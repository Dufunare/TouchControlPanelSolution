# OpenHaptics + Qt 控制面板架构文档（按当前工程更新）

## 1. 文档目标

本文件描述当前仓库已经落地的架构，而不是理想化蓝图。重点回答三件事：

- 现在代码是怎样分层的
- 数据和线程是怎样流动的
- 后续扩展时应该改哪里，避免把工程改乱

当前方案是：

- 后端静态库：`ControlBackend`
- 前端 Qt 程序：`TouchControlPanelApp`
- 共享状态：`shared/DeviceState.h`

## 2. 当前工程分层

### 2.1 设备后端层（ControlBackend）

职责：

- 管理 OpenHaptics 设备初始化与释放
- 注册和取消 scheduler 回调
- 在 servo callback 内读取当前位置
- 保存线程安全的最新状态快照
- 通过最小接口给前端提供状态读取

入口接口：`TouchBackend`（PImpl 结构）

### 2.2 Qt 控制层（DeviceController）

职责：

- 响应 UI 发出的初始化/启动/停止信号
- 驱动后端生命周期
- 用 `QTimer` 轮询后端最新状态
- 把状态和消息转发给 UI

当前轮询周期：`8ms`（约 125 FPS）

### 2.3 Qt 界面层（MainWindow + Widgets）

职责：

- 组织主布局
- 展示状态与日志
- 渲染 3D 坐标场景
- 预留视频显示区域

当前主要控件：

- `VideoWidget`：视频占位/帧显示（含 NO SIGNAL 状态）
- `GLCoordinateWidget`：3D 坐标轴、边框、触笔点位可视化
- `StatusPanelWidget`：状态 + 控制按钮 + 日志

## 3. 当前主界面结构

```text
MainWindow
└─ QSplitter(水平)
   ├─ 左侧 QSplitter(垂直)
   │  ├─ 上：VideoWidget
   │  └─ 下：GLCoordinateWidget
   └─ 右侧：StatusPanelWidget
```

说明：

- 左侧分为视频区和 3D 区，比例默认约 2:1
- 右侧是状态与控制面板
- 这种结构已经支持“视频 + 三维坐标 + 控制面板”的阶段化演进

## 4. 当前数据流与线程模型

```text
Touch 设备
  ↓
OpenHaptics scheduler 回调（后端线程）
  ↓
TouchBackend 原子变量更新（x/y/z、valid、running、sampleCounter）
  ↓
DeviceController(QTimer 8ms，Qt 主线程)
  ↓
MainWindow 分发
  ├─ GLCoordinateWidget::setDeviceState
  └─ StatusPanelWidget::setDeviceState
```

关键规则：

- OpenHaptics 的 `hdGet*` 调用放在 haptics frame 中执行
- QWidget 只在 Qt 主线程更新
- UI 不直接访问 OpenHaptics API，只访问 `TouchBackend`

## 5. 核心类职责（按当前代码）

### `touchpanel::TouchBackend`

- `initialize()`：初始化设备并清理错误状态
- `start()`：注册异步回调并启动 scheduler
- `stop()`：停止采集并取消回调
- `latestState()`：返回快照（位置、初始化状态、有效性、运行状态、采样计数）
- `lastError()`：返回最近错误文本

### `DeviceController`

- 控制后端生命周期
- 维护 `QTimer` 轮询
- 发送 `deviceStateUpdated` 和 `backendMessageChanged`
- 检测 scheduler 异常停止并自动停表、上报消息

### `GLCoordinateWidget`

- 基于 `QOpenGLWidget` 渲染 3D 场景
- 当前包含：边框、RGB 轴、箭头、原点、触笔几何
- 支持鼠标旋转与滚轮缩放
- 局部覆盖文字显示 XYZ 数值

### `StatusPanelWidget`

- 显示初始化状态、采样循环状态、数据有效性、采样计数
- 提供 3 个控制按钮：初始化、启动、停止
- 内置只读日志窗口，支持右键清空日志

### `VideoWidget`

- 管理视频帧显示与无信号状态
- 线程安全接收 `QImage`
- 保证跨线程调用时回到 UI 线程刷新
- 目前 `setFrameFromMat()` 仍为占位实现

## 6. 当前目录与模块边界

```text
TouchControlPanelSolution/
├─ ControlBackend/
│  ├─ include/TouchBackend.h
│  └─ src/TouchBackend.cpp
├─ TouchControlPanelApp/
│  └─ src/
│     ├─ DeviceController.h/.cpp
│     ├─ MainWindow.h/.cpp
│     ├─ main.cpp
│     └─ widgets/
│        ├─ GLCoordinateWidget.h/.cpp
│        ├─ StatusPanelWidget.h/.cpp
│        └─ VideoWidget.h/.cpp
└─ shared/
   └─ DeviceState.h
```

边界约束：

- 共享状态定义只放在 `shared/`
- OpenHaptics 细节只放在 `ControlBackend`
- UI 拼装只在 `MainWindow`
- 功能显示逻辑尽量沉到对应 Widget

## 7. 已实现能力与未实现占位

已实现：

- 设备初始化与采样循环启动/停止
- 位置数据采集与快照读取
- Qt 侧高频轮询和状态展示
- OpenGL 3D 轨迹点位可视化（当前为点位/触笔）
- 视频区域组件化占位（可接入外部图像源）

当前占位/后续方向：

- `VideoWidget::setFrameFromMat()` 尚未接入 OpenCV Mat 转换
- 还未加入按钮状态、速度、力反馈、校准流程
- 还未拆分独立的视频控制器或数据服务

## 8. 扩展建议（结合现状）

### 8.1 增加更多设备状态

推荐顺序：

1. 扩展 `shared/DeviceState.h`
2. 在 `ControlBackend/src/TouchBackend.cpp` 中填充字段
3. 在 `StatusPanelWidget` 或 `GLCoordinateWidget` 增加展示

### 8.2 接入真实视频流

推荐顺序：

1. 保持 `VideoWidget` 仅做显示
2. 新增视频采集/解码服务（后端或独立线程）
3. 在 UI 线程通过 `setFrame()` 投喂 `QImage`
4. 如需 OpenCV，补齐 `setFrameFromMat()` 的像素格式转换

### 8.3 新增业务功能

例如轨迹录制、任务控制、参数面板：

- 新增独立 Widget/Controller
- `MainWindow` 只做装配
- 避免把业务流程直接写入 `MainWindow.cpp`

## 9. 当前与旧文档的关键差异

- 后端项目名是 `ControlBackend`，不是 `TouchBackendLib`
- 主界面已是“三分区”：视频 + 3D + 状态控制
- 轮询周期是 `8ms`，不是 `16ms`
- 状态面板当前主要显示运行状态与日志，XYZ 数字叠加在 3D 视图

以上内容均按当前仓库代码同步更新。
