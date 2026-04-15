# 架构总览（当前实现）

## 1. 总体目标

当前工程目标是构建一个可扩展的 OpenHaptics + Qt 控制面板基础框架，已实现：

- Touch 设备坐标采样
- Qt 前端状态刷新
- OpenGL 3D 可视化
- 视频区域占位

## 2. 三层结构

### 设备后端层

模块：`ControlBackend`

负责：

- OpenHaptics 设备初始化/释放
- scheduler 回调管理
- 坐标采样与快照维护

### Qt 控制层

模块：`DeviceController`

负责：

- 接收 UI 控制事件
- 调用后端 start/stop/initialize
- 以 `8ms` 定时轮询后端状态
- 发送状态与消息信号

### Qt 界面层

模块：`MainWindow + Widgets`

负责：

- 主布局拼装
- 3D 可视化展示
- 状态与日志展示
- 视频画面占位展示

## 3. 当前数据流

```text
[Touch设备]
    ↓
[OpenHaptics scheduler callback]
    ↓
[TouchBackend 原子状态更新]
    ↓
[DeviceController(QTimer 8ms) 读取 latestState()]
    ↓
[MainWindow 分发给各 Widget]
    ├─ GLCoordinateWidget：3D 视图
    └─ StatusPanelWidget：状态与日志
```

## 4. 当前 UI 结构

```text
MainWindow
└─ QSplitter(水平)
   ├─ 左：QSplitter(垂直)
   │  ├─ VideoWidget
   │  └─ GLCoordinateWidget
   └─ 右：StatusPanelWidget
```

## 5. 扩展原则

- 共享状态放 `shared/`
- 设备逻辑放 `ControlBackend`
- 显示逻辑放独立 Widget
- 主窗口只做装配，不堆业务

## 6. 当前关键事实

- 后端项目名已是 `ControlBackend`
- 视频区已存在组件 `VideoWidget`
- 当前轮询频率为 `8ms`
- `StatusPanelWidget` 侧重状态与日志，XYZ 文本叠加在 3D 视图中
