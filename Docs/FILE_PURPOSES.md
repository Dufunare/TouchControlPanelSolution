# 每个文件是做什么的（按当前项目）

## shared

### `shared/DeviceState.h`

前后端共享状态结构。

当前字段：

- `positionMm[3]`：XYZ 位置（mm）
- `initialized`：后端是否初始化
- `valid`：当前采样是否有效
- `schedulerRunning`：scheduler 是否运行
- `sampleCounter`：采样计数

---

## ControlBackend

### `ControlBackend/include/TouchBackend.h`

后端对外公开接口，前端只依赖这个头文件。

主要接口：

- `initialize()`
- `start()`
- `stop()`
- `latestState()`
- `lastError()`

### `ControlBackend/src/TouchBackend.cpp`

后端核心实现，负责：

- OpenHaptics 设备初始化与关闭
- scheduler 异步回调注册与取消
- 在回调中读取 `HD_CURRENT_POSITION`
- 用原子变量维护状态快照
- 错误信息维护与返回

---

## TouchControlPanelApp/src

### `TouchControlPanelApp/src/main.cpp`

程序入口：

- 创建 `QApplication`
- 注册 `touchpanel::DeviceState` 元类型
- 设置 OpenGL 默认格式
- 创建并显示 `MainWindow`

### `TouchControlPanelApp/src/DeviceStateQt.h`

Qt 元类型桥接头，用于信号槽传递 `touchpanel::DeviceState`。

### `TouchControlPanelApp/src/DeviceController.h`

控制器声明，定义：

- 初始化/启动/停止槽函数
- 状态更新和消息更新信号
- 定时轮询槽函数

### `TouchControlPanelApp/src/DeviceController.cpp`

控制器实现，负责：

- 响应状态面板按钮动作
- 在未初始化时自动尝试初始化
- 用 `QTimer` 每 `8ms` 轮询后端
- 在采样循环异常停止时自动停表并提示

### `TouchControlPanelApp/src/MainWindow.h`

主窗口声明，持有：

- `TouchBackend`
- `DeviceController`
- `VideoWidget`
- `GLCoordinateWidget`
- `StatusPanelWidget`

### `TouchControlPanelApp/src/MainWindow.cpp`

主窗口装配：

- 创建控制器与各 Widget
- 构建左右/上下分割布局
- 连接按钮控制和状态分发信号
- 把后端消息同步到状态栏

### `TouchControlPanelApp/src/TouchControlPanelApp.ui`

Qt Designer 生成的基础 UI 文件。当前主界面布局主要由 `MainWindow.cpp` 手写装配。

---

## TouchControlPanelApp/src/widgets

### `widgets/GLCoordinateWidget.h`

OpenGL 3D 显示控件声明。

### `widgets/GLCoordinateWidget.cpp`

OpenGL 3D 显示实现，包含：

- 坐标轴与箭头
- 立方边框
- 原点与触笔几何
- 鼠标旋转与滚轮缩放
- 左上角 XYZ 文本叠加

### `widgets/StatusPanelWidget.h`

状态面板声明，包含状态标签、消息框和控制信号。

### `widgets/StatusPanelWidget.cpp`

状态面板实现，包含：

- 初始化/运行/有效性/计数显示
- 初始化、启动、停止三个按钮
- 时间戳日志输出
- 日志右键菜单（清空）

### `widgets/VideoWidget.h`

视频显示控件声明，提供：

- `setFrame(QImage)`
- `setFrameFromMat(cv::Mat)`（占位）
- 无信号显示切换

### `widgets/VideoWidget.cpp`

视频显示控件实现，包含：

- 线程安全帧缓存
- 跨线程回到 UI 线程刷新
- 自适应缩放显示
- `NO SIGNAL` 覆盖层显示
