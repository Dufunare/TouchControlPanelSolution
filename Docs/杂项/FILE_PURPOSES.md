# 每个文件是做什么的（按当前项目）

## shared

### `shared/DeviceState.h`
前后端共享状态结构，例如触笔的位置、设备是否初始化等，用于多端同步数据格式。

---

## ControlBackend
纯 C++ 库，**不依赖 Qt**。

### `ControlBackend/include/TouchBackend.h` & `ControlBackend/src/TouchBackend.cpp`
- OpenHaptics 设备 API 封装，scheduler 异步回调注册。
- 原子变量维护空间坐标系的快照读取。

### `ControlBackend/include/CommunicationBackend.h` & `ControlBackend/src/CommunicationBackend.cpp`
- 脱离 Qt，采用系统 API (Winsock) 和 `std::thread` 的双工 TCP 客户端通信类。
- 用 `std::function` 提供连接、断开、错误、数据抵达的回调事件。

---

## TouchControlPanelApp/src

### `TouchControlPanelApp/src/main.cpp`
程序入口：创建 QApplication 注册元类型等。

### `TouchControlPanelApp/src/DeviceStateQt.h`
Qt 元类型桥接头。

### `TouchControlPanelApp/src/DeviceController.h` / `.cpp`
- **中枢大脑**：掌控前后端纽带。
- 初始化 `TouchBackend` 和 `CommunicationBackend`。
- 维护双引擎：`m_pollTimer (8ms)` UI同步刷帧；`m_motionTimer (33ms)` 定期对目标网络设备投递 Teleop / Drag 指令包。
- 处理 `StatusPanelWidget` 的日志路由和分发。

### `TouchControlPanelApp/src/MainWindow.h` / `.cpp`
- 界面“主控平台”与积木拼装者。
- 左：视频 + 3D渲染；右：控制交互面板 + 三区状态日志面板。

---

## TouchControlPanelApp/src/widgets

### `widgets/ControlPanelWidget.h` / `.cpp` (新演进)
- 集中处理用户输入与行为指令：
- Touch 的初始化与启停。
- 机械臂网络 IP 重写与连接发起。
- 工作模式切换（Drag、Teleop）。
- 急停指令 (E-stop) 的独立触发下发。

### `widgets/StatusPanelWidget.h` / `.cpp` (职责净化)
- 清除了原先杂乱的按钮。
- 全职日志监控窗口，水平/垂直区域切割为三块：系统常规日志 (Log)、发送报文监控 (TX)、接收报文监控 (RX)。

### `widgets/GLCoordinateWidget.h` / `.cpp`
- OpenGL 3D 显示控件实现（涵盖坐标系、触笔几何形态动态重绘、鼠标交互视角）。

### `widgets/VideoWidget.h` / `.cpp`
- 视频监控与回传的占位/展示区。
