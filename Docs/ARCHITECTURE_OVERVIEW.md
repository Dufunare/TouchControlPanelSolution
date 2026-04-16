# 架构总览（当前实现）

## 1. 总体目标

当前工程目标是构建一个可扩展的 OpenHaptics + Qt + 机械臂控制框架，已实现：

- Touch 设备坐标采样
- 纯 C++ 异步 TCP 网络通信
- Qt 前端双频次调度与状态刷新
- OpenGL 3D 可视化
- 视频区域占位
- 控制与日志面板的职责分离

## 2. 三层结构

### 设备与网络后端层

模块：`ControlBackend`（纯 C++）

负责：

- `TouchBackend`：OpenHaptics 设备初始化/释放，坐标采样与快照维护。
- `CommunicationBackend`：基于 Winsock 和 `std::thread` 的纯 C++ 网络通信，负责与机械臂或中转站收发数据。

### Qt 控制层

模块：`DeviceController`

负责：

- 接收 UI 发起的设备启停、网络连接、运动控制事件。
- 桥接 `TouchBackend` 与 `CommunicationBackend`。
- `m_pollTimer` (`8ms`)：高频轮询获取状态并刷新前台界面。
- `m_motionTimer` (`33ms`)：定时向下游发送 Teleop/Drag 遥操作运动指令。

### Qt 界面层

模块：`MainWindow + Widgets`

负责：

- 主布局拼装。
- 控制交互面板（`ControlPanelWidget`）。
- 日志与通信报文监测面板（`StatusPanelWidget`）。
- 3D 可视化展示（`GLCoordinateWidget`）。
- 视频画面占位展示（`VideoWidget`）。

## 3. 当前数据流

```text
[Touch设备] & [TCP网络连接]
    ↓                 ↓
[OpenHaptics CB] & [C++ 网络接收线程]
    ↓                 ↓
[DeviceController 跨线程整合与定时驱动]
    ├─ m_pollTimer (8ms) -----> 界面全面刷新
    └─ m_motionTimer (33ms) --> 发送控制报文
    ↓
[MainWindow 分发给各 Widget]
    ├─ ControlPanelWidget：处理用户点击指令
    ├─ GLCoordinateWidget：3D 视图刷新
    └─ StatusPanelWidget：显示常规日志、TX日志、RX日志
```

## 4. 当前 UI 结构

```text
MainWindow
└─ QSplitter(水平)
   ├─ 左：QSplitter(垂直)
   │  ├─ VideoWidget
   │  └─ GLCoordinateWidget
   └─ 右：QSplitter(垂直)
      ├─ ControlPanelWidget (触控、网络与运动控制)
      └─ StatusPanelWidget (常规日志、TX/RX 监控)
```

## 5. 关键演进事实

- 网络层摆脱了 Qt，转移为纯 C++ 的 `CommunicationBackend`，以提升核心库的可复用性。
- 面板职责已清晰拆分，原先混合在状态栏的控制逻辑独立至 `ControlPanelWidget`。
- 新增了分类日志记录功能，便于专门追踪 TX/RX 指令。
