# OpenHaptics + Qt + 机械臂控制架构文档（按当前工程更新）

## 1. 文档目标

本文件描述当前仓库已经落地的架构，而不是理想化蓝图。重点回答三件事：

- 现在代码是怎样分层的
- 数据和线程是怎样流动的
- 后续扩展时应该改哪里，避免把工程改乱

当前方案是：

- 后端静态库：`ControlBackend`（含触觉与纯 C++ 通信模块）
- 前端 Qt 程序：`TouchControlPanelApp`
- 共享状态：`shared/DeviceState.h`

## 2. 当前工程分层

### 2.1 设备后端层（ControlBackend）

职责：

- **触觉控制** (`TouchBackend`)：管理 OpenHaptics 设备初始化与释放，注册 scheduler 回调，在 servo callback 内读取当前位置，保存线程安全的最新状态快照。
- **网络通信** (`CommunicationBackend`)：纯 C++ 实现（基于 Winsock 和 `std::thread`），彻底脱离 Qt 网络库。负责与机械臂或中转端进行 TCP 通信，通过 `std::function` 回调向上传递事件。

### 2.2 Qt 控制层（DeviceController）

职责：

- 整合 `TouchBackend` 与 `CommunicationBackend`。
- 响应 UI 发出的触觉设备启动/停止以及机械臂连接/移动信号。
- 用双 `QTimer` 驱动业务：
  - `m_pollTimer`：`8ms` 周期，用于高频轮询后端状态并更新 UI。
  - `m_motionTimer`：`33ms` 周期，用于定期向机械臂下发 Teleop（遥操作）或 Drag（拖拽）指令。
- 把状态、网络接收数据和日志转发给 UI。

### 2.3 Qt 界面层（MainWindow + Widgets）

职责：

- 组织主布局
- 展示状态、通信字节（TX/RX）与日志的专属面板
- 提供触控笔和机械臂控制面板
- 渲染 3D 坐标场景
- 预留视频显示区域

当前主要控件：

- `VideoWidget`：视频占位/帧显示（含 NO SIGNAL 状态）
- `GLCoordinateWidget`：3D 坐标轴、边框、触笔点位可视化
- `ControlPanelWidget`：负责 Touch 初始化/控制，以及机器人控制（支持 IP 覆盖、急停、拖拽/遥操作控制模式）。
- `StatusPanelWidget`：专属日志面板，包含常规日志、TCP TX（发送）字节和 RX（接收）字节显示。

## 3. 当前主界面结构

```text
MainWindow
└─ QSplitter(水平)
   ├─ 左侧 QSplitter(垂直)
   │  ├─ 上：VideoWidget
   │  └─ 下：GLCoordinateWidget
   └─ 右侧 QSplitter(垂直)
      ├─ 上：ControlPanelWidget
      └─ 下：StatusPanelWidget
```

## 4. 当前数据流与线程模型

```text
Touch 设备                            机械臂/中转站
  ↓                                      ↑↓
OpenHaptics callback             CommunicationBackend (C++ std::thread)
  ↓                                      ↑↓
TouchBackend(原子变量)             异步网络回调 (std::function)
  ↓                                      ↑↓
         DeviceController (Qt 主线程)
         ├─ m_pollTimer (8ms): 读取 Touch 状态并刷新 UI
         └─ m_motionTimer (33ms): 读取坐标并下发 TCP 运动指令
  ↓
MainWindow 分发
  ├─ GLCoordinateWidget::setDeviceState
  ├─ StatusPanelWidget (写日志、TX/RX)
  └─ ControlPanelWidget (更新连接状态等)
```

关键规则：

- OpenHaptics 调用在其专属 haptics 线程执行。
- 网络收发在纯 C++ 的后台 `std::thread` 中执行。
- UI 不直接访问 OpenHaptics 或 Winsock，全权由 `DeviceController` 跨线程统筹（Qt 信号槽或定时器）。

## 5. 核心类职责（按当前代码）

### `CommunicationBackend` (新)

- 纯 C++ 的 TCP 客户端实现。
- 支持异步连接、断开、发送和接收。
- 采用 `std::thread` 监听数据，通过注册 `std::function` 回调派发连接成功、断开、数据到达和错误事件。

### `touchpanel::TouchBackend`

- 初始化设备、启动 scheduler 并管理数据快照。

### `DeviceController`

- 控制前后端和网络通讯的生命周期。
- 双定时器：`8ms` 更新显示，`33ms` 运动发包。
- 将从 `CommunicationBackend` 收到的日志、报文转发给 `StatusPanelWidget` 显示。

### `ControlPanelWidget`

- 面板集成了 Touch 设备的启停控制。
- 集成了机械臂控制模块：可手动填写覆盖 IP、建立 TCP 连接、触发急停（E-stop），并支持 Drag / Teleop 模式切换。

### `StatusPanelWidget`

- 不再混合复杂的控制按钮。
- 纯粹的状态与日志输出，划分为三个独立显示区：常规 System Log、TX（发送报文）和 RX（接收报文）。

## 6. 旧文档与当前现状的核心差异

- **网络层已无 Qt 痕迹**：移除了针对 `TransitTcpClient` 等基于 Qt 网络的类，由 `ControlBackend` 中的 `CommunicationBackend` 替代，回归纯 C++。
- **UI布局和职责分离**：原本混在 `StatusPanelWidget` 的控制按钮全部提炼并扩充到了新的 `ControlPanelWidget`。`StatusPanelWidget` 专责日志。
- **精确驱动频率**：确立了 8ms UI刷新与 33ms 运动控制发包的解耦策略。

