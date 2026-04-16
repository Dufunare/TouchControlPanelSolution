# TouchControlPanel 项目开发者指南

## 一、项目背景与目标

本项目为 OpenHaptics 触觉设备和机械臂上位机开发了一套“易扩展、高度解耦”的 Qt 控制面板。

- **后端中立性**：ControlBackend 彻底去 Qt 化，纯 C++ 编写，方便异构平台复用。
- **定时与调度分离**：UI 刷新机制与物理操控发包机制分离，确保响应顺滑与发包稳定。
- **清晰的 UI 模块**：操作区、展示区与日志区各尽其责。

## 二、架构设计理念

### 1. 解耦思想

- **前后端彻底分离**：后端 (TouchBackend, CommunicationBackend) 采用 std::thread 和标准回调，绝不引用 Qt。前端控制器 (DeviceController) 负责接驳各种后端。
- **渲染与逻辑隔离**：界面控件只反映当前数据快照，控制指令全部发送给 Controller，由它调动业务。

### 2. 双定时器机制

DeviceController 使用了两个核心定时器，请开发者严格遵守这种频次划分：

- **8ms `m_pollTimer`**：纯用于读取状态、刷新状态指示器和重绘 3D 界面（约 125 FPS）。
- **33ms `m_motionTimer`**：专门用于 TCP 发送控制报文（如 Drag 或 Teleop 模式下的坐标）。

## 三、各层职责与类说明

| 层级       | 模块 / 典型类                 | 主要职责                     |
| ---------- | --------------------------- | ---------------------------- |
| 设备后层   | TouchBackend                | OpenHaptics API 操作与坐标快照 |
| 通信后层   | CommunicationBackend        | 纯 C++ 的网络收发 (Winsock)     |
| 控制调度   | DeviceController            | 双定时器调度，统一处理设备与网络事件 |
| 交互视图   | ControlPanelWidget          | IP覆盖、连接、急停、触控启停操作面板 |
| 日志视图   | StatusPanelWidget           | 分区记录系统信息、TX报文、RX报文   |
| 共享结构   | shared/DeviceState.h        | 简单轻量的跨线程数据投递结构        |

## 四、当前协作与开发建议

### 1. 添加新通信协议

若是增添 UDP 或是串口，请在 ControlBackend 使用纯 C++ 仿照 CommunicationBackend 开发，然后在 DeviceController 里进行初始化与信号包装，严禁在后端引入 QTcpSocket。

### 2. 修改 UI 或新增控制面板

现在交互已经独立到了 ControlPanelWidget，如果是加机械臂夹爪控制等，请直接在这个 Widget 中加按钮，然后用 Signal 抛给 DeviceController 拼装数据发给网络模块。TX 日志会自发呈现在 StatusPanelWidget。

### 3. 主窗口约束

MainWindow 只做布局分配：左半边给视频和 3D，右半边给控制区和日志区。不要把业务逻辑或者定时器写在这里！
