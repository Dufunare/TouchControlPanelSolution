# Touch Control Panel Solution

基于 OpenHaptics 和纯 C++ 通信框架，为遥操作机械臂精心设计的模块化上位机系统。

## 当前架构亮点

*   **纯 C++ 通信后端**：完全抛弃 Qt 网络库，封装 `CommunicationBackend` (Winsock + std::thread)，便于与 `TouchBackend` 同样实现高跨平台迁移性。
*   **物理发送与界面刷新解耦**：前台展示依靠 `8ms` 极致帧率定时器，后台通信驱动依托 `33ms` 高效稳健定时器同步发送 Teleop / Drag 数据包。
*   **功能分立的前端面板**：
    *   **控制枢纽** (`ControlPanelWidget`)：掌控设备启停、TCP 连接和各种操控模式交互。
    *   **独立监控** (`StatusPanelWidget`)：三独立视图全记录（常规日志、TX 报文通信记录、RX 报文记录），链路问题一目了然。

## 工作空间模块

1.  **ControlBackend**
    - 提供纯粹的 OpenHaptics (TouchBackend) 封装。
    - 提供纯粹的 Windows TCP 网络 (CommunicationBackend) 脱钩式封装。
2.  **TouchControlPanelApp**
    - Qt 上位机界面主进程，集成展示双分屏（视图与面板交火）。
    - 统筹数据双流，跨线程连接界面、底层触觉设备及远端机械臂主机。
3.  **shared**
    - `DeviceState.h` 高效数据快照交互结构体。

## 快速指南

1.  使用 Visual Studio 打开 `TouchControlPanelSolution.slnx`。
2.  编译静态纯 C++ 库 `ControlBackend`。
3.  编译执行 `TouchControlPanelApp`。
4.  在右方 **ControlPanelWidget** 修改 IP 并连接机械臂，初始化设备并操作。
5.  在右侧 **StatusPanelWidget** 监控网络指令与 TX/RX 吞吐率。

## 详细架构参见 Doc 文件夹
- ARCHITECTURE.md
- DEVELOPER_GUIDE.md
