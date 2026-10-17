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

## 下一步实现
1. 完善机械臂状态监控和异常处理机制
2. 统一消息格式，增加 RequestId 以匹配请求与响应
3. 接入视频流显示，设计 `VideoWidget`.



## 详细架构参见 Doc 文件夹
- ARCHITECTURE.md
- DEVELOPER_GUIDE.md

## UVC 摄像头预览（中转站视频链路）

当前控制平台通过 relay_station 已有的 JPEG/UDP 兼容链路观看摄像头：纽曼 UVC 摄像头连接在**中转站电脑**上，由中转站用 OpenCV DirectShow 采集（现有代码设备索引 0、640×480、JPEG 品质 60），无需在控制平台安装摄像头驱动或 OpenCV。视频与机械臂 TCP 控制相互独立。

1. 在中转站电脑插入摄像头，确认 Windows 能识别且未被其他程序占用，启动 relay_station。
2. 控制平台的「视频控制」中填写中转站 IPv4（留空默认使用当前 TCP 连接面板里的 IP，否则退回 `192.168.100.194`），UDP 端口默认 `8886`。
3. 点击「开始采集」。客户端会在随机本地 UDP 端口绑定后向中转站的 `8886` 发送 `VIDEO_START`，每 2 秒续订；中转站向客户端 UDP 源端口回传原始 JPEG 单帧。
4. 视频窗会显示图像和实测接收 FPS。可切换镜像显示，点击「抓拍」保存当前帧为 PNG。点击「停止采集」会发 `VIDEO_STOP` 并关闭 UDP Socket。
5. 若断流超过 2.5 秒，界面显示 NO SIGNAL；5 秒无帧会提示检查摄像头、relay_station、网络和防火墙。注意需允许中转站 `UDP 8886` 入站及控制端的**动态 UDP 回程端口**入站。

当前帧率由中转站采集实现决定，控制平台中帧率输入被禁用以免误导。该链路不是 RTSP、不支持直接打开控制平台本机 USB 摄像头，也不具备跨公网可靠传输与视频分片：中转站 JPEG 超过约 65 KB 的帧会丢弃。在受信任的局域网内使用，摄像头画面不会影响触觉控制和 ServoP 指令逻辑。
