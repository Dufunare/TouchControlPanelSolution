# 后续怎么加新功能（按当前项目）

目前项目确立了明确的“纯 C++ 后端 + Qt Controller 调度 + 分区 Widget”的架构模式。

## 场景 A：新增控制功能或硬件接入

例如：给机械臂新增夹爪控制、或者调整网络配置。

推荐做法：

1. **界面层**：修改 `ControlPanelWidget`，添加新的输入框或按钮（如夹爪操作按钮）。
2. **逻辑层**：在 `DeviceController` 添加公共槽协作函数（如 `onGripperButtonClicked`），从界面接收信号。
3. **后端层**：在 `DeviceController` 内部通过 `m_communicationBackend` 下发纯 C++ 的通讯报文。相关的网络发包结果或内容会自动在 `StatusPanelWidget` 的 TX/RX 日志面板体现。

## 场景 B：增加新的网络通信设备

例如：接入新的力矩传感器串口。

推荐做法：

1. **建立纯 C++ 后端**：在 `ControlBackend/src/` 中创建无 Qt 依赖的设备访问类，通过 `std::thread` 采集，利用 `std::function` 回调派发。
2. **控制器调度**：在 `DeviceController` 持有该对象，并配置数据轮询或事件监听。
3. **展示数据**：若需要监控此传感器字节流，可以将其发送给 `StatusPanelWidget` 中的独立日志框。

## 场景 C：修改轮询或控制流频率

项目目前对频率进行了解耦：
- `m_pollTimer (8ms)`: 面部刷新、UI 同步。
- `m_motionTimer (33ms)`: 业务发包、遥操作下达。

如果未来新的控制器要求控制下发频率为 100Hz (10ms)，请**专门修改 `m_motionTimer`** 的参数，切莫动用 `m_pollTimer` 否则会引爆 UI 重绘占用率。

## 场景 D：接入真实视频流

当前项目已有 `VideoWidget` 作为纯净的显示层。

推荐接入路线：

1. 新增独立的视频服务模块。
2. 将原始图像通过通道转为 `QImage`。
3. 调用 `VideoWidget::setFrame()`。

请勿将大图像耦合进 `DeviceState` 或现有的 8ms 轮询体内。

## 核心延展禁忌检查清单

1. **不可在 `ControlBackend` 引入 `QTcpSocket` 等特定 GUI 框架的网络构件**，必须坚持纯 C++ 标准库或系统 API（如 Winsock）。
2. **不可将发包动作掺和进 `m_pollTimer` (8ms)**，物理下发统一走独立的控制定时器。
3. **`StatusPanelWidget` 不可再塞入控制按钮**，它现为纯粹的“三流输出”监控台（Log, TX, RX）。
