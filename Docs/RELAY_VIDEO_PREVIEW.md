# 中转站 UVC 视频与 3D 预览布局

## 集成来源

本分支复用 PR #2 的 `feat/relay-uvc-video-preview`（55be40c）中的 UDP JPEG 协议和接收器，按 PR #3 当前界面及遥控性能要求重新接入。中转站 master 的 OpenCV UVC 采集协议不变；不需要在控制面板安装 OpenCV 或 QtMultimedia。

- 左侧设置中转站 IPv4（空白使用 192.168.100.194），中央视频区默认 UDP 8886。
- 点击“开始视频”订阅，2s 续订 VIDEO_START；停止或关闭程序发送 VIDEO_STOP。视频连接与 8888/8891 控制连接独立。
- 接收器只接受指定中转站 IP/UDP 端口的 JPEG 单包，限制尺寸与解码工作量，2.5s 无有效画面清空显示并等待恢复；启动超时提示检查中转站、摄像头及 UDP 防火墙。
- 中央显示视频和 3D 两个独立的垂直分割区域；两侧控制/状态栏各自可滚动，停止遥控和软件急停保留在固定工具栏。
- 视频镜像仅影响显示；截图保存最新有效原始帧 PNG。实体遥控期间不弹出文件对话框。
- 视频网络接收、JPEG 解码、续订和断流 watchdog 位于独立 QThread；由 readyRead 到包触发，工作线程单次最多读取64个数据报，只解码排空后的最新候选；解码尝试间隔约33ms，过载主动丢弃而非逐帧排队。线程通过互斥保护的最新 QImage 邮箱发布帧，不跨线程操作 QWidget。GUI 每40ms检查最新帧，最多25次/s刷新；状态 FPS 是接收解码速率，而非远端相机设置或最终绘制速率。
- 3D 最小尺寸由350×240改为180×140；明确禁用 WA_AlwaysStackOnTop，绘制限制在控件 rect，中央 3D 具有独立容器。视频改用 QPainter 在固定控件区域等比例绘制，避免 QLabel pixmap 尺寸反推布局。左栏最低300px、右栏270px，侧栏允许横向滚动，窗口最低940×620。

## 验证

`tests/qt-video-smoke` 和 `.github/workflows/qt-video-smoke.yml` 在 Ubuntu/Qt6 软件 OpenGL 环境编译视频、3D、左右栏组件，使用本地 UDP 模拟订阅、JPEG 接收、PNG 保存、断流与退订，并检查分割区域几何边界。它不编译依赖 Windows/OpenHaptics 的完整 MainWindow，也不接入真实机器人或摄像头。现场必须使用目标 Windows Qt 工程重新生成、验证显卡驱动/DPI下的布局、真实中转站画面及控制周期；未通过现场验证前不得宣称遮挡和遥控性能已完全解决。

本轮没有修改任何机械臂目标生成、工作空间、离合、反馈安全逻辑或中转站 master。

## 性能退化专项：早期视频分支 vs 当前版本

用户实测 `feat/relay-uvc-video-preview`（55be40c）运动更快、无明显追赶等待；集成视频的 3bbdbe1 又比上一版 e12901b 更卡。以下是基于三个固定提交的源码对照，而不是新的实体测量：

- e12901b 与 3bbdbe1 的 DeviceController.cpp/.h、ControlBackend/src/CommunicationBackend.cpp、TouchBackend.cpp 文件 blob SHA 完全相同。故这次新增卡顿不是这些文件中的运动算法被改写，而应调查新增视频/绘制负载、共享 Qt GUI 事件循环和同网段视频流。尚无该版 TickGapMax、反馈年龄和端到端时延数据，不可认定唯一根因。
- 55be40c 的 DeviceController.cpp L276–383 每33ms取最新 Touch，查询基准后直接映射坐标，0.3mm重复过滤；CommunicationBackend.cpp L367–382 逐轴夹取并发送固定RPY的 ServoP。当前 ServoPCommandPlanner.h L60–136 保持20mm目标跳变和20mm实际TCP超前包络，可能按反馈停发；独立25mm跟踪异常停止、离合范围、真实姿态锁定、8891实时反馈门禁依然存在。这是长期运动响应不同的实质，不是视频显示延迟。原始分支无这些保护，不能视为经过认证的速度/加速度/停止距离基线。
- 早期 RelayVideoClient.cpp L21、L99–135 使用 readyRead 直接接收 JPEG；VideoWidget.cpp L106–135 在新帧到达时缩放 QPixmap。3bbdbe1 使用33ms轮询＋40ms GUI 最新帧轮询，并在 paintEvent 每次做图像缩放。现在恢复到包触发且限频/批量排空，继续保留独立解码线程及最新帧邮箱，GUI 仍限25fps。缓存等比缩放的 QPixmap，只有新帧、尺寸、镜像或DPI变化才重建；普通重复绘制只 blit。快速缩放优先降低开销而非画质；原始 PNG 截图不受影响。
- 3bbdbe1 GLCoordinateWidget.cpp L207–287 每次 paintGL 重新生成笔身/姿态环和轨迹，上传三个 VBO。现笔身和姿态环在局部坐标中一次生成/上传，每次变换着色器矩阵即可；轨迹只在内容改变时上传。尖端十字随局部姿态旋转，中心不变。视频订阅期间3D样本触发绘制间隔至少60ms（状态有效性转换除外），没有视频时维持原刷新逻辑；可在中央选项中隐藏3D预览，纯显示行为。
- Touch状态计数标签从约30Hz降到5Hz；初始化/校准/有效性标签仅值变化时更新。实际采样、控制周期、反馈处理、软件停止/急停不变。没有移动 DeviceController 或 RelayStateClient 到其他线程；Qt UI 和控制尚未彻底隔离。
- 之前 Qt Video Preview Smoke (run 45616408226) 已通过；新修改后的隔离 smoke 扩充有效位姿 OpenGL framebuffer 检查。Windows 完整工程、GPU实际绘制性能以及机械臂真实响应均未验证。无新的实际运动测试，不声称已恢复55be40c的运动速度。
