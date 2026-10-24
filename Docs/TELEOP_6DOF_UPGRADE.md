# TouchControlPanel 六自由度遥操作升级：设计与分阶段落地

> 状态：**阶段 A（六维采样与离线映射 + 禁止物理运动）**。本分支没有启用新的真实运动指令。基于 `master@894ff8f`，不以放行 ServoP 的 relay_station PR 为前提。

## 仓库现状与必须消除的风险

- 原来 TouchBackend 仅采集 `HD_CURRENT_POSITION`（XYZ），无姿态；`ServoP(...,150,0,90)` 的后三维始终固定。
- 原来按住笔按钮后同时请求 `RobotMode()` 和 `GetPose()`，但 CommunicationBackend 只允许一个待确认操作；容易冲突。
- 接收端按换行及在 pending 存在时提前消费未完整缓冲；中转站实际采用 `;` 作为 CR 回复帧边界，故仍需重写可靠协议层。
- `lastRobotMode` 和 `lastPose` 未关联连接 generation、请求代次或采集时戳；旧缓存禁止用于实际运动起点。
- `ServoP` 是无应答连续笛卡尔伺服；不能把 30Hz ServoP 无条件改成 30Hz MovL（MovL 为运动队列指令）。
- 现有 `relay_station/master` 的 8888 正式禁用 ServoP、8891 v2 运动同样禁用；其 8891 状态消息中已有 `q_actual`、`tool_vector_actual`、`robot_mode`、`enable_status`、`error_status` 和序号/时间戳。

## 本阶段实际实现

- 读取 OpenHaptics `HD_CURRENT_TRANSFORM` 4x4 列主序矩阵，独立导出 3x3 行主序旋转，校验正交性和右手系；保留原始 XYZ 和 Button1。
- 新增独立 `TeleopGeometry.h` C++17 几何库：相对 clutch 位姿映射、固定轴映射、三对不共线点的轴标定和尺度一致性检验、欧拉角 ZYX 编码、欧拉奇异位形拒绝、工件 XYZ 安全盒拒绝、六关节限位判定、最大步长判定。
- **原 `onMotionTick` 不再调用 sendMotion**：开始遥操作只做 dry-run 六维预览；故即使中转站日后启用 ServoP 也不会因本控制路径触发连续运动。
- GUI 轮询从误用的 33ms 常量改为定义的 8ms。工作空间、关节约束仍属于未配置状态，不会自作主张使用 CR3 的关节范围。
- `tests/teleop_geometry_tests.cpp` 是不依赖 OpenHaptics 和机器人硬件的离线断言测试；可执行 `g++ -std=c++17 tests/teleop_geometry_tests.cpp -o teleop_test && ./teleop_test`。

## 六维坐标系约定（暂为软件假设，不可用于真机）

令 Touch 原点/姿态为 `p_h0,R_h0`、机械臂起点为 `p_r0,R_r0`，正交轴映射矩阵为 `A`，位移倍率 `s`：

```
p_r = p_r0 + s A (p_h - p_h0)
R_r = (A R_h R_h0^-1 A^-1) R_r0
```

本版本内部用于预览的欧拉编码是固定轴 Z-Y-X，即 `Rz*Ry*Rx`。**越疆 V3 文档只明确六个坐标与 Rx/Ry/Rz 角度的单位，不足以证明机器人控制器采用相同的欧拉旋转顺序和角度分支；必须在仿真与低速单轴测试中验证！** 不能把 Touch 的机械 gimbal 角直接当做机器人 J4–J6 或 Rx–Rz。Touch 的 6DOF 末端姿态与机器人关节角不是一一对应。

标定函数只实现纯数学的三点配准，不包含现场采点 UI、持久化、离群值剔除与测试误差统计。三点应在统一且已确认的机器人用户/工具坐标系下采集；超过误差门槛需拒绝。OpenHaptics 设备自身的 `hdCheckCalibration/hdUpdateCalibration` 尚未接入设备生命周期，不应将“空间标定”误称为“设备已校准”。

## 下一阶段 B：可靠通信与实时机器人状态

1. 用独立 Qt 8891 client 完成 `hello`、`hello_ack`、session_id、长度前缀 JSON；解析 `robot_state` 和 `relay_status`，绑定状态 generation、sequence、主机单调时钟。
2. 设置机器人状态时效门限（建议初期不超过 300–500ms，实际门限依硬件和网络测量）；无反馈、状态跨代、模式异常立即切断“运动许可”。高频 GUI 与控制策略分别用快照。
3. 29999/30003 命令分通道维护请求关联、响应解析、错误码、超时；8888 Legacy 没有真正 request_id，应序列化运动命令，避免迟到应答污染新的请求。
4. 在软件命令队列以外实现真实机械臂硬件急停与安全互锁；不能把 TCP EmergencyStop 或 disconnect 当作可靠实体急停。

## 下一阶段 C：安全的运动计划

- 先完成机器人 TCP `User` 与 `Tool` 坐标系明确配置、三点或多点刚体标定和固化、Touch 驱动自身校准。再对候选 6D 位姿做工作空间约束、速度/角速度限制、跳变限制与实际姿态误差限制。
- `MovL`：只能按离散目标点进行有界队列/单命令在途控制，机器人反馈表明完成或可终止后再发送下一目标；按键释放后不能默认为停机，必要时需独立验证 ResetRobot/停止策略。
- `InverseSolution(X,Y,Z,Rx,Ry,Rz,User,Tool,...)`：越疆固件提供逆解，可在运动前取得六关节候选解，并与 `q_actual` 及针对**确切 CR 型号**的关节角、关节速度、奇异点阈值比较；相同末端位姿可能有多种逆解，不能忽略解分支。
- `ServoJ`：越疆文档说明六关节在线跟随推荐 33Hz、无普通回复，应只在完成逆解连续性和网络、关节安全层后，经过独立授权启用。中转站 `8891` v2 运动目前生产禁用，不在本 PR 中绕过。
- 关节角限制并不足以证明安全：还需要连杆、工具、环境的碰撞模型及关节加速度/奇异位形校验。对于未知 CR3 安装方式、TCP、负载和固件参数，所有运动许可必须保持 false。

## 验收路径

1. 无机器人：编译几何测试并运行；所有无效/越界输入被拒绝，按键捕获姿态没有坐标跳跃。
2. 仅 Touch：在主界面观察旋转和 XYZ 的 Dry Run 日志，松开与重新按下按钮时基准重置；不应向 30003 下发 ServoP/MovL。
3. 中转站 + 模拟机器人：验证 8891 状态流，乱序、半包、断线、超时、旧帧保护；独立测试 29999/30003 回复隔离。
4. 有实体安全措施且完成审查后，再进行低速、短距离、单轴真机运动。正式遥操作使能须另一个明确的变更审核。

## 参考

- OpenHaptics Toolkit API Reference Guide 3.5.0：`HD_CURRENT_TRANSFORM`、`HD_CURRENT_GIMBAL_ANGLES`、`hdCheckCalibration`、`hdUpdateCalibration`。
- Dobot TCP/IP 远程控制接口 V3：`MovL`、`ServoJ`、`InverseSolution`、`RobotMode` 及 30004 实时反馈。
- Dobot CR 系列使用手册 V1.8：具体 CR 型号的轴运动范围、奇异位置与安装注意事项。

## 阶段 B 增量：8891 只读状态链路与 Legacy 分帧（2026-10-19）

本次已添加以下代码，但 **Windows Qt 真正构建、8891 真实链路联调及物理设备标定尚未完成**：

- `shared/RelayV2Frame.h`：长度前缀的有界 TCP 流分帧 + Legacy 分号/换行边界解析；禁止零长度、超出 1 MiB 的 v2 JSON 报文，出现非法帧立即锁死解析器，直到显式 reset。
- `TouchControlPanelApp/src/RelayStateClient.{h,cpp}`：独立 `QTcpSocket` 连接中转站 8891；采用 `touchvla_windows` client / `hello` / session UUID；订阅 `robot_state`、`relay_status`，严格校验字段、六元素有限值、递增状态序号、端口及 relay_ready。
- 在上位机单调时钟上保存最近有效反馈时间（400ms 本地时效门槛），不依赖跨设备时钟比较；断开、错误、状态过期时清除预览基准。
- `DeviceController` 捕获 Button1 时使用 8891 实时 `tool_vector_actual` 替换无时戳的 Legacy `GetPose` 缓存；本次修复了先前阶段 A 临时实现中每 Tick 可能重读和偏移位姿的问题，锁定按下时原始 TCP 基准；绝不发送运动命令。
- Legacy 8888 接收的 CR3 分号结束回复改为渐进累积完整报文才解析，不再错误吞掉 TCP 半包；断线/新建连接清空 `GetPose/RobotMode` 缓存。
- 软件急停调用时首先取消本地单槽等待，避免因其他普通命令占用请求槽而直接拒绝；这不保证硬件急停成功，不提供已进入机器人命令队列的取消保障。
- Linux CI `.github/workflows/teleop-offline.yml` 运行无硬件依赖的 `teleop_geometry_tests.cpp` / `relay_v2_frame_tests.cpp`；Windows Qt/HDAPI 构建和端到端模拟设备测试仍需单独实施。

### 下一步技术债

1. 彻底重构旧 8888 请求队列与回包关联，明确迟到应答的孤儿回复处理；软件急停需要单独独立高优先级机制，普通 `PendingCommand` 抢占并不保证指令先于底层排队命令执行。
2. `RelayStateClient` 当前仅向控制层暴露新鲜六维 TCP 位姿，尚未将 `q_actual`、速度、时间序列和校验过的机器人运行状态以独立 `RobotSnapshot` API 输出；后续增加完整不可变快照及控制许可状态机。
3. OpenHaptics servo 回调中的位姿由独立原子字段采样，本分支还需对齐时间戳和一致性序号，避免多字段跨样本混读；应将标定状态接入设备生命周期。
4. 未验证越疆 Rx/Ry/Rz 欧拉角约定、CR 型号关节上下限、工具/用户坐标系；绝不解锁 ServoJ/MovL。


## 阶段 C 增量（仅预览）：完整遥测门禁 + 三点标定界面

- 新增 `shared/RobotTelemetry.h`：按值复制 `RobotSnapshot`，包含 8891 session epoch、状态序号与本机年龄、29999/30003/30004 连接、RobotMode、六轴角度与速度、关节目标、工具六维位姿与速度、User/Tool 索引、拖拽/使能/错误/暂停/队列等标志。
- `evaluateRobotGate()` 对会话、relay_status 新鲜度（750 ms）、机器人状态新鲜度（400 ms）、数据有限性、工作模式、使能、故障、坐标 User/Tool 与已应用标定进行 fail-closed 判断。仅控制 `canCaptureCalibration` 和 `canPreview`；`canSendRobotMotion` 永远是 false（甚至 GateReason::Ok 时）。
- `RelayStateClient::snapshot()` 返回完整只读快照。若 8891 重连、session 不匹配、序号倒退/重复或反馈失效，不重用原有六维预览基准。
- 新增 `TeleopCalibrationDialog`：人工调整到三个位置并分别捕获同一时刻的 Touch/robot XYZ，对共线/小位移、尺度偏差和坐标系变动拒绝。保存 `AppConfigLocation/touch_robot_calibration_v1.json`，使用 `QSaveFile` 原子更新，并注明 `verified_for_motion: false`。现有配置只告知存在，不会自动启用标定。
- 每次点击保存只将标定应用至当前会话的 **Dry Run 预览**；历史持久化配置不可跨启动自动授权。变更标定导致 clutch 基准清空。
- 补充 `tests/robot_telemetry_tests.cpp` 及 GitHub Actions 离线测试。

### 未完成与限制

- 本三点标定仅产生近似旋转/位移比例；不是多点最小二乘拟合，不估计残差与不确定度；未做稳定采集平均、操作指导动图、校准配置与真实设备唯一 ID 绑定。标定完成后不应该用于自动控制机械臂。
- Touch 自身 OpenHaptics `hdCheckCalibration`、`hdUpdateCalibration` 的生命周期尚待接入；控制器的 RPY 分支约定和 robot 底座安装姿态同样待验证。
- 当前没有 Windows / Qt / OpenHaptics 端到端构建验证。必须先验证此构建、无真实运动的 8891 mock 和现场标定采样，才进一步开发离散 MovL 或受控 ServoJ。
- 当前两个仓库主分支仍保持原有行为；此文档对应 Draft PR，不能将“只读状态许可”理解为能进行机械臂实体运动。


## 阶段 D：多点配准、OpenHaptics 校准状态和采样一致性

- 新增无 Qt 依赖的 `shared/MultiPointCalibration.h`：基于 3D 刚体旋转 + 统一比例 + 平移的多点最小二乘配准。要求至少 8 组、最多 64 组输入；用户界面限定最多 32 组。拒绝三维分布退化（共线或共面近似）、位移不足、比例异常、非有限数值；汇报 RMS / 最大残差及对应最差点位。**目前是带尺度的配准，并非严格刚体 1:1 配准，实际缩放系数需经现场验证。**
- `TeleopCalibrationDialog` 允许捕获 8–32 组手动对应点，同一 8891 session、Robot User/Tool 和 Touch 校准纪元须保持不变。保存 JSON schema v2，附带 `rms_mm`、`max_residual_mm`、`sample_count`。老 schema v1 不自动加载；保存仍明确 `verified_for_motion=false`，仅供当前会话的 Dry Run 预览。
- `TouchBackend` 的 OpenHaptics 设备线程检查 `hdCheckCalibration` 并按返回值决定是否调用 `hdUpdateCalibration`（优先 inkwell、其次 auto/encoder reset）。未获得设备校准成功状态时 `DeviceState.valid=false`。UI 单独显示 Touch 自身校准状态。
- 同一 servo 回调提交 XYZ、矩阵、按键及有效标志时使用原子版本标记，Qt 的 latestState() 通过有限次数的版本前后检查避免混合两个采样周期的数据。注意该策略限制的是**同一帧一致性**，不代表跨网络的 Touch 与机器人采样完全同步。
- 预览中若 `calibrationEpoch` 发生变化，立即废弃旧的按键起点，避免设备校准造成坐标跳跃。
- 离线 CI 新增 `tests/multipoint_calibration_tests.cpp`：测试带噪声的旋转/尺度/平移拟合、误差与非有限点、共线与共面拒绝。

### 仍然要求现场验证

1. OpenHaptics SDK 设备支持何种校准风格、是否要求 inkwell，必须由实际 Touch 硬件及驱动确认；自动轮询校准状态不会替代用户的归位动作。设备校准时没有主动输出力，但仍需检查任何未来力反馈环路的安全性。
2. 目前多点拟合没有独立验证集、置信区间或强鲁棒剔除算法；现场采样应在稳定静止姿态下进行，检查对应点同步误差和关节是否在安全工况。
3. `RobotTelemetry` 的运动许可始终关闭；拟合 RMS 达标**不等于**真实机器人运动被认证或允许。
4. 当前 GitHub Actions 只构建无硬件 C++17 测试。Windows Qt 工程/OpenHaptics SDK 的完整构建及实际 8891/Touch 设备验收仍未完成。

## 2026-10-19 Hotfix：Touch 启动后立刻停止 / HD_ILLEGAL_BEGIN

**复现（Windows 现场日志）**：初始化成功，开始采集后立即退出；再次尝试可能出现 `hdScheduleAsynchronous` 失败（HD_INVALID_OPERATION）；设备错误 `hdBeginFrame for a device was called when already in that device's frame, or more than once during a single servoloop tick`。

**静态代码诊断**：上个版本的采样回调在每个 100 次采样检查时直接调用 `hdCheckCalibration` / `hdUpdateCalibration`，随后在**同一 servo tick** 调用 `hdBeginFrame`。这可能违反设备默认的 `HD_ONE_FRAME_LIMIT`，而且 `hdGetError` 在 `hdEndFrame` 之前读取，可能漏掉结束帧产生的错误。原有错误终止路径回调结束后，UI 没有清理旧注册句柄和已启动的调度器就可能重试注册。

**修复**：
- 检查校准状态的普通 tick 只打开/关闭一次帧；检测到 `HD_CALIBRATION_NEEDS_UPDATE` 后将更新延期至**下一个独立 tick**，该更新 tick 不进入正常采样帧。
- 更新后重新检查校准，校准未完成不发布有效位置；不禁用 `HD_ONE_FRAME_LIMIT`。
- 在 `hdEndFrame` 之后读取 HDAPI 错误。异常退出/再次启动时，先 `hdStopScheduler`、再 `hdUnschedule` 旧回调，清理旧错误后重新调度，避免旧回调残留。
- `calibrationPollTicks` 不再依赖只有有效采样才增加的 `sampleCounter`，防止未校准时每个 tick 都重复触发检查。
- 仍由硬件确认 inkwell/auto/encoder reset 的真实支持方式；校准会导致坐标突变，前端 clutch 只在设备校准后恢复。

**待验证**：此 Hotfix 未在 Windows/实体 Touch 设备上编译、运行，属于基于现场错误和 OpenHaptics 3.5 参考文档的修复。请重新生成 ControlBackend 及主程序，测试首次启动、短时间停止再启动、未完成设备归位、点击重置再初始化及连跑数分钟稳定性。若仍出现 `HD_ILLEGAL_BEGIN`，需要采集失败时的 HDAPI 错误码和调度器日志，在驱动和其它注册回调层进一步排查。

## 2026-10-19 follow-up: scheduler runs briefly and stops

The earlier hotfix did not fully isolate calibration checks: `hdCheckCalibration()` still ran in the ordinary sampling tick and there was no cooldown for repeated updates. Reworked the callback into a three-phase scheduler policy (`shared/HapticCalibrationPolicy.h`): a dedicated status-check tick (single HDAPI frame), a separate `hdUpdateCalibration` tick without an extra frame, and normal position/rotation acquisition tick (single frame). A requested update is rate-limited to at most once every 2000 servo ticks and status checks occur approximately every 500 servo ticks. The existing one-frame-per-device limit remains enabled.

Added atomic diagnostic counters and failure-stage logging: `calibration-check`, `calibration-update`, or `sample-frame`, HDAPI numeric code, ticks, update count. On callback failure the UI should report the stage and code via `TouchBackend::lastError()`; an unexpected process-level crash is outside this handler. Linux offline policy tests verify isolation and cooldown, but do not exercise real HDAPI; run Windows/OpenHaptics hardware validation before treating the issue as resolved. The root cause is not conclusively established without the user's new error log.

## Stability regression isolation: 2026-10-19

Compared `master` with PR #3. The new `latestState()` coherence loop can fail after eight retries; previously it returned a default `DeviceState` with `schedulerRunning=false`, making `DeviceController::pollDeviceState()` stop its Qt timer while HDAPI was still sampling. This exactly explains a `采集循环已停止。` line with **no HDAPI error**, though hardware faults can also exist. Fixed the fallback lifecycle flags AND made the UI ask `TouchBackend::isRunning()` directly.

For reproducible comparison with the older working master, the high-rate Touch servo callback no longer invokes `hdCheckCalibration()` or `hdUpdateCalibration()` automatically. It reads XYZ, buttons, and the orientation transform in one frame; normal acquisition no longer depends on a guessed calibration flag. `deviceCalibrated` remains **false/unknown**, so preview-only calibration capture and robot motion must remain unavailable until a separately validated explicit OpenHaptics calibration path is introduced. Do not interpret placing the stylus in the inkwell as proof of a successful HDAPI calibration. The added orientation transform is still a difference from master and requires Windows/hardware validation.

Acceptance: clean rebuild on the branch; only Touch connected, leave stylus docked then move it, verify sample counter stays increasing for several minutes, repeated start-stop and reset, and whether `采集循环已停止` is accompanied by HDAPI error. UI snapshot retry exhaustion must not stop polling. CI only checks pure C++ parts and is not an HDAPI regression test.


## Modern 6DoF GL visualization

The GL widget has been rewritten for read-only visualization:
- Dark background and hierarchical spatial reference grid, RGB world axes, track trail (last 250 points), on-canvas status and raw/display-relative XYZ.
- Procedural solid stylus mesh (nib, tip, body, grip rings) at the **HD_CURRENT_POSITION point**. Its local +Z shaft follows the row-major rotation extracted from HD_CURRENT_TRANSFORM, as do RGB local pose axes. The **local +Z shaft direction is an initial visualization convention** and must be checked against the physical Touch stylus in an actual device rotation test.
- Mouse left-drag orbit, Shift+drag pan, wheel zoom, double-click focus. R resets view, F follows/focuses stylus, C clears trail.
- Display origin and scene camera/trail are visualization only, never used by robot calibration or motion. Changing the GL display origin resets trail and camera target, while the original Touch raw XYZ remains untouched.
- All hardware motion remains disabled in this Draft PR. Only the GL display and independent pose math test have changed, not the validated stable OpenHaptics sample callback.
- Linux CI covers `gl_pose_tests.cpp` and checks multiplication order, rebase and invalid orientation. Windows Qt/OpenGL 3.3 and real stylus orientation require local acceptance testing.

## GL render and sidebar usability follow-up

- Fixed dynamic GL upload lifecycle: `upload()` no longer releases the shader program during `paintGL()`, which previously made the following solid pen and pose-axis draws unreliable/invisible. Shader remains bound until all GL primitives have rendered.
- If XYZ is valid but the orientation flag is unavailable, the stylus uses a **visualization-only identity fallback**, rather than disappearing. This does not validate or alter raw HDAPI orientation.
- Simplified the GL overlay to compact title + status, one XYZ line and one RPY line using 8-9 pt fonts. Deleted verbose shortcuts, redundant raw/view labels and multi-line warning. RPY remains diagnostic ZYX only.
- Added independent vertical QScrollArea wrappers for left control panel and right status/log panel. Center video/GL splitter keeps its own size. The right log edits retain useful minimum heights.
- Requires Windows Qt/OpenGL hands-on check for solid pen visibility, camera geometry, resizing scrollbars, actual RPY orientation and device sampling. Robot motion remains disabled.

## Explicit read-only Touch calibration check (current staged step)

The known-stable continuous 6DOF sampling and GL renderer remain unchanged except for an opt-in query inside the **existing** one-frame-per-tick servo callback. The new `检查 Touch 校准状态（只读）` button requests exactly one `hdCheckCalibration()` on the HDAPI scheduler thread, during an existing sample frame; it never calls `hdUpdateCalibration()`, adds a second frame, or schedules a competing callback. The UI reports NotChecked, Ok, NeedsUpdate, NeedsManualInput or CheckError. `TouchBackend::latestState()` now includes that state, while the existing coordinate publication remains independent of calibration until an explicit verified result is available.

To test safely: run Touch without relay/robot, verify sustained XYZ and GL 6DOF first, click the new check button once with the stylus resting in its inkwell, record the result and whether sampling continues. If `Ok`, the existing preview-only spatial registration can then accept samples when the read-only relay 8891 telemetry gate is also satisfied. If `NeedsUpdate` or `NeedsManualInput`, keep registration blocked until a *separately reviewed* explicit update workflow is implemented; never silently override the flag. Reinitializing or restarting acquisition invalidates the previous check. This build has not been validated with Windows/HDAPI hardware; if the query provokes a scheduler fault, use the app's HDAPI error logging and revert to the last known stable commit.

## Real hardware integration, phase 1: read-only 8891 telemetry and paired point acquisition

- UI shows 8891 read-only robot_state feedback: TCP XYZ/RxRyRz, six joint positions, six joint velocities, mode/enable/drag/running/error, User/Tool, sequence and freshness. Updated at most every 200ms to avoid flooding the right sidebar.
- Calibration dialog explicitly requires each Touch XYZ and robot TCP XYZ to be the **same physical reference point**. Moving devices independently and capturing unrelated positions cannot produce a meaningful registration. Robot movement must be independently taught safely, never by the dialog.
- Each captured pair requires OpenHaptics calibration OK, active Touch samples, fresh 8891 read-only feedback and invariant robot session/User/Tool/Touch calibration epoch. Near-duplicate sample points (<5 mm on either side) are rejected. Existing 8-32 point solver checks 3D spread, fit scale, residuals, and saves preview-only results with verified_for_motion=false.
- No robot motion command has been enabled. Status display is only an observation and does not guarantee safe physical state.
- **Important limitation:** Pair capture is currently instantaneous, not time-window averaged or geometrically independently verified; click only after both tools are stationary and correctly matched to a shared reference. Live-telemetry UI remains unverified in Windows. Pose orientation conventions with the Dobot controller and motion safety are not yet validated.

Onsite progression:
1. Keep hands clear; confirm physical emergency stop and separately controlled teaching method. Do not rely on the software emergency stop as a sole safeguard.
2. Start only Touch, check HDAPI calibration OK, confirm GL position/orientation and stable sample counts.
3. Connect relay 8888 and 8891; verify right sidebar LIVE age, seq advancing, correct TCP/joints/User/Tool, and stale state when disconnecting 8891.
4. For spatial calibration choose 8-20 independent **physical** reference locations well spread across the intended workspace, with Touch nib and robot TCP accurately brought to the *same location* for each pair via external safe teaching. Do not capture while either is moving. If this is not feasible, stop: simultaneous unrelated positions must not be paired.
5. Inspect scale (unit-ratio expected near 1 for matching mm coordinates), RMS, max residual, and repeat an unused reference location for an independent check. A mathematically low residual is not motion certification.
6. Only after a successful calibration and fresh robot states consider on-screen Dry Run. Real control remains disabled.

## 8891 v2 direct TCP proxy fix (live integration)

On Windows with HTTP/system proxy configuration, Qt's `QTcpSocket` for 8891 inherited an incompatible proxy and returned `The proxy type is invalid for this operation`. The control app now sets **per-socket** `QNetworkProxy::NoProxy` in `RelayStateClient` (not a global proxy change); the relay_station/master also explicitly uses NoProxy for robot sockets. Error messages include requested host, port, socket code and reason. This is a direct LAN 8891 connection and should never be sent through the OS web proxy.

Read-only six-DoF preview no longer requires legacy 8888 to be connected: it requires healthy fresh v2 feedback, the saved preview-only mapping applied in the **current** session, verified Touch calibration, enabled idle robot and matching User/Tool. The old generic 8888-or-8891 error is now a diagnostic gate code and v2 status. Physical robot movement remains permanently locked in this branch. If v2 connects but lacks fresh robot status, inspect robot port 30004 feedback and relay readiness; do not bypass the freshness gate.

Verify on Windows after rebuild: Connect TCP, watch 8891 handshake then LIVE sequence and age in the status sidebar; use a PowerShell `Test-NetConnection <relay-ip> -Port 8891` only if TCP connection still fails. Do not interpret a connected socket alone as ready robot feedback.

## Dobot RunQueuedCmd flag and Dry Run gate

Hardware observation: with fresh 8891 feedback, `RobotMode=5`, `EnableStatus=1`, `DragStatus=0`, `RunningStatus=0`, `JogStatus=0`, `PauseCmdFlag=0`, the controller may report `RunQueuedCmd=1`. The Dobot TCP/IP V3 reference calls this **algorithm queue running flag** (byte 1014), not "pending command count". Do **not** send Stop, ClearError or other robot-changing commands merely to force it to zero. The **read-only** Dry Run preview no longer treats this flag alone as non-idle; RobotMode=5, !running, !dragging, !jogging, !paused, fresh session and calibration are still required. `canSendRobotMotion` stays false unconditionally. Real physical motion safety cannot inherit this permissive read-only gate; it needs separate verified queue and command acknowledgement policy.

## Calibration plan selection / stability preflight

The current `mapClutchedPose()` requires a rotation and scale for **relative Touch motion**, not an absolute translation. Existing UI `fitSimilarity()` is appropriate only if all paired positions refer to corresponding physical reference locations, e.g. shared fixtures, or a rigorously designed paired motion trajectory. Unrelated simultaneous Touch / robot positions MUST NOT be paired and fitted.

Two distinct onsite routes remain open: (A) shared physical fiducials -> 8+ non-coplanar absolute point pairs and held-out verification; (B) workspaces do not overlap -> relative displacement direction/scale calibration with an independent safe teaching procedure and dedicated solver/UI still to be implemented. Do not use the existing absolute-pair dialog for B.

Added pure C++ preflight helpers `shared/CalibrationCaptureQuality.h` and offline tests for joint/TCP stationary thresholds, short temporal-window repeatability, duplicate rejection and independent holdout residual. They are not yet wired into the Qt dialog; instantaneous capture is unchanged until the appropriate calibration mode is chosen. These tests do not authorize motion.

## Separate-workspace relative calibration — Qt wizard, onsite procedure

There are now two DIFFERENT UI entries:
- 相对位移与方向标定（分离工作空间）: records paired start/end *displacements*, intended for independent workspaces
- 绝对空间点对标定（仅共同基准点）: original shared-physical-landmark method. Do NOT use this for independently placed devices

The new wizard is read-only. It NEVER issues robot motion commands. It estimates robot_delta = s * R * touch_delta with rotation and scale but no absolute translation.

PRECONDITION: Pairs MUST represent real, deliberately matching movement directions and scale. Fix an EXTERNAL common directional convention (e.g. room-right/room-forward/up shown on each fixture, verified by operator). Arbitrary independent hand and robot movements are not correspondence and MUST NOT be fitted.

1. Keep physical E-stop and an independently authorized low-speed teach workflow available. Verify clearance and correct TCP tool center. No one should rely on software E-stop alone.
2. Initialize Touch, start samples, read-only check HDAPI calibration OK. Connect 8891, verify fresh LIVE seq and matching User/Tool.
3. Both stationary: click ① Capture start (8 independent snapshots, 45ms period; reject excessive motion, stale feedback and session changes).
4. Teach robot movement using a SEPARATE safe robot mechanism, not the wizard; move Touch manually through intentionally corresponding displacement direction/scale. Movements need not be simultaneous. Keep robot TCP orientation nearly constant.
5. Both stationary: check manual correspondence checkbox and click ② Capture end. It checks >15 mm displacement on both devices, endpoint stability, <8° TCP orientation change, freshness and stable session/coordinates.
6. Repeat for >=10 diverse movements, recommended 12–14: positive/negative world X,Y,Z and diagonals; ensure noncoplanar training data, avoid duplicates and narrow direction coverage. Last 2 gestures are reserved as independent holdouts.
7. Click ③ Solve + validate + save. Zero-translation training residual gates: RMS <=6mm and max <=15mm; holdout residual RMS <=8mm and max <=12mm; scale in [0.15,6]. These thresholds are preliminary software test gates, NOT physical-motion approval.
8. On success writes dedicated touch_robot_relative_calibration_v1.json with all delta pairs, rotation, scale, residuals, verified_for_motion=false and orientation_verified=false. Failed fit does not save or apply.
9. Mapping applied only to current relay session and Touch calibration epoch for READ-ONLY Dry Run. Disconnection or changed Touch calibration epoch prevents reusing stale session mapping. Robot physical-motion output remains unconditionally disabled.
10. Dry Run: with a separately confirmed safe idle robot and unchanged User/Tool, press Touch button 1 to capture the clutch baseline, then move Touch gently and inspect printed target coordinates, release to reset. Do NOT consider displayed Rx/Ry/Rz validated by the displacement fit. Verify orientation mapping independently before any future motion work.

Important: A low mathematical residual can result from consistently wrong movement correspondences and is not evidence of safe movement. If the user cannot reliably establish shared directional meaning, stop capture and design a manual axis map / externally referenced alignment first. Qt Windows hardware integration remains untested.
