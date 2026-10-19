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
