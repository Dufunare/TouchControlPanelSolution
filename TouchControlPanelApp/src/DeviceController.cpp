#include "DeviceController.h"
#include "TeleopGeometry.h"
#include "RelayStateClient.h"

#include <cmath>
#include <QString>
#include <QPointer>
#include <QMetaObject>
#include <algorithm>

namespace
{
    constexpr double kTeleopScaleX = 1.0;
    constexpr double kTeleopScaleY = 1.0;
    constexpr double kTeleopScaleZ = 1.0;

    QString decodeBackendText(const std::string& text)
    {
        const QString utf8Text = QString::fromUtf8(text.c_str());
        if (!utf8Text.contains(QChar::ReplacementCharacter))
        {
            return utf8Text;
        }

        return QString::fromLocal8Bit(text.c_str());
    }
}

DeviceController::DeviceController(
    touchpanel::TouchBackend* backend,
    touchpanel::CommunicationBackend* communicationBackend,
    QObject* parent)
    : QObject(parent), m_backend(backend), m_robotBackend(communicationBackend)
{
    m_pollTimer.setInterval(DEVICE_CONTROLLER_POLL_INTERVAL_MS); 
    m_pollTimer.setTimerType(Qt::PreciseTimer);
    m_motionTimer.setInterval(DEVICE_CONTROLLER_MOTION_INTERVAL_MS);
    m_motionTimer.setTimerType(Qt::PreciseTimer);

    connect(&m_pollTimer, &QTimer::timeout, this, &DeviceController::pollDeviceState);
    connect(&m_motionTimer, &QTimer::timeout, this, &DeviceController::onMotionTick);

    m_robotBackend->setMessageCallback([this](const std::string& text)
        {
            emit backendMessageChanged(decodeBackendText(text));
        });

    m_robotBackend->setConnectionCallback([this](bool connected)
        {
            emit tcpConnectionChanged(connected);
        });

    m_robotBackend->setRobotStatusCallback([this](const std::string& text)
        {
            emit robotStatusChanged(decodeBackendText(text));
        });

    m_robotBackend->setTcpTxCallback([this](const std::string& text)
        {
            emit tcpTxMessage(decodeBackendText(text));
        });

    m_robotBackend->setTcpRxCallback([this](const std::string& text)
        {
            emit tcpRxMessage(decodeBackendText(text));
        });
    QPointer<DeviceController> guard(this);
    m_robotBackend->setOperationCallback([guard](const std::string& op,bool ok,const std::string& detail){
        if(!guard)return;
        const QString operation=QString::fromStdString(op),message=QString::fromStdString(detail);
        QMetaObject::invokeMethod(guard.data(),[guard,operation,ok,message]{
            if(guard)guard->handleRobotOperation(operation,ok,message);
        },Qt::QueuedConnection);
    });
    m_robotBackend->setJointSolutionCallback([guard](bool ok,const std::array<double,6>& q,const std::string&){
        if(!guard)return;
        QMetaObject::invokeMethod(guard.data(),[guard,ok,q]{
            if(guard)guard->handleInverseSolution(ok,q);
        },Qt::QueuedConnection);
    });
}

void DeviceController::setRelayStateClient(RelayStateClient* client)
{
    m_relayStateClient = client;
}

bool DeviceController::getTouchSnapshot(touchpanel::DeviceState& state) const
{
    if(!m_backend)return false;
    state=m_backend->latestState();
    return state.valid&&state.schedulerRunning;
}

void DeviceController::applyPreviewCalibration(const touchpanel::teleop::Calibration& calibration,int user,int tool)
{
    invalidateTeleopBaseline();
    m_calibrationApplied=false;
    if(!m_relayStateClient || !m_backend || !calibration.measured ||
       !touchpanel::teleop::properRotation(calibration.touchToBase)) {
        emit backendMessageChanged(QStringLiteral("拒绝应用标定：数据或连接状态无效"));
        return;
    }
    const auto robot=m_relayStateClient->snapshot();
    const auto touch=m_backend->latestState();
    if(!touch.valid || !touch.schedulerRunning || !touch.deviceCalibrated ||
       !touchpanel::evaluateRobotGate(robot).canCaptureCalibration ||
       robot.userIndex!=user || robot.toolIndex!=tool) {
        emit backendMessageChanged(QStringLiteral("拒绝应用标定：机器人/Touch 会话或校准状态已失效"));
        return;
    }
    m_calibration=calibration;
    m_calibrationUser=user;m_calibrationTool=tool;
    m_calibrationSessionEpoch=robot.sessionEpoch;
    m_calibrationTouchEpoch=touch.calibrationEpoch;
    m_calibrationApplied=true;
    emit backendMessageChanged(QStringLiteral("已启用本次会话相对位移标定（仅预览，不启用实体运动）；姿态约定仍待独立验证"));
}

void DeviceController::invalidateTeleopBaseline()
{
    m_lastButtonPressed = false;
    m_motionBaselineReady = false;
    m_previewLastValid = false;
}

void DeviceController::checkTouchCalibration()
{
    if(!m_backend || !m_backend->requestCalibrationCheck()) {
        emit backendMessageChanged(QStringLiteral("请先初始化并开始 Touch 采集，再检查校准状态。"));
        return;
    }
    emit backendMessageChanged(QStringLiteral("已请求只读 HDAPI 校准检查；不会自动更新校准。"));
}

void DeviceController::initializeBackend()
{
    if (m_backend == nullptr)
    {
        emit backendMessageChanged("后端对象为空，无法初始化。");
        return;
    }
    if (m_backend->initialize())
    {
        emit deviceStateUpdated(m_backend->latestState());
        emit backendMessageChanged("Touch 后端初始化成功。");
    }
    else
    {
        emit deviceStateUpdated(m_backend->latestState());
        emit backendMessageChanged(decodeBackendText(m_backend->lastError()));
    }
}

void DeviceController::startStreaming()
{
    if (m_backend == nullptr)
    {
        emit backendMessageChanged("后端对象为空，无法启动。");
        return;
    }

    if (!m_backend->isInitialized())
    {
        emit backendMessageChanged("设备暂未初始化，请先初始化");
        return;
    }

    if (m_backend->start())
    {
        if (!m_pollTimer.isActive())
        {
            m_pollTimer.start();
        }

        emit backendMessageChanged("实时采集已启动，Qt 前端开始刷新坐标。");
    }
    else
    {
        emit backendMessageChanged(decodeBackendText(m_backend->lastError()));
    }
}

void DeviceController::stopStreaming()
{
    if (m_backend == nullptr)
    {
        emit backendMessageChanged("后端对象为空，无法停止。");
        return;
    }

    if (m_pollTimer.isActive())
    {
        m_pollTimer.stop();
    }

    if (m_motionTimer.isActive())
    {
        stopTeleop();
    }

    m_backend->stop();
    emit deviceStateUpdated(m_backend->latestState());
    emit backendMessageChanged("实时采集已停止。");
}

void DeviceController::resetBackend()
{
    if (m_backend == nullptr)
    {
        emit backendMessageChanged("后端对象为空，无法重置。");
        return;
    }

    if (m_pollTimer.isActive())
    {
        m_pollTimer.stop();
    }

    if (m_motionTimer.isActive()) stopTeleop();
    m_backend->reset();
    emit deviceStateUpdated(m_backend->latestState());
    emit backendMessageChanged("连接已重置，错误与回复已清空。");
}

void DeviceController::pollDeviceState()
{
    if (m_backend == nullptr)
    {
        return;
    }

    const auto state = m_backend->latestState();
    emit deviceStateUpdated(state);

    // Snapshot coherence failure must not stop the UI polling timer.
    if (!m_backend->isRunning() && m_pollTimer.isActive())
    {
        m_pollTimer.stop();

        const auto error = decodeBackendText(m_backend->lastError());
        if (!error.isEmpty())
        {
            emit backendMessageChanged(QStringLiteral("采集循环已停止：%1").arg(error));
        }
        else
        {
            emit backendMessageChanged("采集循环已停止。");
        }
    }
}

void DeviceController::connectTransit(const QString& ipOverride, quint16 portOverride)
{
    m_robotBackend->connectTransit(ipOverride.toStdString(), portOverride);
}

void DeviceController::disconnectTransit()
{
    if (m_motionTimer.isActive())
    {
        stopTeleop();
    }

    m_robotBackend->disconnectTransit();
}

void DeviceController::powerOnRobot()
{
    m_robotBackend->powerOn();
}

void DeviceController::enableRobot()
{
    m_robotBackend->enableRobot();
}

void DeviceController::disableRobot()
{
    m_robotBackend->disableRobot();
}

void DeviceController::clearRobotError()
{
    m_robotBackend->clearError();
}

void DeviceController::emergencyStopRobot()
{
    // Never permit the 33ms servo timer to re-issue motion after E-stop.
    m_realMode=RealMode::None;
    m_motionCommandSent=false;
    m_ikPending=false;
    m_stepPrepared=false;
    m_manualPreview=false;
    m_motionTimer.stop();
    invalidateTeleopBaseline();
    m_robotBackend->emergencyStop();
}

void DeviceController::startDragRobot()
{
    m_robotBackend->startDrag();
}

void DeviceController::stopDragRobot()
{
    m_robotBackend->stopDrag();
}

void DeviceController::setManualPreviewMapping(const touchpanel::teleop::ManualAxisMap& mapping)
{
    if(!touchpanel::teleop::validManualAxisMap(mapping)) {
        emit backendMessageChanged(QStringLiteral("手动轴映射无效：三个输出轴应各对应不同的 Touch 轴"));
        return;
    }
    m_manualMap=mapping;
    invalidateTeleopBaseline();
}

void DeviceController::startManualPreview()
{
    if(m_realMode!=RealMode::None) {
        emit backendMessageChanged(QStringLiteral("先停止实体遥控，再切换预览模式"));
        return;
    }
    if(!touchpanel::teleop::validManualAxisMap(m_manualMap) ||
       !m_backend || !m_backend->isRunning() || !m_relayStateClient) {
        emit backendMessageChanged(QStringLiteral("手动预览失败：Touch 采集或 8891 遥测未启动"));
        return;
    }
    const auto touch=m_backend->latestState();
    const auto robot=m_relayStateClient->snapshot();
    if(!touch.valid || !touch.deviceCalibrated ||
       !touchpanel::evaluateRobotGate(robot).canCaptureCalibration) {
        emit backendMessageChanged(QStringLiteral("手动预览失败：需要 HDAPI 已确认校准与新鲜机器人反馈"));
        return;
    }
    m_manualPreview=true;
    invalidateTeleopBaseline();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    emit backendMessageChanged(QStringLiteral("手动三轴预览已开启：Button1 离合，锁定 TCP 姿态，不下发实体运动"));
}

void DeviceController::startTeleop()
{
    if(m_realMode!=RealMode::None) {
        emit backendMessageChanged(QStringLiteral("先停止实体遥控，再切换六维预览"));
        return;
    }
    m_manualPreview=false;
    if (m_backend == nullptr || m_robotBackend == nullptr)
    {
        emit backendMessageChanged("后端对象为空，无法开始控制。");
        return;
    }

    if (!m_backend->isInitialized() || !m_backend->isRunning())
    {
        emit backendMessageChanged("开始控制失败：Touch 尚未初始化或未开始采集。");
        return;
    }

    // Preview is read-only: relay 8891 feedback is sufficient. Do not force
    // legacy 8888 connectivity merely to perform a local pose calculation.
    if(m_relayStateClient==nullptr) {
        emit backendMessageChanged(QStringLiteral("六维预览失败：8891 遥测客户端未初始化"));
        return;
    }
    const auto touch=m_backend->latestState();
    if(!touch.valid || !touch.orientationValid || !touch.deviceCalibrated) {
        emit backendMessageChanged(QStringLiteral("六维预览失败：需要 Touch 有效六维采样与 HDAPI 已确认校准"));
        return;
    }
    const auto robotSnapshot=m_relayStateClient->snapshot();
    if(m_calibrationApplied &&
       (robotSnapshot.sessionEpoch!=m_calibrationSessionEpoch ||
        touch.calibrationEpoch!=m_calibrationTouchEpoch)) {
        m_calibrationApplied=false;
        emit backendMessageChanged(QStringLiteral("旧空间标定已失效，自动退回无标定固定轴映射预览"));
    }
    // Read-only preview intentionally does not require a saved spatial fit.
    // Evaluate normal feedback/idle gates using the robot's current User/Tool.
    const auto decision=touchpanel::evaluateRobotGate(robotSnapshot,
        true,robotSnapshot.userIndex,robotSnapshot.toolIndex);
    if(!decision.canPreview) {
        emit backendMessageChanged(QStringLiteral("六维预览未启用：%1；%2")
            .arg(QString::fromUtf8(touchpanel::gateReasonChinese(decision.reason)) +
                 QStringLiteral(" [") + QString::fromStdString(touchpanel::idleGateDiagnostics(m_relayStateClient->snapshot())) + QStringLiteral("]"))
            .arg(m_relayStateClient->statusDescription()));
        return;
    }

    m_lastButtonPressed = false;
    m_motionBaselineReady = false;
    m_previewLastValid = false;
    m_modeQueryCooldownTicks = 0;
    m_poseQueryCooldownTicks = 0;
    m_hasLastSent = false;

    if (!m_motionTimer.isActive())
    {
        m_motionTimer.start();
    }

    emit backendMessageChanged("六维预览已启用：不要求空间标定。未标定时使用默认轴映射；仅计算目标，不发送实体运动。");
}

void DeviceController::stopTeleop()
{
    if(m_realMode!=RealMode::None) {
        haltRealMotion(QStringLiteral("操作员停止遥控"));
        return;
    }
    if (m_motionTimer.isActive())
    {
        m_motionTimer.stop();
    }

    m_lastButtonPressed = false;
    m_motionBaselineReady = false;
    m_hasLastSent = false;
    m_previewLastValid = false;
    m_manualPreview=false;
    emit backendMessageChanged("已停止预览；没有发送实体遥操作运动。");
}

void DeviceController::onMotionTick()
{
    if(m_realMode==RealMode::SingleStep) {
        monitorSingleStep();
        return;
    }
    if(m_realMode==RealMode::Continuous) {
        runContinuousTick();
        return;
    }
    if (!m_backend || !m_relayStateClient ||
        !m_backend->isRunning() || !m_relayStateClient->hasFreshState()) {
        invalidateTeleopBaseline();
        return;
    }
    const auto st = m_backend->latestState();
    if (!st.valid || !st.orientationValid || !st.schedulerRunning) {
        invalidateTeleopBaseline();
        return;
    }
    if (!st.button1Pressed) {
        invalidateTeleopBaseline();
        return;
    }
    const auto robotSnapshot=m_relayStateClient->snapshot();
    if(!m_manualPreview && m_calibrationApplied &&
       (robotSnapshot.sessionEpoch!=m_calibrationSessionEpoch ||
        st.calibrationEpoch!=m_calibrationTouchEpoch)) {
        m_calibrationApplied=false;
        invalidateTeleopBaseline(); // Restart the clutch using default axes.
        return;
    }
    const auto gate=touchpanel::evaluateRobotGate(robotSnapshot,
        true,robotSnapshot.userIndex,robotSnapshot.toolIndex);
    if(m_manualPreview) {
        // Read-only mapping needs fresh robot feedback, not a saved spatial fit.
        // Any error or motion invalidates the clutch; never send real commands.
        if(!gate.canCaptureCalibration || robotSnapshot.error ||
           robotSnapshot.running || robotSnapshot.dragging ||
           robotSnapshot.jogging || robotSnapshot.paused) {
            invalidateTeleopBaseline();return;
        }
    } else if(!gate.canPreview) {invalidateTeleopBaseline();return;}
    if (!m_lastButtonPressed) {
        // Re-capture after each release or invalidation. Never use legacy GetPose cache.
        if (!m_relayStateClient->tryGetFreshPose(m_robotPoseOrigin)) return;
        m_clutchCalibrationEpoch=st.calibrationEpoch;
        m_touchOrigin = st.positionMm;
        m_touchRotationOrigin = st.rotationMatrix;
        m_motionBaselineReady = true;
        m_lastButtonPressed = true;
        m_previewLastValid = false;
        emit backendMessageChanged(QStringLiteral("[DRY RUN] 已使用新鲜 8891 机器人位姿捕获六维基准"));
        return;
    }
    if (!m_motionBaselineReady || st.calibrationEpoch!=m_clutchCalibrationEpoch ||
        !st.deviceCalibrated) {invalidateTeleopBaseline();return;}
    touchpanel::teleop::Clutch anchor;
    anchor.valid = true;
    anchor.touchPosition = m_touchOrigin;
    anchor.touchRotation = m_touchRotationOrigin;
    anchor.robotPose = m_robotPoseOrigin;
    touchpanel::teleop::Pose6 requested{};
    const bool mapped=m_manualPreview ?
        touchpanel::teleop::manualPositionPreview(
            m_manualMap,touchpanel::teleop::sub(st.positionMm,m_touchOrigin),
            m_robotPoseOrigin,requested) :
        touchpanel::teleop::mapClutchedPose(
            m_calibrationApplied ? m_calibration : touchpanel::teleop::Calibration{},
            anchor,st.positionMm,st.rotationMatrix,requested);
    if (!mapped) {
        invalidateTeleopBaseline();
        emit backendMessageChanged(QStringLiteral("[DRY RUN] 无效的旋转/姿态，已停止当前预览"));
        return;
    }
    if (m_previewLastValid &&
        touchpanel::teleop::smallStep(requested,m_previewLast,0.25,0.25))return;
    m_previewLast = requested;
    m_previewLastValid = true;
    static unsigned previewTick = 0;
    if (++previewTick % 10 != 0) return;
    emit backendMessageChanged(
        QStringLiteral("[DRY RUN / NO ROBOT MOTION] XYZ=(%1,%2,%3) RxRyRz=(%4,%5,%6)")
        .arg(requested[0],0,'f',2).arg(requested[1],0,'f',2)
        .arg(requested[2],0,'f',2).arg(requested[3],0,'f',2)
        .arg(requested[4],0,'f',2).arg(requested[5],0,'f',2));
}


void DeviceController::prepareSingleStep()
{
    if(m_realMode!=RealMode::None || !m_manualPreview ||
       !m_motionBaselineReady || !m_previewLastValid ||
       !m_relayStateClient || !m_robotBackend->isTransitConnected()) {
        emit backendMessageChanged(QStringLiteral("单步准备失败：请先开启平移预览，按住 Touch Button 1 移动至目标，再捕获单步"));
        return;
    }
    const auto snapshot=m_relayStateClient->snapshot();
    touchpanel::teleop::StepProposal proposal;
    if(!touchpanel::teleop::proposeSmallPositionStep(snapshot,m_previewLast,
         m_robotPoseOrigin,snapshot.userIndex,snapshot.toolIndex,proposal)) {
        emit backendMessageChanged(QStringLiteral("单步准备失败：目标过近、超出当前离合范围，或机器人反馈/空闲状态不满足条件"));
        return;
    }
    m_stepProposal=proposal;
    m_stepExpectedStart=snapshot.tcpPose;
    m_stepStartSeq=snapshot.stateSeq;
    m_motionSession=snapshot.sessionEpoch;
    m_motionUser=snapshot.userIndex;m_motionTool=snapshot.toolIndex;
    m_stepPrepared=true;
    emit singleStepConfirmationNeeded(
        QStringLiteral("将通过 MovL 执行单次 %1 mm 位移。\n目标 XYZ=(%2,%3,%4) mm\n机器人当前姿态保持不变。\n确认周围人员、障碍物及物理急停状态后执行？")
        .arg(proposal.distanceMm,0,'f',2)
        .arg(proposal.pose[0],0,'f',2).arg(proposal.pose[1],0,'f',2)
        .arg(proposal.pose[2],0,'f',2));
}

void DeviceController::confirmSingleStep()
{
    if(!m_stepPrepared || m_realMode!=RealMode::None ||
       !m_robotBackend->isTransitConnected() || !m_relayStateClient)return;
    m_stepPrepared=false;
    const auto snapshot=m_relayStateClient->snapshot();
    if(snapshot.sessionEpoch!=m_motionSession ||
       snapshot.userIndex!=m_motionUser || snapshot.toolIndex!=m_motionTool ||
       !touchpanel::teleop::idleForExperimentalStep(snapshot,m_motionUser,m_motionTool) ||
       touchpanel::teleop::norm({snapshot.tcpPose[0]-m_stepExpectedStart[0],
                                 snapshot.tcpPose[1]-m_stepExpectedStart[1],
                                 snapshot.tcpPose[2]-m_stepExpectedStart[2]})>0.5 ||
       std::abs(snapshot.tcpPose[3]-m_stepExpectedStart[3])>0.3 ||
       std::abs(snapshot.tcpPose[4]-m_stepExpectedStart[4])>0.3 ||
       std::abs(snapshot.tcpPose[5]-m_stepExpectedStart[5])>0.3) {
        emit backendMessageChanged(QStringLiteral("单步取消：确认前机器人位置、会话或状态发生了变化"));
        return;
    }
    m_realMode=RealMode::SingleStep;
    m_manualPreview=false;
    m_stepAwaitingReply=true;
    m_stepReplyAccepted=false;
    m_motionCommandSent=true;
    invalidateTeleopBaseline();
    m_stepTimer.start();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    m_robotBackend->sendLinearStep(m_stepProposal.pose,m_motionUser,m_motionTool);
    emit backendMessageChanged(QStringLiteral("单步 MovL 已下发；等待应答及 8891 到位确认"));
}

void DeviceController::startContinuousServo()
{
    if(m_realMode!=RealMode::None || !m_backend ||
       !m_backend->isRunning() || !m_relayStateClient ||
       !m_robotBackend->isTransitConnected() ||
       !touchpanel::teleop::validManualAxisMap(m_manualMap)) {
        emit backendMessageChanged(QStringLiteral("连续遥控未启动：请确认 8888、8891、Touch 和轴映射"));
        return;
    }
    const auto st=m_backend->latestState();
    const auto s=m_relayStateClient->snapshot();
    if(!st.valid || !st.deviceCalibrated ||
       !touchpanel::teleop::idleForExperimentalStep(s,s.userIndex,s.toolIndex)) {
        emit backendMessageChanged(QStringLiteral("连续遥控未启动：机器人应处于使能空闲，且 Touch 校准与反馈有效"));
        return;
    }
    m_realMode=RealMode::Continuous;
    m_manualPreview=false;
    m_motionSession=s.sessionEpoch;
    m_motionUser=s.userIndex;m_motionTool=s.toolIndex;
    m_motionCommandSent=false;
    m_ikPending=false;
    invalidateTeleopBaseline();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    emit backendMessageChanged(QStringLiteral("连续 ServoJ 已就绪：仅按住 Touch Button1 时生成新目标；松开将请求停止机器人"));
}

void DeviceController::handleRobotOperation(const QString& operation,bool success,
                                           const QString& detail)
{
    if(operation==QStringLiteral("MovL") && m_realMode==RealMode::SingleStep) {
        if(!success) {
            haltRealMotion(QStringLiteral("MovL 命令应答失败：")+detail);
            return;
        }
        m_stepAwaitingReply=false;
        m_stepReplyAccepted=true;
        emit backendMessageChanged(QStringLiteral("单步命令已接收，等待机器人实际到位"));
    }
    if(operation==QStringLiteral("InverseSolution") &&
       m_realMode==RealMode::Continuous && !success) {
        haltRealMotion(QStringLiteral("逆解请求失败：")+detail);
    }
}

void DeviceController::handleInverseSolution(bool success,const std::array<double,6>& joints)
{
    if(m_realMode!=RealMode::Continuous || !m_ikPending)return;
    m_ikPending=false;
    if(!success) {
        haltRealMotion(QStringLiteral("逆解返回失败"));
        return;
    }
    const auto st=m_backend->latestState();
    const auto s=m_relayStateClient->snapshot();
    if(!m_motionBaselineReady || !m_lastButtonPressed ||
       !st.valid || !st.button1Pressed || !st.deviceCalibrated ||
       st.calibrationEpoch!=m_clutchCalibrationEpoch ||
       !m_relayStateClient->hasFreshState() ||
       s.sessionEpoch!=m_motionSession ||
       s.userIndex!=m_motionUser || s.toolIndex!=m_motionTool ||
       !s.enabled || s.error || s.dragging || s.jogging || s.paused ||
       (s.robotMode!=5 && s.robotMode!=7) ||
       !touchpanel::teleop::acceptableJointSolution(s,m_ikJointOrigin,joints)) {
        haltRealMotion(QStringLiteral("逆解不连续、过期或关节状态失效"));
        return;
    }
    const auto target=touchpanel::teleop::limitedServoTarget(s,joints);
    m_robotBackend->sendServoJointTarget(target);
    m_motionCommandSent=true;
    m_servoTime.restart();
}

void DeviceController::monitorSingleStep()
{
    if(!m_relayStateClient || !m_stepTimer.isValid()) {
        haltRealMotion(QStringLiteral("单步反馈监控不可用"));return;
    }
    const auto s=m_relayStateClient->snapshot();
    if(!m_relayStateClient->hasFreshState() ||
       s.sessionEpoch!=m_motionSession || s.userIndex!=m_motionUser ||
       s.toolIndex!=m_motionTool || s.error || s.dragging ||
       m_stepTimer.elapsed()>4000) {
        haltRealMotion(QStringLiteral("单步超时、断线或机器人状态异常"));return;
    }
    if(m_stepReplyAccepted && s.stateSeq>m_stepStartSeq &&
       touchpanel::teleop::stepArrived(s,m_stepProposal,m_motionUser,m_motionTool) &&
       touchpanel::teleop::norm({s.tcpPose[0]-m_stepExpectedStart[0],
                                s.tcpPose[1]-m_stepExpectedStart[1],
                                s.tcpPose[2]-m_stepExpectedStart[2]})>=0.25) {
        m_realMode=RealMode::None;
        m_motionCommandSent=false;
        m_motionTimer.stop();
        emit backendMessageChanged(QStringLiteral("单步结束：已收到命令成功应答并由 8891 确认位置到达"));
    }
}

void DeviceController::runContinuousTick()
{
    if(!m_backend || !m_relayStateClient || !m_robotBackend ||
       !m_backend->isRunning() || !m_relayStateClient->hasFreshState() ||
       !m_robotBackend->isTransitConnected()) {
        haltRealMotion(QStringLiteral("连续遥控链路不再有效"));return;
    }
    const auto st=m_backend->latestState();
    const auto s=m_relayStateClient->snapshot();
    if(!st.valid || !st.deviceCalibrated || !st.orientationValid ||
       s.sessionEpoch!=m_motionSession ||
       s.userIndex!=m_motionUser || s.toolIndex!=m_motionTool ||
       !s.enabled || s.error || s.dragging || s.jogging || s.paused ||
       (s.robotMode!=5 && s.robotMode!=7) ||
       s.feedbackAgeMs<0 || s.feedbackAgeMs>150 ||
       s.relayStatusAgeMs<0 || s.relayStatusAgeMs>350) {
        haltRealMotion(QStringLiteral("连续遥控状态/会话/反馈失效"));return;
    }
    if(!st.button1Pressed) {
        if(m_lastButtonPressed || m_motionBaselineReady)
            haltRealMotion(QStringLiteral("Touch Button1 已松开"));
        return;
    }
    if(!m_lastButtonPressed) {
        m_touchOrigin=st.positionMm;
        m_touchRotationOrigin=st.rotationMatrix;
        m_robotPoseOrigin=s.tcpPose;
        m_ikJointOrigin=s.jointPosition;
        m_clutchCalibrationEpoch=st.calibrationEpoch;
        m_motionBaselineReady=true;m_lastButtonPressed=true;
        emit backendMessageChanged(QStringLiteral("连续控制：已捕获 Touch 和机器人实际基准"));
        return;
    }
    if(st.calibrationEpoch!=m_clutchCalibrationEpoch) {
        haltRealMotion(QStringLiteral("Touch 校准周期发生变化"));return;
    }
    if(m_ikPending) {
        if(m_ikTimer.elapsed()>900)
            haltRealMotion(QStringLiteral("InverseSolution 回复超时"));
        return;
    }
    touchpanel::teleop::Pose6 requested{};
    if(!touchpanel::teleop::manualPositionPreview(
        m_manualMap,touchpanel::teleop::sub(st.positionMm,m_touchOrigin),
        m_robotPoseOrigin,requested)) {
        haltRealMotion(QStringLiteral("目标生成失败"));return;
    }
    const touchpanel::teleop::Vec3 delta={
        requested[0]-m_robotPoseOrigin[0],
        requested[1]-m_robotPoseOrigin[1],
        requested[2]-m_robotPoseOrigin[2]};
    if(touchpanel::teleop::norm(delta)>15.0) {
        haltRealMotion(QStringLiteral("离合行程超过 15 mm；请释放后重新建立基准"));return;
    }
    const touchpanel::teleop::Vec3 tracking={
        requested[0]-s.tcpPose[0],requested[1]-s.tcpPose[1],
        requested[2]-s.tcpPose[2]};
    const double step=touchpanel::teleop::norm(tracking);
    if(step<0.15)return;
    if(step>0.6) {
        for(int i=0;i<3;++i)requested[i]=s.tcpPose[i]+0.6*tracking[i]/step;
    }
    requested[3]=s.tcpPose[3];requested[4]=s.tcpPose[4];requested[5]=s.tcpPose[5];
    m_ikPending=true;
    m_ikTimer.start();
    m_robotBackend->requestInverseSolution(requested,m_motionUser,m_motionTool,s.jointPosition);
}

void DeviceController::haltRealMotion(const QString& cause)
{
    const bool sent=m_motionCommandSent;
    m_realMode=RealMode::None;
    m_motionCommandSent=false;
    m_stepPrepared=false;m_stepAwaitingReply=false;m_stepReplyAccepted=false;
    m_ikPending=false;
    m_manualPreview=false;
    invalidateTeleopBaseline();
    m_motionTimer.stop();
    if(sent && m_robotBackend && m_robotBackend->isTransitConnected())
        m_robotBackend->requestControlledStop();
    emit backendMessageChanged(QStringLiteral("实体遥控已终止：")+cause+
        (sent?QStringLiteral("；已请求 ResetRobot 停止/清队列（非物理急停）"):
              QStringLiteral("；未发送运动指令")));
}

