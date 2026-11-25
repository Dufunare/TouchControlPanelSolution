#include "DeviceController.h"
#include "TeleopGeometry.h"
#include "RelayStateClient.h"

#include <cmath>
#include <QString>
#include <QStringList>
#include <QPointer>
#include <QMetaObject>
#include <algorithm>

namespace
{
    constexpr double kTeleopScaleX = 1.0;
    constexpr double kTeleopScaleY = 1.0;
    constexpr double kTeleopScaleZ = 1.0;

    bool servoPMeasuredIdle(const touchpanel::RobotSnapshot& s)
    {
        // RunQueuedCmd is a queue-execution mode flag, not a reliable
        // queue-depth/empty indicator. Use measured idle and stop ACK.
        if(s.robotMode!=5 || s.running || s.error || s.dragging ||
           s.jogging || s.paused ||
           !touchpanel::finiteSix(s.tcpVelocity) ||
           !touchpanel::finiteSix(s.jointVelocity))
            return false;
        for(int i=0;i<3;++i)if(std::abs(s.tcpVelocity[i])>2.0)return false;
        for(double velocity:s.jointVelocity)
            if(std::abs(velocity)>1.0)return false;
        return true;
    }

    QString servoPWorkspaceProblem(const touchpanel::teleop::Pose6& pose)
    {
        using namespace touchpanel::teleop;
        const auto result=checkServoPWorkspace(pose);
        if(result.code==ServoPWorkspaceCode::Allowed)return {};
        if(result.code==ServoPWorkspaceCode::XYZOutside)
            return QStringLiteral("实际 TCP %1=%2 mm，超出实验软件范围 [%3, %4] mm。"
                                  "请由现场人员按设备规定确认安全位置；不会自动回位。")
                .arg(QStringLiteral("XYZ").mid(result.axis,1))
                .arg(result.value,0,'f',2).arg(result.minimum,0,'f',0)
                .arg(result.maximum,0,'f',0);
        return QStringLiteral("实际 TCP 位姿数值/姿态无效，不能作为离合基准。");
    }

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
    // Member QObjects migrate with the controller. Their destructors
    // detach from the parent before QObject base destruction.
    m_pollTimer.setParent(this);
    m_motionTimer.setParent(this);
    m_statusTimer.setParent(this);
    m_statusTimer.setInterval(250);
    connect(&m_statusTimer,&QTimer::timeout,this,[this]{
        const QString status=servoPWorkspaceStatus();
        if(status!=m_lastServoPStatus) {
            m_lastServoPStatus=status;
            emit servoPStatusChanged(status);
        }
    });
    m_statusTimer.start();
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
    if(client)connect(client,&RelayStateClient::orientationProbeFinished,
                       this,&DeviceController::backendMessageChanged);
}

bool DeviceController::realMotionActive() const
{
    return m_realMode!=RealMode::None;
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

void DeviceController::onRelayStateInvalidated()
{
    m_orientationAudit.clear();
    m_singleStepVerified=false;
    if(m_realMode!=RealMode::None)
        haltRealMotion(QStringLiteral("8891 遥测失效，已终止实体运动"));
    else
        invalidateTeleopBaseline();
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
    m_singleStepVerified=false;
    if(m_realMode!=RealMode::None)haltRealMotion(QStringLiteral("机器人下使能"));
    m_robotBackend->disableRobot();
}

void DeviceController::clearRobotError()
{
    m_robotBackend->clearError();
}

void DeviceController::emergencyStopRobot()
{
    // Never permit the 33ms servo timer to re-issue motion after E-stop.
    if(m_realMode==RealMode::CartesianServoP)
        m_pollTimer.setInterval(DEVICE_CONTROLLER_POLL_INTERVAL_MS);
    m_realMode=RealMode::None;
    m_cartesianServoP.reset();
    m_motionCommandSent=false;
    m_ikPending=false;
    m_stepPrepared=false;
    m_singleStepVerified=false;
    m_manualPreview=false;
    m_motionTimer.stop();
    invalidateTeleopBaseline();
    m_robotBackend->emergencyStop();
}

void DeviceController::startDragRobot()
{
    m_singleStepVerified=false;
    if(m_realMode!=RealMode::None)haltRealMotion(QStringLiteral("切换到拖拽模式"));
    m_robotBackend->startDrag();
}

void DeviceController::stopDragRobot()
{
    m_robotBackend->stopDrag();
}

void DeviceController::setManualPreviewMapping(const touchpanel::teleop::ManualAxisMap& mapping)
{
    if(m_realMode!=RealMode::None) {
        emit backendMessageChanged(QStringLiteral("实体遥控期间禁止修改轴映射；请先停止"));
        return;
    }
    if(!touchpanel::teleop::validManualAxisMap(mapping)) {
        emit backendMessageChanged(
            QStringLiteral("手动轴映射无效：机器人 X=%1、Y=%2、Z=%3（0/1/2=+X/+Y/+Z，3/4/5=-X/-Y/-Z）；比例=%4，死区=%5 mm")
                .arg(mapping.output[0]).arg(mapping.output[1]).arg(mapping.output[2])
                .arg(mapping.scale,0,'f',2).arg(mapping.deadzoneMm,0,'f',1));
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
    m_fixedAxisPosePreview=false;
    invalidateTeleopBaseline();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    emit backendMessageChanged(QStringLiteral("手动三轴预览已开启：Button1 离合，锁定 TCP 姿态，不下发实体运动"));
}

void DeviceController::resetOrientationEvidence()
{
    m_orientationAudit.clear();
    emit backendMessageChanged(QStringLiteral("[ROTATION AUDIT] 已清除本会话姿态证据；不发送运动"));
}

void DeviceController::captureOrientationEvidence()
{
    using namespace touchpanel::teleop;
    if(realMotionActive() || !m_relayStateClient) {
        emit backendMessageChanged(QStringLiteral("[ROTATION AUDIT] 请先停止实体遥控；只读取反馈"));
        return;
    }
    const auto state=m_relayStateClient->snapshot();
    const auto captured=m_orientationAudit.capture(state);
    if(captured==OrientationCapture::Invalid) {
        emit backendMessageChanged(QStringLiteral(
            "[ROTATION AUDIT] 拒绝采样：需要 User=0/Tool=0、新鲜空闲反馈、"
            "有效 ActualQuaternion 及静止关节；不自动切换坐标系"));
        return;
    }
    const auto report=m_orientationAudit.report();
    QStringList errors;
    for(std::size_t i=0;i<kRotationOrders.size();++i)
        errors<<QStringLiteral("%1=%2°").arg(QString::fromLatin1(kRotationOrders[i].data(),3))
            .arg(report.worstError[i],0,'f',3);
    const auto& pose=state.tcpPose;
    const auto& q=state.actualQuaternion;
    const QString status=report.unique?
        QStringLiteral("候选 %1 与样本一致（仅姿态数值证据，不授权运动）")
            .arg(QString::fromLatin1(kRotationOrders[report.bestOrder].data(),3)):
        QStringLiteral("证据不足/约定不匹配，实体旋转仍未授权");
    emit backendMessageChanged(QStringLiteral(
        "[ROTATION AUDIT] session=%1 seq=%2 %3 distinct=%4 "
        "RPY=(%5,%6,%7) Q=(%8,%9,%10,%11) | %12 | %13")
        .arg(state.sessionEpoch).arg(state.stateSeq)
        .arg(captured==OrientationCapture::Added?QStringLiteral("采集"):
             captured==OrientationCapture::Duplicate?QStringLiteral("重复姿态"):QStringLiteral("样本已满"))
        .arg(report.count).arg(pose[3],0,'f',4).arg(pose[4],0,'f',4)
        .arg(pose[5],0,'f',4).arg(q[0],0,'f',7).arg(q[1],0,'f',7)
        .arg(q[2],0,'f',7).arg(q[3],0,'f',7)
        .arg(errors.join(QStringLiteral(" "))).arg(status));
}

void DeviceController::probeRotationCandidate()
{
    using namespace touchpanel::teleop;
    if(realMotionActive() || !m_relayStateClient || !m_backend ||
       !m_fixedAxisPosePreview || !m_motionBaselineReady || !m_previewLastValid) {
        emit backendMessageChanged(QStringLiteral("[ROTATION IK] 请先开启固定轴六维只读预览，抓取当前离合目标"));
        return;
    }
    const auto s=m_relayStateClient->snapshot();
    const auto st=m_backend->latestState();
    if(!m_orientationAudit.supportsZyx(s) || !idleForExperimentalStep(s,0,0) ||
       s.sessionEpoch!=m_fixedPreviewSession ||
       s.userIndex!=m_fixedPreviewUser || s.toolIndex!=m_fixedPreviewTool ||
       !st.valid || !st.schedulerRunning || !st.deviceCalibrated ||
       !st.orientationValid || !st.button1Pressed ||
       st.calibrationEpoch!=m_clutchCalibrationEpoch ||
       st.button1ReleaseCounter!=m_previewReleaseCounter ||
       servoPCartesianDistance(s.tcpPose,m_robotPoseOrigin)>0.5 ||
       rotationDistanceDeg(rpyToMatrix({s.tcpPose[3],s.tcpPose[4],s.tcpPose[5]}),
           rpyToMatrix({m_robotPoseOrigin[3],m_robotPoseOrigin[4],m_robotPoseOrigin[5]}))>0.5) {
        emit backendMessageChanged(QStringLiteral("[ROTATION IK] 需要本会话唯一 ZYX 证据、User/Tool=0、稳定机器人及有效当前离合"));
        return;
    }
    Clutch anchor;anchor.valid=true;anchor.touchPosition=m_touchOrigin;
    anchor.touchRotation=m_touchRotationOrigin;anchor.robotPose=m_robotPoseOrigin;
    Pose6 pose{};
    if(!fixedAxisPosePreview(m_manualMap,anchor,st.positionMm,st.rotationMatrix,
            {m_previewLast[3],m_previewLast[4],m_previewLast[5]},pose) ||
       !m_relayStateClient->requestOrientationIkProbe(pose)) {
        emit backendMessageChanged(QStringLiteral("[ROTATION IK] 目标不可计算/超出实验工作空间/已有诊断在途"));
        return;
    }
    emit backendMessageChanged(QStringLiteral("[ROTATION IK] 已请求 V3 InverseSolution（29999，仅计算，不发送运动）"));
}

void DeviceController::startFixedAxisPosePreview()
{
    if(realMotionActive() || !m_backend || !m_backend->isRunning() ||
       !m_relayStateClient || !touchpanel::teleop::validManualAxisMap(m_manualMap)) {
        emit backendMessageChanged(QStringLiteral("六维固定轴预览不可启动：先停止实体控制并确认采集、映射与反馈"));
        return;
    }
    const auto touch=m_backend->latestState();
    const auto robot=m_relayStateClient->snapshot();
    if(!touch.valid || !touch.orientationValid || !touch.deviceCalibrated ||
       !touchpanel::evaluateRobotGate(robot,true,robot.userIndex,robot.toolIndex).canPreview) {
        emit backendMessageChanged(QStringLiteral("六维固定轴预览需要有效 Touch 姿态及新鲜空闲机器人反馈"));
        return;
    }
    m_manualPreview=false;
    m_fixedAxisPosePreview=true;
    m_previewReleaseCounter=touch.button1ReleaseCounter;
    invalidateTeleopBaseline();
    m_stepPrepared=false;
    if(!m_motionTimer.isActive())m_motionTimer.start();
    emit backendMessageChanged(QStringLiteral(
        "[6DOF DRY RUN] 固定轴平移及旋转预览；平移使用所选比例，旋转角度1:1。"
        "采用相对旋转矩阵；不发送任何机器人运动指令。"));
}

void DeviceController::startTeleop()
{
    if(m_realMode!=RealMode::None) {
        emit backendMessageChanged(QStringLiteral("先停止实体遥控，再切换六维预览"));
        return;
    }
    m_manualPreview=false;
    m_fixedAxisPosePreview=false;
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
    m_fixedAxisPosePreview=false;
    emit backendMessageChanged("已停止预览；没有发送实体遥操作运动。");
}

void DeviceController::onMotionTick()
{
    if(m_realMode==RealMode::SingleStep) {
        monitorSingleStep();
        return;
    }
    if(m_realMode==RealMode::CartesianServoP) {
        runServoPTick();
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
    // Position-only manual preview does not need a valid stylus rotation.
    // Six-DoF orientation preview retains its explicit orientation gate.
    if (!st.valid || !st.schedulerRunning ||
        (!m_manualPreview && !st.orientationValid)) {
        invalidateTeleopBaseline();
        return;
    }
    if(m_fixedAxisPosePreview && st.button1ReleaseCounter!=m_previewReleaseCounter) {
        m_previewReleaseCounter=st.button1ReleaseCounter;
        invalidateTeleopBaseline();
        return;
    }
    if (!st.button1Pressed) {
        invalidateTeleopBaseline();
        return;
    }
    const auto robotSnapshot=m_relayStateClient->snapshot();
    if(m_fixedAxisPosePreview && m_motionBaselineReady &&
       (robotSnapshot.sessionEpoch!=m_fixedPreviewSession ||
        robotSnapshot.userIndex!=m_fixedPreviewUser ||
        robotSnapshot.toolIndex!=m_fixedPreviewTool)) {
        invalidateTeleopBaseline();
        return;
    }
    if(!m_manualPreview && !m_fixedAxisPosePreview && m_calibrationApplied &&
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
        if(m_fixedAxisPosePreview) {
            m_fixedPreviewSession=robotSnapshot.sessionEpoch;
            m_fixedPreviewUser=robotSnapshot.userIndex;
            m_fixedPreviewTool=robotSnapshot.toolIndex;
        }
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
    const touchpanel::teleop::Vec3 referenceRpy=m_previewLastValid?
        touchpanel::teleop::Vec3{m_previewLast[3],m_previewLast[4],m_previewLast[5]}:
        touchpanel::teleop::Vec3{m_robotPoseOrigin[3],m_robotPoseOrigin[4],m_robotPoseOrigin[5]};
    const bool mapped=m_manualPreview ?
        touchpanel::teleop::manualPositionPreview(
            m_manualMap,touchpanel::teleop::sub(st.positionMm,m_touchOrigin),
            m_robotPoseOrigin,requested) :
        m_fixedAxisPosePreview ?
        touchpanel::teleop::fixedAxisPosePreview(
            m_manualMap,anchor,st.positionMm,st.rotationMatrix,referenceRpy,requested) :
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
    m_stepLastArrivalSeq=snapshot.stateSeq;
    m_stepArrivalStableSamples=0;
    m_motionSession=snapshot.sessionEpoch;
    m_motionUser=snapshot.userIndex;m_motionTool=snapshot.toolIndex;
    m_stepPrepared=true;
    emit backendMessageChanged(
        QStringLiteral("[STEP PLAN] 当前机器人 XYZ=(%1,%2,%3), "
                       "Touch 预览 XYZ=(%4,%5,%6), "
                       "本次 MovL 目标 XYZ=(%7,%8,%9), 位移=%10 mm, seq=%11")
        .arg(snapshot.tcpPose[0],0,'f',3).arg(snapshot.tcpPose[1],0,'f',3)
        .arg(snapshot.tcpPose[2],0,'f',3)
        .arg(m_previewLast[0],0,'f',3).arg(m_previewLast[1],0,'f',3)
        .arg(m_previewLast[2],0,'f',3)
        .arg(proposal.pose[0],0,'f',3).arg(proposal.pose[1],0,'f',3)
        .arg(proposal.pose[2],0,'f',3)
        .arg(proposal.distanceMm,0,'f',3).arg(snapshot.stateSeq));
    emit singleStepConfirmationNeeded(
        QStringLiteral("将通过 MovL 执行单次 %1 mm 位移。\n目标 XYZ=(%2,%3,%4) mm\n机器人当前姿态保持不变。\n确认周围人员、障碍物及物理急停状态后执行？")
        .arg(proposal.distanceMm,0,'f',2)
        .arg(proposal.pose[0],0,'f',2).arg(proposal.pose[1],0,'f',2)
        .arg(proposal.pose[2],0,'f',2));
}

void DeviceController::confirmSingleStep()
{
    if(m_relayStateClient && m_relayStateClient->orientationProbeInterlock()) {
        emit backendMessageChanged(QStringLiteral("只读姿态诊断仍在途或Dashboard链路不确定，禁止启动实体控制；查看诊断日志"));
        return;
    }
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
    m_stepLastArrivalSeq=snapshot.stateSeq;
    m_stepArrivalStableSamples=0;
    m_motionCommandSent=true;
    invalidateTeleopBaseline();
    m_stepTimer.start();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    m_robotBackend->sendLinearStep(m_stepProposal.pose,m_motionUser,m_motionTool);
    emit backendMessageChanged(QStringLiteral("单步 MovL 已下发；等待应答及 8891 到位确认"));
}

QString DeviceController::servoPWorkspaceStatus() const
{
    if(!m_relayStateClient || !m_relayStateClient->hasFreshState())
        return QStringLiteral("工作空间：等待 8891 有效反馈");
    const auto s=m_relayStateClient->snapshot();
    const auto problem=servoPWorkspaceProblem(s.tcpPose);
    if(!problem.isEmpty())return QStringLiteral("ServoP 不可启动：")+problem;
    if(m_realMode==RealMode::CartesianServoP) {
        if(m_servoPFeedbackHolding)return QStringLiteral("离合：反馈暂失，已停发；恢复后松开再按");
        switch(m_cartesianServoP.phase()) {
        case touchpanel::teleop::ServoPPhase::NeedRelease:
            return QStringLiteral("离合：先松开 Button1，等待就绪");
        case touchpanel::teleop::ServoPPhase::Ready:
            return QStringLiteral("离合：就绪，可按下 Button1 重新抓取");
        case touchpanel::teleop::ServoPPhase::Active:
            if(m_servoPTrackingHolding)return QStringLiteral("离合：跟踪保护暂停");
            if(m_servoPWallMask)return QStringLiteral("离合：软件边界限制部分轴");
            return QStringLiteral("离合：控制中，松开可重新定位 Touch");
        case touchpanel::teleop::ServoPPhase::WaitingForIdle:
            if(m_cartesianServoP.releaseStopPending())
                return QStringLiteral("离合：已请求软件停止，等待应答");
            return QStringLiteral("离合：等待实测停稳；提前按下须松开再按");
        }
    }
    return QStringLiteral("工作空间：当前 TCP 在实验软件范围内（非物理安全确认）");
}

void DeviceController::startContinuousServoP()
{
    if(m_relayStateClient && m_relayStateClient->orientationProbeInterlock()) {
        emit backendMessageChanged(QStringLiteral("只读姿态诊断仍在途或Dashboard链路不确定，禁止启动实体控制；查看诊断日志"));
        return;
    }
    if(m_realMode!=RealMode::None || !m_backend ||
       !m_backend->isRunning() || !m_relayStateClient || !m_robotBackend ||
       !m_robotBackend->isTransitConnected() ||
       !touchpanel::teleop::validManualAxisMap(m_manualMap)) {
        emit backendMessageChanged(QStringLiteral(
            "ServoP 未启动：请停止旧模式，确认 Touch/8888/8891 与轴映射"));
        return;
    }
    const auto st=m_backend->latestState();
    const auto s=m_relayStateClient->snapshot();
    if(!st.valid || !st.deviceCalibrated || !st.schedulerRunning ||
       !touchpanel::teleop::continuousTelemetryAgesHealthy(s) ||
       !s.enabled || s.error || s.dragging || s.jogging || s.paused ||
       !servoPMeasuredIdle(s)) {
        emit backendMessageChanged(QStringLiteral(
            "ServoP 未启动：需有效校准的 Touch、8891 新鲜状态及空闲已使能机器人"
            "（Mode=%1, Running=%2, FeedbackAge=%3ms, RelayAge=%4ms）")
            .arg(s.robotMode).arg(s.running)
            .arg(s.feedbackAgeMs).arg(s.relayStatusAgeMs));
        return;
    }
    const auto workspaceProblem=servoPWorkspaceProblem(s.tcpPose);
    if(!workspaceProblem.isEmpty()) {
        emit backendMessageChanged(QStringLiteral("[SERVOP START BLOCKED] ")+workspaceProblem+
            QStringLiteral(" 未进入遥控待命，未发送运动指令。"));
        return;
    }
    // A single fixed-axis live controller; no legacy trajectory mode.
    m_cartesianServoP.reset();
    m_realMode=RealMode::CartesianServoP;
    // Presentation only: keep the 33ms control and telemetry cadence,
    // but reduce competing GUI state/GL refresh during physical ServoP.
    m_pollTimer.setInterval(66);
    m_motionSession=s.sessionEpoch;
    m_motionUser=s.userIndex;
    m_motionTool=s.toolIndex;
    m_motionCommandSent=false;
    m_manualPreview=false;
    m_servoPReleaseTimer.invalidate();
    m_servoPTrackingHolding=false;
    m_servoPFeedbackHolding=false;
    m_servoPTrackingHoldTimer.invalidate();
    m_servoPTrackingLogTimer.invalidate();
    m_servoPStatsTimer.start();
    m_servoPTickTimer.invalidate();
    m_servoPSendTimer.invalidate();
    m_servoPMaxTickGapMs=0;
    m_servoPMaxSendUs=0;
    m_servoPTotalSendUs=0;
    m_servoPSendSamples=0;
    m_servoPSentPackets=0;
    m_servoPHasTarget=false;
    m_servoPWorkspaceBlocked=false;
    m_servoPWorkspaceAxis=-1;
    m_servoPWallMask=0;
    m_servoPWallLogTimer.invalidate();
    m_servoPWorkspaceLogTimer.invalidate();
    m_invalidTouchTicks=0;
    m_staleTouchTicks=0;
    m_lastTouchSampleCounter=st.sampleCounter;
    m_servoPLastReleaseCounter=st.button1ReleaseCounter;
    invalidateTeleopBaseline();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    emit backendMessageChanged(QStringLiteral(
        "ServoP 同步优先遥控已待命：先松开 Button1，等待就绪后按下抓取两端坐标。"
        "按 master 直接计算位置，保护层丢弃的位移不补发。"
        "松开立即停发；若本次发过运动指令则请求一次 ResetRobot 软件停止。"
        "确认应答及新鲜停稳反馈后才允许重新离合。"
        "保留固定轴映射、比例、工作空间和指令安全包络。"));
}

void DeviceController::startContinuousServo()
{
    if(m_relayStateClient && m_relayStateClient->orientationProbeInterlock()) {
        emit backendMessageChanged(QStringLiteral("只读姿态诊断仍在途或Dashboard链路不确定，禁止启动实体控制；查看诊断日志"));
        return;
    }
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
    if(!m_singleStepVerified || m_verifiedStepSession!=s.sessionEpoch) {
        emit backendMessageChanged(QStringLiteral("连续控制未放行：本次 8891 会话尚未完成一次受控单步到位验收"));
        return;
    }
    m_realMode=RealMode::Continuous;
    m_manualPreview=false;
    m_motionSession=s.sessionEpoch;
    m_motionUser=s.userIndex;m_motionTool=s.toolIndex;
    m_motionCommandSent=false;
    m_haveServoTarget=false;
    m_clutchRangeSaturated=false;
    m_ikPending=false;
    m_continuousStopping=false;
    m_stopReplyAccepted=false;
    m_releaseTicks=0;
    m_ikQueries=0;m_ikAccepted=0;m_servoCommands=0;
    m_ikRttSumMs=0;m_ikRttMaxMs=0;m_ikRttSamples=0;
    m_invalidTouchTicks=0;
    m_staleTouchTicks=0;
    m_lastTouchSampleCounter=st.sampleCounter;
    m_continuousStatsTimer.restart();
    invalidateTeleopBaseline();
    if(!m_motionTimer.isActive())m_motionTimer.start();
    emit backendMessageChanged(QStringLiteral("连续 ServoJ 已就绪：仅按住 Touch Button1 时生成新目标；松开将请求停止机器人"));
}

void DeviceController::handleRobotOperation(const QString& operation,bool success,
                                           const QString& detail)
{
    // Ignore benign unsolicited ServoJ success echoes in the transport.
    // Explicit errors from the motion controller are always fatal for the
    // current continuous session; never continue on an ErrorID != 0.
    if(operation==QStringLiteral("ServoJ") && !success &&
       m_realMode==RealMode::Continuous) {
        haltRealMotion(QStringLiteral("ServoJ 控制器拒绝指令：")+detail);
        return;
    }
    if(operation==QStringLiteral("ServoP") && !success &&
       m_realMode==RealMode::CartesianServoP) {
        haltRealMotion(QStringLiteral("ServoP 控制器拒绝指令：")+detail);
        return;
    }
    if(operation==QStringLiteral("MovL") && m_realMode==RealMode::SingleStep) {
        if(!success) {
            haltRealMotion(QStringLiteral("MovL 命令应答失败：")+detail);
            return;
        }
        m_stepAwaitingReply=false;
        m_stepReplyAccepted=true;
        emit backendMessageChanged(QStringLiteral("单步命令已接收，等待机器人实际到位"));
    }
    if(operation==QStringLiteral("ResetRobot") &&
       m_realMode==RealMode::CartesianServoP &&
       m_cartesianServoP.releaseStopPending()) {
        const auto observed=m_relayStateClient?
            m_relayStateClient->snapshot():touchpanel::RobotSnapshot{};
        if(!success || observed.sessionEpoch!=m_motionSession ||
           !touchpanel::teleop::continuousTelemetryAgesHealthy(observed) ||
           !m_cartesianServoP.acknowledgeReleaseStop(observed.stateSeq)) {
            haltRealMotion(QStringLiteral("ServoP：松手停止确认失败：")+detail);
            return;
        }
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP STOP ACK] 软件停止已应答；仍需后续三次不同的有效空闲反馈。"));
        return;
    }
    if(operation==QStringLiteral("ResetRobot") &&
       m_realMode==RealMode::Continuous && m_continuousStopping) {
        if(!success) {
            haltRealMotion(QStringLiteral("离合释放后的停止命令失败：")+detail);
        } else {
            m_stopReplyAccepted=true;
            emit backendMessageChanged(QStringLiteral("停止命令已应答，继续等待 8891 确认机器人空闲"));
        }
        return;
    }
    if(operation==QStringLiteral("InverseSolution") &&
       m_realMode==RealMode::Continuous && !success && !m_continuousStopping) {
        haltRealMotion(QStringLiteral("逆解请求失败：")+detail);
    }
}

void DeviceController::handleInverseSolution(bool success,const std::array<double,6>& joints)
{
    if(m_realMode!=RealMode::Continuous || !m_ikPending)return;
    m_ikPending=false;
    const int ikElapsed=m_ikTimer.isValid()?static_cast<int>(m_ikTimer.elapsed()):0;
    if(ikElapsed>0) {
        m_ikRttSumMs+=ikElapsed;
        m_ikRttMaxMs=std::max(m_ikRttMaxMs,ikElapsed);
        ++m_ikRttSamples;
    }
    // Release cancels pending IK; its late reply must never move the robot
    // or turn an intentional clutch release into a fault.
    if(m_continuousStopping || !m_backend->latestState().button1Pressed)return;
    if(!success) {
        haltRealMotion(QStringLiteral("逆解返回失败"));
        return;
    }
    const auto st=m_backend->latestState();
    const auto s=m_relayStateClient->snapshot();
    bool validGeometry=touchpanel::teleop::acceptableJointSolution(
        s,m_ikJointOrigin,joints);
    if(!m_motionBaselineReady || !m_lastButtonPressed ||
       !st.valid || !st.button1Pressed || !st.deviceCalibrated ||
       st.calibrationEpoch!=m_clutchCalibrationEpoch ||
       !m_relayStateClient->hasFreshState() ||
       s.sessionEpoch!=m_motionSession ||
       s.userIndex!=m_motionUser || s.toolIndex!=m_motionTool ||
       !s.enabled || s.error || s.dragging || s.jogging || s.paused ||
       (s.robotMode!=5 && s.robotMode!=7) ||
       !m_ikTimer.isValid() || m_ikTimer.elapsed()>touchpanel::teleop::kIKReplyAcceptMs ||
       !validGeometry) {
        double maxMeasured=0,maxFromAnchor=0;
        for(int i=0;i<6;++i) {
            maxMeasured=std::max(maxMeasured,std::abs(joints[i]-s.jointPosition[i]));
            maxFromAnchor=std::max(maxFromAnchor,std::abs(joints[i]-m_ikJointOrigin[i]));
        }
        haltRealMotion(QStringLiteral(
            "逆解结果不符合连续控制条件：IK耗时=%1ms，Mode=%2，"
            "8891延迟=%3ms，Button1=%4，关节相对实际最大差=%5°(限3°)，"
            "相对离合起点最大差=%6°(限8°)，几何检查=%7")
            .arg(m_ikTimer.isValid()?m_ikTimer.elapsed():-1)
            .arg(s.robotMode).arg(s.feedbackAgeMs).arg(st.button1Pressed)
            .arg(maxMeasured,0,'f',2).arg(maxFromAnchor,0,'f',2)
            .arg(validGeometry?QStringLiteral("通过"):QStringLiteral("拒绝")));
        return;
    }
    const auto target=touchpanel::teleop::limitedServoTarget(s,joints);
    if(!m_robotBackend->sendServoJointTarget(target)) {
        haltRealMotion(QStringLiteral("ServoJ 发送失败"));
        return;
    }
    m_latestServoTarget=target;
    m_haveServoTarget=true;
    m_motionCommandSent=true;
    ++m_ikAccepted;
    ++m_servoCommands;
    m_servoTime.restart();
}

void DeviceController::monitorSingleStep()
{
    if(!m_relayStateClient || !m_stepTimer.isValid()) {
        haltRealMotion(QStringLiteral("单步反馈监控不可用"));
        return;
    }
    const auto s=m_relayStateClient->snapshot();
    const touchpanel::teleop::Vec3 delta={
        s.tcpPose[0]-m_stepExpectedStart[0],
        s.tcpPose[1]-m_stepExpectedStart[1],
        s.tcpPose[2]-m_stepExpectedStart[2]};
    const touchpanel::teleop::Vec3 error={
        s.tcpPose[0]-m_stepProposal.pose[0],
        s.tcpPose[1]-m_stepProposal.pose[1],
        s.tcpPose[2]-m_stepProposal.pose[2]};
    const double measured=touchpanel::teleop::norm(delta);
    const double remaining=touchpanel::teleop::norm(error);
    const auto elapsed=m_stepTimer.elapsed();
    if(!m_relayStateClient->hasFreshState() ||
       s.sessionEpoch!=m_motionSession || s.userIndex!=m_motionUser ||
       s.toolIndex!=m_motionTool || s.error || s.dragging ||
       elapsed>7000) {
        const QString cause=QStringLiteral(
            "单步未完成：耗时 %1 ms，命令应答=%2，"
            "机器人实际位移=%3 mm（计划 %4 mm），"
            "距离目标=%5 mm，Mode=%6，8891年龄=%7 ms，seq=%8")
            .arg(elapsed).arg(m_stepReplyAccepted?QStringLiteral("成功"):QStringLiteral("未成功"))
            .arg(measured,0,'f',3).arg(m_stepProposal.distanceMm,0,'f',3)
            .arg(remaining,0,'f',3).arg(s.robotMode)
            .arg(s.feedbackAgeMs).arg(s.stateSeq);
        haltRealMotion(cause);
        return;
    }
    // Do not treat one early reply + one position sample as proof of arrival.
    // Require real movement and independent, settled, successive 8891 states.
    if(s.stateSeq<=m_stepLastArrivalSeq)return;
    m_stepLastArrivalSeq=s.stateSeq;
    const double minimumEvidence=std::max(0.5,m_stepProposal.distanceMm-0.8);
    const bool arrived=m_stepReplyAccepted &&
        s.stateSeq>m_stepStartSeq && elapsed>=150 &&
        measured>=minimumEvidence &&
        touchpanel::teleop::stepArrived(
            s,m_stepProposal,m_motionUser,m_motionTool);
    if(!arrived) {
        m_stepArrivalStableSamples=0;
        return;
    }
    if(++m_stepArrivalStableSamples<3)return;
    m_realMode=RealMode::None;
    m_motionCommandSent=false;
    m_singleStepVerified=true;
    m_verifiedStepSession=s.sessionEpoch;
    m_motionTimer.stop();
    emit backendMessageChanged(
        QStringLiteral("[STEP VERIFIED] 计划=%1 mm 实际=%2 mm "
                       "剩余误差=%3 mm，耗时=%4 ms，"
                       "起点=(%5,%6,%7)，实测终点=(%8,%9,%10)，seq=%11。"
                       "单步经过持续空闲反馈确认。")
        .arg(m_stepProposal.distanceMm,0,'f',3)
        .arg(measured,0,'f',3)
        .arg(remaining,0,'f',3)
        .arg(elapsed)
        .arg(m_stepExpectedStart[0],0,'f',3)
        .arg(m_stepExpectedStart[1],0,'f',3)
        .arg(m_stepExpectedStart[2],0,'f',3)
        .arg(s.tcpPose[0],0,'f',3)
        .arg(s.tcpPose[1],0,'f',3)
        .arg(s.tcpPose[2],0,'f',3)
        .arg(s.stateSeq));
}

void DeviceController::runServoPTick()
{
    const qint64 actualGapMs=m_servoPTickTimer.isValid()?
        m_servoPTickTimer.restart():DEVICE_CONTROLLER_MOTION_INTERVAL_MS;
    if(!m_servoPTickTimer.isValid())m_servoPTickTimer.start();
    m_servoPMaxTickGapMs=std::max(m_servoPMaxTickGapMs,actualGapMs);
    const bool touchRunning=m_backend && m_backend->isRunning();
    const bool legacyConnected=m_robotBackend && m_robotBackend->isTransitConnected();
    const bool feedbackFresh=m_relayStateClient && m_relayStateClient->hasFreshState();
    const auto robot=m_relayStateClient
        ? m_relayStateClient->snapshot() : touchpanel::RobotSnapshot{};
    const auto st=m_backend?m_backend->latestState():touchpanel::DeviceState{};
    const bool releaseObserved=st.valid &&
        st.button1ReleaseCounter!=m_servoPLastReleaseCounter;
    // A brief feedback gap cannot authorize ANY new motion. Disarm
    // clutch; existing 400ms watchdog still escalates persistent loss.
    if(touchRunning && legacyConnected &&
       touchpanel::teleop::servoPRecoverableFeedbackGap(
           robot,m_motionSession,m_motionUser,m_motionTool)) {
        // Do not erase an in-flight stop handshake and accidentally
        // rearm after a telemetry interruption. A release during the
        // gap must not be treated as permission to finish old motion.
        if(m_cartesianServoP.phase()==touchpanel::teleop::ServoPPhase::WaitingForIdle ||
           (m_motionCommandSent && st.valid &&
            (releaseObserved || !st.button1Pressed))) {
            haltRealMotion(QStringLiteral("ServoP：离合释放/停止期间反馈不新鲜"));
            return;
        }
        if(!m_servoPFeedbackHolding) {
            m_servoPFeedbackHolding=true;
            m_cartesianServoP.reset();
            m_servoPHasTarget=false;
            m_servoPTrackingHolding=false;
            m_servoPTrackingHoldTimer.invalidate();
            emit backendMessageChanged(QStringLiteral(
                "[SERVOP FEEDBACK HOLD] 8891年龄=%1ms；"
                "立即停止下发并撤销离合。待恢复后须松开Button1并重新按下。")
                .arg(robot.feedbackAgeMs));
        }
        return;
    }
    if(!touchRunning || !legacyConnected || !feedbackFresh ||
       !touchpanel::teleop::continuousTelemetryAgesHealthy(robot)) {
        const auto gate=touchpanel::evaluateRobotGate(robot,false);
        haltRealMotion(QStringLiteral(
            "ServoP 通路失效：Touch=%1 Legacy8888=%2 8891=%3 "
            "状态=%4 feedbackAge=%5ms relayAge=%6ms")
            .arg(touchRunning).arg(legacyConnected).arg(feedbackFresh)
            .arg(QString::fromUtf8(touchpanel::gateReasonChinese(gate.reason)))
            .arg(robot.feedbackAgeMs).arg(robot.relayStatusAgeMs));
        return;
    }
    const auto workspaceProblem=servoPWorkspaceProblem(robot.tcpPose);
    if(!workspaceProblem.isEmpty()) {
        haltRealMotion(QStringLiteral("[SERVOP ACTUAL WORKSPACE] ")+workspaceProblem);
        return;
    }
    if(m_servoPFeedbackHolding) {
        if(m_motionCommandSent && st.valid &&
           (releaseObserved || !st.button1Pressed)) {
            haltRealMotion(QStringLiteral("ServoP：反馈恢复时已检测到离合释放"));
            return;
        }
        m_servoPFeedbackHolding=false;
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP FEEDBACK RECOVERED] 反馈已恢复；"
            "必须松开Button1并等待机器人停稳，才可重新抓取基准"));
    }
    if(!st.schedulerRunning || !st.deviceCalibrated ||
       robot.sessionEpoch!=m_motionSession ||
       robot.userIndex!=m_motionUser || robot.toolIndex!=m_motionTool ||
       !robot.enabled || robot.error || robot.dragging || robot.jogging ||
       robot.paused || (robot.robotMode!=5 && robot.robotMode!=7)) {
        haltRealMotion(QStringLiteral(
            "ServoP 状态变化：Mode=%1 Enabled=%2 Error=%3 Drag=%4 "
            "Jog=%5 Pause=%6 Session=%7 User/Tool=%8/%9")
            .arg(robot.robotMode).arg(robot.enabled).arg(robot.error)
            .arg(robot.dragging).arg(robot.jogging).arg(robot.paused)
            .arg(robot.sessionEpoch).arg(robot.userIndex).arg(robot.toolIndex));
        return;
    }
    if(!st.valid) {
        if(++m_invalidTouchTicks>=3)
            haltRealMotion(QStringLiteral("ServoP：连续 3 次无有效 Touch XYZ，停止发送"));
        return;
    }
    m_invalidTouchTicks=0;
    if(st.sampleCounter<=m_lastTouchSampleCounter) {
        if(++m_staleTouchTicks>=3)
            haltRealMotion(QStringLiteral("ServoP：Touch 采样序号未增长，停止发送"));
        return;
    }
    m_staleTouchTicks=0;
    m_lastTouchSampleCounter=st.sampleCounter;
    m_servoPLastReleaseCounter=st.button1ReleaseCounter;
    if(m_cartesianServoP.phase()==touchpanel::teleop::ServoPPhase::Active &&
       st.calibrationEpoch!=m_clutchCalibrationEpoch) {
        haltRealMotion(QStringLiteral("ServoP：Touch 校准代次变化"));
        return;
    }
    m_servoPFollowMetrics.observeActual(robot.tcpPose,robot.stateSeq);
    const bool idle=servoPMeasuredIdle(robot);
    // Monitor the last transmitted setpoint even when no new candidate is
    // generated: an immobile stylus must not hide a persistent lag.
    if(m_cartesianServoP.phase()==touchpanel::teleop::ServoPPhase::Active &&
       m_servoPHasTarget) {
        const auto& accepted=m_cartesianServoP.lastAcceptedTarget();
        const touchpanel::teleop::Vec3 actualError={
            accepted[0]-robot.tcpPose[0],accepted[1]-robot.tcpPose[1],
            accepted[2]-robot.tcpPose[2]};
        const double lag=touchpanel::teleop::norm(actualError);
        if(!std::isfinite(lag) ||
           lag>touchpanel::teleop::kServoPTrackingErrorMm ||
           (m_servoPTrackingHolding && m_servoPTrackingHoldTimer.isValid() &&
            m_servoPTrackingHoldTimer.elapsed()>
                touchpanel::teleop::kServoPTrackingHoldTimeoutMs &&
            lag>touchpanel::teleop::kServoPTrackingResumeMm)) {
            haltRealMotion(QStringLiteral(
                "ServoP：已发送目标跟踪持续异常：LastLag=%1mm")
                .arg(lag,0,'f',2));
            return;
        }
    }
    // A software stop ACK plus distinct post-ACK idle feedback is
    // required before any new clutch. Keep the original 1.5s release
    // deadline; missing ACK/idle terminates this teleop session.
    if(m_cartesianServoP.phase()==touchpanel::teleop::ServoPPhase::WaitingForIdle &&
       m_servoPReleaseTimer.isValid() &&
       m_servoPReleaseTimer.elapsed()>1500 &&
       (m_cartesianServoP.releaseStopPending() || !idle)) {
        haltRealMotion(QStringLiteral(
            "ServoP：松手后 1.5 秒内未确认停止应答/实际停稳"));
        return;
    }
    const auto outcome=m_cartesianServoP.tick(
        st.button1Pressed,st.positionMm,robot.tcpPose,idle,
        robot.stateSeq,m_manualMap,releaseObserved);
    using Action=touchpanel::teleop::ServoPAction;
    if(outcome.action==Action::Invalid) {
        haltRealMotion(QStringLiteral("ServoP：相对映射/工作空间目标无效，已停止"));
        return;
    }
    // Record every active sample, including planner HOLD/None, rather than
    // measuring only the frames that happened to be sent.
    if(outcome.intentValid) {
        // Record the actual controller's immutable, unmodified input intent,
        // including HOLD frames. Never recompute from a mutable GUI mapping.
        m_servoPFollowMetrics.observeMappedDemand(outcome.intent,robot.tcpPose);
        if(outcome.feedbackLimited)m_servoPFollowMetrics.noteLeadLimit();
        if(outcome.wallHoldAxisMask)m_servoPFollowMetrics.noteWorkspaceHold();
    }
    if(outcome.intentValid) {
        // Optional virtual wall freezes outward axes at the last SENT
        // safe target, not at the boundary. Other axes remain subject to
        // speed, tracking, feedback and workspace checks.
        if(outcome.wallHoldAxisMask!=m_servoPWallMask) {
            const bool reportWall=!m_servoPWallLogTimer.isValid() ||
                m_servoPWallLogTimer.elapsed()>=2000;
            if(outcome.wallHoldAxisMask && reportWall) {
                QStringList axes;
                for(int i=0;i<3;++i)if(outcome.wallHoldAxisMask&(1U<<i))
                    axes<<QStringLiteral("XYZ").mid(i,1);
                emit backendMessageChanged(QStringLiteral(
                    "[SERVOP WALL SLIDE] %1轴保持上次安全目标，其他轴可继续；"
                    "不扩大软件边界，不等于物理防碰撞")
                    .arg(axes.join(QStringLiteral(","))));
            } else if(m_servoPWallMask && reportWall) {
                emit backendMessageChanged(QStringLiteral(
                    "[SERVOP WALL RESUMED] 已退出边界切向保持"));
            }
            if(reportWall)m_servoPWallLogTimer.restart();
            m_servoPWallMask=outcome.wallHoldAxisMask;
        }
    }
    if(outcome.action==Action::TrackingHold) {
        // The current Touch goal remains a goal, NOT a queued packet.
        // Stop emitting farther-ahead targets until the actual robot
        // catches up, or the operator moves back toward the last target.
        m_servoPFollowMetrics.noteTrackingHold();
        if(!m_servoPTrackingHolding) {
            m_servoPTrackingHolding=true;
            m_servoPTrackingHoldTimer.start();
        }
        if(!m_servoPTrackingLogTimer.isValid() ||
           m_servoPTrackingLogTimer.elapsed()>=2000) {
            emit backendMessageChanged(QStringLiteral(
                "[SERVOP TRACKING HOLD] 最新Touch目标暂不可下发，"
                "等待8891实际TCP跟上；不排队补发旧目标"));
            m_servoPTrackingLogTimer.restart();
        }
        // Still run the rate/hold statistics below; no motion is sent.
    } else if(outcome.action==Action::RegripBlocked) {
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP REGRIP BLOCKED] 此次按下早于停止确认；"
            "等待应答及三次不同的有效停稳反馈，松开后重新按下。"
            "不会在保持按下期间自动开始运动。"));
    } else if(outcome.action==Action::Ready) {
        m_servoPReleaseTimer.invalidate();
        // Release stop was acknowledged and measured idle (or this
        // clutch never transmitted); no outstanding motion from it.
        m_motionCommandSent=false;
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP READY] 已检测到 Button1 松开及连续 8891 静止反馈；"
            "下一次按下将重新捕获双方的当前坐标"));
    } else if(outcome.action==Action::Captured) {
        m_clutchCalibrationEpoch=st.calibrationEpoch;
        m_servoPHasTarget=false;
        m_servoPWorkspaceBlocked=false;
        m_servoPWorkspaceAxis=-1;
        m_servoPWallMask=0;
        m_servoPWallLogTimer.invalidate();
        m_servoPWorkspaceLogTimer.invalidate();
        m_servoPFollowMetrics.reset(robot.tcpPose,robot.stateSeq);
        m_servoPSentPackets=0;
        m_servoPTrackingHolding=false;
        m_servoPTrackingHoldTimer.invalidate();
        m_servoPTrackingLogTimer.invalidate();
        m_servoPStatsTimer.restart();
        m_servoPReleaseTimer.invalidate();
        m_servoPMaxTickGapMs=0;
        m_servoPMaxSendUs=0;
        m_servoPTotalSendUs=0;
        m_servoPSendSamples=0;
        const auto& p=m_cartesianServoP.anchorRobot();
        const auto& h=m_cartesianServoP.anchorTouch();
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP CLUTCH] 本次新基准：Robot=(%1,%2,%3) "
            "RPY=(%4,%5,%6) Touch=(%7,%8,%9)。"
            "按下这一帧不发送运动目标。")
            .arg(p[0],0,'f',2).arg(p[1],0,'f',2).arg(p[2],0,'f',2)
            .arg(p[3],0,'f',2).arg(p[4],0,'f',2).arg(p[5],0,'f',2)
            .arg(h[0],0,'f',2).arg(h[1],0,'f',2).arg(h[2],0,'f',2));
    } else if(outcome.action==Action::Released) {
        m_servoPWallMask=0;
        m_servoPReleaseTimer.start();
        m_servoPHasTarget=false;
        m_servoPTrackingHolding=false;
        m_servoPTrackingHoldTimer.invalidate();
        // Pure clutch release already prevents any further ServoP sends.
        // Request software stop only once per clutch that sent a command;
        // a no-motion clutch does not generate unnecessary ResetRobot.
        if(outcome.releaseStopRequired) {
            m_robotBackend->requestControlledStop();
            emit backendMessageChanged(QStringLiteral(
                "[SERVOP RELEASE STOP] 已停发 ServoP，已请求 ResetRobot；"
                "等待停止应答和后续实测停稳，非物理急停。"));
        } else {
            emit backendMessageChanged(QStringLiteral(
                "[SERVOP RELEASE] 本次未发送运动指令；等待实测停稳。"));
        }
    } else if(outcome.action==Action::Send) {
        const auto& anchorPose=m_cartesianServoP.anchorRobot();
        // Never silently change the already captured robot orientation.
        for(int i=3;i<6;++i) {
            if(std::abs(std::remainder(robot.tcpPose[i]-anchorPose[i],360.0))>8.0) {
                haltRealMotion(QStringLiteral("ServoP：末端姿态偏离锁定基准超过 8°"));
                return;
            }
        }
        // Exactly one live command arbiter. The independent hard
        // last-sent/actual check above runs even when no command is sent.
        const double lastLag=touchpanel::teleop::servoPCartesianDistance(
            m_cartesianServoP.lastAcceptedTarget(),robot.tcpPose);
        if(!std::isfinite(lastLag) ||
           lastLag>touchpanel::teleop::kServoPTrackingErrorMm) {
            haltRealMotion(QStringLiteral(
                "ServoP：已发送目标实际跟踪误差=%1mm 超过硬限制")
                .arg(lastLag,0,'f',2));
            return;
        }
        if(m_servoPTrackingHolding) {
            m_servoPTrackingHolding=false;
            m_servoPTrackingHoldTimer.invalidate();
            if(!m_servoPTrackingLogTimer.isValid() ||
               m_servoPTrackingLogTimer.elapsed()>=2000) {
                emit backendMessageChanged(QStringLiteral(
                    "[SERVOP TRACKING RESUMED] 新Touch目标通过仲裁"));
                m_servoPTrackingLogTimer.restart();
            }
        }
        m_servoPSendTimer.start();
        const auto send=m_robotBackend->sendServoCartesianTarget(outcome.target);
        const qint64 sendUs=m_servoPSendTimer.nsecsElapsed()/1000;
        m_servoPMaxSendUs=std::max(m_servoPMaxSendUs,sendUs);
        m_servoPTotalSendUs+=sendUs;
        ++m_servoPSendSamples;
        using SendCode=touchpanel::CommunicationBackend::ServoPSendCode;
        if(send.code==SendCode::OutsideWorkspace) {
            m_servoPFollowMetrics.noteWorkspaceHold();
            // The requested target is unsafe but the control CHANNEL is
            // healthy. Do not ResetRobot on every outward attempt. Keep
            // the LAST SUCCESSFULLY SENT target as rate limiter origin,
            // so reversing the stylus can generate a safe setpoint.
            m_cartesianServoP.rejectUnsentTarget();
            // Keep log suppression across HOLD/RESUMED flips. Continuous
            // per-frame log changes can starve the Qt 8891 event loop.
            const bool report=!m_servoPWorkspaceLogTimer.isValid() ||
                m_servoPWorkspaceLogTimer.elapsed()>=2000;
            if(report) {
                const QString axis=QStringLiteral("XYZ").mid(send.axis,1);
                emit backendMessageChanged(QStringLiteral(
                    "[SERVOP WORKSPACE HOLD] %1目标=%2mm，允许区间=[%3,%4]mm，"
                    "当前机器人%1=%5mm。拒绝这帧越界命令，不下发、不重置；"
                    "保持 Button1 向允许的方向移动可继续控制。")
                    .arg(axis)
                    .arg(send.value,0,'f',3)
                    .arg(send.minimum,0,'f',3)
                    .arg(send.maximum,0,'f',3)
                    .arg(robot.tcpPose[send.axis],0,'f',3));
                m_servoPWorkspaceLogTimer.restart();
            }
            m_servoPWorkspaceBlocked=true;
            m_servoPWorkspaceAxis=send.axis;
            return;
        }
        if(send.code!=SendCode::Sent) {
            m_cartesianServoP.rejectUnsentTarget();
            const QString reason=
                send.code==SendCode::Disconnected?
                    QStringLiteral("8888 已断开"):
                send.code==SendCode::InvalidTarget?
                    QStringLiteral("ServoP 姿态/数值无效"):
                    QStringLiteral("TCP 发送失败");
            haltRealMotion(QStringLiteral("ServoP：%1；按既有保护请求停止").arg(reason));
            return;
        }
        // Local write succeeded; even if the state-machine commit fails,
        // the fail-stop must know a motion command may have reached robot.
        m_motionCommandSent=true;
        const auto sentFrom=m_cartesianServoP.lastAcceptedTarget();
        if(!m_cartesianServoP.acknowledgeSentTarget(outcome.target)) {
            haltRealMotion(QStringLiteral("ServoP：已发送目标与离合状态不同步，停止"));
            return;
        }
        m_servoPFollowMetrics.observeSent(sentFrom,outcome.target);
        if(m_servoPWorkspaceBlocked) {
            if(!m_servoPWorkspaceLogTimer.isValid() ||
               m_servoPWorkspaceLogTimer.elapsed()>=2000) {
                emit backendMessageChanged(QStringLiteral(
                    "[SERVOP WORKSPACE RESUMED] 新目标已回到软件工作空间，"
                    "沿合法方向发送；未更改软件边界"));
                m_servoPWorkspaceLogTimer.restart();
            }
            m_servoPWorkspaceBlocked=false;
            m_servoPWorkspaceAxis=-1;
        }
        m_lastServoPCartesianTarget=outcome.target;
        m_servoPHasTarget=true;
        ++m_servoPSentPackets;
    }
    // A held stationary hand never generates a catch-up command. Clear a
    // cosmetic HOLD once measured lag is back inside the old resume
    // threshold; the independent hard tracking gate remains active.
    if(outcome.action==Action::None && m_servoPTrackingHolding &&
       m_cartesianServoP.phase()==touchpanel::teleop::ServoPPhase::Active &&
       m_servoPHasTarget &&
       touchpanel::teleop::servoPCartesianDistance(
           m_cartesianServoP.lastAcceptedTarget(),robot.tcpPose)<=
               touchpanel::teleop::kServoPTrackingResumeMm) {
        m_servoPTrackingHolding=false;
        m_servoPTrackingHoldTimer.invalidate();
    }
    if(m_cartesianServoP.phase()==touchpanel::teleop::ServoPPhase::Active &&
       m_servoPStatsTimer.isValid() &&
       m_servoPStatsTimer.elapsed()>=2000) {
        const double seconds=m_servoPStatsTimer.elapsed()/1000.0;
        const auto& anchorPose=m_cartesianServoP.anchorRobot();
        const touchpanel::teleop::Vec3 actualDelta={
            robot.tcpPose[0]-anchorPose[0],
            robot.tcpPose[1]-anchorPose[1],
            robot.tcpPose[2]-anchorPose[2]};
        double error=0.0;
        if(m_servoPHasTarget) {
            const touchpanel::teleop::Vec3 pending={
                m_lastServoPCartesianTarget[0]-robot.tcpPose[0],
                m_lastServoPCartesianTarget[1]-robot.tcpPose[1],
                m_lastServoPCartesianTarget[2]-robot.tcpPose[2]};
            error=touchpanel::teleop::norm(pending);
        }
        const auto& follow=m_servoPFollowMetrics.window();
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP FOLLOW] 8891实际TCP路径速度=%1mm/s "
            "已发送设定点路径速度=%2mm/s 按发送次数=%3 "
            "FeedbackLead限幅=%4 TrackingHold帧=%5 WorkspaceHold帧=%6 "
            "目标模式=%7 前视限幅=%8mm 原始映射距实机=%9mm "
            "窗口峰值=%10mm（原始绝对映射仅诊断，不补偿）")
            .arg(follow.actualPathMm/seconds,0,'f',1)
            .arg(follow.setpointPathMm/seconds,0,'f',1)
            .arg(follow.sent).arg(follow.leadLimited)
            .arg(follow.trackingHeld).arg(follow.workspaceHeld)
            .arg(QStringLiteral("master式直接位置 / 不补偿受限位移"))
            .arg(touchpanel::teleop::kServoPCommandLeadMm,0,'f',1)
            .arg(follow.latestMappedLagMm,0,'f',1)
            .arg(follow.maxMappedLagMm,0,'f',1));
        m_servoPFollowMetrics.clearWindow();
        emit backendMessageChanged(QStringLiteral(
            "[SERVOP RATE] 实际发送=%1/s 机器人相对基准位移=%2mm "
            "最新目标误差=%3mm，8891 Age=%4ms RelayAge=%5ms "
            "RobotMode=%6 Button1=%7 TickGapMax=%8ms "
            "TcpSendAvg=%9ms TcpSendMax=%10ms")
            .arg(m_servoPSentPackets/seconds,0,'f',1)
            .arg(touchpanel::teleop::norm(actualDelta),0,'f',2)
            .arg(error,0,'f',2).arg(robot.feedbackAgeMs)
            .arg(robot.relayStatusAgeMs).arg(robot.robotMode)
            .arg(st.button1Pressed)
            .arg(m_servoPMaxTickGapMs)
            .arg(m_servoPSendSamples?
                 m_servoPTotalSendUs/1000.0/m_servoPSendSamples:0.0,0,'f',2)
            .arg(m_servoPMaxSendUs/1000.0,0,'f',2));
        m_servoPStatsTimer.restart();
        m_servoPSentPackets=0;
        m_servoPMaxTickGapMs=0;
        m_servoPMaxSendUs=0;
        m_servoPTotalSendUs=0;
        m_servoPSendSamples=0;
    }
}

void DeviceController::runContinuousTick()
{
    const bool touchRunning=m_backend && m_backend->isRunning();
    const bool legacyConnected=m_robotBackend && m_robotBackend->isTransitConnected();
    const bool stateFresh=m_relayStateClient && m_relayStateClient->hasFreshState();
    if(!touchRunning || !legacyConnected || !stateFresh) {
        const auto snapshot=m_relayStateClient
            ? m_relayStateClient->snapshot() : touchpanel::RobotSnapshot{};
        const auto decision=touchpanel::evaluateRobotGate(snapshot,false);
        haltRealMotion(QStringLiteral(
            "连续遥控链路失效：Touch运行=%1 8888连接=%2 8891新鲜=%3，"
            "门禁=%4，FeedbackAge=%5ms RelayAge=%6ms "
            "RelayReady=%7 Ports=%8/%9/%10 Session=%11")
                .arg(touchRunning).arg(legacyConnected).arg(stateFresh)
                .arg(QString::fromUtf8(touchpanel::gateReasonChinese(decision.reason)))
                .arg(snapshot.feedbackAgeMs).arg(snapshot.relayStatusAgeMs)
                .arg(snapshot.relayReady).arg(snapshot.port29999Connected)
                .arg(snapshot.port30003Connected).arg(snapshot.port30004Connected)
                .arg(snapshot.sessionEpoch));
        return;
    }
    const auto st=m_backend->latestState();
    const auto s=m_relayStateClient->snapshot();
    // HDAPI calibration and scheduler lifecycle are real safety gates.
    // A transient incoherent Qt snapshot is a missed sample, not proof
    // of permanent device failure; handle it separately below.
    if(!st.schedulerRunning || !st.deviceCalibrated ||
       s.sessionEpoch!=m_motionSession ||
       s.userIndex!=m_motionUser || s.toolIndex!=m_motionTool ||
       !s.enabled || s.error || s.dragging || s.jogging || s.paused ||
       (s.robotMode!=5 && s.robotMode!=7) ||
       // Do not mistake an occasionally late 200ms relay heartbeat for
       // stale high-rate robot feedback: keep a strict 250ms state limit
       // and a separate 750ms relay_status heartbeat limit.
       !touchpanel::teleop::continuousTelemetryAgesHealthy(s)) {
        haltRealMotion(QStringLiteral(
            "连续遥控状态不符合要求：Touch运行=%1 校准=%2 Mode=%3 "
            "Enabled=%4 Drag=%5 Running=%6 Jog=%7 Pause=%8 Error=%9 "
            "FeedbackAge=%10ms (max250) RelayAge=%11ms (max750) "
            "RelayReady=%12 Ports=%13/%14/%15 Session=%16")
            .arg(st.schedulerRunning).arg(st.deviceCalibrated).arg(s.robotMode)
            .arg(s.enabled).arg(s.dragging).arg(s.running)
            .arg(s.jogging).arg(s.paused).arg(s.error)
            .arg(s.feedbackAgeMs).arg(s.relayStatusAgeMs).arg(s.relayReady)
            .arg(s.port29999Connected).arg(s.port30003Connected)
            .arg(s.port30004Connected).arg(s.sessionEpoch));
        return;
    }

    // ResetRobot is a software stop command. We MUST wait for its ACK and
    // fresh 8891 evidence of physical idle before accepting another clutch.
    if(m_continuousStopping) {
        if(!m_stopTimer.isValid() || m_stopTimer.elapsed()>5000) {
            haltRealMotion(QStringLiteral("离合释放后停止未能在 5 秒内确认"));
            return;
        }
        bool still=false;
        for(double speed:s.jointVelocity) {
            if(!std::isfinite(speed)||std::abs(speed)>1.0)still=true;
        }
        for(int i=0;i<3;++i) {
            if(!std::isfinite(s.tcpVelocity[i])||std::abs(s.tcpVelocity[i])>2.0)
                still=true;
        }
        if(m_stopReplyAccepted && s.robotMode==5 && !s.running &&
           !still && st.valid && !st.button1Pressed) {
            m_continuousStopping=false;
            m_stopReplyAccepted=false;
            m_motionCommandSent=false;
            m_haveServoTarget=false;
            m_ikPending=false;
            m_releaseTicks=0;
            invalidateTeleopBaseline();
            emit backendMessageChanged(QStringLiteral(
                "离合停止已由 8891 确认；连续遥控仍处于待命状态，重新按下 Button1 可捕获新基准"));
        }
        return;
    }

    // Do not send a ServoJ target while the current Touch frame is invalid.
    // The 1 kHz producer can temporarily overlap a Qt snapshot read; it
    // previously caused a complete exit on a single failed eight-retry read.
    if(!st.valid) {
        if(++m_invalidTouchTicks>=3) {
            haltRealMotion(QStringLiteral(
                "Touch 连续 %1 个控制周期没有可用 XYZ 快照；"
                "采样计数=%2，姿态有效=%3，调度器仍运行=%4")
                .arg(m_invalidTouchTicks).arg(st.sampleCounter)
                .arg(st.orientationValid).arg(st.schedulerRunning));
        }
        return; // Pause this tick, and NEVER replay a stale motion target.
    }
    m_invalidTouchTicks=0;
    if(st.sampleCounter<=m_lastTouchSampleCounter) {
        if(++m_staleTouchTicks>=3) {
            haltRealMotion(QStringLiteral(
                "Touch 采样计数连续 %1 个控制周期未增长：sample=%2")
                .arg(m_staleTouchTicks).arg(st.sampleCounter));
        }
        return;
    }
    m_staleTouchTicks=0;
    m_lastTouchSampleCounter=st.sampleCounter;

    // The previous ServoP controller stayed armed across clutch releases.
    // Keep that workflow, but request a bounded software stop after each
    // active ServoJ gesture; never resume until idle/stop are confirmed.
    if(!st.button1Pressed) {
        if(++m_releaseTicks<2)return; // filter one transient HDAPI tick
        if(m_motionCommandSent || m_ikPending) {
            m_continuousStopping=true;
            m_haveServoTarget=false;
            m_stopReplyAccepted=false;
            m_stopTimer.start();
            m_ikPending=false;
            m_robotBackend->requestControlledStop();
            emit backendMessageChanged(QStringLiteral(
                "Touch Button1 已释放；请求停止本次 ServoJ，等待机器人实际停稳"));
        } else {
            invalidateTeleopBaseline();
        }
        return;
    }
    m_releaseTicks=0;
    if(!m_lastButtonPressed) {
        if(s.robotMode!=5 || s.running) {
            emit backendMessageChanged(QStringLiteral(
                "等待机器人空闲才能重新捕获连续控制基准"));
            return;
        }
        m_haveServoTarget=false;
        m_clutchRangeSaturated=false;
        m_touchOrigin=st.positionMm;
        m_touchRotationOrigin=st.rotationMatrix;
        m_robotPoseOrigin=s.tcpPose;
        m_ikJointOrigin=s.jointPosition;
        m_clutchCalibrationEpoch=st.calibrationEpoch;
        m_motionBaselineReady=true;m_lastButtonPressed=true;
        emit backendMessageChanged(QStringLiteral("连续控制：已捕获新的 Touch/机器人实际基准"));
        return;
    }
    if(st.calibrationEpoch!=m_clutchCalibrationEpoch) {
        haltRealMotion(QStringLiteral("Touch 校准周期发生变化"));
        return;
    }
    // ServoJ is an online 33Hz interface, whereas remote dashboard IK
    // replies arrive at an irregular rate. Repeat ONLY the latest validated
    // joint target while it is bounded in time AND all fresh 8891/Touch
    // gates still hold. Repeated ServoJ writes do NOT create new IK targets.
    if(m_haveServoTarget && m_servoTime.isValid() &&
       m_servoTime.elapsed()<=touchpanel::teleop::kMaxValidatedSetpointHoldMs) {
        if(!m_robotBackend->sendServoJointTarget(m_latestServoTarget)) {
            haltRealMotion(QStringLiteral("ServoJ 保持发送失败"));
            return;
        }
        ++m_servoCommands;
    }
    if(m_ikPending) {
        if(m_ikTimer.elapsed()>touchpanel::teleop::kIKRequestTimeoutMs)
            haltRealMotion(QStringLiteral("InverseSolution 在 %1ms 内没有应答；停止连续发送")
                .arg(touchpanel::teleop::kIKRequestTimeoutMs));
        return;
    }
    touchpanel::teleop::Pose6 requested{};
    if(!touchpanel::teleop::manualPositionPreview(
        m_manualMap,touchpanel::teleop::sub(st.positionMm,m_touchOrigin),
        m_robotPoseOrigin,requested)) {
        haltRealMotion(QStringLiteral("目标生成失败"));
        return;
    }
    // Saturate the requested target at the same 15mm per-clutch limit.
    // Reaching that limit must NOT repeatedly reset the physical robot:
    // a normal Touch gesture can exceed 15mm almost instantly.
    touchpanel::teleop::ClutchBound bounded{};
    if(!touchpanel::teleop::boundClutchTarget(
         requested,m_robotPoseOrigin,bounded)) {
        haltRealMotion(QStringLiteral("离合目标不是有限数值，停止控制"));
        return;
    }
    const double displacement=bounded.rawDistanceMm;
    requested=bounded.pose;
    if(bounded.saturated && !m_clutchRangeSaturated) {
        emit backendMessageChanged(
            QStringLiteral("[CLUTCH LIMIT] Touch 映射原始位移=%1mm；"
                           "机器人目标软限幅为 %2mm。"
                           "保持 Button1 可继续接近限幅终点，"
                           "松开并确认停稳后可重新建立基准。")
              .arg(bounded.rawDistanceMm,0,'f',2)
              .arg(touchpanel::teleop::kClutchTrialLimitMm,0,'f',1));
    }
    m_clutchRangeSaturated=bounded.saturated;
    const touchpanel::teleop::Vec3 tracking={
        requested[0]-s.tcpPose[0],requested[1]-s.tcpPose[1],
        requested[2]-s.tcpPose[2]};
    const double distance=touchpanel::teleop::norm(tracking);
    // Report effective control throughput even if the Touch generates zero
    // IK requests: otherwise a stationary or deadzoned input stays silent.
    if(m_continuousStatsTimer.isValid() && m_continuousStatsTimer.elapsed()>=2000) {
        const double seconds=m_continuousStatsTimer.elapsed()/1000.0;
        const touchpanel::teleop::Vec3 reached={
            s.tcpPose[0]-m_robotPoseOrigin[0],
            s.tcpPose[1]-m_robotPoseOrigin[1],
            s.tcpPose[2]-m_robotPoseOrigin[2]};
        const touchpanel::teleop::Vec3 clipped={
            requested[0]-m_robotPoseOrigin[0],
            requested[1]-m_robotPoseOrigin[1],
            requested[2]-m_robotPoseOrigin[2]};
        emit backendMessageChanged(
            QStringLiteral("[SERVO RATE] IK请求=%1/s 成功逆解=%2/s ServoJ发送=%3/s，"
                           "Touch原始映射=%4mm 软限目标=%5mm 机器人实测=%6mm "
                           "当前目标误差=%7mm，Mode=%8 8891延迟=%9ms "
                           "RelayAge=%10ms 增益=%11 IK-RTT均值=%12ms 峰值=%13ms")
                .arg(m_ikQueries/seconds,0,'f',1)
                .arg(m_ikAccepted/seconds,0,'f',1)
                .arg(m_servoCommands/seconds,0,'f',1)
                .arg(displacement,0,'f',2)
                .arg(touchpanel::teleop::norm(clipped),0,'f',2)
                .arg(touchpanel::teleop::norm(reached),0,'f',2)
                .arg(distance,0,'f',2)
                .arg(s.robotMode).arg(s.feedbackAgeMs)
                .arg(s.relayStatusAgeMs).arg(m_manualMap.scale,0,'f',2)
                .arg(m_ikRttSamples?m_ikRttSumMs/m_ikRttSamples:-1)
                .arg(m_ikRttSamples?m_ikRttMaxMs:-1));
        m_continuousStatsTimer.restart();
        m_ikQueries=0;m_ikAccepted=0;m_servoCommands=0;
        m_ikRttSumMs=0;m_ikRttMaxMs=0;m_ikRttSamples=0;
    }
    if(distance<0.15)return;
    if(distance>1.2) {
        for(int i=0;i<3;++i)
            requested[i]=s.tcpPose[i]+1.2*tracking[i]/distance;
    }
    // The new remote IK workflow is position-only, not a six-axis stylus
    // orientation command. Use current robot TCP angles unmodified.
    requested[3]=s.tcpPose[3];requested[4]=s.tcpPose[4];requested[5]=s.tcpPose[5];
    m_ikPending=true;
    m_ikTimer.start();
    ++m_ikQueries;
    m_robotBackend->requestInverseSolution(requested,m_motionUser,m_motionTool,s.jointPosition);
}

void DeviceController::haltRealMotion(const QString& cause)
{
    const bool sent=m_motionCommandSent;
    if(m_realMode==RealMode::CartesianServoP)
        m_pollTimer.setInterval(DEVICE_CONTROLLER_POLL_INTERVAL_MS);
    m_realMode=RealMode::None;
    m_cartesianServoP.reset();
    m_servoPTrackingHolding=false;
    m_servoPFeedbackHolding=false;
    m_servoPTrackingHoldTimer.invalidate();
    m_motionCommandSent=false;
    m_haveServoTarget=false;
    m_clutchRangeSaturated=false;
    m_continuousStopping=false;
    m_stopReplyAccepted=false;
    m_releaseTicks=0;
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

