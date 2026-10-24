#include "DeviceController.h"
#include "TeleopGeometry.h"
#include "RelayStateClient.h"

#include <cmath>
#include <QString>

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
        emit backendMessageChanged(QStringLiteral("六维预览未启用：标定绑定的设备或 8891 会话已变化，必须重新标定"));
        return;
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
        invalidateTeleopBaseline();
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
