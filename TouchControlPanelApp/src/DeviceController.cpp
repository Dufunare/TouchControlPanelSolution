#include "DeviceController.h"
#include "TeleopGeometry.h"

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

    if (!state.schedulerRunning && m_pollTimer.isActive())
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

void DeviceController::startTeleop()
{
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

    if (!m_robotBackend->isTransitConnected())
    {
        emit backendMessageChanged("开始控制失败：中转站 TCP 未连接。");
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

    emit backendMessageChanged("六维映射预览已启用（仅计算，不下发运动命令）。需完成标定、逆解和安全闭环后才能启用实体运动。");
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
    emit backendMessageChanged("已停止六维映射预览；本版本无遥操作运动下发。");
}

void DeviceController::onMotionTick()
{
    if (m_backend == nullptr || m_robotBackend == nullptr) return;
    if (!m_backend->isRunning() || !m_robotBackend->isTransitConnected()) {
        m_lastButtonPressed = false;
        m_motionBaselineReady = false;
        return;
    }

    const auto st = m_backend->latestState();
    if (!st.valid || !st.orientationValid || !st.schedulerRunning) {
        m_motionBaselineReady = false;
        m_previewLastValid = false;
        return;
    }

    if (!st.button1Pressed) {
        m_lastButtonPressed = false;
        m_motionBaselineReady = false;
        m_previewLastValid = false;
        return;
    }

    // Do not submit competing pending requests (the existing backend has one slot).
    if (!m_lastButtonPressed) {
        m_motionBaselineReady = false;
        m_previewLastValid = false;
        m_robotBackend->requestRobotMode();
        m_modeQueryCooldownTicks = 10;
        m_poseQueryCooldownTicks = 0;
    }
    m_lastButtonPressed = true;
    if (!m_robotBackend->isRobotModeReady()) {
        if (--m_modeQueryCooldownTicks <= 0) {
            m_robotBackend->requestRobotMode();
            m_modeQueryCooldownTicks = 10;
        }
        return;
    }

    if (!m_motionBaselineReady) {
        if (--m_poseQueryCooldownTicks <= 0) {
            m_robotBackend->requestCurrentPose();
            m_poseQueryCooldownTicks = 10;
        }
        std::array<double, 6> pose{};
        if (!m_robotBackend->tryGetLastPose(pose)) return;

        m_touchOrigin = st.positionMm;
        m_touchRotationOrigin = st.rotationMatrix;
        m_robotOrigin = { pose[0], pose[1], pose[2] };
        m_motionBaselineReady = true;
        m_previewLast = pose;
        m_previewLastValid = true;
        emit backendMessageChanged("六维映射起点已捕获。注意：现有 GetPose 缓存尚不支持新鲜度证明；仅可预览。");
        return;
    }

    touchpanel::teleop::Clutch clutch;
    clutch.valid = true;
    clutch.touchPosition = m_touchOrigin;
    clutch.touchRotation = m_touchRotationOrigin;
    clutch.robotPose = m_previewLast;
    // Anchor robot orientation is frozen at the first sample, not each tick.
    // The following static capture will be replaced by a freshness-bound 8891 pose.
    std::array<double, 6> pose{};
    if (!m_robotBackend->tryGetLastPose(pose)) return;
    clutch.robotPose = pose;

    touchpanel::teleop::Calibration mapping; // fallback axis convention, not field-calibrated
    touchpanel::teleop::Pose6 desired{};
    if (!touchpanel::teleop::mapClutchedPose(mapping, clutch,
                                            st.positionMm, st.rotationMatrix, desired)) {
        m_motionBaselineReady = false;
        emit backendMessageChanged("六维预览已中止：无效姿态矩阵或欧拉角奇异区间。");
        return;
    }

    if (m_previewLastValid &&
        touchpanel::teleop::smallStep(desired, m_previewLast, 0.25, 0.25)) return;
    m_previewLast = desired;
    m_previewLastValid = true;

    static unsigned previewTick = 0;
    if (++previewTick % 10 != 0) return;
    emit backendMessageChanged(
        QStringLiteral("[DRY RUN / NO ROBOT MOTION] XYZ=(%1,%2,%3) RxRyRz=(%4,%5,%6)")
        .arg(desired[0], 0, 'f', 2).arg(desired[1], 0, 'f', 2)
        .arg(desired[2], 0, 'f', 2).arg(desired[3], 0, 'f', 2)
        .arg(desired[4], 0, 'f', 2).arg(desired[5], 0, 'f', 2));
}
