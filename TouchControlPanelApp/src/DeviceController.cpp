#include "DeviceController.h"

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
    touchpanel::CommunicationBackend& communicationBackend,
    QObject* parent)
    : QObject(parent), m_backend(backend), m_robotBackend(communicationBackend)
{
    m_pollTimer.setInterval(8); // 约 125 FPS 的前端刷新频率
    m_pollTimer.setTimerType(Qt::PreciseTimer);
    m_motionTimer.setInterval(33);
    m_motionTimer.setTimerType(Qt::PreciseTimer);

    connect(&m_pollTimer, &QTimer::timeout, this, &DeviceController::pollDeviceState);
    connect(&m_motionTimer, &QTimer::timeout, this, &DeviceController::onMotionTick);

    m_robotBackend.setMessageCallback([this](const std::string& text)
        {
            emit backendMessageChanged(decodeBackendText(text));
        });

    m_robotBackend.setConnectionCallback([this](bool connected)
        {
            emit tcpConnectionChanged(connected);
        });

    m_robotBackend.setRobotStatusCallback([this](const std::string& text)
        {
            emit robotStatusChanged(decodeBackendText(text));
        });

    m_robotBackend.setTcpTxCallback([this](const std::string& text)
        {
            emit tcpTxMessage(decodeBackendText(text));
        });

    m_robotBackend.setTcpRxCallback([this](const std::string& text)
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
        emit backendMessageChanged("Touch 后端初始化成功。");
    }
    else
    {
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
        initializeBackend();
        if (!m_backend->isInitialized())
        {
            return;
        }
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
    m_robotBackend.connectTransit(ipOverride.toStdString(), portOverride);
}

void DeviceController::disconnectTransit()
{
    if (m_motionTimer.isActive())
    {
        stopTeleop();
    }

    m_robotBackend.disconnectTransit();
}

void DeviceController::powerOnRobot()
{
    m_robotBackend.powerOn();
}

void DeviceController::enableRobot()
{
    m_robotBackend.enableRobot();
}

void DeviceController::disableRobot()
{
    m_robotBackend.disableRobot();
}

void DeviceController::clearRobotError()
{
    m_robotBackend.clearError();
}

void DeviceController::emergencyStopRobot()
{
    m_robotBackend.emergencyStop();
}

void DeviceController::startDragRobot()
{
    m_robotBackend.startDrag();
}

void DeviceController::stopDragRobot()
{
    m_robotBackend.stopDrag();
}

void DeviceController::startTeleop()
{
    if (m_backend == nullptr)
    {
        emit backendMessageChanged("后端对象为空，无法开始控制。");
        return;
    }

    if (!m_backend->isInitialized() || !m_backend->isRunning())
    {
        emit backendMessageChanged("开始控制失败：Touch 尚未初始化或未开始采集。");
        return;
    }

    if (!m_robotBackend.isTransitConnected())
    {
        emit backendMessageChanged("开始控制失败：中转站 TCP 未连接。");
        return;
    }

    m_lastButtonPressed = false;
    m_motionBaselineReady = false;
    m_modeQueryCooldownTicks = 0;
    m_poseQueryCooldownTicks = 0;
    m_hasLastSent = false;

    if (!m_motionTimer.isActive())
    {
        m_motionTimer.start();
    }

    emit backendMessageChanged("已启动 33ms 运动发送。按住 Touch 按钮后开始跟随。");
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
    emit backendMessageChanged("已停止运动发送。");
}

void DeviceController::onMotionTick()
{
    if (m_backend == nullptr)
    {
        return;
    }

    if (!m_backend->isRunning() || !m_robotBackend.isTransitConnected())
    {
        return;
    }

    const auto st = m_backend->latestState();
    if (!st.valid)
    {
        return;
    }

    if (!st.button1Pressed)
    {
        m_lastButtonPressed = false;
        m_motionBaselineReady = false;
        m_hasLastSent = false;
        return;
    }

    if (!m_lastButtonPressed)
    {
        m_robotBackend.requestRobotMode();
        m_robotBackend.requestCurrentPose();
        m_modeQueryCooldownTicks = 10;
        m_poseQueryCooldownTicks = 10;
        emit backendMessageChanged("检测到按钮按下，正在同步 RobotMode 和 GetPose 基准...");
    }
    m_lastButtonPressed = true;

    if (!m_robotBackend.isRobotModeReady())
    {
        if (m_modeQueryCooldownTicks <= 0)
        {
            m_robotBackend.requestRobotMode();
            m_modeQueryCooldownTicks = 10;
        }
        else
        {
            --m_modeQueryCooldownTicks;
        }
        return;
    }

    if (!m_motionBaselineReady)
    {
        std::array<double, 6> pose{};

        if (!m_robotBackend.tryGetLastPose(pose))
        {
            if (m_poseQueryCooldownTicks <= 0)
            {
                m_robotBackend.requestCurrentPose();
                m_poseQueryCooldownTicks = 10;
            }
            else
            {
                --m_poseQueryCooldownTicks;
            }
            return;
        }

        m_touchOrigin = { st.positionMm[0], st.positionMm[1], st.positionMm[2] };
        m_robotOrigin = { pose[0], pose[1], pose[2] };
        m_motionBaselineReady = true;
        m_hasLastSent = false;
        emit backendMessageChanged(
            QStringLiteral("GetPose 基准同步完成：Robot(%1,%2,%3) Touch(%4,%5,%6)，开始下发 ServoP。")
                .arg(m_robotOrigin[0], 0, 'f', 3)
                .arg(m_robotOrigin[1], 0, 'f', 3)
                .arg(m_robotOrigin[2], 0, 'f', 3)
                .arg(m_touchOrigin[0], 0, 'f', 3)
                .arg(m_touchOrigin[1], 0, 'f', 3)
                .arg(m_touchOrigin[2], 0, 'f', 3));
    }

    const double dx = st.positionMm[0] - m_touchOrigin[0];
    const double dy = st.positionMm[1] - m_touchOrigin[1];
    const double dz = st.positionMm[2] - m_touchOrigin[2];

    // Touch 局部坐标 -> 机械臂笛卡尔坐标（先用简化映射，后续可按现场标定调整）
    const double x = m_robotOrigin[0] + dx * kTeleopScaleX;
    const double y = m_robotOrigin[1] - dz * kTeleopScaleY;
    const double z = m_robotOrigin[2] + dy * kTeleopScaleZ;

    constexpr double kMotionThresholdMm = 0.3;
    if (m_hasLastSent)
    {
        const double dx = x - m_lastSent[0];
        const double dy = y - m_lastSent[1];
        const double dz = z - m_lastSent[2];
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance < kMotionThresholdMm)
        {
            return;
        }
    }

    m_robotBackend.sendMotion(x, y, z);
    m_lastSent = { x, y, z };
    m_hasLastSent = true;
}
