#include "DeviceController.h"

#include <QString>

namespace
{
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

DeviceController::DeviceController(touchpanel::TouchBackend* backend, QObject* parent)
    : QObject(parent), m_backend(backend)
{
    m_pollTimer.setInterval(8); // 约 125 FPS 的前端刷新频率
    m_pollTimer.setTimerType(Qt::PreciseTimer);

    connect(&m_pollTimer, &QTimer::timeout, this, &DeviceController::pollDeviceState);

    connect(&m_robotFacade, &RobotControllerFacade::backendMessageChanged,
        this, &DeviceController::backendMessageChanged);
    connect(&m_robotFacade, &RobotControllerFacade::tcpConnectionChanged,
        this, &DeviceController::tcpConnectionChanged);
    connect(&m_robotFacade, &RobotControllerFacade::robotStatusChanged,
        this, &DeviceController::robotStatusChanged);
    connect(&m_robotFacade, &RobotControllerFacade::tcpTxMessage,
        this, &DeviceController::tcpTxMessage);
    connect(&m_robotFacade, &RobotControllerFacade::tcpRxMessage,
        this, &DeviceController::tcpRxMessage);
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
    m_robotFacade.connectTransit(ipOverride, portOverride);
}

void DeviceController::disconnectTransit()
{
    m_robotFacade.disconnectTransit();
}

void DeviceController::powerOnRobot()
{
    m_robotFacade.powerOn();
}

void DeviceController::enableRobot()
{
    m_robotFacade.enableRobot();
}

void DeviceController::emergencyStopRobot()
{
    m_robotFacade.emergencyStop();
}

void DeviceController::startDragRobot()
{
    m_robotFacade.startDrag();
}

void DeviceController::stopDragRobot()
{
    m_robotFacade.stopDrag();
}
