#include "RobotControllerFacade.h"

RobotControllerFacade::RobotControllerFacade(QObject* parent)
	: QObject(parent)
{
	m_pendingTimer.setSingleShot(true);

	connect(&m_pendingTimer, &QTimer::timeout, this, [this]()
		{
			finishPending(false, QStringLiteral("等待机械臂回复超时。"));
		});

	connect(&m_tcpClient, &TransitTcpClient::connected, this,
		[this]()
		{
			m_tcpConnected = true;
			emit tcpConnectionChanged(true);
			emit backendMessageChanged(QStringLiteral("中转站 TCP 已建立连接。"));
		});

	connect(&m_tcpClient, &TransitTcpClient::disconnected, this, &RobotControllerFacade::onTcpDisconnected);

	connect(&m_tcpClient, &TransitTcpClient::connectionStateChanged, this,
		[this](bool connected, const QString& detail)
		{
			m_tcpConnected = connected;
			emit tcpConnectionChanged(connected);
			emit backendMessageChanged(detail);
		});

	connect(&m_tcpClient, &TransitTcpClient::lineSent, this,
		[this](const QString& raw)
		{
			emit tcpTxMessage(raw);
		});

	connect(&m_tcpClient, &TransitTcpClient::lineReceived, this, &RobotControllerFacade::onLineReceived);

	connect(&m_tcpClient, &TransitTcpClient::errorOccurred, this,
		[this](const QString& text)
		{
			emit backendMessageChanged(text);
			if (m_hasPending)
			{
				finishPending(false, text);
			}
		});

	emit robotStatusChanged(QStringLiteral("未知（等待机械臂上报）"));
}

void RobotControllerFacade::connectTransit(const QString& ipOverride, quint16 portOverride)
{
	const QString host = ipOverride.trimmed().isEmpty() ? m_defaultIp : ipOverride.trimmed();
	const quint16 port = (portOverride == 0) ? m_defaultPort : portOverride;

	emit backendMessageChanged(QStringLiteral("正在连接中转站 %1:%2 ...").arg(host).arg(port));
	m_tcpClient.connectToHost(host, port);
}

void RobotControllerFacade::disconnectTransit()
{
	m_tcpClient.disconnectFromHost();
}

void RobotControllerFacade::powerOn()
{
	sendRobotCommand(QStringLiteral("PowerOn"), QStringLiteral("PowerOn()"), 15000);
}

void RobotControllerFacade::enableRobot()
{
	sendRobotCommand(QStringLiteral("EnableRobot"), QStringLiteral("EnableRobot()"), 5000);
}

void RobotControllerFacade::emergencyStop()
{
	sendRobotCommand(QStringLiteral("EmergencyStop"), QStringLiteral("EmergencyStop()"), 5000);
}

void RobotControllerFacade::startDrag()
{
	sendRobotCommand(QStringLiteral("StartDrag"), QStringLiteral("StartDrag()"), 5000);
}

void RobotControllerFacade::stopDrag()
{
	sendRobotCommand(QStringLiteral("StopDrag"), QStringLiteral("StopDrag()"), 5000);
}

void RobotControllerFacade::onLineReceived(const QString& rawLine)
{
	emit tcpRxMessage(rawLine);

	const TransitParsedFrame frame = m_parser.parse(rawLine);

	if (m_hasPending)
	{
		const QString detail = frame.payload.isEmpty() ? rawLine : frame.payload;
		finishPending(true, detail);
		return;
	}

	emit backendMessageChanged(QStringLiteral("收到中转站消息：%1").arg(frame.payload.isEmpty() ? rawLine : frame.payload));
}

void RobotControllerFacade::onTcpDisconnected()
{
	m_tcpConnected = false;
	emit tcpConnectionChanged(false);

	if (m_hasPending)
	{
		finishPending(false, QStringLiteral("连接断开，命令未完成。"));
	}
}

void RobotControllerFacade::sendRobotCommand(const QString& operation, const QString& command, int timeoutMs)
{
	if (!m_tcpConnected)
	{
		emit backendMessageChanged(QStringLiteral("未连接中转站，无法执行 %1。").arg(operation));
		emit robotOperationFinished(operation, false, QStringLiteral("TCP 未连接"));
		return;
	}

	if (m_hasPending)
	{
		emit backendMessageChanged(QStringLiteral("当前仍有命令在执行，请稍后再试。"));
		emit robotOperationFinished(operation, false, QStringLiteral("上一条命令尚未完成"));
		return;
	}

	m_pending.operation = operation;
	m_pending.command = command;
	m_hasPending = true;

	const QString packet = m_parser.pack(29999, command);
	m_tcpClient.sendLine(packet);

	emit backendMessageChanged(QStringLiteral("已发送 %1，等待机械臂回复... ").arg(command));
	m_pendingTimer.start(timeoutMs);
}

void RobotControllerFacade::finishPending(bool success, const QString& detail)
{
	if (!m_hasPending)
	{
		return;
	}

	const QString operation = m_pending.operation;
	m_pending = {};
	m_hasPending = false;
	m_pendingTimer.stop();

	emit robotOperationFinished(operation, success, detail);

	if (success)
	{
		emit backendMessageChanged(QStringLiteral("%1 执行成功：%2").arg(operation).arg(detail));
	}
	else
	{
		emit backendMessageChanged(QStringLiteral("%1 执行失败：%2").arg(operation).arg(detail));
	}
}
