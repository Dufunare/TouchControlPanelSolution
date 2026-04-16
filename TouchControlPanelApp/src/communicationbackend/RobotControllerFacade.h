#pragma once

#include <QObject>
#include <QTimer>
#include <QtGlobal>

#include "TransitProtocolParser.h"
#include "TransitTcpClient.h"

class RobotControllerFacade : public QObject
{
	Q_OBJECT

public:
	explicit RobotControllerFacade(QObject* parent = nullptr);

public slots:
	void connectTransit(const QString& ipOverride = QString(), quint16 portOverride = 0);
	void disconnectTransit();

	void powerOn();
	void enableRobot();
	void emergencyStop();
	void startDrag();
	void stopDrag();

signals:
	void backendMessageChanged(const QString& text);
	void tcpTxMessage(const QString& raw);
	void tcpRxMessage(const QString& raw);
	void tcpConnectionChanged(bool connected);
	void robotStatusChanged(const QString& text);
	void robotOperationFinished(const QString& operation, bool success, const QString& detail);

private slots:
	void onLineReceived(const QString& rawLine);
	void onTcpDisconnected();

private:
	void sendRobotCommand(const QString& operation, const QString& command, int timeoutMs);
	void finishPending(bool success, const QString& detail);

	struct PendingCommand
	{
		QString operation;
		QString command;
	};

	TransitTcpClient m_tcpClient;
	TransitProtocolParser m_parser;

	QString m_defaultIp = QStringLiteral("127.0.0.1");
	quint16 m_defaultPort = 9000;

	bool m_tcpConnected = false;
	bool m_hasPending = false;
	PendingCommand m_pending;
	QTimer m_pendingTimer;
};
