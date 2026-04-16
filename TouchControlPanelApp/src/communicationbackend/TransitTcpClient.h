#pragma once

#include <QObject>
#include <QString>
#include <QThread>

#include "TransitTcpWorker.h"

class TransitTcpClient : public QObject
{
	Q_OBJECT

public:
	explicit TransitTcpClient(QObject* parent = nullptr);
	~TransitTcpClient() override;

public slots:
	void connectToHost(const QString& ip, quint16 port);
	void disconnectFromHost();
	void sendLine(const QString& line);

signals:
	void connected();
	void disconnected();
	void connectionStateChanged(bool connected, const QString& detail);
	void lineReceived(const QString& rawLine);
	void lineSent(const QString& rawLine);
	void errorOccurred(const QString& errorText);

private:
	QThread m_workerThread;
	TransitTcpWorker* m_worker = nullptr;

signals:
	void connectRequested(const QString& ip, quint16 port);
	void disconnectRequested();
	void sendLineRequested(const QString& line);
};
