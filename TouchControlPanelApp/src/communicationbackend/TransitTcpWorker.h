#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QtGlobal>

class QTcpSocket;

class TransitTcpWorker : public QObject
{
	Q_OBJECT

public slots:
	void initialize();
	void connectToHost(const QString& ip, quint16 port);
	void disconnectFromHost();
	void sendLine(const QString& line);

private slots:
	void onReadyRead();

signals:
	void connected();
	void disconnected();
	void connectionStateChanged(bool connected, const QString& detail);
	void lineReceived(const QString& rawLine);
	void lineSent(const QString& rawLine);
	void errorOccurred(const QString& errorText);

private:
	QTcpSocket* m_socket = nullptr;
	QByteArray m_buffer;
};
