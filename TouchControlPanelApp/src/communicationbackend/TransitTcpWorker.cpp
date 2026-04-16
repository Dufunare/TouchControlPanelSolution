#include "TransitTcpWorker.h"

#include <QAbstractSocket>
#include <QTcpSocket>

void TransitTcpWorker::initialize()
{
	if (m_socket != nullptr)
	{
		return;
	}

	m_socket = new QTcpSocket(this);
	connect(m_socket, &QTcpSocket::connected, this, [this]()
		{
			emit connected();
			emit connectionStateChanged(true, QStringLiteral("已连接中转站。"));
		});

	connect(m_socket, &QTcpSocket::disconnected, this, [this]()
		{
			emit disconnected();
			emit connectionStateChanged(false, QStringLiteral("中转站连接已断开。"));
		});

	connect(m_socket, &QTcpSocket::readyRead, this, &TransitTcpWorker::onReadyRead);
	connect(m_socket, &QTcpSocket::errorOccurred, this,
		[this](QAbstractSocket::SocketError)
		{
			const QString errorText = m_socket->errorString();
			emit errorOccurred(errorText);
			emit connectionStateChanged(false, QStringLiteral("连接异常：%1").arg(errorText));
		});
}

void TransitTcpWorker::connectToHost(const QString& ip, quint16 port)
{
	if (m_socket == nullptr)
	{
		return;
	}

	if (m_socket->state() == QAbstractSocket::ConnectedState)
	{
		emit connectionStateChanged(true, QStringLiteral("已连接中，无需重复连接。"));
		return;
	}

	m_socket->abort();
	m_socket->connectToHost(ip, port);
}

void TransitTcpWorker::disconnectFromHost()
{
	if (m_socket == nullptr)
	{
		return;
	}

	if (m_socket->state() == QAbstractSocket::UnconnectedState)
	{
		emit connectionStateChanged(false, QStringLiteral("当前并未连接设备。"));
		return;
	}

	m_socket->disconnectFromHost();
}

void TransitTcpWorker::sendLine(const QString& line)
{
	if (m_socket == nullptr)
	{
		return;
	}

	if (m_socket->state() != QAbstractSocket::ConnectedState)
	{
		emit errorOccurred(QStringLiteral("发送失败：TCP 未连接。"));
		return;
	}

	QByteArray payload = line.toUtf8();
	if (!payload.endsWith('\n'))
	{
		payload.append('\n');
	}

	const qint64 written = m_socket->write(payload);
	if (written <= 0)
	{
		emit errorOccurred(QStringLiteral("发送失败：%1").arg(m_socket->errorString()));
		return;
	}

	emit lineSent(line);
}

void TransitTcpWorker::onReadyRead()
{
	if (m_socket == nullptr)
	{
		return;
	}

	m_buffer.append(m_socket->readAll());

	while (true)
	{
		const int lineEnd = m_buffer.indexOf('\n');
		if (lineEnd < 0)
		{
			break;
		}

		QByteArray line = m_buffer.left(lineEnd);
		m_buffer.remove(0, lineEnd + 1);

		if (line.endsWith('\r'))
		{
			line.chop(1);
		}

		emit lineReceived(QString::fromUtf8(line));
	}
}
