#include "TransitTcpClient.h"

TransitTcpClient::TransitTcpClient(QObject* parent)
	: QObject(parent)
{
	m_worker = new TransitTcpWorker();
	m_worker->moveToThread(&m_workerThread);

	connect(&m_workerThread, &QThread::started, m_worker, &TransitTcpWorker::initialize);

	connect(this, &TransitTcpClient::connectRequested, m_worker, &TransitTcpWorker::connectToHost, Qt::QueuedConnection);
	connect(this, &TransitTcpClient::disconnectRequested, m_worker, &TransitTcpWorker::disconnectFromHost, Qt::QueuedConnection);
	connect(this, &TransitTcpClient::sendLineRequested, m_worker, &TransitTcpWorker::sendLine, Qt::QueuedConnection);

	connect(m_worker, &TransitTcpWorker::connected, this, &TransitTcpClient::connected);
	connect(m_worker, &TransitTcpWorker::disconnected, this, &TransitTcpClient::disconnected);
	connect(m_worker, &TransitTcpWorker::connectionStateChanged, this, &TransitTcpClient::connectionStateChanged);
	connect(m_worker, &TransitTcpWorker::lineReceived, this, &TransitTcpClient::lineReceived);
	connect(m_worker, &TransitTcpWorker::lineSent, this, &TransitTcpClient::lineSent);
	connect(m_worker, &TransitTcpWorker::errorOccurred, this, &TransitTcpClient::errorOccurred);

	connect(&m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);

	m_workerThread.start();
}

TransitTcpClient::~TransitTcpClient()
{
	m_workerThread.quit();
	m_workerThread.wait();
}

void TransitTcpClient::connectToHost(const QString& ip, quint16 port)
{
	emit connectRequested(ip, port);
}

void TransitTcpClient::disconnectFromHost()
{
	emit disconnectRequested();
}

void TransitTcpClient::sendLine(const QString& line)
{
	emit sendLineRequested(line);
}
