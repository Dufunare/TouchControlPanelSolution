#include "RelayVideoClient.h"

#include <QNetworkDatagram>
#include <QBuffer>
#include <QImageReader>
#include <QThread>

namespace
{
    constexpr int kHeartbeatMs = 2000;   // relay_station times out after 5000 ms.
    constexpr int kFrameStaleMs = 2500;
    constexpr int kStartupTimeoutMs = 5000;
    constexpr qsizetype kMaxJpegDatagramBytes = 65507;
    constexpr int kDrainIntervalMs = 33;
    constexpr int kMaxDatagramsPerDrain = 64;
    constexpr qint64 kMaxPixels = 4096LL * 2160;
}

RelayVideoClient::RelayVideoClient(QObject* parent)
    : QObject(parent)
{
    // QObject member children migrate with this worker to its video thread.
    // Their member destructors detach them before QObject base destruction.
    m_socket.setParent(this);
    m_heartbeat.setParent(this);
    m_signalWatchdog.setParent(this);
    m_drainTimer.setParent(this);
    m_drainTimer.setInterval(kDrainIntervalMs);
    connect(&m_drainTimer, &QTimer::timeout, this, &RelayVideoClient::readDatagrams);
    m_heartbeat.setInterval(kHeartbeatMs);
    connect(&m_heartbeat, &QTimer::timeout, this, &RelayVideoClient::sendHeartbeat);

    m_signalWatchdog.setInterval(500);
    connect(&m_signalWatchdog, &QTimer::timeout, this, &RelayVideoClient::checkSignal);
    connect(&m_socket, &QUdpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        if (m_active) {
            emit message(QStringLiteral("视频 UDP Socket 错误：%1").arg(m_socket.errorString()));
        }
    });
}

RelayVideoClient::~RelayVideoClient()
{
    stop();
}

void RelayVideoClient::publishState(const QString& detail)
{
    emit stateChanged(m_active, m_receiving, detail);
}

void RelayVideoClient::start(const QString& relayHost, quint16 relayPort)
{
    stop();

    QHostAddress address;
    // The relay uses an IPv4 address for the legacy camera subscription.
    if (!address.setAddress(relayHost.trimmed()) ||
        address.protocol() != QAbstractSocket::IPv4Protocol || relayPort == 0) {
        publishState(QStringLiteral("无效的中转站 IPv4 地址或端口"));
        emit message(QStringLiteral("视频启动失败：请输入有效的中转站 IPv4 地址。"));
        return;
    }

    if (!m_socket.bind(QHostAddress::AnyIPv4, 0)) {
        const QString error = m_socket.errorString();
        publishState(QStringLiteral("无法绑定本地 UDP 端口：%1").arg(error));
        emit message(QStringLiteral("视频启动失败：%1").arg(error));
        return;
    }

    m_relayAddress = address;
    m_relayPort = relayPort;
    m_active = true;
    m_receiving = false;
    m_timeoutReported = false;
    m_framesInWindow = 0;
    m_lastFrame.start();
    m_fpsWindow.start();

    sendHeartbeat();
    m_heartbeat.start();
    m_signalWatchdog.start();
    m_drainTimer.start();
    publishState(QStringLiteral("等待中转站 UVC 摄像头画面…"));
}

void RelayVideoClient::sendHeartbeat()
{
    if (!m_active) return;
    const QByteArray command("VIDEO_START");
    const qint64 bytes = m_socket.writeDatagram(command, m_relayAddress, m_relayPort);
    if (bytes != command.size()) {
        emit message(QStringLiteral("VIDEO_START 发送失败：%1").arg(m_socket.errorString()));
    }
}

void RelayVideoClient::stop()
{
    m_heartbeat.stop();
    m_signalWatchdog.stop();
    m_drainTimer.stop();

    if (m_active) {
        m_socket.writeDatagram("VIDEO_STOP", m_relayAddress, m_relayPort);
    }
    m_active = false;
    m_receiving = false;
    m_framesInWindow = 0;
    m_socket.close();
    emit frameLost();
    publishState(QStringLiteral("已停止"));
}

void RelayVideoClient::readDatagrams()
{
    if (!m_active) return;
    // Bounded worker-side drain. Decode only the latest valid JPEG
    // header in this batch; never enqueue all received video frames.
    QByteArray latest;
    for (int n=0; n<kMaxDatagramsPerDrain && m_socket.hasPendingDatagrams(); ++n) {
        const QNetworkDatagram datagram=m_socket.receiveDatagram();
        if (datagram.senderAddress()!=m_relayAddress ||
            datagram.senderPort()!=m_relayPort) continue;
        const QByteArray bytes=datagram.data();
        if (bytes.size()<4 || bytes.size()>kMaxJpegDatagramBytes ||
            static_cast<unsigned char>(bytes.front())!=0xff ||
            static_cast<unsigned char>(bytes.at(1))!=0xd8) continue;
        latest=bytes;
    }
    if (latest.isEmpty() || m_socket.hasPendingDatagrams()) return;
    QBuffer buffer(&latest);
    if (!buffer.open(QIODevice::ReadOnly)) return;
    QImageReader reader(&buffer,"JPEG");
    const QSize size=reader.size();
    if (!size.isValid() || size.width()>4096 || size.height()>4096 ||
        qint64(size.width())*size.height()>kMaxPixels) return;
    const QImage frame=reader.read();
    if (frame.isNull()) return;
    m_lastFrame.restart();
    const bool wasReceiving=m_receiving;
    m_receiving=true;
    m_timeoutReported=false;
    ++m_framesInWindow;
    emit frameReady(frame);
    if (!wasReceiving || m_fpsWindow.elapsed()>=1000) {
        const double fps=m_fpsWindow.elapsed()>0
            ? 1000.0*m_framesInWindow/m_fpsWindow.elapsed() : 0.0;
        publishState(QStringLiteral("接收中 · %1×%2 · %3 FPS")
            .arg(frame.width()).arg(frame.height()).arg(fps,0,'f',1));
        m_framesInWindow=0;
        m_fpsWindow.restart();
    }
}

void RelayVideoClient::checkSignal()
{
    if (!m_active || !m_lastFrame.isValid()) return;
    const qint64 elapsedMs = m_lastFrame.elapsed();
    if (m_receiving && elapsedMs > kFrameStaleMs) {
        m_receiving = false;
        m_framesInWindow = 0;
        m_fpsWindow.restart();
        emit frameLost();
        publishState(QStringLiteral("视频信号中断，正在重试订阅…"));
    } else if (!m_receiving && elapsedMs >= kStartupTimeoutMs && !m_timeoutReported) {
        m_timeoutReported=true;
        publishState(QStringLiteral("未收到视频：检查中转站、摄像头及 UDP 8886"));
        emit message(QStringLiteral("视频等待超时：检查中转站已启动、摄像头已连接及防火墙 UDP 8886/回程端口。"));
    }
}
