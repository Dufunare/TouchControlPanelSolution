#pragma once

#include <QObject>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QImage>
#include <QTimer>
#include <QUdpSocket>

// Receives the relay_station legacy UDP 8886 JPEG stream. The camera itself
// is opened on the relay computer (UVC / DirectShow), not on this PC.
class RelayVideoClient final : public QObject
{
    Q_OBJECT

public:
    explicit RelayVideoClient(QObject* parent = nullptr);
    ~RelayVideoClient() override;

    void start(const QString& relayHost, quint16 relayPort = 8886);
    void stop();
    bool isActive() const { return m_active; }

signals:
    void frameReady(const QImage& frame);
    void frameLost();
    void stateChanged(bool active, bool receiving, const QString& detail);
    void message(const QString& text);

private:
    void sendHeartbeat();
    void readDatagrams();
    void checkSignal();
    void publishState(const QString& detail);

    QUdpSocket m_socket;
    QTimer m_heartbeat;
    QTimer m_signalWatchdog;
    QHostAddress m_relayAddress;
    quint16 m_relayPort = 0;
    QElapsedTimer m_lastFrame;
    QElapsedTimer m_fpsWindow;
    bool m_active = false;
    bool m_receiving = false;
    int m_framesInWindow = 0;
};
