#pragma once
#include <QObject>
#include <QAbstractSocket>
#include <QElapsedTimer>
#include <QJsonObject>
#include <QTcpSocket>
#include <QTimer>
#include <array>
#include "RelayV2Frame.h"
#include "RobotTelemetry.h"

class RelayStateClient final : public QObject {
    Q_OBJECT
public:
    explicit RelayStateClient(QObject* parent=nullptr);
    void connectToRelay(const QString& host,quint16 port=8891);
    void disconnectRelay();
    bool hasFreshState() const;
    bool tryGetFreshPose(std::array<double,6>& pose) const;
    touchpanel::RobotSnapshot snapshot() const;
    QString statusDescription() const;
signals:
    void statusChanged(const QString& description);
    void stateInvalidated();
    void telemetryUpdated(const QString& summary);
private:
    void readIncoming();
    void handleFrame(const QByteArray& bytes);
    void invalidate(const QString& reason);
    void sendHello();
    void refreshStatus();
    QString buildTelemetrySummary() const;
    QTcpSocket m_socket;
    QTimer m_watchdog;
    QElapsedTimer m_stateAge;
    touchpanel::protocol::V2FrameCodec m_codec;
    QString m_session;
    QString m_host;
    bool m_handshaken=false, m_ready=false, m_valid=false;
    quint64 m_sessionEpoch=0;
    QElapsedTimer m_relayAge;
    QElapsedTimer m_summaryEmitAge;
    QString m_lastTelemetrySummary;
    touchpanel::RobotSnapshot m_snapshot;
    quint64 m_sequence=0;
    std::array<double,6> m_pose{},m_joints{};
    QString m_status=QStringLiteral("未连接 v2 遥测");
};
