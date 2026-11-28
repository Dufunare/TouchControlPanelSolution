#pragma once
#include <QObject>
#include <QAbstractSocket>
#include <QElapsedTimer>
#include <QJsonObject>
#include <QTcpSocket>
#include <QTimer>
#include <array>
#include <chrono>
#include <mutex>
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
    // Owner thread only. This sends an allowlisted Dashboard calculation,
    // never a motion command. No diagnostic result grants motion authority.
    bool requestOrientationIkProbe(const std::array<double,6>& pose);
    bool orientationProbePending() const {return !m_probeId.isEmpty();}
    // An ambiguous/timed-out dashboard reply may leave the relay FIFO
    // out of alignment. Do not enter a real mode in this process afterwards.
    bool orientationProbeInterlock() const {return orientationProbePending()||m_probeUncertain;}
signals:
    void statusChanged(const QString& description);
    void stateInvalidated();
    void telemetryUpdated(const QString& summary);
    void orientationProbeFinished(const QString& summary);
    void cameraObservationReceived(const QJsonObject& observation,qint64 receivedAtMs);
private:
    void readIncoming();
    void handleFrame(const QByteArray& bytes);
    void invalidate(const QString& reason);
    void sendHello();
    void refreshStatus();
    void publishReadOnlySnapshot();
    void finishOrientationProbe(const QString& result,bool uncertain=false);
    void handleOrientationProbeReply(const QJsonObject& object);
    QString buildTelemetrySummary() const;
    QTcpSocket m_socket;
    QTimer m_watchdog;
    QElapsedTimer m_stateAge;
    touchpanel::protocol::V2FrameCodec m_codec;
    QString m_session;
    QString m_host;
    quint16 m_port=8891;
    bool m_handshaken=false, m_ready=false, m_valid=false;
    quint64 m_sessionEpoch=0;
    QElapsedTimer m_relayAge;
    QElapsedTimer m_summaryEmitAge;
    QString m_lastTelemetrySummary;
    touchpanel::RobotSnapshot m_snapshot;
    quint64 m_sequence=0;
    quint64 m_probeCounter=100;
    QString m_probeId,m_probeCommand;
    bool m_probeUncertain=false;
    QElapsedTimer m_probeAge;
    touchpanel::RobotSnapshot m_probeStart{};
    std::array<double,6> m_pose{},m_joints{};
    QString m_status=QStringLiteral("未连接 v2 遥测");
    mutable std::mutex m_publicMutex;
    touchpanel::RobotSnapshot m_publicSnapshot{};
    QString m_publicStatus=QStringLiteral("未连接 v2 遥测");
    std::chrono::steady_clock::time_point m_publicTime{};
};
