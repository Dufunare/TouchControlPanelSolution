#include "RelayStateClient.h"
#include "OrientationConventionAudit.h"
#include "TeleopExecutionPolicy.h"
#include "ServoPWorkspace.h"
#include <QRegularExpression>
#include <QStringList>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>
#include <QNetworkProxy>
#include <QThread>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr int kFreshMs=400; // stricter than relay_station's 500ms readiness
constexpr qsizetype kReadChunk=64*1024;
bool parseU64(const QJsonValue& value,quint64& result) {
    if(!value.isString())return false;
    const QString s=value.toString();
    if(s.isEmpty()||s.size()>20)return false;
    for(const QChar c:s)if(c<'0'||c>'9')return false;
    bool ok=false;const auto n=s.toULongLong(&ok,10);
    if(!ok)return false;result=n;return true;
}
bool six(const QJsonValue& v,std::array<double,6>& out) {
    if(!v.isArray())return false;
    const auto a=v.toArray();if(a.size()!=6)return false;
    for(int i=0;i<6;++i) {
        if(!a[i].isDouble()||!std::isfinite(a[i].toDouble()))return false;
        out[i]=a[i].toDouble();
    }
    return true;
}
// Accept numeric formatting differences, but insist on the complete
// echoed pose, user/tool, near-selection flag and six seed joints.
// The relay request ID alone is insufficient: its controller ledger
// matches replies in FIFO order, not by echoed command contents.
bool parseIkEcho(const QString& call,std::array<double,15>& values) {
    static const QRegularExpression re(
        QStringLiteral(R"(^\s*InverseSolution\s*\(([^{}()]+),\s*\{([^{}()]+)\}\s*\)\s*$)"),
        QRegularExpression::CaseInsensitiveOption);
    const auto match=re.match(call);
    if(!match.hasMatch())return false;
    const auto first=match.captured(1).split(',');
    const auto seed=match.captured(2).split(',');
    if(first.size()!=9 || seed.size()!=6)return false;
    for(int i=0;i<15;++i) {
        bool ok=false;
        values[i]=(i<9?first[i]:seed[i-9]).trimmed().toDouble(&ok);
        if(!ok||!std::isfinite(values[i]))return false;
    }
    return true;
}
bool matchingIkEcho(const QString& expected,const QString& received) {
    std::array<double,15> a{},b{};
    if(!parseIkEcho(expected,a)||!parseIkEcho(received,b))return false;
    for(int i=0;i<15;++i)
        if(std::abs(a[i]-b[i])>(i>=6&&i<=8?0.0:0.001))return false;
    return true;
}
QString sixText(const std::array<double,6>& a) {
    QStringList tokens;
    for(double x:a)tokens<<QString::number(x,'f',3);
    return tokens.join(',');
}
}
RelayStateClient::RelayStateClient(QObject* parent):QObject(parent) {
    m_socket.setParent(this);
    m_watchdog.setParent(this);
    // The relay runs on a trusted LAN. Never inherit HTTP/HTTPS system
    // proxies (including transparent VPN proxy settings) for raw TCP 8891.
    // QTcpSocket may otherwise fail with UnsupportedSocketOperationError:
    // "The proxy type is invalid for this operation".
    m_socket.setProxy(QNetworkProxy::NoProxy);
    m_watchdog.setInterval(100);
    connect(&m_watchdog,&QTimer::timeout,this,&RelayStateClient::refreshStatus);
    connect(&m_socket,&QTcpSocket::connected,this,&RelayStateClient::sendHello);
    connect(&m_socket,&QTcpSocket::readyRead,this,&RelayStateClient::readIncoming);
    connect(&m_socket,&QTcpSocket::disconnected,this,[this]{invalidate(QStringLiteral("v2 连接已断开"));});
    connect(&m_socket,&QTcpSocket::errorOccurred,this,[this](QAbstractSocket::SocketError err){
        const QString cause=(err==QAbstractSocket::UnsupportedSocketOperationError)
            ? QStringLiteral("TCP 代理配置不兼容；此连接已设置 NoProxy")
            : m_socket.errorString();
        invalidate(QStringLiteral("v2 %1:%2 连接错误 [%3]: %4")
            .arg(m_host).arg(m_port).arg(static_cast<int>(err)).arg(cause));
    });
    m_watchdog.start();
}
void RelayStateClient::connectToRelay(const QString& host,quint16 port) {
    disconnectRelay();
    m_host=host.trimmed();
    m_port=port;
    m_socket.setProxy(QNetworkProxy::NoProxy);
    m_status=QStringLiteral("正在直接连接 v2 %1:%2（NoProxy）…")
        .arg(m_host).arg(m_port);
    emit statusChanged(m_status);
    m_socket.connectToHost(m_host,m_port);
}
void RelayStateClient::disconnectRelay() {
    m_socket.abort();
    invalidate(QStringLiteral("v2 遥测已断开"));
}
void RelayStateClient::invalidate(const QString& why) {
    if(orientationProbePending())finishOrientationProbe(QStringLiteral("会话失效；结果作废"));
    const bool wasValid=m_valid||m_ready||m_handshaken;
    m_valid=false;m_ready=false;m_handshaken=false;m_session.clear();
    m_sequence=0;m_stateAge.invalidate();m_relayAge.invalidate();m_codec.reset();
    m_snapshot=touchpanel::RobotSnapshot{};
    if(m_status!=why){m_status=why;emit statusChanged(m_status);}
    publishReadOnlySnapshot();
    if(wasValid)emit stateInvalidated();
}
void RelayStateClient::sendHello() {
    // Fresh 8891 robot_state/relay_status messages must not be held behind
    // delayed ACK/Nagle on a latency-sensitive control feedback socket.
    m_socket.setSocketOption(QAbstractSocket::LowDelayOption,1);
    m_codec.reset();m_session.clear();m_handshaken=false;m_ready=false;m_valid=false;
    m_snapshot=touchpanel::RobotSnapshot{};m_stateAge.invalidate();m_relayAge.invalidate();
    const QJsonObject hello{{"v",2},{"type","hello"},{"request_id","1"},
        {"session_id",""},{"client","touchvla_windows"},
        {"capabilities",QJsonArray{QStringLiteral("robot_state"),QStringLiteral("relay_status")}}};
    const QByteArray json=QJsonDocument(hello).toJson(QJsonDocument::Compact);
    const auto wire=touchpanel::protocol::V2FrameCodec::encode(
        std::string_view(json.constData(),static_cast<std::size_t>(json.size())));
    m_socket.write(wire.data(),static_cast<qint64>(wire.size()));
    m_status=QStringLiteral("v2 握手中...");
    emit statusChanged(m_status);
}
void RelayStateClient::readIncoming() {
    while(m_socket.bytesAvailable()>0) {
        const QByteArray chunk=m_socket.read(kReadChunk);
        if(chunk.isEmpty())break;
        std::vector<std::string> frames;
        if(!m_codec.feed(chunk.constData(),static_cast<size_t>(chunk.size()),frames)) {
            invalidate(QStringLiteral("v2 报文长度/缓存非法"));
            m_socket.abort();return;
        }
        for(const auto& frame:frames) {
            handleFrame(QByteArray(frame.data(),static_cast<qsizetype>(frame.size())));
            if(m_socket.state()!=QAbstractSocket::ConnectedState)return;
        }
    }
    // Safety state transitions still invalidate immediately in handleFrame,
    // but cosmetic status/telemetry text updates happen once per read.
    refreshStatus();
}
void RelayStateClient::handleFrame(const QByteArray& bytes) {
    QJsonParseError error;
    const auto doc=QJsonDocument::fromJson(bytes,&error);
    if(error.error!=QJsonParseError::NoError||!doc.isObject()) {
        invalidate(QStringLiteral("v2 JSON 解析失败"));m_socket.abort();return;
    }
    const auto o=doc.object();
    if(o.value("v").toInt(-1)!=2||!o.value("type").isString()||
       !o.value("session_id").isString()||!o.value("request_id").isString()) {
        invalidate(QStringLiteral("v2 消息结构非法"));m_socket.abort();return;
    }
    const QString type=o.value("type").toString(),session=o.value("session_id").toString();
    if(!m_handshaken) {
        if(type!="hello_ack"||o.value("request_id").toString()!="1"||session.isEmpty()) {
            invalidate(QStringLiteral("v2 握手响应不符"));m_socket.abort();return;
        }
        m_handshaken=true;m_session=session;++m_sessionEpoch;
        m_snapshot.sessionEstablished=true;m_snapshot.sessionEpoch=m_sessionEpoch;
        m_status=QStringLiteral("v2 握手完成；等待机器人状态");
        emit statusChanged(m_status);return;
    }
    if(session!=m_session) {
        invalidate(QStringLiteral("v2 会话不一致"));m_socket.abort();return;
    }
    if((type=="reply" || type=="error") && orientationProbePending() &&
       o.value("request_id").toString()==m_probeId) {
        handleOrientationProbeReply(o);
        return;
    }
    if(type=="relay_status") {
        if(!o.value("relay_ready").isBool()||!o.value("port_29999_connected").isBool()||
           !o.value("port_30003_connected").isBool()||!o.value("port_30004_connected").isBool()) {
            invalidate(QStringLiteral("v2 中转站状态格式错误"));m_socket.abort();return;
        }
        const bool ready=o.value("relay_ready").toBool()&&
            o.value("port_29999_connected").toBool()&&
            o.value("port_30003_connected").toBool()&&
            o.value("port_30004_connected").toBool();
        if(m_ready&&!ready) {m_valid=false;emit stateInvalidated();}
        m_ready=ready;
        m_relayAge.restart();
        m_snapshot.relayStatusSeen=true;
        m_snapshot.relayReady=ready;
        m_snapshot.port29999Connected=o.value("port_29999_connected").toBool();
        m_snapshot.port30003Connected=o.value("port_30003_connected").toBool();
        m_snapshot.port30004Connected=o.value("port_30004_connected").toBool();
        return;
    }
    if(type=="robot_state") {
        quint64 seq=0;std::array<double,6> pose{},joints{};
        const auto rv=o.value("robot_mode");
        if(!o.value("valid").isBool()||!o.value("valid").toBool()||
           !parseU64(o.value("state_seq"),seq)||seq==0||
           !rv.isDouble()||!six(o.value("tool_vector_actual"),pose)||
           !six(o.value("q_actual"),joints)||!o.value("error_status").isBool()||
           !o.value("enable_status").isBool()||
           !o.value("drag_status").isBool()) {
            if(m_valid){m_valid=false;emit stateInvalidated();}
            return;
        }
        // Prevent wraparound/reordered feedback within one v2 session.
        if(seq<=m_sequence)return;
        m_sequence=seq;
        m_pose=pose;m_joints=joints;
        auto velocity=std::array<double,6>{},tcpVelocity=std::array<double,6>{},target=std::array<double,6>{};
        const auto uv=o.value("user"),tv=o.value("tool");
        if(!six(o.value("qd_actual"),velocity)||!six(o.value("tcp_speed_actual"),tcpVelocity)||
           !six(o.value("q_target"),target)||!uv.isDouble()||!tv.isDouble()||
           !o.value("running_status").isBool()||!o.value("jog_status").isBool()||
           !o.value("pause_cmd_flag").isBool()||!o.value("run_queued_cmd").isBool()) {
            m_valid=false;emit stateInvalidated();return;
        }
        m_snapshot.feedbackSeen=true;m_snapshot.feedbackValid=true;
        m_snapshot.stateSeq=seq;m_snapshot.robotMode=rv.toInt(-1);
        m_snapshot.tcpPose=pose;m_snapshot.jointPosition=joints;
        // Optional extension already emitted by relay_station. Never retain
        // a quaternion from the preceding sample if this one lacks it.
        m_snapshot.actualQuaternion={};
        m_snapshot.actualQuaternionValid=false;
        const auto quaternion=o.value("actual_quaternion").toArray();
        if(quaternion.size()==4) {
            std::array<double,4> q{};
            bool numeric=true;
            for(int i=0;i<4;++i) {
                if(!quaternion[i].isDouble()) {numeric=false;break;}
                q[i]=quaternion[i].toDouble();
            }
            touchpanel::teleop::Mat3 rotation{};
            if(numeric && touchpanel::teleop::quaternionToRotation(q,rotation)) {
                m_snapshot.actualQuaternion=q;
                m_snapshot.actualQuaternionValid=true;
            }
        }
        m_snapshot.jointVelocity=velocity;m_snapshot.tcpVelocity=tcpVelocity;
        m_snapshot.jointTarget=target;
        m_snapshot.userIndex=uv.toInt(-1);m_snapshot.toolIndex=tv.toInt(-1);
        m_snapshot.enabled=o.value("enable_status").toBool();
        m_snapshot.dragging=o.value("drag_status").toBool();
        m_snapshot.error=o.value("error_status").toBool();
        m_snapshot.running=o.value("running_status").toBool();
        m_snapshot.jogging=o.value("jog_status").toBool();
        m_snapshot.paused=o.value("pause_cmd_flag").toBool();
        m_snapshot.queuedMotion=o.value("run_queued_cmd").toBool();
        // Motion permission remains separately disabled: this is observation only.
        m_valid=true;m_stateAge.restart();
        return;
    }
    if(type=="error") {
        m_status=QStringLiteral("v2 服务端错误: %1").arg(o.value("code").toString());
        emit statusChanged(m_status);
    }
}
bool RelayStateClient::requestOrientationIkProbe(const std::array<double,6>& pose) {
    using namespace touchpanel::teleop;
    if(QThread::currentThread()!=thread() || orientationProbePending())return false;
    const auto s=snapshot();
    if(!idleForExperimentalStep(s,0,0) || feedbackZyxErrorDeg(s)>0.5 ||
       checkServoPWorkspace(pose).code!=ServoPWorkspaceCode::Allowed)return false;
    // V3 section 2.3: InverseSolution is a calculation on 29999.
    // Construct exclusively from finite typed values, not arbitrary text.
    m_probeCommand=QStringLiteral("InverseSolution(%1,0,0,1,{%2})")
        .arg(sixText(pose)).arg(sixText(s.jointPosition));
    m_probeId=QString::number(++m_probeCounter);
    m_probeStart=s;m_probeAge.start();
    const QJsonObject command{{"v",2},{"type","command"},{"request_id",m_probeId},
        {"session_id",m_session},{"target_port",29999},{"command",m_probeCommand}};
    const auto json=QJsonDocument(command).toJson(QJsonDocument::Compact);
    const auto wire=touchpanel::protocol::V2FrameCodec::encode(
        std::string_view(json.constData(),static_cast<size_t>(json.size())));
    if(wire.empty() || m_socket.write(wire.data(),static_cast<qint64>(wire.size()))!=
                      static_cast<qint64>(wire.size())) {
        finishOrientationProbe(QStringLiteral("诊断请求写入失败；不授权运动"));
        return false;
    }
    return true;
}
void RelayStateClient::finishOrientationProbe(const QString& result) {
    m_probeId.clear();m_probeCommand.clear();m_probeAge.invalidate();m_probeStart={};
    emit orientationProbeFinished(QStringLiteral("[ROTATION IK / READ ONLY] ")+result);
}
void RelayStateClient::handleOrientationProbeReply(const QJsonObject& o) {
    using namespace touchpanel::teleop;
    if(o.value("type").toString()!="reply" || o.value("target_port").toInt()!=29999 ||
       o.value("command_name").toString().compare(QStringLiteral("InverseSolution"),Qt::CaseInsensitive)!=0 ||
       !o.value("controller_code").isDouble() || o.value("controller_code").toInt(-1)!=0) {
        finishOrientationProbe(QStringLiteral("控制器/中转站拒绝逆解，或应答格式不匹配"));
        return;
    }
    static const QRegularExpression reply(
        QStringLiteral(R"(^\s*0\s*,\s*\{([^{}]*)\}\s*,\s*(InverseSolution\s*\(.*\))\s*;?\s*$)"),
        QRegularExpression::CaseInsensitiveOption);
    const auto match=reply.match(o.value("raw_reply").toString());
    if(!match.hasMatch() || !matchingIkEcho(m_probeCommand,match.captured(2))) {
        finishOrientationProbe(QStringLiteral("逆解回显与完整请求不匹配；丢弃结果"));
        return;
    }
    const auto fields=match.captured(1).split(',');
    if(fields.size()!=6) {
        finishOrientationProbe(QStringLiteral("逆解返回的关节数无效"));
        return;
    }
    std::array<double,6> joints{};
    for(int i=0;i<6;++i) {
        bool ok=false;joints[i]=fields[i].trimmed().toDouble(&ok);
        if(!ok||!std::isfinite(joints[i])) {
            finishOrientationProbe(QStringLiteral("逆解返回非有限关节值"));return;
        }
    }
    const auto now=snapshot();
    bool sameStart=idleForExperimentalStep(now,0,0) &&
        now.sessionEpoch==m_probeStart.sessionEpoch &&
        feedbackZyxErrorDeg(now)<=0.5 && m_probeAge.isValid() &&
        m_probeAge.elapsed()<=1500;
    double maxDelta=0;
    for(int i=0;i<6;++i) {
        if(std::abs(now.jointPosition[i]-m_probeStart.jointPosition[i])>0.2)
            sameStart=false;
        maxDelta=std::max(maxDelta,std::abs(joints[i]-m_probeStart.jointPosition[i]));
    }
    if(!sameStart) {
        finishOrientationProbe(QStringLiteral("逆解期间机器人状态变化或超时；丢弃结果"));return;
    }
    const auto msg=QStringLiteral("seq=%1 目标=%2 解=(%3) 最大关节差=%4° RTT=%5ms；仅终点逆解证据，不保证路径、限位或无碰撞")
        .arg(m_probeStart.stateSeq).arg(m_probeCommand).arg(sixText(joints))
        .arg(maxDelta,0,'f',3).arg(m_probeAge.elapsed());
    finishOrientationProbe(msg);
}

bool RelayStateClient::hasFreshState() const {
    const auto state=snapshot();
    return state.sessionEstablished &&
           touchpanel::evaluateRobotGate(state,false).canCaptureCalibration;
}
bool RelayStateClient::tryGetFreshPose(std::array<double,6>& pose) const {
    const auto state=snapshot();
    if(!state.sessionEstablished ||
       !touchpanel::evaluateRobotGate(state,false).canCaptureCalibration)return false;
    pose=state.tcpPose;return true;
}
touchpanel::RobotSnapshot RelayStateClient::snapshot() const {
    if(QThread::currentThread()!=thread()) {
        // Only GUI read-only tools use this cache. Live control reads
        // the full-rate state directly on the socket's owning thread.
        std::lock_guard<std::mutex> lock(m_publicMutex);
        auto result=m_publicSnapshot;
        const auto extra=std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now()-m_publicTime).count();
        const auto age=[extra](int prior) {
            if(prior<0)return prior;
            return static_cast<int>(std::min<std::int64_t>(
                std::numeric_limits<int>::max(),
                static_cast<std::int64_t>(prior)+std::max<std::int64_t>(0,extra)));
        };
        result.feedbackAgeMs=age(result.feedbackAgeMs);
        result.relayStatusAgeMs=age(result.relayStatusAgeMs);
        if(result.feedbackAgeMs>kFreshMs)result.feedbackValid=false;
        return result;
    }
    auto result=m_snapshot;
    result.sessionEstablished=m_handshaken&&m_socket.state()==QAbstractSocket::ConnectedState;
    result.relayStatusAgeMs=m_relayAge.isValid()?static_cast<int>(m_relayAge.elapsed()):-1;
    result.feedbackAgeMs=m_stateAge.isValid()?static_cast<int>(m_stateAge.elapsed()):-1;
    result.feedbackValid=m_valid;
    return result;
}

QString RelayStateClient::buildTelemetrySummary() const {
    const auto s=snapshot();
    if(!s.sessionEstablished)
        return QStringLiteral("8891 未连接");
    if(!s.feedbackSeen)
        return QStringLiteral("等待首个有效 robot_state…");
    const QString freshness=(!s.feedbackValid||s.feedbackAgeMs<0||s.feedbackAgeMs>400)
        ? QStringLiteral("STALE") : QStringLiteral("LIVE");
    const auto& p=s.tcpPose;
    const auto& q=s.jointPosition;
    const auto& v=s.jointVelocity;
    return QStringLiteral(
        "%1 · age %2 ms · seq %3\n"
        "TCP mm/deg  X %4  Y %5  Z %6  Rx %7  Ry %8  Rz %9\n"
        "Joints deg  J1 %10  J2 %11  J3 %12  J4 %13  J5 %14  J6 %15\n"
        "qd deg/s    %16  %17  %18  %19  %20  %21\n"
        "Mode %22 · Enabled %23 · Drag %24 · Running %25 · Error %26 · User %27 / Tool %28\n"
        "Jog %29 · Pause %30 · Queued %31\n"
        "Actual Q [wxyz] %32 · ZYX Δ %33")
        .arg(freshness).arg(s.feedbackAgeMs).arg(s.stateSeq)
        .arg(p[0],0,'f',1).arg(p[1],0,'f',1).arg(p[2],0,'f',1)
        .arg(p[3],0,'f',1).arg(p[4],0,'f',1).arg(p[5],0,'f',1)
        .arg(q[0],0,'f',1).arg(q[1],0,'f',1).arg(q[2],0,'f',1)
        .arg(q[3],0,'f',1).arg(q[4],0,'f',1).arg(q[5],0,'f',1)
        .arg(v[0],0,'f',1).arg(v[1],0,'f',1).arg(v[2],0,'f',1)
        .arg(v[3],0,'f',1).arg(v[4],0,'f',1).arg(v[5],0,'f',1)
        .arg(s.robotMode).arg(s.enabled?QStringLiteral("yes"):QStringLiteral("no"))
        .arg(s.dragging?QStringLiteral("yes"):QStringLiteral("no"))
        .arg(s.running?QStringLiteral("yes"):QStringLiteral("no"))
        .arg(s.error?QStringLiteral("YES"):QStringLiteral("no"))
        .arg(s.userIndex).arg(s.toolIndex)
        .arg(s.jogging?QStringLiteral("yes"):QStringLiteral("no"))
        .arg(s.paused?QStringLiteral("yes"):QStringLiteral("no"))
        .arg(s.queuedMotion?QStringLiteral("yes"):QStringLiteral("no"))
        .arg(s.actualQuaternionValid?
            QStringLiteral("%1, %2, %3, %4")
            .arg(s.actualQuaternion[0],0,'f',5).arg(s.actualQuaternion[1],0,'f',5)
            .arg(s.actualQuaternion[2],0,'f',5).arg(s.actualQuaternion[3],0,'f',5):
            QStringLiteral("缺失/无效"))
        .arg(std::isfinite(touchpanel::teleop::feedbackZyxErrorDeg(s))?
            QString::number(touchpanel::teleop::feedbackZyxErrorDeg(s),'f',3)+QStringLiteral("°"):
            QStringLiteral("未核对"));
}

QString RelayStateClient::statusDescription() const {
    if(QThread::currentThread()==thread())return m_status;
    std::lock_guard<std::mutex> lock(m_publicMutex);
    return m_publicStatus;
}
void RelayStateClient::publishReadOnlySnapshot() {
    const auto value=snapshot();
    std::lock_guard<std::mutex> lock(m_publicMutex);
    m_publicSnapshot=value;
    m_publicStatus=m_status;
    m_publicTime=std::chrono::steady_clock::now();
}
void RelayStateClient::refreshStatus() {
    if(orientationProbePending() &&
       (!touchpanel::teleop::idleForExperimentalStep(snapshot(),0,0) ||
        !m_probeAge.isValid() || m_probeAge.elapsed()>1500))
        finishOrientationProbe(QStringLiteral("诊断超时或空闲反馈失效；不重试、不授权运动"));
    if(m_valid&&m_stateAge.isValid()&&m_stateAge.elapsed()>kFreshMs) {
        m_valid=false;emit stateInvalidated();
    }
    const QString newStatus=hasFreshState()
       ? QStringLiteral("v2 遥测正常（只读，运动锁定）· seq %1").arg(m_sequence)
       : (!m_handshaken ? m_status :
          (!m_ready ? QStringLiteral("v2 在线；中转站未就绪")
                    : QStringLiteral("v2 无有效新鲜机器人状态")));
    // robot_state can arrive hundreds of times per second, but the status
    // label contains a different seq on EVERY frame. Emitting statusChanged
    // for each frame overwhelms the main-window UI and can delay processing
    // the independent 200ms relay_status heartbeat. Keep actual snapshots
    // full-rate, but throttle cosmetic labels to 5 Hz. Safety transitions
    // (connected -> not ready / invalid) remain immediate.
    const bool uiRefreshDue=!m_summaryEmitAge.isValid() ||
        m_summaryEmitAge.elapsed()>=200;
    const QString previousClass=m_status.section(QStringLiteral("· seq"),0,0);
    const QString newClass=newStatus.section(QStringLiteral("· seq"),0,0);
    if(newStatus!=m_status && (previousClass!=newClass || uiRefreshDue)) {
        m_status=newStatus;
        emit statusChanged(m_status);
    }
    if(uiRefreshDue) {
        const QString summary=buildTelemetrySummary();
        if(summary!=m_lastTelemetrySummary) {
            m_lastTelemetrySummary=summary;
            emit telemetryUpdated(summary);
        }
        m_summaryEmitAge.restart();
    }
    publishReadOnlySnapshot();
}
