#include "RelayStateClient.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpServer>
#include <QThread>
#include <atomic>
#include <functional>
#include <iostream>
#include <stdexcept>

static void require(bool ok,const char* message) {
    if(!ok)throw std::runtime_error(message);
}
static bool waitFor(const std::function<bool()>& predicate,int ms=3000) {
    QElapsedTimer timer;timer.start();
    while(timer.elapsed()<ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents,20);
        if(predicate())return true;
        QThread::msleep(2);
    }
    return predicate();
}
static QByteArray frame(QJsonObject value) {
    value["v"]=2;
    value["session_id"]="local-test";
    if(!value.contains("request_id"))value["request_id"]="";
    const auto json=QJsonDocument(value).toJson(QJsonDocument::Compact);
    const auto wire=touchpanel::protocol::V2FrameCodec::encode(
        std::string_view(json.constData(),static_cast<size_t>(json.size())));
    return QByteArray(wire.data(),static_cast<qsizetype>(wire.size()));
}
static QByteArray robotState(int seq,int quaternionKind=1) {
    const QJsonArray zero{0,0,0,0,0,0};
    QJsonObject object{{"type","robot_state"},{"state_seq",QString::number(seq)},
        {"valid",true},{"robot_mode",5},
        {"tool_vector_actual",QJsonArray{10,-100,300,0,0,0}},
        {"q_actual",zero},{"qd_actual",zero},{"tcp_speed_actual",zero},
        {"q_target",zero},{"error_status",false},{"enable_status",true},
        {"drag_status",false},{"running_status",false},{"jog_status",false},
        {"pause_cmd_flag",false},{"run_queued_cmd",false},{"user",0},{"tool",0}};
    if(quaternionKind==1)object["actual_quaternion"]=QJsonArray{1,0,0,0};
    if(quaternionKind==2)object["actual_quaternion"]=QJsonArray{0,0,0,0};
    return frame(object);
}
struct Worker {
    QThread thread;
    RelayStateClient* client=new RelayStateClient;
    std::atomic<unsigned> invalidations{0};
    Worker() {
        client->moveToThread(&thread);
        QObject::connect(&thread,&QThread::finished,client,&QObject::deleteLater);
        QObject::connect(client,&RelayStateClient::stateInvalidated,client,
            [this]{invalidations.fetch_add(1);});
        thread.start();
    }
    ~Worker() {
        QMetaObject::invokeMethod(client,&RelayStateClient::disconnectRelay,
                                  Qt::BlockingQueuedConnection);
        thread.quit();thread.wait();
    }
};
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        QTcpServer server;
        require(server.listen(QHostAddress::LocalHost,0),"local test server");
        Worker worker;
        const auto port=server.serverPort();
        QMetaObject::invokeMethod(worker.client,[client=worker.client,port]{
            client->connectToRelay(QStringLiteral("127.0.0.1"),port);
        },Qt::QueuedConnection);
        require(waitFor([&]{return server.hasPendingConnections();}),"worker TCP connect");
        auto* socket=server.nextPendingConnection();
        require(socket!=nullptr,"server socket");
        require(waitFor([&]{return socket->bytesAvailable()>0;}),"worker hello");
        socket->readAll();
        const auto hello=frame({{"type","hello_ack"},{"request_id","1"}});
        const auto relay=frame({{"type","relay_status"},{"relay_ready",true},
            {"port_29999_connected",true},{"port_30003_connected",true},
            {"port_30004_connected",true}});
        require(socket->write(hello+relay+robotState(1))>0,"send mock state");
        socket->flush();
        require(waitFor([&]{return worker.client->hasFreshState();}),"cross-thread fresh snapshot");
        const auto live=worker.client->snapshot();
        require(live.stateSeq==1 && live.tcpPose[0]==10 && live.feedbackValid &&
                live.actualQuaternionValid && live.actualQuaternion[0]==1,
                "published state and quaternion");
        // Deliberately block the GUI/main event loop. Worker must still
        // expire stale telemetry and signal invalidation while it is blocked.
        const auto before=worker.invalidations.load();
        QThread::msleep(650);
        require(worker.invalidations.load()>before,"worker watchdog during GUI stall");
        require(!worker.client->hasFreshState(),"published state expires");
        require(socket->write(relay+robotState(2))>0,"refresh mock state");
        socket->flush();
        require(waitFor([&]{return worker.client->hasFreshState() &&
            worker.client->snapshot().stateSeq==2;}),"worker feedback recovery");
        require(socket->write(robotState(3,0))>0,"missing quaternion");
        socket->flush();
        require(waitFor([&]{const auto s=worker.client->snapshot();
            return s.stateSeq==3 && s.feedbackValid && !s.actualQuaternionValid;
        }),"legacy relay must clear optional quaternion without breaking XYZ");
        require(socket->write(robotState(4,2))>0,"invalid quaternion");
        socket->flush();
        require(waitFor([&]{const auto s=worker.client->snapshot();
            return s.stateSeq==4 && s.feedbackValid && !s.actualQuaternionValid;
        }),"malformed quaternion must not be inherited");
        // Exercise the real read-only 8891 diagnostic request against a
        // local fake relay. No physical robot or motion command is involved.
        require(socket->write(relay+robotState(5,1))>0,"fresh diagnostic state");
        socket->flush();
        require(waitFor([&]{const auto s=worker.client->snapshot();
            return s.stateSeq==5 && s.actualQuaternionValid && s.feedbackAgeMs<150;
        }),"diagnostic state restored");
        QString report;
        QObject::connect(worker.client,&RelayStateClient::orientationProbeFinished,
            &app,[&](const QString& result){report=result;});
        const auto probe=[&](int seq){
            require(socket->write(relay+robotState(seq,1))>0,"fresh probe state");
            socket->flush();
            require(waitFor([&]{return worker.client->snapshot().stateSeq==static_cast<unsigned>(seq);}),
                    "wait for probe sample");
            bool requested=false;
            QMetaObject::invokeMethod(worker.client,[&]{
                requested=worker.client->requestOrientationIkProbe({10,-100,300,1,2,3});
            },Qt::BlockingQueuedConnection);
            require(requested,"read-only IK request accepted");
            touchpanel::protocol::V2FrameCodec codec;
            std::vector<std::string> decoded;
            require(waitFor([&]{
                const QByteArray bytes=socket->readAll();
                if(!bytes.isEmpty() && !codec.feed(bytes.constData(),static_cast<size_t>(bytes.size()),decoded))
                    throw std::runtime_error("malformed diagnostic frame");
                return !decoded.empty();
            }),"receive diagnostic request");
            const auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(decoded.front()));
            const auto request=doc.object();
            require(request.value("type").toString()=="command" &&
                request.value("target_port").toInt()==29999 &&
                request.value("command").toString().startsWith("InverseSolution("),
                "only allowlisted dashboard calculation may be emitted");
            return request;
        };
        auto req=probe(6);
        const auto call=req.value("command").toString();
        auto replyObject=QJsonObject{{"type","reply"},{"request_id",req.value("request_id")},
            {"target_port",29999},{"command_name","InverseSolution"},{"controller_code",0},
            {"raw_reply",QStringLiteral("0,{0,0,0,0,0,0},")+call+";"}};
        require(socket->write(frame(replyObject))>0,"diagnostic answer");
        socket->flush();
        require(waitFor([&]{return report.contains("RTT=");}),"matched read-only IK evidence");
        report.clear();
        req=probe(7);
        replyObject["request_id"]=req.value("request_id");
        replyObject["raw_reply"]=QStringLiteral("0,{0,0,0,0,0,0},InverseSolution(11,-100,300,1,2,3,0,0,1,{0,0,0,0,0,0});");
        require(socket->write(frame(replyObject))>0,"mismatched diagnostic answer");
        socket->flush();
        require(waitFor([&]{return !report.isEmpty();}) && !report.contains("RTT="),
                "mismatched echo must not be accepted as IK evidence");
        std::cout<<"Thread-owned relay, quaternion compatibility, IK echo validation and watchdog passed\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<e.what()<<"\n";return 1;
    }
}
