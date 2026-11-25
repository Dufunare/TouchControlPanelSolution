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
        {"tool_vector_actual",QJsonArray{10,-100,300,175,2,17}},
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
        std::cout<<"Thread-owned relay, quaternion compatibility and independent watchdog passed\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<e.what()<<"\n";return 1;
    }
}
