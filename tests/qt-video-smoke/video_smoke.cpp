#include "RelayVideoClient.h"
#include "VideoWidget.h"
#include "GLCoordinateWidget.h"
#include "ControlPanelWidget.h"
#include "StatusPanelWidget.h"
#include <QApplication>
#include <QBuffer>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QNetworkDatagram>
#include <QScrollArea>
#include <QSplitter>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QThread>
#include <QVBoxLayout>
#include <functional>
#include <iostream>
#include <stdexcept>

static void require(bool value,const char* message) {
    if(!value)throw std::runtime_error(message);
}
static bool waitFor(const std::function<bool()>& test,int timeout=3000) {
    QElapsedTimer timer;timer.start();
    while(timer.elapsed()<timeout) {
        QCoreApplication::processEvents(QEventLoop::AllEvents,20);
        if(test())return true;
        QThread::msleep(5);
    }
    return test();
}
struct VideoWorker {
    QThread thread;
    RelayVideoClient* client=new RelayVideoClient;
    VideoWorker(VideoWidget& video) {
        client->moveToThread(&thread);
        QObject::connect(&thread,&QThread::finished,client,&QObject::deleteLater);
        QObject::connect(client,&RelayVideoClient::frameReady,&video,&VideoWidget::setFrame,Qt::DirectConnection);
        QObject::connect(client,&RelayVideoClient::frameLost,&video,&VideoWidget::showNoSignal,Qt::DirectConnection);
        thread.start();
    }
    ~VideoWorker() {
        QMetaObject::invokeMethod(client,&RelayVideoClient::stop,Qt::BlockingQueuedConnection);
        thread.quit();thread.wait();
    }
};
int main(int argc,char** argv) {
    QSurfaceFormat format;format.setDepthBufferSize(24);
    format.setVersion(3,3);format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);
    QApplication app(argc,argv);
    try {
        QSplitter root(Qt::Horizontal);
        auto* controls=new QScrollArea;
        controls->setWidgetResizable(true);controls->setMinimumWidth(300);
        controls->setWidget(new ControlPanelWidget);
        auto* status=new QScrollArea;
        status->setWidgetResizable(true);status->setMinimumWidth(270);
        status->setWidget(new StatusPanelWidget);
        auto* center=new QSplitter(Qt::Vertical);
        auto* video=new VideoWidget;
        auto* gl=new GLCoordinateWidget;
        center->addWidget(video);center->addWidget(gl);
        center->setChildrenCollapsible(false);
        root.addWidget(controls);root.addWidget(center);root.addWidget(status);
        root.setChildrenCollapsible(false);
        root.resize(940,620);root.show();
        require(waitFor([&]{return controls->width()>=300 && gl->width()>=180;}),
                "preview layout minimums");
        require(!controls->geometry().intersects(center->geometry()),"sidebar overlaps center");
        require(!status->geometry().intersects(center->geometry()),"right sidebar overlaps center");
        QUdpSocket relay;
        require(relay.bind(QHostAddress(QHostAddress::LocalHost),quint16(0)),"loopback bind");
        VideoWorker worker(*video);
        const quint16 port=relay.localPort();
        QMetaObject::invokeMethod(worker.client,[client=worker.client,port]{
            client->start(QStringLiteral("127.0.0.1"),port);
        },Qt::QueuedConnection);
        // A local socket simulates the legacy relay; no robot or Touch is used.
        QNetworkDatagram subscribe;
        require(waitFor([&]{
            while(relay.hasPendingDatagrams()) {
                auto d=relay.receiveDatagram();
                if(d.data()=="VIDEO_START"){subscribe=d;return true;}
            }
            return false;
        }),"VIDEO_START subscription");
        QImage source(640,480,QImage::Format_RGB32);source.fill(Qt::green);
        QByteArray jpeg;QBuffer buffer(&jpeg);
        require(buffer.open(QIODevice::WriteOnly)&&source.save(&buffer,"JPEG"),"JPEG plugin");
        require(relay.writeDatagram(jpeg,subscribe.senderAddress(),subscribe.senderPort())==jpeg.size(),
                "send JPEG");
        require(waitFor([&]{return video->hasSignal();}),"decoded worker frame");
        QTemporaryDir dir;require(dir.isValid(),"temporary dir");
        const QString path=dir.filePath(QStringLiteral("frame.png"));
        require(video->saveFrame(path),"save screenshot");
        require(QImage(path).size()==source.size(),"screenshot dimensions");
        root.resize(1440,860);QCoreApplication::processEvents();
        require(!controls->geometry().intersects(center->geometry()),"resize overlaps sidebar");
        require(waitFor([&]{return !video->hasSignal();},4000),"lost-signal watchdog");
        QMetaObject::invokeMethod(worker.client,&RelayVideoClient::stop,Qt::QueuedConnection);
        require(waitFor([&]{
            while(relay.hasPendingDatagrams())
                if(relay.receiveDatagram().data()=="VIDEO_STOP")return true;
            return false;
        }),"VIDEO_STOP unsubscribe");
        std::cout<<"Qt video loopback, snapshot, watchdog and splitter smoke passed\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<e.what()<<"\n";return 1;
    }
}
