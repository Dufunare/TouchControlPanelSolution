#include "widgets/VideoWidget.h"
#include <QApplication>
#include <QThread>
#include <QFontDatabase>
#include <QTemporaryDir>
#include <iostream>
#include <stdexcept>
#include <limits>
static void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
int main(int argc,char** argv){QApplication app(argc,argv);try{
    // The offscreen Windows platform may not enumerate system fonts.
    const int fontId=QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
    if(fontId>=0)app.setFont(QFont(QFontDatabase::applicationFontFamilies(fontId).first(),10));
    CameraWarningState state;
    QJsonObject observation{{"valid",true},{"gap_mm",40.0},{"age_ms",10.0}};
    state.receive(observation,1000,1000);
    require(!state.warning(1000),"default warning too early");
    state.setThreshold(50,1000);require(state.warning(1000),"raising threshold did not warn immediately");
    state.setThreshold(20,1000);require(!state.warning(1000),"lowering threshold did not clear old hold");
    state.setThreshold(40,1000);require(state.warning(1000),"inclusive threshold boundary");
    require(!state.fresh(1251)&&state.warning(1251),"expired reading should retain only historical warning");
    require(state.text(1251).contains(QStringLiteral("未知")),"stale distance shown as current");
    require(!state.warning(3001),"warning hold did not expire");
    state.setThreshold(500,3001);require(!state.warning(3001),"threshold update revived stale reading");
    state.receive(observation,3000,3300);require(!state.fresh(3300),"GUI queue delay ignored");
    observation["age_ms"]=240.0;state.receive(observation,4000,4020);
    require(!state.fresh(4020),"relay age and queue delay not combined");
    observation["age_ms"]=0;observation["gap_mm"]="0";state.receive(observation,5000,5000);
    require(!state.fresh(5000),"invalid distance interpreted as zero");
    state.setThreshold(std::numeric_limits<double>::quiet_NaN(),5000);
    require(state.threshold()==30,"invalid saved threshold fallback");
    state.receive({},6000,6000);require(!state.fresh(6000),"missing extension treated as valid");
    VideoWidget widget;widget.resize(640,360);QImage frame(640,360,QImage::Format_RGB32);
    frame.fill(QColor(60,75,90));widget.setFrame(frame);widget.updateFrame();
    observation={{"valid",true},{"gap_mm",40.0},{"age_ms",0.0}};
    widget.setCameraObservation(observation,CameraWarningState::nowMs());
    widget.setWarningThreshold(50);widget.show();app.processEvents();
    require(widget.cameraWarningText().contains(QStringLiteral("距离过近")),"widget did not re-evaluate threshold");
    require(widget.grab().toImage().pixelColor(2,2).red()>100,"warning banner not painted");
    QTemporaryDir screenshots;require(screenshots.isValid(),"screenshot directory");
    const auto screenshotPath=screenshots.filePath("warning.png");
    require(widget.saveFrame(screenshotPath),"warning screenshot failed");
    const QImage screenshot(screenshotPath);
    require(screenshot.size()==frame.size() && screenshot.pixelColor(2,2).red()>100,
            "native-sized screenshot omitted local warning");
    if(argc==2)require(widget.grab().save(QString::fromLocal8Bit(argv[1])),"preview save failed");
    widget.setWarningThreshold(20);
    require(!widget.cameraWarningText().contains(QStringLiteral("距离过近")),"widget kept previous warning");
    QThread::msleep(300);widget.updateFrame();
    require(widget.cameraWarningText().contains(QStringLiteral("未知")),"frozen video retained current distance");
    std::cout<<"Manual threshold, immediate re-evaluation, warning hold, stale/malformed observations and video overlay passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
