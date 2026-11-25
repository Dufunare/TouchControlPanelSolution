#include "MainWindow.h"

#include <QSplitter>
#include <QThread>
#include <QMetaObject>
#include <QCheckBox>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QDateTime>
#include <QStandardPaths>
#include <QToolBar>
#include <QLabel>
#include <QTimer>
#include <QScrollArea>
#include <QFrame>
#include <QLayout>
#include <QSizePolicy>
#include <QStatusBar>
#include <QList>

#include "DeviceController.h"
#include "RelayStateClient.h"
#include "RelayVideoClient.h"
#include "widgets/VideoWidget.h"
#include "TeleopCalibrationDialog.h"
#include "RelativeCalibrationDialog.h"
#include <QPushButton>
#include <QToolButton>
#include <QMessageBox>
#include <QCloseEvent>
#include <QVBoxLayout>
#include "widgets/ControlPanelWidget.h"
#include "widgets/GLCoordinateWidget.h"
#include "widgets/StatusPanelWidget.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    //窗口设置
    setWindowTitle(QStringLiteral("TouchControlPanel · Teleoperation"));
    resize(1440, 860);
    setMinimumSize(940,620);

	//组件初始化
    m_relayStateClient = new RelayStateClient(this);
    m_controller = new DeviceController(&m_backend, &m_communicationBackend, this);
    m_controller->setRelayStateClient(m_relayStateClient);
    m_controlPanel = new ControlPanelWidget(this);
    m_glWidget = new GLCoordinateWidget(this);
    m_videoWidget = new VideoWidget(this);
    m_videoThread = new QThread(this);
    m_videoThread->setObjectName(QStringLiteral("RelayVideoThread"));
    m_videoClient = new RelayVideoClient();
    m_videoClient->moveToThread(m_videoThread);
    connect(m_videoThread,&QThread::finished,m_videoClient,&QObject::deleteLater);
    // These two receiver methods ONLY publish into a mutex-protected
    // QImage mailbox. They do not touch widgets from the worker thread.
    connect(m_videoClient,&RelayVideoClient::frameReady,
            m_videoWidget,&VideoWidget::setFrame,Qt::DirectConnection);
    connect(m_videoClient,&RelayVideoClient::frameLost,
            m_videoWidget,&VideoWidget::showNoSignal,Qt::DirectConnection);
    m_videoThread->start();

    // Less frequently used calibration and GL presentation tools remain
    // accessible, but do not occupy the control panel by default.
    auto* advancedToggle=new QToolButton(m_controlPanel);
    advancedToggle->setText(QStringLiteral("标定与 GL 显示 · 高级"));
    advancedToggle->setCheckable(true);
    advancedToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    advancedToggle->setArrowType(Qt::RightArrow);
    auto* advancedPanel=new QWidget(m_controlPanel);
    auto* advancedLayout=new QVBoxLayout(advancedPanel);
    advancedLayout->setContentsMargins(0,0,0,0);

    auto* calibrationCheckButton=new QPushButton(QStringLiteral("检查 Touch 校准状态（只读）"),advancedPanel);
    auto* relativeCalibrationButton=new QPushButton(QStringLiteral("相对位移与方向标定"),advancedPanel);
    auto* calibrationButton=new QPushButton(QStringLiteral("绝对空间点对标定"),advancedPanel);
    auto* originButton=new QPushButton(QStringLiteral("将当前位置设为 GL 原点"),advancedPanel);
    auto* resetOriginButton=new QPushButton(QStringLiteral("恢复 GL 原始坐标"),advancedPanel);
    for(auto* button:{calibrationCheckButton,relativeCalibrationButton,
                      calibrationButton,originButton,resetOriginButton})
        advancedLayout->addWidget(button);
    if(auto* layout=qobject_cast<QVBoxLayout*>(m_controlPanel->layout())) {
        // ControlPanelWidget finishes with a stretch.
        layout->insertWidget(layout->count()-1,advancedToggle);
        layout->insertWidget(layout->count()-1,advancedPanel);
    }
    advancedPanel->setVisible(false);
    connect(advancedToggle,&QToolButton::toggled,advancedPanel,&QWidget::setVisible);
    connect(advancedToggle,&QToolButton::toggled,advancedToggle,
        [advancedToggle](bool expanded){
            advancedToggle->setArrowType(expanded?Qt::DownArrow:Qt::RightArrow);
        });
    connect(calibrationCheckButton,&QPushButton::clicked,
            m_controller,&DeviceController::checkTouchCalibration);
    connect(relativeCalibrationButton,&QPushButton::clicked,this,[this]{
        RelativeCalibrationDialog dialog(m_controller,m_relayStateClient,this);
        connect(&dialog,&RelativeCalibrationDialog::calibrationSaved,m_controller,
            &DeviceController::applyPreviewCalibration);
        dialog.exec();
    });
    connect(calibrationButton,&QPushButton::clicked,this,[this]{
        TeleopCalibrationDialog dialog(m_controller,m_relayStateClient,this);
        connect(&dialog,&TeleopCalibrationDialog::calibrationSaved,m_controller,
            &DeviceController::applyPreviewCalibration);
        dialog.exec();
    });
    connect(originButton,&QPushButton::clicked,this,[this]{
        const bool ok=m_glWidget->captureDisplayOrigin();
        statusBar()->showMessage(ok?
            QStringLiteral("GL 显示原点已更新；不影响机器人或 Touch 原始坐标"):
            QStringLiteral("请先启动 Touch 有效采集"),5000);
    });
    connect(resetOriginButton,&QPushButton::clicked,
            m_glWidget,&GLCoordinateWidget::clearDisplayOrigin);
    m_statusPanel = new StatusPanelWidget(this);

	//布局设置
    // Sidebars have independent scroll viewports. No preview is a child
    // of, or stacked above, either sidebar.
    auto* controlsScroll = new QScrollArea(this);
    controlsScroll->setObjectName(QStringLiteral("controlsScroll"));
    controlsScroll->setFrameShape(QFrame::NoFrame);
    controlsScroll->setWidgetResizable(true);
    controlsScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    controlsScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    controlsScroll->setMinimumWidth(300);
    m_controlPanel->layout()->setSizeConstraint(QLayout::SetMinimumSize);
    controlsScroll->setWidget(m_controlPanel);

    auto* statusScroll = new QScrollArea(this);
    statusScroll->setObjectName(QStringLiteral("statusScroll"));
    statusScroll->setFrameShape(QFrame::NoFrame);
    statusScroll->setWidgetResizable(true);
    statusScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    statusScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    statusScroll->setMinimumWidth(270);
    m_statusPanel->layout()->setSizeConstraint(QLayout::SetMinimumSize);
    statusScroll->setWidget(m_statusPanel);

    auto* videoPanel=new QWidget(this);
    auto* videoLayout=new QVBoxLayout(videoPanel);
    videoLayout->setContentsMargins(4,4,4,4);
    videoLayout->setSpacing(4);
    auto* videoTools=new QHBoxLayout();
    auto* videoStart=new QPushButton(QStringLiteral("开始视频"),videoPanel);
    auto* videoStop=new QPushButton(QStringLiteral("停止"),videoPanel);
    auto* videoSnapshot=new QPushButton(QStringLiteral("截图"),videoPanel);
    auto* videoMirror=new QCheckBox(QStringLiteral("镜像"),videoPanel);
    auto* videoPort=new QSpinBox(videoPanel);
    videoPort->setRange(1,65535);
    videoPort->setValue(8886);
    videoPort->setToolTip(QStringLiteral("UDP 视频端口；IP 使用左侧中转站 IPv4"));
    videoPort->setPrefix(QStringLiteral("UDP "));
    videoTools->addWidget(videoStart);
    videoTools->addWidget(videoStop);
    videoTools->addWidget(videoSnapshot);
    videoTools->addWidget(videoMirror);
    videoTools->addWidget(videoPort);
    videoTools->addStretch(1);
    auto* videoState=new QLabel(QStringLiteral("UVC 视频 · 已停止"),videoPanel);
    videoState->setWordWrap(true);
    videoLayout->addLayout(videoTools);
    videoLayout->addWidget(videoState);
    videoLayout->addWidget(m_videoWidget,1);
    videoStop->setEnabled(false);
    videoSnapshot->setEnabled(false);
    connect(videoStart,&QPushButton::clicked,this,[this,videoPort,videoStart]{
        const QString host=m_controlPanel->relayHost();
        const quint16 port=static_cast<quint16>(videoPort->value());
        videoStart->setEnabled(false);
        QMetaObject::invokeMethod(m_videoClient,[client=m_videoClient,host,port]{
            client->start(host,port);
        },Qt::QueuedConnection);
    });
    connect(videoStop,&QPushButton::clicked,this,[this]{
        QMetaObject::invokeMethod(m_videoClient,&RelayVideoClient::stop,Qt::QueuedConnection);
    });
    connect(videoMirror,&QCheckBox::toggled,m_videoWidget,&VideoWidget::setMirrored);
    connect(m_videoClient,&RelayVideoClient::stateChanged,this,
        [videoStart,videoStop,videoSnapshot,videoPort,videoState](bool active,bool receiving,const QString& detail){
            videoStart->setEnabled(!active);
            videoStop->setEnabled(active);
            videoPort->setEnabled(!active);
            videoSnapshot->setEnabled(receiving);
            videoState->setText(QStringLiteral("UVC 视频 · ")+detail);
        });
    connect(m_videoClient,&RelayVideoClient::message,this,[this](const QString& text){
        m_statusPanel->setBackendMessage(text);
        statusBar()->showMessage(text,6000);
    });
    connect(videoSnapshot,&QPushButton::clicked,this,[this]{
        // Do not open a nested native save dialog during real motion.
        if(m_controller->realMotionActive()) {
            statusBar()->showMessage(QStringLiteral("请先停止实体遥控，再保存视频截图"),5000);
            return;
        }
        const QString folder=QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
        const QString proposed=folder+QStringLiteral("/TouchVideo_")+
            QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))+
            QStringLiteral(".png");
        const QString path=QFileDialog::getSaveFileName(this,QStringLiteral("保存视频截图"),
            proposed,QStringLiteral("PNG 图片 (*.png)"));
        if(path.isEmpty())return;
        const bool ok=m_videoWidget->saveFrame(path);
        statusBar()->showMessage(ok?QStringLiteral("截图已保存：")+path:
            QStringLiteral("截图失败：无有效画面或文件不可写"),6000);
    });

    auto* glPanel=new QWidget(this);
    auto* glLayout=new QVBoxLayout(glPanel);
    glLayout->setContentsMargins(4,4,4,4);
    glLayout->addWidget(m_glWidget,1);
    auto* previewSplitter=new QSplitter(Qt::Vertical,this);
    previewSplitter->setObjectName(QStringLiteral("previewSplitter"));
    previewSplitter->addWidget(videoPanel);
    previewSplitter->addWidget(glPanel);
    previewSplitter->setChildrenCollapsible(false);
    previewSplitter->setStretchFactor(0,2);
    previewSplitter->setStretchFactor(1,1);
    previewSplitter->setSizes({440,260});
    previewSplitter->setMinimumWidth(270);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->setObjectName(QStringLiteral("mainSplitter"));
    splitter->addWidget(controlsScroll);
    splitter->addWidget(previewSplitter);
    splitter->addWidget(statusScroll);
    splitter->setStretchFactor(0,0);
    splitter->setStretchFactor(1,1);
    splitter->setStretchFactor(2,0);
    splitter->setChildrenCollapsible(false);
    splitter->setSizes({325,780,335});

    setCentralWidget(splitter);
    auto* operationBar=addToolBar(QStringLiteral("遥控操作"));
    operationBar->setMovable(false);
    operationBar->setFloatable(false);
    auto* stopButton=new QPushButton(QStringLiteral("停止遥控"),operationBar);
    auto* emergencyButton=new QPushButton(QStringLiteral("软件急停（非物理急停）"),operationBar);
    emergencyButton->setStyleSheet(QStringLiteral("QPushButton { background:#b91c1c; color:white; font-weight:bold; padding:8px; }"));
    operationBar->addWidget(stopButton);
    operationBar->addWidget(emergencyButton);
    connect(stopButton,&QPushButton::clicked,m_controller,&DeviceController::stopTeleop);
    connect(emergencyButton,&QPushButton::clicked,m_controller,&DeviceController::emergencyStopRobot);
    auto* workspaceLabel=new QLabel(this);
    workspaceLabel->setWordWrap(true);
    workspaceLabel->setMaximumWidth(600);
    statusBar()->addPermanentWidget(workspaceLabel,1);
    auto* workspaceTimer=new QTimer(this);
    connect(workspaceTimer,&QTimer::timeout,this,[this,workspaceLabel]{
        const auto text=m_controller->servoPWorkspaceStatus();
        if(workspaceLabel->text()!=text)workspaceLabel->setText(text);
    });
    workspaceLabel->setText(m_controller->servoPWorkspaceStatus());
    workspaceTimer->start(250);
    statusBar()->showMessage("准备就绪：请先初始化设备。");

	//信号与槽连接
    connect(m_controlPanel, &ControlPanelWidget::initializeRequested,
        m_controller, &DeviceController::initializeBackend);
    connect(m_controlPanel, &ControlPanelWidget::startRequested,
        m_controller, &DeviceController::startStreaming);
    connect(m_controlPanel, &ControlPanelWidget::stopRequested,
        m_controller, &DeviceController::stopStreaming);
    connect(m_controlPanel, &ControlPanelWidget::resetRequested,
        m_controller, &DeviceController::resetBackend);
    connect(m_controlPanel, &ControlPanelWidget::robotConnectRequested,
        m_controller, &DeviceController::connectTransit);
    connect(m_controlPanel, &ControlPanelWidget::robotConnectRequested, this,
        [this](const QString& ip, quint16) {
            m_relayStateClient->connectToRelay(ip.trimmed().isEmpty() ?
                QStringLiteral("192.168.100.194") : ip.trimmed(), 8891);
        });
    connect(m_controlPanel,&ControlPanelWidget::relayV2ConnectRequested,this,[this](const QString& ip){
        m_relayStateClient->connectToRelay(ip.trimmed().isEmpty()?QStringLiteral("192.168.100.194"):ip.trimmed(),8891);
    });
    connect(m_controlPanel,&ControlPanelWidget::relayV2DisconnectRequested,m_relayStateClient,&RelayStateClient::disconnectRelay);
    connect(m_controlPanel, &ControlPanelWidget::robotDisconnectRequested,
        m_controller, &DeviceController::disconnectTransit);
    connect(m_controlPanel, &ControlPanelWidget::robotDisconnectRequested,
        m_relayStateClient, &RelayStateClient::disconnectRelay);
    connect(m_controlPanel, &ControlPanelWidget::powerOnRequested,
        m_controller, &DeviceController::powerOnRobot);
    connect(m_controlPanel, &ControlPanelWidget::enableRobotRequested,
        m_controller, &DeviceController::enableRobot);
    connect(m_controlPanel, &ControlPanelWidget::disableRobotRequested,
        m_controller, &DeviceController::disableRobot);
    connect(m_controlPanel, &ControlPanelWidget::clearErrorRequested,
        m_controller, &DeviceController::clearRobotError);
    connect(m_controlPanel, &ControlPanelWidget::emergencyStopRequested,
        m_controller, &DeviceController::emergencyStopRobot);
    connect(m_controlPanel, &ControlPanelWidget::startDragRequested,
        m_controller, &DeviceController::startDragRobot);
    connect(m_controlPanel, &ControlPanelWidget::stopDragRequested,
        m_controller, &DeviceController::stopDragRobot);
    connect(m_controlPanel, &ControlPanelWidget::startTeleopRequested,
        m_controller, &DeviceController::startTeleop);
    connect(m_controlPanel,&ControlPanelWidget::manualPreviewRequested,this,
        [this](const std::array<int,3>& axes,double gain,double deadzone){
            touchpanel::teleop::ManualAxisMap mapping;
            mapping.output=axes;mapping.scale=gain;mapping.deadzoneMm=deadzone;
            m_controller->setManualPreviewMapping(mapping);
            if(touchpanel::teleop::validManualAxisMap(mapping))m_controller->startManualPreview();
        });
    connect(m_controlPanel,&ControlPanelWidget::singleStepRequested,
        m_controller,&DeviceController::prepareSingleStep);
    connect(m_controller,&DeviceController::singleStepConfirmationNeeded,this,
        [this](const QString& message){
            if(QMessageBox::question(this,QStringLiteral("确认机器人单步运动"),
                 message,QMessageBox::Yes|QMessageBox::No,QMessageBox::No)==QMessageBox::Yes)
                m_controller->confirmSingleStep();
        });
    connect(m_controlPanel,&ControlPanelWidget::continuousServoPRequested,this,
        [this](const std::array<int,3>& axes,double gain,double deadzone,double clutchTravelMm){
            touchpanel::teleop::ManualAxisMap mapping;
            mapping.output=axes;mapping.scale=gain;mapping.deadzoneMm=deadzone;
            if(!touchpanel::teleop::validManualAxisMap(mapping)) {
                statusBar()->showMessage(QStringLiteral("ServoP 轴映射或比例无效"),5000);
                return;
            }
            if(m_controller->realMotionActive()) {
                statusBar()->showMessage(QStringLiteral("先停止当前实体控制，再启动 ServoP"),5000);
                return;
            }
            const auto choice=QMessageBox::warning(this,
                QStringLiteral("确认 ServoP 实体连续遥控"),
                QStringLiteral(
                    "将通过 8888→30003 连续发送 ServoP，控制真实机械臂。\n"
                    "按下 Button1 捕获此刻 Touch 与机器人坐标，按下当帧绝不发运动指令。\n"
                    "松开立即停止发送；机械臂可能执行最后一个目标，"
                    "必须等待反馈停稳，重新按下才捕获新的坐标基准。\n"
                    "默认坐标映射 X←Touch X, Y←−Touch Z, Z←Touch Y，末端姿态固定在基准姿态。\n"
                    "本版为实时位置直接响应（不再使用旧版速度追赶器）；"
                    "单次目标跳变和实际TCP超前均限制为20mm，仍可能有突发动作。\n"
                    "先在隔离的安全空间内确认方向；独立物理急停必须随手可用。"),
                QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
            if(choice!=QMessageBox::Yes)return;
            if(!m_controller->setServoPClutchTravelLimit(clutchTravelMm)) {
                statusBar()->showMessage(QStringLiteral("ServoP 单次离合范围无效"),5000);
                return;
            }
            m_controller->setManualPreviewMapping(mapping);
            m_controller->startContinuousServoP();
        });
    connect(m_controlPanel,&ControlPanelWidget::continuousServoRequested,this,
        [this](const std::array<int,3>& axes,double gain,double deadzone){
            touchpanel::teleop::ManualAxisMap mapping;
            mapping.output=axes;mapping.scale=gain;mapping.deadzoneMm=deadzone;
            if(!touchpanel::teleop::validManualAxisMap(mapping) || gain>2.0) {
                statusBar()->showMessage(QStringLiteral("ServoJ 比例仍限制为2倍，且每个 Touch 轴只能使用一次"),5000);
                return;
            }
            if(m_controller->realMotionActive()) {
                statusBar()->showMessage(QStringLiteral("先停止当前实体遥控模式"),5000);
                return;
            }
            const auto choice=QMessageBox::warning(this,QStringLiteral("真实连续遥控"),
                QStringLiteral("即将允许 Touch Button1 控制真实机器人 ServoJ。\n"
                    "开始前请确保机械臂周围清空、安全范围已确认、物理急停可触及。\n"
                    "松开 Button1 将通过 ResetRobot 请求停止。\n"
                    "仅启用平移；末端姿态锁定。是否继续？"),
                QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
            if(choice!=QMessageBox::Yes)return;
            m_controller->setManualPreviewMapping(mapping);
            m_controller->startContinuousServo();
        });
    connect(m_controlPanel, &ControlPanelWidget::stopTeleopRequested,
        m_controller, &DeviceController::stopTeleop);

    connect(m_relayStateClient, &RelayStateClient::telemetryUpdated,
        m_statusPanel, &StatusPanelWidget::setRobotTelemetryText);
    connect(m_relayStateClient, &RelayStateClient::statusChanged, this,
        [this](const QString& status) {m_statusPanel->setRobotStatusText(status);});
    connect(m_relayStateClient, &RelayStateClient::stateInvalidated, m_controller,
        &DeviceController::onRelayStateInvalidated);

    connect(m_controller, &DeviceController::deviceStateUpdated, this,
        [this](const touchpanel::DeviceState& state)
        {
            m_glWidget->setDeviceState(state);
            m_statusPanel->setDeviceState(state);
        });

    connect(m_controller, &DeviceController::backendMessageChanged,
        m_statusPanel, &StatusPanelWidget::setBackendMessage);

    connect(m_controller, &DeviceController::tcpConnectionChanged,
        m_statusPanel, &StatusPanelWidget::setTcpConnected);
    connect(m_controller, &DeviceController::robotStatusChanged,
        m_statusPanel, &StatusPanelWidget::setRobotStatusText);
    connect(m_controller, &DeviceController::tcpTxMessage,
        m_statusPanel, &StatusPanelWidget::appendTcpTxMessage);
    connect(m_controller, &DeviceController::tcpRxMessage,
        m_statusPanel, &StatusPanelWidget::appendTcpRxMessage);

    connect(m_controller, &DeviceController::backendMessageChanged, this,
        [this](const QString& message)
        {
            statusBar()->showMessage(message);
        });
}


MainWindow::~MainWindow()
{
    // Stop subscription and socket/timers on their owning thread before
    // the video mailbox/widget is destroyed. No worker references robot I/O.
    if(m_videoThread && m_videoThread->isRunning()) {
        QMetaObject::invokeMethod(m_videoClient,&RelayVideoClient::stop,
                                  Qt::BlockingQueuedConnection);
        m_videoThread->quit();
        m_videoThread->wait();
    }
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if(m_controller && m_controller->realMotionActive()) {
        m_controller->stopTeleop();
        QMessageBox::warning(this,QStringLiteral("已请求停止遥控"),
            QStringLiteral("已发送软件停止请求，请确认 8891 反馈显示机器人停止。"
                            "如运动未停止，请立即使用物理急停。确认后再关闭窗口。"));
        event->ignore();
        return;
    }
    QMainWindow::closeEvent(event);
}
