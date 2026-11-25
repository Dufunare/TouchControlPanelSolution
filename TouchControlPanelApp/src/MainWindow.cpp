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
    m_controlThread=new QThread(this);
    m_controlThread->setObjectName(QStringLiteral("TeleopControlThread"));
    m_relayStateClient = new RelayStateClient();
    m_controller = new DeviceController(&m_backend, &m_communicationBackend);
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
    auto* orientationEvidenceButton=new QPushButton(QStringLiteral("采集当前姿态证据（只读）"),advancedPanel);
    orientationEvidenceButton->setToolTip(QStringLiteral("读取同一反馈帧的 RPY 与实际四元数，比较六种欧拉角顺序。需要 User=0/Tool=0；不发送运动。"));
    auto* orientationResetButton=new QPushButton(QStringLiteral("清除姿态证据"),advancedPanel);
    auto* orientationProbeButton=new QPushButton(QStringLiteral("验证六维预览目标逆解（只读）"),advancedPanel);
    orientationProbeButton->setToolTip(QStringLiteral("对当前固定轴六维预览目标调用控制器 InverseSolution；仅计算，不执行。"));
    auto* relativeCalibrationButton=new QPushButton(QStringLiteral("相对位移与方向标定"),advancedPanel);
    auto* calibrationButton=new QPushButton(QStringLiteral("绝对空间点对标定"),advancedPanel);
    auto* originButton=new QPushButton(QStringLiteral("将当前位置设为 GL 原点"),advancedPanel);
    auto* resetOriginButton=new QPushButton(QStringLiteral("恢复 GL 原始坐标"),advancedPanel);
    for(auto* button:{calibrationCheckButton,orientationEvidenceButton,
                      orientationResetButton,orientationProbeButton,relativeCalibrationButton,
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
    connect(orientationEvidenceButton,&QPushButton::clicked,
            m_controller,&DeviceController::captureOrientationEvidence);
    connect(orientationResetButton,&QPushButton::clicked,
            m_controller,&DeviceController::resetOrientationEvidence);
    connect(orientationProbeButton,&QPushButton::clicked,
            m_controller,&DeviceController::probeRotationCandidate);
    connect(relativeCalibrationButton,&QPushButton::clicked,this,[this]{
        if(m_controller->realMotionActive())return;
        RelativeCalibrationDialog dialog(m_controller,m_relayStateClient,this);
        connect(&dialog,&RelativeCalibrationDialog::calibrationSaved,this,
            [this](const touchpanel::teleop::Calibration& calibration,int user,int tool){
                QMetaObject::invokeMethod(m_controller,[controller=m_controller,calibration,user,tool]{
                    controller->applyPreviewCalibration(calibration,user,tool);
                },Qt::QueuedConnection);
            });
        dialog.exec();
    });
    connect(calibrationButton,&QPushButton::clicked,this,[this]{
        if(m_controller->realMotionActive())return;
        TeleopCalibrationDialog dialog(m_controller,m_relayStateClient,this);
        connect(&dialog,&TeleopCalibrationDialog::calibrationSaved,this,
            [this](const touchpanel::teleop::Calibration& calibration,int user,int tool){
                QMetaObject::invokeMethod(m_controller,[controller=m_controller,calibration,user,tool]{
                    controller->applyPreviewCalibration(calibration,user,tool);
                },Qt::QueuedConnection);
            });
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
    auto* show3D=new QCheckBox(QStringLiteral("3D 预览"),videoPanel);
    show3D->setChecked(true);
    show3D->setToolTip(QStringLiteral("仅控制显示；隐藏 3D 可减少绘制负载，不影响实体控制"));
    auto* videoPort=new QSpinBox(videoPanel);
    videoPort->setRange(1,65535);
    videoPort->setValue(8886);
    videoPort->setToolTip(QStringLiteral("UDP 视频端口；IP 使用左侧中转站 IPv4"));
    videoPort->setPrefix(QStringLiteral("UDP "));
    videoTools->addWidget(videoStart);
    videoTools->addWidget(videoStop);
    videoTools->addWidget(videoSnapshot);
    videoTools->addStretch(1);
    auto* videoOptions=new QHBoxLayout();
    videoOptions->addWidget(videoMirror);
    videoOptions->addWidget(show3D);
    videoOptions->addWidget(videoPort);
    videoOptions->addStretch(1);
    auto* videoState=new QLabel(QStringLiteral("UVC 视频 · 已停止"),videoPanel);
    videoState->setWordWrap(true);
    videoLayout->addLayout(videoTools);
    videoLayout->addLayout(videoOptions);
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
        [this,videoStart,videoStop,videoSnapshot,videoPort,videoState](bool active,bool receiving,const QString& detail){
            m_glWidget->setVideoActive(active);
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
    connect(show3D,&QCheckBox::toggled,glPanel,&QWidget::setVisible);
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
    connect(m_controller,&DeviceController::servoPStatusChanged,workspaceLabel,&QLabel::setText);
    connect(workspaceTimer,&QTimer::timeout,this,[this]{
        m_controlPanel->setMappingLocked(m_controller->realMotionActive());
    });
    workspaceLabel->setText(QStringLiteral("工作空间：等待 8891 有效反馈"));
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
            const QString host=ip.trimmed().isEmpty()?QStringLiteral("192.168.100.194"):ip.trimmed();
            QMetaObject::invokeMethod(m_relayStateClient,[client=m_relayStateClient,host]{
                client->connectToRelay(host,8891);
            },Qt::QueuedConnection);
        });
    connect(m_controlPanel,&ControlPanelWidget::relayV2ConnectRequested,this,[this](const QString& ip){
        const QString host=ip.trimmed().isEmpty()?QStringLiteral("192.168.100.194"):ip.trimmed();
        QMetaObject::invokeMethod(m_relayStateClient,[client=m_relayStateClient,host]{
            client->connectToRelay(host,8891);
        },Qt::QueuedConnection);
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
            QMetaObject::invokeMethod(m_controller,[controller=m_controller,mapping]{
                if(controller->realMotionActive())return;
                controller->setManualPreviewMapping(mapping);
                if(touchpanel::teleop::validManualAxisMap(mapping))controller->startManualPreview();
            },Qt::QueuedConnection);
        });
    connect(m_controlPanel,&ControlPanelWidget::fixedAxisPosePreviewRequested,this,
        [this](const std::array<int,3>& axes,double gain,double deadzone){
            touchpanel::teleop::ManualAxisMap mapping;
            mapping.output=axes;mapping.scale=gain;mapping.deadzoneMm=deadzone;
            QMetaObject::invokeMethod(m_controller,[controller=m_controller,mapping]{
                if(controller->realMotionActive() || !touchpanel::teleop::validManualAxisMap(mapping))return;
                controller->setManualPreviewMapping(mapping);
                controller->startFixedAxisPosePreview();
            },Qt::QueuedConnection);
        });
    connect(m_controlPanel,&ControlPanelWidget::singleStepRequested,
        m_controller,&DeviceController::prepareSingleStep);
    connect(m_controller,&DeviceController::singleStepConfirmationNeeded,this,
        [this](const QString& message){
            if(QMessageBox::question(this,QStringLiteral("确认机器人单步运动"),
                 message,QMessageBox::Yes|QMessageBox::No,QMessageBox::No)==QMessageBox::Yes)
                QMetaObject::invokeMethod(m_controller,&DeviceController::confirmSingleStep,Qt::QueuedConnection);
        });
    connect(m_controlPanel,&ControlPanelWidget::continuousServoPRequested,this,
        [this](const std::array<int,3>& axes,double gain,double deadzone){
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
            QString mappingDescription;
            for(int axis=0;axis<3;++axis) {
                const int code=axes[axis];
                if(axis)mappingDescription+=QStringLiteral("；");
                mappingDescription+=QStringLiteral("机器人 %1 ← %2Touch %3")
                    .arg(QStringLiteral("XYZ").mid(axis,1))
                    .arg(code>=3?QStringLiteral("−"):QStringLiteral("+"))
                    .arg(QStringLiteral("XYZ").mid(code%3,1));
            }
            const auto choice=QMessageBox::warning(this,
                QStringLiteral("确认固定轴 ServoP 遥控"),
                QStringLiteral(
                    "%1\n移动比例：%2×；每轴输入死区：%3 mm\n"
                    "使用当前机器人 User/Tool 坐标约定及固定末端姿态，不需要设备间点对标定。\n"
                    "按住前先确认离合就绪；首次按下只记录两端实际位置，不发送运动。\n"
                    "master 式直接位置目标；保护层丢弃的位移不补偿，实际总位移可能小于比例。\n"
                    "松开停发，发过运动指令时请求 ResetRobot 软件停止；"
                    "应答和实测停稳后才能重新按下。\n"
                    "不限制单次离合总行程；原软件工作空间、20mm 指令限制和反馈异常停止仍有效，"
                    "均不是物理碰撞保护。必须确认方向和独立物理急停。")
                    .arg(mappingDescription).arg(gain,0,'f',2).arg(deadzone,0,'f',2),
                QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
            if(choice!=QMessageBox::Yes)return;
            QMetaObject::invokeMethod(m_controller,[controller=m_controller,mapping]{
                if(controller->realMotionActive())return;
                controller->setManualPreviewMapping(mapping);
                controller->startContinuousServoP();
            },Qt::QueuedConnection);
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
            QMetaObject::invokeMethod(m_controller,[controller=m_controller,mapping]{
                if(controller->realMotionActive())return;
                controller->setManualPreviewMapping(mapping);
                controller->startContinuousServo();
            },Qt::QueuedConnection);
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
    // One owner for live control state, 8891 socket, timer and stop ACKs.
    // Widget updates and other GUI work never run on this thread.
    m_relayStateClient->moveToThread(m_controlThread);
    m_controller->moveToThread(m_controlThread);
    connect(m_controlThread,&QThread::finished,m_controller,&QObject::deleteLater);
    connect(m_controlThread,&QThread::finished,m_relayStateClient,&QObject::deleteLater);
    m_controlThread->start();
}


MainWindow::~MainWindow()
{
    // Tear down backend callbacks and sockets before deleting their owner.
    // Normal closeEvent requests a motion stop before permitting exit.
    if(m_controlThread && m_controlThread->isRunning()) {
        QMetaObject::invokeMethod(m_controller,[this]{
            m_controller->stopTeleop();
            m_controller->stopStreaming();
            m_controller->disconnectTransit();
            m_relayStateClient->disconnectRelay();
            m_controller->resetBackend();
            m_communicationBackend.setMessageCallback({});
            m_communicationBackend.setConnectionCallback({});
            m_communicationBackend.setRobotStatusCallback({});
            m_communicationBackend.setTcpTxCallback({});
            m_communicationBackend.setTcpRxCallback({});
            m_communicationBackend.setOperationCallback({});
            m_communicationBackend.setJointSolutionCallback({});
        },Qt::BlockingQueuedConnection);
        m_controlThread->quit();
        m_controlThread->wait();
    }
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
        QMetaObject::invokeMethod(m_controller,&DeviceController::stopTeleop,Qt::QueuedConnection);
        QMessageBox::warning(this,QStringLiteral("已请求停止遥控"),
            QStringLiteral("已发送软件停止请求，请确认 8891 反馈显示机器人停止。"
                            "如运动未停止，请立即使用物理急停。确认后再关闭窗口。"));
        event->ignore();
        return;
    }
    QMainWindow::closeEvent(event);
}
