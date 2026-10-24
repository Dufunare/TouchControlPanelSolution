#include "MainWindow.h"

#include <QSplitter>
#include <QScrollArea>
#include <QFrame>
#include <QLayout>
#include <QSizePolicy>
#include <QStatusBar>
#include <QList>

#include "DeviceController.h"
#include "RelayStateClient.h"
#include "TeleopCalibrationDialog.h"
#include "RelativeCalibrationDialog.h"
#include <QPushButton>
#include <QVBoxLayout>
#include "widgets/ControlPanelWidget.h"
#include "widgets/GLCoordinateWidget.h"
#include "widgets/StatusPanelWidget.h"
#include "widgets/VideoWidget.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    //窗口设置
    setWindowTitle("TouchControlPanel(阶段二)");
    resize(1440, 860);

	//组件初始化
    m_relayStateClient = new RelayStateClient(this);
    m_controller = new DeviceController(&m_backend, &m_communicationBackend, this);
    m_controller->setRelayStateClient(m_relayStateClient);
    m_controlPanel = new ControlPanelWidget(this);
    auto* calibrationCheckButton=new QPushButton(QStringLiteral("检查 Touch 校准状态（只读）"),m_controlPanel);
    if(auto* layout=qobject_cast<QVBoxLayout*>(m_controlPanel->layout()))
        layout->insertWidget(3,calibrationCheckButton);
    connect(calibrationCheckButton,&QPushButton::clicked,m_controller,&DeviceController::checkTouchCalibration);
    auto* relativeCalibrationButton=new QPushButton(QStringLiteral("相对位移与方向标定（分离工作空间）"),m_controlPanel);
    if(auto* layout=qobject_cast<QVBoxLayout*>(m_controlPanel->layout()))
        layout->insertWidget(3,relativeCalibrationButton);
    connect(relativeCalibrationButton,&QPushButton::clicked,this,[this]{
        RelativeCalibrationDialog dialog(m_controller,m_relayStateClient,this);
        connect(&dialog,&RelativeCalibrationDialog::calibrationSaved,m_controller,
            &DeviceController::applyPreviewCalibration);
        dialog.exec();
    });
    auto* calibrationButton=new QPushButton(QStringLiteral("绝对空间点对标定（仅共同基准点）"),m_controlPanel);
    if(auto* layout=qobject_cast<QVBoxLayout*>(m_controlPanel->layout()))layout->insertWidget(3,calibrationButton);
    connect(calibrationButton,&QPushButton::clicked,this,[this] {
        TeleopCalibrationDialog dialog(m_controller,m_relayStateClient,this);
        connect(&dialog,&TeleopCalibrationDialog::calibrationSaved,m_controller,
            &DeviceController::applyPreviewCalibration);
        dialog.exec();
    });
    m_videoWidget = new VideoWidget(this);
    m_glWidget = new GLCoordinateWidget(this);
    auto* originButton = new QPushButton(QStringLiteral("将当前位置设为 GL 显示原点"), m_controlPanel);
    auto* resetOriginButton = new QPushButton(QStringLiteral("恢复 GL 原始坐标"), m_controlPanel);
    if (auto* layout = qobject_cast<QVBoxLayout*>(m_controlPanel->layout())) {
        layout->insertWidget(4, originButton);
        layout->insertWidget(5, resetOriginButton);
    }
    connect(originButton, &QPushButton::clicked, this, [this] {
        const bool ok=m_glWidget->captureDisplayOrigin();
        statusBar()->showMessage(ok ? QStringLiteral("GL 显示原点已更新（不影响 Touch 原始坐标或机械臂）") :
            QStringLiteral("请先启动有效的 Touch 采集，再设置显示原点。"),5000);
    });
    connect(resetOriginButton, &QPushButton::clicked,m_glWidget,&GLCoordinateWidget::clearDisplayOrigin);
    m_statusPanel = new StatusPanelWidget(this);

	//布局设置
    auto* leftSplitter = new QSplitter(Qt::Vertical, this);
    leftSplitter->addWidget(m_videoWidget);
    leftSplitter->addWidget(m_glWidget);
    leftSplitter->setStretchFactor(0, 1);
    leftSplitter->setStretchFactor(1, 1);
    leftSplitter->setChildrenCollapsible(false);
    leftSplitter->setSizes({ 380, 400 });

    // The two sidebars grow vertically as controls and logs are added.
    // Independent scroll areas keep the center video/GL splitter visible
    // even when the main window becomes short. Existing widget pointers and
    // signal connections remain unchanged after setWidget() reparents them.
    auto* controlsScroll = new QScrollArea(this);
    controlsScroll->setObjectName(QStringLiteral("controlsScroll"));
    controlsScroll->setFrameShape(QFrame::NoFrame);
    controlsScroll->setWidgetResizable(true);
    controlsScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    controlsScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    controlsScroll->setMinimumWidth(220);
    m_controlPanel->layout()->setSizeConstraint(QLayout::SetMinimumSize);
    controlsScroll->setWidget(m_controlPanel);

    auto* statusScroll = new QScrollArea(this);
    statusScroll->setObjectName(QStringLiteral("statusScroll"));
    statusScroll->setFrameShape(QFrame::NoFrame);
    statusScroll->setWidgetResizable(true);
    statusScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    statusScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    statusScroll->setMinimumWidth(255);
    m_statusPanel->layout()->setSizeConstraint(QLayout::SetMinimumSize);
    statusScroll->setWidget(m_statusPanel);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(controlsScroll);
    splitter->addWidget(leftSplitter);
    splitter->addWidget(statusScroll);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 4);
    splitter->setStretchFactor(2, 2);
    splitter->setChildrenCollapsible(false);
    splitter->setSizes({ 280, 820, 420 });

    setCentralWidget(splitter);
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
    connect(m_controlPanel, &ControlPanelWidget::stopTeleopRequested,
        m_controller, &DeviceController::stopTeleop);

    connect(m_relayStateClient, &RelayStateClient::telemetryUpdated,
        m_statusPanel, &StatusPanelWidget::setRobotTelemetryText);
    connect(m_relayStateClient, &RelayStateClient::statusChanged, this,
        [this](const QString& status) {m_statusPanel->setRobotStatusText(status);});
    connect(m_relayStateClient, &RelayStateClient::stateInvalidated, m_controller,
        &DeviceController::invalidateTeleopBaseline);

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

