#include "MainWindow.h"

#include <QSplitter>
#include <QStatusBar>
#include <QList>

#include "DeviceController.h"
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
    m_controller = new DeviceController(&m_backend, &m_communicationBackend, this);
    m_controlPanel = new ControlPanelWidget(this);
    m_videoWidget = new VideoWidget(this);
    m_glWidget = new GLCoordinateWidget(this);
    m_statusPanel = new StatusPanelWidget(this);

	//布局设置
    auto* leftSplitter = new QSplitter(Qt::Vertical, this);
    leftSplitter->addWidget(m_videoWidget);
    leftSplitter->addWidget(m_glWidget);
    leftSplitter->setStretchFactor(0, 2);
    leftSplitter->setStretchFactor(1, 1);
    leftSplitter->setChildrenCollapsible(false);
    leftSplitter->setSizes({ 520, 260 });

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(m_controlPanel);
    splitter->addWidget(leftSplitter);
    splitter->addWidget(m_statusPanel);
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
    connect(m_controlPanel, &ControlPanelWidget::robotDisconnectRequested,
        m_controller, &DeviceController::disconnectTransit);
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

