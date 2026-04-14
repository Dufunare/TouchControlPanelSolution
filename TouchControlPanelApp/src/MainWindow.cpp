#include "MainWindow.h"

#include <QSplitter>
#include <QStatusBar>

#include "DeviceController.h"
#include "widgets/GLCoordinateWidget.h"
#include "widgets/StatusPanelWidget.h"
#include "widgets/VideoWidget.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("OpenHaptics + Qt Touch 控制面板（阶段 1 骨架）");
    resize(1280, 800);

    m_controller = new DeviceController(&m_backend, this);
    m_videoWidget = new VideoWidget(this);
    m_glWidget = new GLCoordinateWidget(this);
    m_statusPanel = new StatusPanelWidget(this);

    auto* leftSplitter = new QSplitter(Qt::Vertical, this);
    leftSplitter->addWidget(m_videoWidget);
    leftSplitter->addWidget(m_glWidget);
    leftSplitter->setStretchFactor(0, 2);
    leftSplitter->setStretchFactor(1, 1);
    leftSplitter->setChildrenCollapsible(false);
    leftSplitter->setSizes({ 520, 260 });

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(leftSplitter);
    splitter->addWidget(m_statusPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 1);

    setCentralWidget(splitter);
    statusBar()->showMessage("准备就绪：请先初始化设备。");

    connect(m_statusPanel, &StatusPanelWidget::initializeRequested,
        m_controller, &DeviceController::initializeBackend);
    connect(m_statusPanel, &StatusPanelWidget::startRequested,
        m_controller, &DeviceController::startStreaming);
    connect(m_statusPanel, &StatusPanelWidget::stopRequested,
        m_controller, &DeviceController::stopStreaming);

    connect(m_controller, &DeviceController::deviceStateUpdated, this,
        [this](const touchpanel::DeviceState& state)
        {
            m_glWidget->setDeviceState(state);
            m_statusPanel->setDeviceState(state);
        });

    connect(m_controller, &DeviceController::backendMessageChanged,
        m_statusPanel, &StatusPanelWidget::setBackendMessage);

    connect(m_controller, &DeviceController::backendMessageChanged, this,
        [this](const QString& message)
        {
            statusBar()->showMessage(message);
        });
}
