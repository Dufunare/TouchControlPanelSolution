#include "ControlPanelWidget.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

ControlPanelWidget::ControlPanelWidget(QWidget *parent)
    : QWidget(parent)
{
    auto *rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(12);

    auto *touchGroup = new QGroupBox("Touch 控制", this);
    auto *touchLayout = new QVBoxLayout(touchGroup);

    m_initializeButton = new QPushButton("1. 初始化设备", touchGroup);
    m_startButton = new QPushButton("2. 开始采集", touchGroup);
    m_stopButton = new QPushButton("3. 停止采集", touchGroup);
    m_resetButton = new QPushButton("4. 重置连接", touchGroup);

    touchLayout->addWidget(m_initializeButton);
    touchLayout->addWidget(m_startButton);
    touchLayout->addWidget(m_stopButton);
    touchLayout->addWidget(m_resetButton);

    auto *robotGroup = new QGroupBox("中转站与机械臂控制", this);
    auto *robotLayout = new QVBoxLayout(robotGroup);

    auto* endpointLayout = new QFormLayout();
    m_transitIpEdit = new QLineEdit(robotGroup);
    m_transitIpEdit->setPlaceholderText("留空使用默认 IP");
    m_transitPortSpin = new QSpinBox(robotGroup);
    m_transitPortSpin->setRange(0, 65535);
    m_transitPortSpin->setValue(0);
    m_transitPortSpin->setToolTip("0 表示使用默认端口");
    endpointLayout->addRow("中转站 IP：", m_transitIpEdit);
    endpointLayout->addRow("中转站端口：", m_transitPortSpin);

    m_robotConnectButton = new QPushButton("建立 TCP 连接", robotGroup);
    m_robotDisconnectButton = new QPushButton("断开 TCP 连接", robotGroup);
    m_powerOnButton = new QPushButton("PowerOn()", robotGroup);
    m_enableRobotButton = new QPushButton("EnableRobot()", robotGroup);
    m_disableRobotButton = new QPushButton("DisableRobot()", robotGroup);
    m_clearErrorButton = new QPushButton("ClearError()", robotGroup);
    m_emergencyStopButton = new QPushButton("EmergencyStop()", robotGroup);
    m_startDragButton = new QPushButton("StartDrag()", robotGroup);
    m_stopDragButton = new QPushButton("StopDrag()", robotGroup);
    m_startTeleopButton = new QPushButton("开始控制（按住按钮跟随）", robotGroup);
    m_stopTeleopButton = new QPushButton("停止控制", robotGroup);

    robotLayout->addLayout(endpointLayout);
    robotLayout->addWidget(m_robotConnectButton);
    robotLayout->addWidget(m_robotDisconnectButton);
    robotLayout->addWidget(m_powerOnButton);
    robotLayout->addWidget(m_enableRobotButton);
    robotLayout->addWidget(m_disableRobotButton);
    robotLayout->addWidget(m_clearErrorButton);
    robotLayout->addWidget(m_emergencyStopButton);
    robotLayout->addWidget(m_startDragButton);
    robotLayout->addWidget(m_stopDragButton);
    robotLayout->addWidget(m_startTeleopButton);
    robotLayout->addWidget(m_stopTeleopButton);

    rootLayout->addWidget(touchGroup);
    rootLayout->addWidget(robotGroup);
    rootLayout->addStretch(1);

    connect(m_initializeButton, &QPushButton::clicked, this, &ControlPanelWidget::initializeRequested);
    connect(m_startButton, &QPushButton::clicked, this, &ControlPanelWidget::startRequested);
    connect(m_stopButton, &QPushButton::clicked, this, &ControlPanelWidget::stopRequested);
    connect(m_resetButton, &QPushButton::clicked, this, &ControlPanelWidget::resetRequested);

    connect(m_robotConnectButton, &QPushButton::clicked, this,
        [this]()
        {
            emit robotConnectRequested(m_transitIpEdit->text(), static_cast<quint16>(m_transitPortSpin->value()));
        });
    connect(m_robotDisconnectButton, &QPushButton::clicked, this, &ControlPanelWidget::robotDisconnectRequested);
    connect(m_powerOnButton, &QPushButton::clicked, this, &ControlPanelWidget::powerOnRequested);
    connect(m_enableRobotButton, &QPushButton::clicked, this, &ControlPanelWidget::enableRobotRequested);
    connect(m_disableRobotButton, &QPushButton::clicked, this, &ControlPanelWidget::disableRobotRequested);
    connect(m_clearErrorButton, &QPushButton::clicked, this, &ControlPanelWidget::clearErrorRequested);
    connect(m_emergencyStopButton, &QPushButton::clicked, this, &ControlPanelWidget::emergencyStopRequested);
    connect(m_startDragButton, &QPushButton::clicked, this, &ControlPanelWidget::startDragRequested);
    connect(m_stopDragButton, &QPushButton::clicked, this, &ControlPanelWidget::stopDragRequested);
    connect(m_startTeleopButton, &QPushButton::clicked, this, &ControlPanelWidget::startTeleopRequested);
    connect(m_stopTeleopButton, &QPushButton::clicked, this, &ControlPanelWidget::stopTeleopRequested);
}
