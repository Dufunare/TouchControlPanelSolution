#include "ControlPanelWidget.h"

#include <QGroupBox>
#include <QPushButton>
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

    touchLayout->addWidget(m_initializeButton);
    touchLayout->addWidget(m_startButton);
    touchLayout->addWidget(m_stopButton);

    auto *robotGroup = new QGroupBox("机械臂控制", this);
    auto *robotLayout = new QVBoxLayout(robotGroup);

    m_robotConnectButton = new QPushButton("连接机械臂", robotGroup);
    m_robotDisconnectButton = new QPushButton("断开机械臂", robotGroup);

    robotLayout->addWidget(m_robotConnectButton);
    robotLayout->addWidget(m_robotDisconnectButton);

    rootLayout->addWidget(touchGroup);
    rootLayout->addWidget(robotGroup);
    rootLayout->addStretch(1);

    connect(m_initializeButton, &QPushButton::clicked, this, &ControlPanelWidget::initializeRequested);
    connect(m_startButton, &QPushButton::clicked, this, &ControlPanelWidget::startRequested);
    connect(m_stopButton, &QPushButton::clicked, this, &ControlPanelWidget::stopRequested);

    connect(m_robotConnectButton, &QPushButton::clicked, this, &ControlPanelWidget::robotConnectRequested);
    connect(m_robotDisconnectButton, &QPushButton::clicked, this, &ControlPanelWidget::robotDisconnectRequested);
}
