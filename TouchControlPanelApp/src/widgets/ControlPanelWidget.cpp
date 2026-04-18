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

    m_initializeButton = new QPushButton("初始化设备", touchGroup);
    m_startButton = new QPushButton("开始采集", touchGroup);
    m_stopButton = new QPushButton("停止采集", touchGroup);
    m_resetButton = new QPushButton("重置连接", touchGroup);

    touchLayout->addWidget(m_initializeButton);
    touchLayout->addWidget(m_startButton);
    touchLayout->addWidget(m_stopButton);
    touchLayout->addWidget(m_resetButton);

    auto *TCPGroup = new QGroupBox("中转站控制", this);
    auto *TCPLayout = new QVBoxLayout(TCPGroup);

    auto* endpointLayout = new QFormLayout();
    m_transitIpEdit = new QLineEdit(TCPGroup);
    m_transitIpEdit->setPlaceholderText("留空使用默认 IP");
    m_transitPortSpin = new QSpinBox(TCPGroup);
    m_transitPortSpin->setRange(0, 65535);
    m_transitPortSpin->setValue(0);
    m_transitPortSpin->setToolTip("0 表示使用默认端口");
    endpointLayout->addRow("中转站 IP：", m_transitIpEdit);
    endpointLayout->addRow("中转站端口：", m_transitPortSpin);

    m_robotConnectButton = new QPushButton("建立 TCP 连接", TCPGroup);
    m_robotDisconnectButton = new QPushButton("断开 TCP 连接", TCPGroup);

    TCPLayout->addLayout(endpointLayout);
    TCPLayout->addWidget(m_robotConnectButton);
    TCPLayout->addWidget(m_robotDisconnectButton);


    
QGroupBox *robotGroup = new QGroupBox("机械臂控制");
QVBoxLayout *mainVLayout = new QVBoxLayout(robotGroup);
m_emergencyStopButton = new QPushButton("急停");
m_emergencyStopButton->setStyleSheet("background-color: red; color: white; font-weight: bold; height: 50px;");
mainVLayout->addWidget(m_emergencyStopButton);
mainVLayout->addSpacing(10); 


    m_powerOnButton = new QPushButton("上电", robotGroup);
    m_enableRobotButton = new QPushButton("使能", robotGroup);
    m_disableRobotButton = new QPushButton("下使能", robotGroup);
    m_clearErrorButton = new QPushButton("清除错误码", robotGroup);

QGridLayout *stateGridLayout = new QGridLayout();
stateGridLayout->addWidget(m_powerOnButton, 0, 0);
stateGridLayout->addWidget(m_enableRobotButton, 1, 0);
stateGridLayout->addWidget(m_disableRobotButton, 1, 1);
stateGridLayout->addWidget(m_clearErrorButton, 0, 1);
mainVLayout->addLayout(stateGridLayout); // 将网格布局塞入主垂直布局

// 4. 模式控制 (水平布局)
    m_startDragButton = new QPushButton("启动拖拽", robotGroup);
    m_stopDragButton = new QPushButton("停止拖拽", robotGroup);
QHBoxLayout *modeHLayout = new QHBoxLayout();
modeHLayout->addWidget(m_startDragButton);
modeHLayout->addWidget(m_stopDragButton); // 把按钮挤到左边，或者填满其他模式按钮
mainVLayout->addLayout(modeHLayout);


m_startTeleopButton = new QPushButton("开始控制（按住按钮跟随）", robotGroup);
m_stopTeleopButton = new QPushButton("停止控制", robotGroup);
QVBoxLayout* teleopLayout = new QVBoxLayout();
teleopLayout->addWidget(m_startTeleopButton);
teleopLayout->addWidget(m_stopTeleopButton);
mainVLayout->addLayout(teleopLayout);







    rootLayout->addWidget(touchGroup);
    rootLayout->addWidget(TCPGroup);
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
