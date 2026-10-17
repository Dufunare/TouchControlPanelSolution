#include "ControlPanelWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

ControlPanelWidget::ControlPanelWidget(QWidget *parent)
    : QWidget(parent)
{
    //根布局
    auto *rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(12);

    //touch控制区域
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

    //中转站控制区域
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

    //机械臂控制区
    QGroupBox* robotGroup = new QGroupBox("机械臂控制");
    QVBoxLayout* mainVLayout = new QVBoxLayout(robotGroup);

    m_emergencyStopButton = new QPushButton("急停");
    m_powerOnButton = new QPushButton("上电", robotGroup);
    m_enableRobotButton = new QPushButton("使能", robotGroup);
    m_disableRobotButton = new QPushButton("下使能", robotGroup);
    m_clearErrorButton = new QPushButton("清除错误码", robotGroup);
    m_startDragButton = new QPushButton("启动拖拽", robotGroup);
    m_stopDragButton = new QPushButton("停止拖拽", robotGroup);
    m_startTeleopButton = new QPushButton("开始控制（按住按钮跟随）", robotGroup);
    m_stopTeleopButton = new QPushButton("停止控制", robotGroup);

    m_emergencyStopButton->setStyleSheet("background-color: red; color: white; font-weight: bold; height: 50px;");
    mainVLayout->addWidget(m_emergencyStopButton);
    mainVLayout->addSpacing(10);

    QGridLayout* stateGridLayout = new QGridLayout();
    stateGridLayout->addWidget(m_powerOnButton, 0, 0);
    stateGridLayout->addWidget(m_enableRobotButton, 1, 0);
    stateGridLayout->addWidget(m_disableRobotButton, 1, 1);
    stateGridLayout->addWidget(m_clearErrorButton, 0, 1);
    mainVLayout->addLayout(stateGridLayout);

    QHBoxLayout* modeHLayout = new QHBoxLayout();
    modeHLayout->addWidget(m_startDragButton);
    modeHLayout->addWidget(m_stopDragButton); 
    mainVLayout->addLayout(modeHLayout);

    QVBoxLayout* teleopLayout = new QVBoxLayout();
    teleopLayout->addWidget(m_startTeleopButton);
    teleopLayout->addWidget(m_stopTeleopButton);
    mainVLayout->addLayout(teleopLayout);

    //视频控制区
    auto* videoGroup = new QGroupBox("视频控制", this);
    auto* videoRootLayout = new QVBoxLayout(videoGroup);
    auto* videoFormLayout = new QFormLayout();

    m_videoSourceCombo = new QComboBox(videoGroup);
    m_videoSourceCombo->addItem("中转站 UVC 摄像头 (JPEG/UDP)");

    m_videoSourceEdit = new QLineEdit(videoGroup);
    m_videoSourceEdit->setPlaceholderText("192.168.100.194 (中转站 IPv4)");
    m_videoPortSpin = new QSpinBox(videoGroup);
    m_videoPortSpin->setRange(1, 65535);
    m_videoPortSpin->setValue(8886);
    m_videoPortSpin->setToolTip("中转站视频控制监听端口，默认 8886");

    m_videoFpsSpin = new QSpinBox(videoGroup);
    m_videoFpsSpin->setRange(1, 120);
    m_videoFpsSpin->setValue(30);
    m_videoFpsSpin->setSuffix(" FPS");
    m_videoFpsSpin->setEnabled(false);
    m_videoFpsSpin->setToolTip("当前帧率由中转站摄像头决定，暂不支持远程设置");

    m_videoMirrorCheck = new QCheckBox("镜像显示", videoGroup);

    videoFormLayout->addRow("视频来源：", m_videoSourceCombo);
    videoFormLayout->addRow("中转站 IPv4：", m_videoSourceEdit);
    videoFormLayout->addRow("视频 UDP 端口：", m_videoPortSpin);
    videoFormLayout->addRow("采集帧率（远端）：", m_videoFpsSpin);
    videoFormLayout->addRow(QString(), m_videoMirrorCheck);
    videoRootLayout->addLayout(videoFormLayout);

    auto* videoButtonLayout = new QHBoxLayout();
    m_videoStartButton = new QPushButton("开始采集", videoGroup);
    m_videoStopButton = new QPushButton("停止采集", videoGroup);
    m_videoSnapshotButton = new QPushButton("抓拍", videoGroup);

    m_videoStartButton->setToolTip("占位按钮：后续接入 VideoWidget/后端采集逻辑");
    m_videoStopButton->setToolTip("占位按钮：后续接入 VideoWidget/后端采集逻辑");
    m_videoStartButton->setToolTip("订阅中转站通过 UDP 8886 发送的 UVC 摄像头 JPEG 画面");
    m_videoStopButton->setToolTip("发送 VIDEO_STOP 并停止接收");
    m_videoSnapshotButton->setToolTip("将当前画面保存为 PNG");
    m_videoStatusLabel = new QLabel("已停止", videoGroup);
    m_videoStatusLabel->setWordWrap(true);

    videoButtonLayout->addWidget(m_videoStartButton);
    videoButtonLayout->addWidget(m_videoStopButton);
    videoButtonLayout->addWidget(m_videoSnapshotButton);
    videoRootLayout->addLayout(videoButtonLayout);
    videoRootLayout->addWidget(m_videoStatusLabel);
    
    //加入跟布局
    rootLayout->addWidget(touchGroup);
    rootLayout->addWidget(TCPGroup);
    rootLayout->addWidget(robotGroup);
    rootLayout->addWidget(videoGroup);
    rootLayout->addStretch(1);

    //信号与槽连线
    setVideoState(false, false, QStringLiteral("已停止"));
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
    connect(m_videoStartButton, &QPushButton::clicked, this, [this] {
        const QString ip = m_videoSourceEdit->text().trimmed().isEmpty()
            ? m_transitIpEdit->text().trimmed() : m_videoSourceEdit->text().trimmed();
        emit videoStartRequested(ip.isEmpty() ? QStringLiteral("192.168.100.194") : ip,
                                 static_cast<quint16>(m_videoPortSpin->value()));
    });
    connect(m_videoStopButton, &QPushButton::clicked, this, &ControlPanelWidget::videoStopRequested);
    connect(m_videoSnapshotButton, &QPushButton::clicked, this, &ControlPanelWidget::videoSnapshotRequested);
}

void ControlPanelWidget::setVideoState(bool active, bool receiving, const QString& detail)
{
    m_videoStartButton->setEnabled(!active);
    m_videoStopButton->setEnabled(active);
    m_videoSnapshotButton->setEnabled(receiving);
    m_videoStatusLabel->setText(detail);
}
