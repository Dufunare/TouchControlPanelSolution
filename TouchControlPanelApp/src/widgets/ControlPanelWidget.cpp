#include "ControlPanelWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QToolButton>
#include <QSignalBlocker>
#include "ManualTeleop.h"
#include <QVBoxLayout>

ControlPanelWidget::ControlPanelWidget(QWidget *parent)
    : QWidget(parent)
{
    //根布局
    auto *rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(8);

    //touch控制区域
    auto *touchGroup = new QGroupBox("Touch 控制", this);
    auto *touchLayout = new QVBoxLayout(touchGroup);

    m_initializeButton = new QPushButton("初始化设备", touchGroup);
    m_startButton = new QPushButton("开始采集", touchGroup);
    m_stopButton = new QPushButton("停止采集", touchGroup);
    m_resetButton = new QPushButton("重置连接", touchGroup);

    auto* touchGrid=new QGridLayout();
    touchGrid->addWidget(m_initializeButton,0,0);
    touchGrid->addWidget(m_startButton,0,1);
    touchGrid->addWidget(m_stopButton,1,0);
    touchGrid->addWidget(m_resetButton,1,1);
    touchLayout->addLayout(touchGrid);

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

    m_robotConnectButton = new QPushButton("连接中转站", TCPGroup);
    m_robotDisconnectButton = new QPushButton("断开", TCPGroup);

    TCPLayout->addLayout(endpointLayout);
    auto* connectionRow=new QHBoxLayout();
    connectionRow->addWidget(m_robotConnectButton);
    connectionRow->addWidget(m_robotDisconnectButton);
    TCPLayout->addLayout(connectionRow);

    auto* relayToolsToggle = new QToolButton(TCPGroup);
    relayToolsToggle->setText(QStringLiteral("8891 独立管理"));
    relayToolsToggle->setCheckable(true);
    relayToolsToggle->setArrowType(Qt::RightArrow);
    relayToolsToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    auto* relayTools = new QWidget(TCPGroup);
    auto* relayToolsLayout = new QHBoxLayout(relayTools);
    relayToolsLayout->setContentsMargins(0,0,0,0);
    relayTools->setVisible(false);
    connect(relayToolsToggle,&QToolButton::toggled,relayTools,&QWidget::setVisible);
    connect(relayToolsToggle,&QToolButton::toggled,relayToolsToggle,
        [relayToolsToggle](bool expanded){relayToolsToggle->setArrowType(expanded?Qt::DownArrow:Qt::RightArrow);});
    TCPLayout->addWidget(relayToolsToggle);
    TCPLayout->addWidget(relayTools);

    m_relayV2ConnectButton=new QPushButton(QStringLiteral("单独连接 8891 遥测（只读）"),TCPGroup);
    m_relayV2DisconnectButton=new QPushButton(QStringLiteral("单独断开 8891 遥测"),TCPGroup);
    relayToolsLayout->addWidget(m_relayV2ConnectButton);
    relayToolsLayout->addWidget(m_relayV2DisconnectButton);

    //机械臂控制区
    QGroupBox* robotGroup = new QGroupBox("遥控");
    QVBoxLayout* mainVLayout = new QVBoxLayout(robotGroup);

    m_emergencyStopButton = new QPushButton("软件急停（非物理急停）");
    m_powerOnButton = new QPushButton("上电", robotGroup);
    m_enableRobotButton = new QPushButton("使能", robotGroup);
    m_disableRobotButton = new QPushButton("下使能", robotGroup);
    m_clearErrorButton = new QPushButton("清除错误码", robotGroup);
    m_startDragButton = new QPushButton("启动拖拽", robotGroup);
    m_stopDragButton = new QPushButton("停止拖拽", robotGroup);
    m_startTeleopButton = new QPushButton("六维预览（标定可选）", robotGroup);
    m_stopTeleopButton = new QPushButton(QStringLiteral("停止预览 / 实体控制"), robotGroup);

    m_emergencyStopButton->setStyleSheet("background-color: red; color: white; font-weight: bold; height: 50px;");
    // Stop controls are placed in the fixed main-window toolbar.
    mainVLayout->addSpacing(4);

    QVBoxLayout* teleopLayout = new QVBoxLayout();
    // Preview is in the advanced mapping section; stop stays visible in the toolbar.
    mainVLayout->addLayout(teleopLayout);
    auto* manualGroup=new QGroupBox(QStringLiteral("固定设备坐标轴 · 平移遥控"),robotGroup);
    auto* manualForm=new QFormLayout(manualGroup);
    auto* mappingDetails=new QWidget(manualGroup);
    auto* detailsForm=new QFormLayout(mappingDetails);
    detailsForm->setContentsMargins(0,0,0,0);
    const QStringList choices={QStringLiteral("+Touch X"),QStringLiteral("+Touch Y"),QStringLiteral("+Touch Z"),
                               QStringLiteral("-Touch X"),QStringLiteral("-Touch Y"),QStringLiteral("-Touch Z")};
    for(int i=0;i<3;++i) {
        m_axisCombos[i]=new QComboBox(manualGroup);
        m_axisCombos[i]->addItems(choices);
        m_axisCombos[i]->setCurrentIndex(m_axisSelection[i]);
        m_axisCombos[i]->setToolTip(QStringLiteral("每个 Touch 轴只使用一次；选择冲突轴时自动交换"));
        manualForm->addRow(QStringLiteral("机器人 %1 ←").arg(QStringLiteral("XYZ").at(i)),m_axisCombos[i]);
        connect(m_axisCombos[i],QOverload<int>::of(&QComboBox::currentIndexChanged),
                this,[this,i](int selected){setAxisChoice(i,selected);});
    }
    m_resetAxesButton=new QPushButton(QStringLiteral("恢复早期分支轴向"),manualGroup);
    manualForm->addRow(m_resetAxesButton);
    connect(m_resetAxesButton,&QPushButton::clicked,this,[this]{
        m_axisSelection={{0,5,1}};
        for(int i=0;i<3;++i) {
            const QSignalBlocker block(m_axisCombos[i]);
            m_axisCombos[i]->setCurrentIndex(m_axisSelection[i]);
        }
    });
    m_axisGain=new QDoubleSpinBox(manualGroup);
    m_axisGain->setRange(0.05,touchpanel::teleop::kManualTeleopMaximumScale);m_axisGain->setDecimals(2);
    m_axisGain->setSingleStep(0.05);m_axisGain->setValue(1.0);
    m_axisDeadzone=new QDoubleSpinBox(manualGroup);
    m_axisDeadzone->setRange(0,10);m_axisDeadzone->setSuffix(QStringLiteral(" mm"));
    // Exact signed-axis mapping by default. The separate 0.3 mm
    // duplicate-target filter in the ServoP guard still applies.
    m_axisDeadzone->setValue(0.0);
    m_axisGain->setSuffix(QStringLiteral(" ×"));
    m_axisGain->setToolTip(QStringLiteral("Touch 位移乘以此比例；不是机器人速度设置。启动后锁定。"));
    manualForm->addRow(QStringLiteral("移动比例"),m_axisGain);
    // ServoP now uses the latest Touch target directly, with a fixed
    // bounded forward setpoint and independent hard 8891 safeguards.
    // Do not expose ineffective legacy speed/feedback-horizon knobs.
    detailsForm->addRow(QStringLiteral("每轴输入死区"),m_axisDeadzone);
    m_axisDeadzone->setToolTip(QStringLiteral("设为 0 可使用精确的轴向位移比例；仍保留重复目标过滤。"));
    auto* previewButton=new QPushButton(QStringLiteral("启用平移预览（不发送运动）"),manualGroup);
    detailsForm->addRow(previewButton);
    auto* posePreviewButton=new QPushButton(QStringLiteral("固定轴六维预览（只读）"),manualGroup);
    posePreviewButton->setToolTip(QStringLiteral("同一离合内映射 XYZ 和相对姿态；不下发机械臂旋转"));
    detailsForm->addRow(posePreviewButton);
    connect(posePreviewButton,&QPushButton::clicked,this,[this]{
        emit fixedAxisPosePreviewRequested(m_axisSelection,m_axisGain->value(),m_axisDeadzone->value());
    });
    detailsForm->addRow(m_startTeleopButton);
    connect(previewButton,&QPushButton::clicked,this,[this]{
        emit manualPreviewRequested(m_axisSelection,m_axisGain->value(),m_axisDeadzone->value());
    });
    // One mutually exclusive physical mode selector. ServoP is the
    // original field-tested Cartesian stream; the IK-based ServoJ path
    // stays available as an explicit experimental alternative.
    m_realModeSelector=new QComboBox(manualGroup);
    m_realModeSelector->addItems({
        QStringLiteral("ServoP · 六自由度连续跟随（默认）"),
        QStringLiteral("ServoJ · 逆运动学连续跟随（实验）"),
        QStringLiteral("ServoP · 仅平移（锁定姿态）")});
    m_realModeSelector->setCurrentIndex(0);
    detailsForm->addRow(QStringLiteral("实体控制模式"),m_realModeSelector);
    auto* actions=new QHBoxLayout();
    auto* stepButton=new QPushButton(QStringLiteral("执行单步（确认）"),manualGroup);
    auto* servoButton=new QPushButton(QStringLiteral("启动 ServoP 连续遥控"),manualGroup);
    stepButton->setToolTip(QStringLiteral("先用平移预览生成目标，按住 Touch 按钮时捕获一个小幅目标，再人工确认执行"));
    servoButton->setToolTip(QStringLiteral("先松开 Touch Button1；重新按下时捕获当前机器人实际坐标。需要物理急停。"));
    detailsForm->addRow(stepButton);
    manualForm->addRow(servoButton);
    auto* mappingToggle=new QToolButton(manualGroup);
    mappingToggle->setText(QStringLiteral("软件限制、预览与实验模式"));
    mappingToggle->setCheckable(true);
    mappingToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    mappingToggle->setArrowType(Qt::RightArrow);
    manualForm->addRow(mappingToggle);
    manualForm->addRow(mappingDetails);
    mappingDetails->hide();
    connect(mappingToggle,&QToolButton::toggled,mappingDetails,&QWidget::setVisible);
    connect(mappingToggle,&QToolButton::toggled,mappingToggle,[mappingToggle](bool on){mappingToggle->setArrowType(on?Qt::DownArrow:Qt::RightArrow);});
    delete actions;
    connect(stepButton,&QPushButton::clicked,this,&ControlPanelWidget::singleStepRequested);
    connect(m_realModeSelector,QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,[this,servoButton](int mode){
        // Changing mode while the robot is running cannot silently transfer
        // control to another sender. Stop first; starting needs a new click.
        emit stopTeleopRequested();
        servoButton->setText(mode==0 ?
            QStringLiteral("启动 ServoP 连续遥控"):
            mode==2?QStringLiteral("启动 ServoP 平移遥控"):
            QStringLiteral("启动 ServoJ 连续遥控（实验）"));
    });
    connect(servoButton,&QPushButton::clicked,this,[this]{
        if(m_realModeSelector->currentIndex()==0)
            emit continuousServoPRequested(
                m_axisSelection,m_axisGain->value(),m_axisDeadzone->value());
        else if(m_realModeSelector->currentIndex()==2)
            emit translationServoPRequested(
                m_axisSelection,m_axisGain->value(),m_axisDeadzone->value());
        else
            emit continuousServoRequested(
                m_axisSelection,m_axisGain->value(),m_axisDeadzone->value());
    });
    mainVLayout->addWidget(manualGroup);

    // Robot management functions remain available but hidden by default.
    auto* adminToggle=new QToolButton(robotGroup);
    adminToggle->setText(QStringLiteral("机器人管理 · 高级"));
    adminToggle->setCheckable(true);
    adminToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    adminToggle->setArrowType(Qt::RightArrow);
    auto* adminPanel=new QWidget(robotGroup);
    auto* adminLayout=new QVBoxLayout(adminPanel);
    adminLayout->setContentsMargins(0,0,0,0);
    QGridLayout* stateGridLayout = new QGridLayout();
    auto* powerRow=new QHBoxLayout();
    powerRow->addWidget(m_powerOnButton);
    powerRow->addWidget(m_enableRobotButton);
    mainVLayout->insertLayout(0,powerRow);
    stateGridLayout->addWidget(m_disableRobotButton, 1, 1);
    stateGridLayout->addWidget(m_clearErrorButton, 0, 1);
    adminLayout->addLayout(stateGridLayout);

    QHBoxLayout* modeHLayout = new QHBoxLayout();
    modeHLayout->addWidget(m_startDragButton);
    modeHLayout->addWidget(m_stopDragButton); 
    adminLayout->addLayout(modeHLayout);


    mainVLayout->addWidget(adminToggle);
    mainVLayout->addWidget(adminPanel);
    adminPanel->setVisible(false);
    connect(adminToggle,&QToolButton::toggled,adminPanel,&QWidget::setVisible);
    connect(adminToggle,&QToolButton::toggled,adminToggle,
        [adminToggle](bool expanded){adminToggle->setArrowType(expanded?Qt::DownArrow:Qt::RightArrow);});

    //加入跟布局
    m_emergencyStopButton->hide();
    m_stopTeleopButton->hide();
    rootLayout->addWidget(touchGroup);
    rootLayout->addWidget(TCPGroup);
    rootLayout->addWidget(robotGroup);
    rootLayout->addStretch(1);

    //信号与槽连线
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
    connect(m_relayV2ConnectButton,&QPushButton::clicked,this,[this]{
        emit relayV2ConnectRequested(m_transitIpEdit->text());
    });
    connect(m_relayV2DisconnectButton,&QPushButton::clicked,this,&ControlPanelWidget::relayV2DisconnectRequested);
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

QString ControlPanelWidget::relayHost() const
{
    const QString host=m_transitIpEdit->text().trimmed();
    return host.isEmpty()?QStringLiteral("192.168.100.194"):host;
}

void ControlPanelWidget::setMappingLocked(bool locked)
{
    if(m_mappingLocked==locked)return;
    m_mappingLocked=locked;
    for(auto* combo:m_axisCombos)if(combo)combo->setEnabled(!locked);
    if(m_axisGain)m_axisGain->setEnabled(!locked);
    if(m_axisDeadzone)m_axisDeadzone->setEnabled(!locked);
    if(m_resetAxesButton)m_resetAxesButton->setEnabled(!locked);
    // Keep the mode selector's existing stop-before-switch behavior.
}

void ControlPanelWidget::setAxisChoice(int robotAxis,int sourceCode)
{
    m_axisSelection=touchpanel::teleop::swapManualAxisSelection(
        m_axisSelection,robotAxis,sourceCode);
    for(int i=0;i<3;++i) {
        if(m_axisCombos[i]) {
            const QSignalBlocker blocker(m_axisCombos[i]);
            m_axisCombos[i]->setCurrentIndex(m_axisSelection[i]);
        }
    }
}
