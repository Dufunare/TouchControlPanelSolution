#include "StatusPanelWidget.h"

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace
{
    QLabel* createValueLabel(const QString& text)
    {
        auto* label = new QLabel(text);
        label->setMinimumWidth(120);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        return label;
    }
} // namespace

StatusPanelWidget::StatusPanelWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(12);

    auto* introGroup = new QGroupBox("阶段 1：Touch 坐标采集 + 3D 点显示", this);
    auto* introLayout = new QVBoxLayout(introGroup);
    auto* introText = new QLabel(
        "左侧区域负责 3D 坐标系可视化；右侧区域负责状态展示和控制。"
        "\n后续你可以在这里继续加入按钮区、视频区、参数区、日志区等模块。", introGroup);
    introText->setWordWrap(true);
    introLayout->addWidget(introText);

    auto* stateGroup = new QGroupBox("后端状态", this);
    auto* stateLayout = new QFormLayout(stateGroup);

    m_initializedValue = createValueLabel("否");
    m_schedulerValue = createValueLabel("未启动");
    m_validValue = createValueLabel("无效");
    m_counterValue = createValueLabel("0");

    stateLayout->addRow("是否初始化：", m_initializedValue);
    stateLayout->addRow("采样循环：", m_schedulerValue);
    stateLayout->addRow("数据有效性：", m_validValue);
    stateLayout->addRow("采样计数：", m_counterValue);

    auto* coordGroup = new QGroupBox("实时坐标（mm）", this);
    auto* coordLayout = new QFormLayout(coordGroup);

    m_xValue = createValueLabel("0.000");
    m_yValue = createValueLabel("0.000");
    m_zValue = createValueLabel("0.000");

    coordLayout->addRow("X：", m_xValue);
    coordLayout->addRow("Y：", m_yValue);
    coordLayout->addRow("Z：", m_zValue);

    auto* controlGroup = new QGroupBox("控制按钮", this);
    auto* controlLayout = new QVBoxLayout(controlGroup);

    m_initializeButton = new QPushButton("1. 初始化设备", controlGroup);
    m_startButton = new QPushButton("2. 启动实时采集", controlGroup);
    m_stopButton = new QPushButton("3. 停止采集", controlGroup);

    controlLayout->addWidget(m_initializeButton);
    controlLayout->addWidget(m_startButton);
    controlLayout->addWidget(m_stopButton);

    auto* messageGroup = new QGroupBox("消息输出", this);
    auto* messageLayout = new QVBoxLayout(messageGroup);

    m_messageValue = new QLabel("请先点击“初始化设备”。", messageGroup);
    m_messageValue->setWordWrap(true);
    m_messageValue->setTextInteractionFlags(Qt::TextSelectableByMouse);
    messageLayout->addWidget(m_messageValue);

    rootLayout->addWidget(introGroup);
    rootLayout->addWidget(stateGroup);
    rootLayout->addWidget(coordGroup);
    rootLayout->addWidget(controlGroup);
    rootLayout->addWidget(messageGroup);
    rootLayout->addStretch(1);

    connect(m_initializeButton, &QPushButton::clicked, this, &StatusPanelWidget::initializeRequested);
    connect(m_startButton, &QPushButton::clicked, this, &StatusPanelWidget::startRequested);
    connect(m_stopButton, &QPushButton::clicked, this, &StatusPanelWidget::stopRequested);
}

void StatusPanelWidget::setDeviceState(const touchpanel::DeviceState& state)
{
    m_initializedValue->setText(state.initialized ? "是" : "否");
    m_schedulerValue->setText(state.schedulerRunning ? "运行中" : "已停止");
    m_validValue->setText(state.valid ? "有效" : "无效");
    m_counterValue->setText(QString::number(state.sampleCounter));

    m_xValue->setText(QString::number(state.positionMm[0], 'f', 3));
    m_yValue->setText(QString::number(state.positionMm[1], 'f', 3));
    m_zValue->setText(QString::number(state.positionMm[2], 'f', 3));
}

void StatusPanelWidget::setBackendMessage(const QString& text)
{
    m_messageValue->setText(text);
}
