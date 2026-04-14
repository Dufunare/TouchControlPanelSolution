#include "StatusPanelWidget.h"

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMenu>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDateTime>
#include <QTextOption>

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
    rootLayout->setContentsMargins(12, 12,12, 0);
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

    m_messageValue = new QPlainTextEdit(messageGroup);
    m_messageValue->setReadOnly(true);
    m_messageValue->setMaximumBlockCount(500);
    m_messageValue->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    m_messageValue->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
    m_messageValue->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_messageValue->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_messageValue->setContextMenuPolicy(Qt::CustomContextMenu);
    m_messageValue->setPlaceholderText("运行日志会显示在这里...");
    m_messageValue->setStyleSheet(
        "QPlainTextEdit {"
        "  background-color: #0D1117;"
        "  color: #D7E2F0;"
        "  border: 1px solid #2B3240;"
        "  selection-background-color: #264F78;"
        "  font-family: Consolas, 'Courier New', monospace;"
        "  font-size: 10pt;"
        "}");
    m_messageValue->appendPlainText("请先点击“初始化设备”。");
    messageLayout->addWidget(m_messageValue);

    connect(m_messageValue, &QPlainTextEdit::customContextMenuRequested, this,
        [this](const QPoint& pos)
        {
            QMenu* menu = m_messageValue->createStandardContextMenu();
            menu->addSeparator();
            QAction* clearAction = menu->addAction("清空日志");

            const QAction* selected = menu->exec(m_messageValue->mapToGlobal(pos));
            if (selected == clearAction)
            {
                m_messageValue->clear();
            }

            delete menu;
        });

    rootLayout->addWidget(introGroup);
    rootLayout->addWidget(stateGroup);
    rootLayout->addWidget(controlGroup);
    rootLayout->addWidget(messageGroup);
   

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
}

void StatusPanelWidget::setBackendMessage(const QString& text)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    m_messageValue->appendPlainText(QStringLiteral("[%1] %2").arg(ts, text));
}
