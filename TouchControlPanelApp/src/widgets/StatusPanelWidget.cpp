#include "StatusPanelWidget.h"

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMenu>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QDateTime>
#include <QTextOption>
#include <QFont>
#include <QToolButton>

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

    auto* stateGroup = new QGroupBox("Touch 状态", this);
    auto* stateLayout = new QFormLayout(stateGroup);

    m_initializedValue = createValueLabel("否");
    m_schedulerValue = createValueLabel("未启动");
    m_validValue = createValueLabel("无效");
    m_counterValue = createValueLabel("0");
    m_touchCalibrationValue = createValueLabel("待确认");

    stateLayout->addRow("是否初始化：", m_initializedValue);
    stateLayout->addRow("采样循环：", m_schedulerValue);
    stateLayout->addRow("数据有效性：", m_validValue);
    stateLayout->addRow("采样计数：", m_counterValue);
    stateLayout->addRow("设备自身校准：", m_touchCalibrationValue);

    auto* messageGroup = new QGroupBox("消息输出", this);
    auto* messageLayout = new QVBoxLayout(messageGroup);

    auto* commStateGroup = new QGroupBox("通信状态", this);
    auto* commStateLayout = new QFormLayout(commStateGroup);
    m_tcpStateValue = createValueLabel("未连接");
    m_robotStateValue = createValueLabel("未知");
    commStateLayout->addRow("TCP 状态：", m_tcpStateValue);
    commStateLayout->addRow("机械臂状态：", m_robotStateValue);

    auto* telemetryGroup = new QGroupBox("机器人实时反馈（8891，只读）", this);
    auto* telemetryLayout = new QVBoxLayout(telemetryGroup);
    m_robotTelemetryValue = createValueLabel("等待 8891 遥测…");
    m_robotTelemetryValue->setMinimumWidth(0);
    m_robotTelemetryValue->setWordWrap(true);
    m_robotTelemetryValue->setTextInteractionFlags(Qt::TextSelectableByMouse);
    QFont telemetryFont(QStringLiteral("Consolas")); telemetryFont.setPointSize(8);
    m_robotTelemetryValue->setFont(telemetryFont);
    telemetryLayout->addWidget(m_robotTelemetryValue);

    m_messageValue = new QPlainTextEdit(messageGroup);
    m_messageValue->setReadOnly(true);
    m_messageValue->setMinimumHeight(110);
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

    auto* txGroup = new QGroupBox("TCP 发送报文", this);
    auto* txLayout = new QVBoxLayout(txGroup);
    m_txValue = new QPlainTextEdit(txGroup);
    m_txValue->setReadOnly(true);
    m_txValue->setMinimumHeight(135);
    m_txValue->setMaximumBlockCount(500);
    m_txValue->setPlaceholderText("发送报文将显示在这里...");
    txLayout->addWidget(m_txValue);

    auto* rxGroup = new QGroupBox("TCP 接收报文", this);
    auto* rxLayout = new QVBoxLayout(rxGroup);
    m_rxValue = new QPlainTextEdit(rxGroup);
    m_rxValue->setReadOnly(true);
    m_rxValue->setMinimumHeight(135);
    m_rxValue->setMaximumBlockCount(500);
    m_rxValue->setPlaceholderText("接收报文将显示在这里...");
    rxLayout->addWidget(m_rxValue);

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

    rootLayout->addWidget(stateGroup);
    rootLayout->addWidget(commStateGroup);
    rootLayout->addWidget(telemetryGroup);
    rootLayout->addWidget(messageGroup);

    auto* rawToggle=new QToolButton(this);
    rawToggle->setText(QStringLiteral("TCP 原始收发报文 · 高级"));
    rawToggle->setCheckable(true);
    rawToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    rawToggle->setArrowType(Qt::RightArrow);
    auto* rawPanel=new QWidget(this);
    auto* rawLayout=new QVBoxLayout(rawPanel);
    rawLayout->setContentsMargins(0,0,0,0);
    rawLayout->addWidget(txGroup);
    rawLayout->addWidget(rxGroup);
    rootLayout->addWidget(rawToggle);
    rootLayout->addWidget(rawPanel);
    rawPanel->setVisible(false);
    connect(rawToggle,&QToolButton::toggled,rawPanel,&QWidget::setVisible);
    connect(rawToggle,&QToolButton::toggled,rawToggle,
        [rawToggle](bool expanded){
            rawToggle->setArrowType(expanded?Qt::DownArrow:Qt::RightArrow);
        });
}

void StatusPanelWidget::setDeviceState(const touchpanel::DeviceState& state)
{
    m_initializedValue->setText(state.initialized ? "是" : "否");
    m_schedulerValue->setText(state.schedulerRunning ? "运行中" : "已停止");
    m_validValue->setText(state.valid ? "有效" : "无效");
    m_counterValue->setText(QString::number(state.sampleCounter));
    switch(state.calibrationStatus) {
    case touchpanel::TouchCalibrationStatus::Ok:
        m_touchCalibrationValue->setText(QStringLiteral("HDAPI 已确认校准")); break;
    case touchpanel::TouchCalibrationStatus::NeedsUpdate:
        m_touchCalibrationValue->setText(QStringLiteral("需更新校准（未执行）")); break;
    case touchpanel::TouchCalibrationStatus::NeedsManualInput:
        m_touchCalibrationValue->setText(QStringLiteral("需要笔筒/手动操作")); break;
    case touchpanel::TouchCalibrationStatus::CheckError:
        m_touchCalibrationValue->setText(QStringLiteral("校准查询失败")); break;
    default: m_touchCalibrationValue->setText(QStringLiteral("尚未检查（自动更新关闭）")); break;
    }
}

void StatusPanelWidget::setBackendMessage(const QString& text)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    m_messageValue->appendPlainText(QStringLiteral("[%1] %2").arg(ts, text));
}

void StatusPanelWidget::setTcpConnected(bool connected)
{
    m_tcpStateValue->setText(connected ? QStringLiteral("已连接") : QStringLiteral("未连接"));
}

void StatusPanelWidget::setRobotStatusText(const QString& text)
{
    m_robotStateValue->setText(text);
}

void StatusPanelWidget::setRobotTelemetryText(const QString& text)
{
    m_robotTelemetryValue->setText(text);
}

void StatusPanelWidget::appendTcpTxMessage(const QString& text)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    m_txValue->appendPlainText(QStringLiteral("[%1] %2").arg(ts, text));
}

void StatusPanelWidget::appendTcpRxMessage(const QString& text)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    m_rxValue->appendPlainText(QStringLiteral("[%1] %2").arg(ts, text));
}
