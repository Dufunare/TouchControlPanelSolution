#pragma once

#include <QWidget>

#include "DeviceState.h"

class QLabel;
class QPlainTextEdit;

class StatusPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StatusPanelWidget(QWidget* parent = nullptr);

    void setDeviceState(const touchpanel::DeviceState& state);
    void setBackendMessage(const QString& text);
    void setTcpConnected(bool connected);
    void setRobotStatusText(const QString& text);
    void appendTcpTxMessage(const QString& text);
    void appendTcpRxMessage(const QString& text);

private:
    QLabel* m_initializedValue = nullptr;
    QLabel* m_schedulerValue = nullptr;
    QLabel* m_validValue = nullptr;
    QLabel* m_counterValue = nullptr;

    QLabel* m_tcpStateValue = nullptr;
    QLabel* m_robotStateValue = nullptr;

    QPlainTextEdit* m_messageValue = nullptr;
    QPlainTextEdit* m_txValue = nullptr;
    QPlainTextEdit* m_rxValue = nullptr;
};
