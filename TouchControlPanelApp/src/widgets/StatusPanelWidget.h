#pragma once

#include <QWidget>

#include "DeviceState.h"

class QLabel;
class QPushButton;

class StatusPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StatusPanelWidget(QWidget* parent = nullptr);

    void setDeviceState(const touchpanel::DeviceState& state);
    void setBackendMessage(const QString& text);

signals:
    void initializeRequested();
    void startRequested();
    void stopRequested();

private:
    QLabel* m_initializedValue = nullptr;
    QLabel* m_schedulerValue = nullptr;
    QLabel* m_validValue = nullptr;
    QLabel* m_counterValue = nullptr;

    QLabel* m_xValue = nullptr;
    QLabel* m_yValue = nullptr;
    QLabel* m_zValue = nullptr;

    QLabel* m_messageValue = nullptr;

    QPushButton* m_initializeButton = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
};
