#pragma once

#include <QWidget>

class QPushButton;

class ControlPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanelWidget(QWidget* parent = nullptr);

signals:
    void initializeRequested();
    void startRequested();
    void stopRequested();

    void robotConnectRequested();
    void robotDisconnectRequested();

private:
    QPushButton* m_initializeButton = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;

    QPushButton* m_robotConnectButton = nullptr;
    QPushButton* m_robotDisconnectButton = nullptr;
};
