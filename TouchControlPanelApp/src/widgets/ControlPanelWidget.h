#pragma once

#include <QWidget>
#include <QtGlobal>

class QPushButton;
class QLineEdit;
class QSpinBox;

class ControlPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanelWidget(QWidget* parent = nullptr);

signals:
    void initializeRequested();
    void startRequested();
    void stopRequested();
    void resetRequested();

    void robotDisconnectRequested();
    void robotConnectRequested(const QString& ip, quint16 port);
    void powerOnRequested();
    void enableRobotRequested();
    void disableRobotRequested();
    void clearErrorRequested();
    void emergencyStopRequested();
    void startDragRequested();
    void stopDragRequested();

private:
    QPushButton* m_initializeButton = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_resetButton = nullptr;

    QPushButton* m_robotConnectButton = nullptr;
    QPushButton* m_robotDisconnectButton = nullptr;
    QPushButton* m_powerOnButton = nullptr;
    QPushButton* m_enableRobotButton = nullptr;
    QPushButton* m_disableRobotButton = nullptr;
    QPushButton* m_clearErrorButton = nullptr;
    QPushButton* m_emergencyStopButton = nullptr;
    QPushButton* m_startDragButton = nullptr;
    QPushButton* m_stopDragButton = nullptr;

    QLineEdit* m_transitIpEdit = nullptr;
    QSpinBox* m_transitPortSpin = nullptr;
};
