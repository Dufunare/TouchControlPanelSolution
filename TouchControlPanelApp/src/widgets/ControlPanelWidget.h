#pragma once

#include <QWidget>
#include <QtGlobal>
#include <array>

class QPushButton;
class QLineEdit;
class QSpinBox;
class QComboBox;
class QCheckBox;
class QDoubleSpinBox;

class ControlPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanelWidget(QWidget* parent = nullptr);

signals:
    //touch控制信号
    void initializeRequested();
    void startRequested();
    void stopRequested();
    void resetRequested();

    //tcp连接管理信号
    void robotDisconnectRequested();
    void robotConnectRequested(const QString& ip, quint16 port);
    void relayV2ConnectRequested(const QString& ip);
    void relayV2DisconnectRequested();

	//机械臂控制信号
    void powerOnRequested();
    void enableRobotRequested();
    void disableRobotRequested();
    void clearErrorRequested();
    void emergencyStopRequested();
    void startDragRequested();
    void stopDragRequested();
    void startTeleopRequested();
    void stopTeleopRequested();
    void manualPreviewRequested(const std::array<int,3>& axes,double gain,double deadzone);
    void singleStepRequested();
    void continuousServoRequested(const std::array<int,3>& axes,double gain,double deadzone);
    void continuousServoPRequested(const std::array<int,3>& axes,double gain,double deadzone);

private:
    void setAxisChoice(int robotAxis,int sourceCode);
    std::array<int,3> m_axisSelection{{0,5,1}}; // match working master ServoP axis convention
    std::array<QComboBox*,3> m_axisCombos{{nullptr,nullptr,nullptr}};
    QDoubleSpinBox* m_axisGain=nullptr;
    QDoubleSpinBox* m_axisDeadzone=nullptr;
    QComboBox* m_realModeSelector=nullptr;

	//touch控制按钮
    QPushButton* m_initializeButton = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_resetButton = nullptr;

	//tcp连接管理
    QPushButton* m_robotConnectButton = nullptr;
    QPushButton* m_robotDisconnectButton = nullptr;
    QPushButton* m_relayV2ConnectButton = nullptr;
    QPushButton* m_relayV2DisconnectButton = nullptr;
    QLineEdit* m_transitIpEdit = nullptr;
    QSpinBox* m_transitPortSpin = nullptr;

	//机械臂控制按钮
    QPushButton* m_powerOnButton = nullptr;
    QPushButton* m_enableRobotButton = nullptr;
    QPushButton* m_disableRobotButton = nullptr;
    QPushButton* m_clearErrorButton = nullptr;
    QPushButton* m_emergencyStopButton = nullptr;
    QPushButton* m_startDragButton = nullptr;
    QPushButton* m_stopDragButton = nullptr;
    QPushButton* m_startTeleopButton = nullptr;
    QPushButton* m_stopTeleopButton = nullptr;

};
