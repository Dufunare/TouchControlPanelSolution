#pragma once

#include <QWidget>
#include <QtGlobal>

class QPushButton;
class QLineEdit;
class QSpinBox;
class QComboBox;
class QCheckBox;

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

private:

	//touch控制按钮
    QPushButton* m_initializeButton = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_resetButton = nullptr;

	//tcp连接管理
    QPushButton* m_robotConnectButton = nullptr;
    QPushButton* m_robotDisconnectButton = nullptr;
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

    //视频控制占位
    QComboBox* m_videoSourceCombo = nullptr;
    QLineEdit* m_videoSourceEdit = nullptr;
    QSpinBox* m_videoFpsSpin = nullptr;
    QCheckBox* m_videoMirrorCheck = nullptr;
    QPushButton* m_videoStartButton = nullptr;
    QPushButton* m_videoStopButton = nullptr;
    QPushButton* m_videoSnapshotButton = nullptr;


};
