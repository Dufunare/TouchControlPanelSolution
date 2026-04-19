#pragma once

#include <QObject>
#include <array>
#include <QTimer>
#include <QtGlobal>

#include "DeviceStateQt.h"
#include "CommunicationBackend.h"
#include "TouchBackend.h"

#define DEVICE_CONTROLLER_POLL_INTERVAL_MS 8 // 约 125 FPS 的前端刷新频率
#define DEVICE_CONTROLLER_MOTION_INTERVAL_MS 33 // 33ms 为机械臂官方文档推荐的远程控制发送频率

class DeviceController : public QObject
{
    Q_OBJECT

public:
    explicit DeviceController(
        touchpanel::TouchBackend* backend,
        touchpanel::CommunicationBackend* communicationBackend,
        QObject* parent = nullptr);

public slots:
    void initializeBackend();
    void startStreaming();
    void stopStreaming();
    void resetBackend();

    void connectTransit(const QString& ipOverride = QString(), quint16 portOverride = 0);
    void disconnectTransit();
    void powerOnRobot();
    void enableRobot();
    void disableRobot();
    void clearRobotError();
    void emergencyStopRobot();
    void startDragRobot();
    void stopDragRobot();
    void startTeleop();
    void stopTeleop();

signals:
    void deviceStateUpdated(const touchpanel::DeviceState& state);
    void backendMessageChanged(const QString& message);

    void tcpConnectionChanged(bool connected);
    void robotStatusChanged(const QString& status);
    void tcpTxMessage(const QString& message);
    void tcpRxMessage(const QString& message);

private slots:
    void pollDeviceState();
    void onMotionTick();

private:
    touchpanel::TouchBackend* m_backend = nullptr;
    touchpanel::CommunicationBackend* m_robotBackend = nullptr;
    QTimer m_pollTimer;
    QTimer m_motionTimer;
    bool m_lastButtonPressed = false;
    bool m_motionBaselineReady = false;
    int m_modeQueryCooldownTicks = 0;
    int m_poseQueryCooldownTicks = 0;
    std::array<double, 3> m_touchOrigin{ 0.0, 0.0, 0.0 };
    std::array<double, 3> m_robotOrigin{ 0.0, 0.0, 0.0 };
    std::array<double, 3> m_lastSent{ 0.0, 0.0, 0.0 };
    bool m_hasLastSent = false;
};