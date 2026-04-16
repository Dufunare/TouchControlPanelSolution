#pragma once

#include <QObject>
#include <QTimer>
#include <QtGlobal>

#include "DeviceStateQt.h"
#include "communicationbackend/RobotControllerFacade.h"
#include "TouchBackend.h"

class DeviceController : public QObject
{
    Q_OBJECT

public:
    explicit DeviceController(touchpanel::TouchBackend* backend, QObject* parent = nullptr);

public slots:
    void initializeBackend();
    void startStreaming();
    void stopStreaming();
    void resetBackend();

    void connectTransit(const QString& ipOverride = QString(), quint16 portOverride = 0);
    void disconnectTransit();
    void powerOnRobot();
    void enableRobot();
    void emergencyStopRobot();
    void startDragRobot();
    void stopDragRobot();

signals:
    void deviceStateUpdated(const touchpanel::DeviceState& state);
    void backendMessageChanged(const QString& message);

    void tcpConnectionChanged(bool connected);
    void robotStatusChanged(const QString& status);
    void tcpTxMessage(const QString& message);
    void tcpRxMessage(const QString& message);

private slots:
    void pollDeviceState();

private:
    touchpanel::TouchBackend* m_backend = nullptr;
    RobotControllerFacade m_robotFacade;
    QTimer m_pollTimer;
};
