#pragma once

#include <QObject>
#include <QTimer>

#include "DeviceStateQt.h"
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

signals:
    void deviceStateUpdated(const touchpanel::DeviceState& state);
    void backendMessageChanged(const QString& message);

private slots:
    void pollDeviceState();

private:
    touchpanel::TouchBackend* m_backend = nullptr;
    QTimer m_pollTimer;
};
