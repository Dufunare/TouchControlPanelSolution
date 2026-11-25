#pragma once

#include <QMainWindow>

#include "CommunicationBackend.h"
#include "TouchBackend.h"

class DeviceController;
class ControlPanelWidget;
class GLCoordinateWidget;
class StatusPanelWidget;
class RelayStateClient;
class RelayVideoClient;
class VideoWidget;
class QThread;
class QCloseEvent;

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;
protected:
    void closeEvent(QCloseEvent* event) override;

private:
    touchpanel::TouchBackend m_backend;
    touchpanel::CommunicationBackend m_communicationBackend;
    DeviceController* m_controller = nullptr;
    RelayStateClient* m_relayStateClient = nullptr;
    QThread* m_controlThread = nullptr;
    QThread* m_videoThread = nullptr;
    RelayVideoClient* m_videoClient = nullptr;
    VideoWidget* m_videoWidget = nullptr;
    ControlPanelWidget* m_controlPanel = nullptr;
    GLCoordinateWidget* m_glWidget = nullptr;
    StatusPanelWidget* m_statusPanel = nullptr;
};
