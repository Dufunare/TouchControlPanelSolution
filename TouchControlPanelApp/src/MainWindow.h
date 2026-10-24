#pragma once

#include <QMainWindow>

#include "CommunicationBackend.h"
#include "TouchBackend.h"

class DeviceController;
class ControlPanelWidget;
class GLCoordinateWidget;
class StatusPanelWidget;
class VideoWidget;
class RelayStateClient;
class QCloseEvent;

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;
protected:
    void closeEvent(QCloseEvent* event) override;

private:
    touchpanel::TouchBackend m_backend;
    touchpanel::CommunicationBackend m_communicationBackend;
    DeviceController* m_controller = nullptr;
    RelayStateClient* m_relayStateClient = nullptr;
    ControlPanelWidget* m_controlPanel = nullptr;
    VideoWidget* m_videoWidget = nullptr;
    GLCoordinateWidget* m_glWidget = nullptr;
    StatusPanelWidget* m_statusPanel = nullptr;
};
