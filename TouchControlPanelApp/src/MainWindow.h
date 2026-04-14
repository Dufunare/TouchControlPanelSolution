#pragma once

#include <QMainWindow>

#include "TouchBackend.h"

class DeviceController;
class GLCoordinateWidget;
class StatusPanelWidget;

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private:
    touchpanel::TouchBackend m_backend;
    DeviceController* m_controller = nullptr;
    GLCoordinateWidget* m_glWidget = nullptr;
    StatusPanelWidget* m_statusPanel = nullptr;
};
