#pragma once
#include <QDialog>
#include <array>
#include "TeleopGeometry.h"
#include "MultiPointCalibration.h"
#include <vector>
class QLabel;
class QPushButton;
class DeviceController;
class RelayStateClient;

// Calibration pairs can only be captured manually. This UI never sends commands.
class TeleopCalibrationDialog : public QDialog {
    Q_OBJECT
public:
    TeleopCalibrationDialog(DeviceController* touch, RelayStateClient* robot, QWidget* parent=nullptr);
signals:
    void calibrationSaved(const touchpanel::teleop::Calibration& calibration,int userIndex,int toolIndex);
private:
    void capture();
    void solveAndSave();
    void loadSaved();
    void resetSamples();
    void showStatus(const QString& info);
    DeviceController* m_touch;
    RelayStateClient* m_robot;
    QLabel* m_status=nullptr;
    QPushButton* m_capture=nullptr;
    QPushButton* m_solve=nullptr;
    std::vector<touchpanel::teleop::Pair> m_samples;
    quint64 m_touchCalibrationEpoch=0;
    int m_count=0;
    int m_user=-1,m_tool=-1;
    quint64 m_epoch=0;
};
