#pragma once

#include <QDialog>
#include <QTimer>
#include <QtGlobal>
#include <array>
#include <vector>

#include "RelativeMotionCalibration.h"
#include "CalibrationCaptureQuality.h"
#include "RobotTelemetry.h"
#include "DeviceState.h"

class QLabel;
class QPushButton;
class QCheckBox;
class DeviceController;
class RelayStateClient;

// Offline/read-only registration. The dialog NEVER sends any robot command.
class RelativeCalibrationDialog final : public QDialog {
    Q_OBJECT
public:
    explicit RelativeCalibrationDialog(DeviceController* touch,
                                       RelayStateClient* robot,
                                       QWidget* parent=nullptr);
signals:
    void calibrationSaved(const touchpanel::teleop::Calibration& calibration,
                          int userIndex,int toolIndex);
private:
    enum class Phase { Start, End };
    void captureStart();
    void captureEnd();
    void beginCapture(Phase phase);
    void captureTick();
    void finishCapture();
    void resetSamples();
    void fitAndSave();
    void describe(const QString& message);
    void updateControls();
    bool readValidPair(touchpanel::teleop::Pair& pair,
                       touchpanel::RobotSnapshot& robot,
                       touchpanel::DeviceState& touch) const;
    bool matchingSession(const touchpanel::RobotSnapshot& robot,
                         const touchpanel::DeviceState& touch) const;
    static double angularDifference(double a,double b);

    DeviceController* m_touch=nullptr;
    RelayStateClient* m_robot=nullptr;
    QLabel* m_instructions=nullptr;
    QLabel* m_status=nullptr;
    QLabel* m_progress=nullptr;
    QPushButton* m_start=nullptr;
    QPushButton* m_end=nullptr;
    QPushButton* m_fit=nullptr;
    QPushButton* m_reset=nullptr;
    QCheckBox* m_confirm=nullptr;
    QTimer m_timer;
    Phase m_phase=Phase::Start;
    bool m_capturing=false;
    bool m_haveStart=false;
    std::vector<touchpanel::teleop::Pair> m_window;
    std::vector<touchpanel::teleop::Pair> m_gestures;
    touchpanel::teleop::Pair m_origin{};
    std::array<double,3> m_robotOrientation{};
    quint64 m_sessionEpoch=0;
    quint64 m_touchEpoch=0;
    quint64 m_previousStateSeq=0;
    std::uint64_t m_previousTouchCounter=0;
    int m_user=-1;
    int m_tool=-1;
};
