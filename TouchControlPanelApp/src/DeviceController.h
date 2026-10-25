#pragma once

#include <QObject>
#include <array>
#include <QTimer>
#include <QElapsedTimer>
#include <QString>
#include "TeleopExecutionPolicy.h"
#include <QtGlobal>

#include "DeviceStateQt.h"
#include "CommunicationBackend.h"
#include "TouchBackend.h"
#include "TeleopGeometry.h"
#include "ManualTeleop.h"
#include "RelativeServoP.h"

#define DEVICE_CONTROLLER_POLL_INTERVAL_MS 8 // 约 125 FPS 的前端刷新频率
#define DEVICE_CONTROLLER_MOTION_INTERVAL_MS 33 // 33ms 为机械臂官方文档推荐的远程控制发送频率

class RelayStateClient;

class DeviceController : public QObject
{
    Q_OBJECT

public:
    explicit DeviceController(
        touchpanel::TouchBackend* backend,
        touchpanel::CommunicationBackend* communicationBackend,
        QObject* parent = nullptr);

    void setRelayStateClient(RelayStateClient* client);
    bool getTouchSnapshot(touchpanel::DeviceState& state) const;
    bool realMotionActive() const;
    void applyPreviewCalibration(const touchpanel::teleop::Calibration& calibration,int user,int tool);
    void setManualPreviewMapping(const touchpanel::teleop::ManualAxisMap& mapping);

public slots:
    void invalidateTeleopBaseline();
    void onRelayStateInvalidated();
    void checkTouchCalibration();
    // touch设备管理
    void initializeBackend();
    void startStreaming();
    void stopStreaming();
    void resetBackend();

    // tcp连接管理
    void connectTransit(const QString& ipOverride = QString(), quint16 portOverride = 0);
    void disconnectTransit();

    //机械比状态控制
    void powerOnRobot();
    void enableRobot();
    void disableRobot();
    void clearRobotError();
    void emergencyStopRobot();
    void startDragRobot();
    void stopDragRobot();
    void startTeleop();
    void startManualPreview();
    void prepareSingleStep();
    void confirmSingleStep();
    void startContinuousServo();
    void startContinuousServoP();
    void stopTeleop();

signals:
    void deviceStateUpdated(const touchpanel::DeviceState& state);
    void backendMessageChanged(const QString& message);

    void tcpConnectionChanged(bool connected);
    void robotStatusChanged(const QString& status);
    void tcpTxMessage(const QString& message);
    void tcpRxMessage(const QString& message);
    void singleStepConfirmationNeeded(const QString& summary);

private slots:
    void pollDeviceState();
    void onMotionTick();
    void handleRobotOperation(const QString& operation,bool success,const QString& detail);
    void handleInverseSolution(bool success,const std::array<double,6>& joints);
    void runContinuousTick();
    void runServoPTick();
    void monitorSingleStep();
    void haltRealMotion(const QString& cause);

private:
    RelayStateClient* m_relayStateClient = nullptr;
    touchpanel::teleop::Calibration m_calibration{};
    bool m_calibrationApplied=false;
    bool m_manualPreview=false;
    touchpanel::teleop::ManualAxisMap m_manualMap{};
    int m_calibrationUser=-1,m_calibrationTool=-1;
    std::uint64_t m_calibrationSessionEpoch=0,m_calibrationTouchEpoch=0;
    touchpanel::TouchBackend* m_backend = nullptr;
    touchpanel::CommunicationBackend* m_robotBackend = nullptr;
    QTimer m_pollTimer;
    QTimer m_motionTimer;
    bool m_lastButtonPressed = false;
    bool m_motionBaselineReady = false;
    std::uint64_t m_clutchCalibrationEpoch=0;
    int m_modeQueryCooldownTicks = 0;
    int m_poseQueryCooldownTicks = 0;
    std::array<double, 3> m_touchOrigin{ 0.0, 0.0, 0.0 };
    std::array<double, 9> m_touchRotationOrigin{ 1,0,0,0,1,0,0,0,1 };
    std::array<double, 6> m_robotPoseOrigin{};
    std::array<double, 6> m_previewLast{};
    bool m_previewLastValid = false;
    std::array<double, 3> m_robotOrigin{ 0.0, 0.0, 0.0 };
    std::array<double, 3> m_lastSent{ 0.0, 0.0, 0.0 };
    bool m_hasLastSent = false;
    enum class RealMode { None, SingleStep, Continuous, CartesianServoP };
    RealMode m_realMode=RealMode::None;
    bool m_stepPrepared=false;
    bool m_stepAwaitingReply=false;
    bool m_stepReplyAccepted=false;
    bool m_ikPending=false;
    bool m_motionCommandSent=false;
    touchpanel::teleop::RelativeServoP m_cartesianServoP;
    QElapsedTimer m_servoPReleaseTimer;
    QElapsedTimer m_servoPStatsTimer;
    std::uint64_t m_servoPLastStateSeq=0;
    int m_servoPSentPackets=0;
    bool m_servoPWasSaturated=false;
    bool m_haveServoTarget=false;
    bool m_clutchRangeSaturated=false;
    std::array<double,6> m_latestServoTarget{};
    bool m_continuousStopping=false;
    bool m_stopReplyAccepted=false;
    int m_releaseTicks=0;
    int m_invalidTouchTicks=0;
    int m_staleTouchTicks=0;
    std::uint64_t m_lastTouchSampleCounter=0;
    int m_ikQueries=0,m_ikAccepted=0,m_servoCommands=0;
    int m_ikRttSumMs=0,m_ikRttMaxMs=0,m_ikRttSamples=0;
    QElapsedTimer m_stopTimer;
    QElapsedTimer m_continuousStatsTimer;
    int m_motionUser=-1,m_motionTool=-1;
    std::uint64_t m_motionSession=0;
    std::array<double,6> m_ikJointOrigin{};
    std::array<double,6> m_stepExpectedStart{};
    touchpanel::teleop::StepProposal m_stepProposal{};
    QElapsedTimer m_stepTimer;
    QElapsedTimer m_ikTimer;
    QElapsedTimer m_servoTime;
    std::uint64_t m_stepStartSeq=0;
    std::uint64_t m_stepLastArrivalSeq=0;
    int m_stepArrivalStableSamples=0;
    bool m_singleStepVerified=false;
    std::uint64_t m_verifiedStepSession=0;
};