#pragma once
#include "ManualTeleop.h"
#include "OrientationConventionAudit.h"
#include "ServoPCommandPlanner.h"
#include "TeleopExecutionPolicy.h"

namespace touchpanel::teleop {
// Local six-axis command/feedback guards. Keep the previous rotational
// step, lead, tracking and joint-velocity thresholds unchanged. These are
// software monitoring thresholds, not certified physical limits. The
// installed controller's safety settings and physical safeguards remain
// authoritative. No fixed TCP or session-centered travel sphere.
inline constexpr double kServoPRotationStepDeg=0.20;
inline constexpr double kServoPRotationDeadbandDeg=0.05;
inline constexpr double kServoPRotationLeadDeg=1.0;
inline constexpr double kServoPRotationHardTrackingDeg=1.5;
inline constexpr double kServoPJointTrackingDeg=3.0;
inline constexpr double kServoPJointSpeedDegPerSec=5.0;

inline Vec3 servoPRpy(const Pose6& p) {return {p[3],p[4],p[5]};}
inline Mat3 servoPRotation(const Pose6& p) {return rpyToMatrix(servoPRpy(p));}
inline bool servoPRotationPoseValid(const Pose6& p) {
    if(checkServoPWorkspace(p).code!=ServoPWorkspaceCode::Allowed)return false;
    Vec3 converted{};
    return matrixToRpy(servoPRotation(p),servoPRpy(p),converted);
}

enum class ServoP6DFault {
    None, Link, Sequence, Session, Frames, FeedbackAge, RelayAge, RobotState,
    NonfiniteFeedback, Quaternion, OrientationMismatch, Pose,
    JointRange, JointTracking, JointSpeed
};
struct ServoP6DCheck {
    ServoP6DFault fault=ServoP6DFault::None;
    int joint=-1;
    double value=0.0;
    double limit=0.0;
    bool ok() const {return fault==ServoP6DFault::None;}
};
// Same thresholds as the preceding boolean guard. The check returns the
// actual rejecting condition from the very same snapshot used for control.
inline ServoP6DCheck checkServoP6DFeedback(const RobotSnapshot& s,
                                           std::uint64_t session,
                                           Mat3& measured) {
    using F=ServoP6DFault;
    if(!s.sessionEstablished || !s.feedbackValid || !s.relayStatusSeen ||
       !s.relayReady || !s.port29999Connected || !s.port30003Connected ||
       !s.port30004Connected)return {F::Link};
    if(!s.feedbackSeen || s.stateSeq==0)return {F::Sequence};
    if(s.sessionEpoch==0 || s.sessionEpoch!=session)return {F::Session};
    if(s.userIndex!=0 || s.toolIndex!=0)return {F::Frames};
    if(s.feedbackAgeMs<0 || s.feedbackAgeMs>150)
        return {F::FeedbackAge,-1,static_cast<double>(s.feedbackAgeMs),150.0};
    if(s.relayStatusAgeMs<0 || s.relayStatusAgeMs>350)
        return {F::RelayAge,-1,static_cast<double>(s.relayStatusAgeMs),350.0};
    if(!s.enabled || s.error || s.dragging || s.jogging || s.paused ||
       (s.robotMode!=5 && s.robotMode!=7))return {F::RobotState};
    if(!finiteSix(s.jointPosition) || !finiteSix(s.jointTarget) ||
       !finiteSix(s.jointVelocity) || !finiteSix(s.tcpVelocity) ||
       !finiteSix(s.tcpPose))return {F::NonfiniteFeedback};
    if(!defaultFrameOrientation(s,measured))return {F::Quaternion};
    const double orientationError=feedbackZyxErrorDeg(s);
    if(!std::isfinite(orientationError) || orientationError>0.5)
        return {F::OrientationMismatch,-1,orientationError,0.5};
    if(!servoPRotationPoseValid(s.tcpPose))return {F::Pose};
    for(int i=0;i<6;++i) {
        const double nominal=i==2?155.0:360.0;
        const double jointLimit=nominal-kServoPJointTrackingDeg;
        const double excursion=std::max(std::abs(s.jointPosition[i]),std::abs(s.jointTarget[i]));
        if(excursion>jointLimit)return {F::JointRange,i,excursion,jointLimit};
        const double error=std::abs(s.jointTarget[i]-s.jointPosition[i]);
        if(error>kServoPJointTrackingDeg)
            return {F::JointTracking,i,error,kServoPJointTrackingDeg};
        const double speed=std::abs(s.jointVelocity[i]);
        if(speed>kServoPJointSpeedDegPerSec)
            return {F::JointSpeed,i,speed,kServoPJointSpeedDegPerSec};
    }
    return {};
}
inline bool servoP6DFeedbackHealthy(const RobotSnapshot& s,
                                    std::uint64_t session,
                                    Mat3& measured) {
    return checkServoP6DFeedback(s,session,measured).ok();
}

// The translation arbiter is unchanged. Check the final combined target
// against BOTH Cartesian and angular bounds, including pure rotations.
inline bool servoP6DCandidateAllowed(const Pose6& candidate,const Pose6& last,
                                    const Pose6& actual,const Mat3& measured) {
    if(!servoPRotationPoseValid(candidate) || !servoPRotationPoseValid(last) ||
       !servoPRotationPoseValid(actual) || !properRotation(measured))return false;
    constexpr double tolerance=1e-7;
    if(servoPCartesianDistance(candidate,last)>kServoPCommandMaxJumpMm+tolerance ||
       servoPCartesianDistance(candidate,actual)>kServoPCommandLeadMm+tolerance)
        return false;
    double step=0;
    for(int i=3;i<6;++i)step+=std::abs(candidate[i]-last[i]);
    const auto rotation=servoPRotation(candidate);
    return step<=kServoPRotationStepDeg+tolerance &&
           rotationDistanceDeg(rotation,servoPRotation(last))<=kServoPRotationStepDeg+tolerance &&
           rotationDistanceDeg(rotation,measured)<=kServoPRotationLeadDeg+tolerance;
}
} // namespace touchpanel::teleop
