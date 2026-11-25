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

inline bool servoP6DFeedbackHealthy(const RobotSnapshot& s,
                                    std::uint64_t session,
                                    Mat3& measured) {
    if(!continuousTelemetryAgesHealthy(s) || !s.feedbackSeen || s.stateSeq==0 ||
       s.sessionEpoch==0 || s.sessionEpoch!=session ||
       s.userIndex!=0 || s.toolIndex!=0 || s.feedbackAgeMs>150 ||
       s.relayStatusAgeMs>350 || !s.enabled || s.error || s.dragging ||
       s.jogging || s.paused || (s.robotMode!=5 && s.robotMode!=7) ||
       !finiteSix(s.jointPosition) || !finiteSix(s.jointTarget) ||
       !finiteSix(s.jointVelocity) || !finiteSix(s.tcpVelocity) ||
       !defaultFrameOrientation(s,measured) || feedbackZyxErrorDeg(s)>0.5 ||
       !servoPRotationPoseValid(s.tcpPose))
        return false;
    // A fixed startup joint excursion would prevent ordinary translation.
    // Instead monitor the local difference between actual and controller
    // target joints. Retain the prior 3-degree threshold and speed check,
    // and require both joints inside nominal mechanical limits with margin.
    // This is NOT a substitute for the installed limits or path validation.
    for(int i=0;i<6;++i) {
        const double nominal=i==2?155.0:360.0;
        if(std::abs(s.jointPosition[i])>nominal-kServoPJointTrackingDeg ||
           std::abs(s.jointTarget[i])>nominal-kServoPJointTrackingDeg ||
           std::abs(s.jointTarget[i]-s.jointPosition[i])>kServoPJointTrackingDeg ||
           std::abs(s.jointVelocity[i])>kServoPJointSpeedDegPerSec)return false;
    }
    return true;
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
