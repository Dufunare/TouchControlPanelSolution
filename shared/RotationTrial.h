#pragma once
#include "ManualTeleop.h"
#include "OrientationConventionAudit.h"
#include "ServoPCommandPlanner.h"
#include "TeleopExecutionPolicy.h"

namespace touchpanel::teleop {
// Narrow first-commissioning command envelope. NOT rated physical velocity,
// collision avoidance, joint-space path validation or a safety function.
// Keep all existing translation/stop gates and controller-side protection.
// No GUI widening; a wider trial requires a separate engineering review.
inline constexpr double kRotationTrialTravelDeg=2.0;
inline constexpr double kRotationTrialStepDeg=0.20;
inline constexpr double kRotationTrialDeadbandDeg=0.05;
inline constexpr double kRotationTrialLeadDeg=1.0;
inline constexpr double kRotationTrialHardTrackingDeg=1.5;
inline constexpr double kRotationTrialHardTravelDeg=3.0;
inline constexpr double kRotationTrialDriftMm=2.0;
inline constexpr double kRotationTrialJointExcursionDeg=3.0;
inline constexpr double kRotationTrialJointSpeedDegPerSec=5.0;

inline Vec3 servoPRpy(const Pose6& p) {return {p[3],p[4],p[5]};}
inline Mat3 servoPRotation(const Pose6& p) {return rpyToMatrix(servoPRpy(p));}
inline bool rotationTrialPoseValid(const Pose6& p) {
    if(checkServoPWorkspace(p).code!=ServoPWorkspaceCode::Allowed)return false;
    // Never enter an Euler representation singularity (distinct from
    // physical robot kinematic singularities).
    Vec3 converted{};
    return matrixToRpy(servoPRotation(p),servoPRpy(p),converted);
}
inline bool rotationTrialFeedbackHealthy(const RobotSnapshot& s,
                                         const RobotSnapshot& origin,
                                         Mat3& measured) {
    if(!continuousTelemetryAgesHealthy(s) || !s.feedbackSeen || s.stateSeq==0 ||
       s.sessionEpoch==0 || s.sessionEpoch!=origin.sessionEpoch ||
       s.userIndex!=0 || s.toolIndex!=0 || s.feedbackAgeMs>150 ||
       s.relayStatusAgeMs>350 || !s.enabled || s.error || s.dragging ||
       s.jogging || s.paused || (s.robotMode!=5 && s.robotMode!=7) ||
       !finiteSix(s.jointPosition) || !finiteSix(s.jointTarget) ||
       !finiteSix(s.jointVelocity) || !finiteSix(s.tcpVelocity) ||
       !finiteSix(origin.jointPosition) ||
       !defaultFrameOrientation(s,measured) || feedbackZyxErrorDeg(s)>0.5 ||
       !rotationTrialPoseValid(s.tcpPose) || !rotationTrialPoseValid(origin.tcpPose) ||
       servoPCartesianDistance(s.tcpPose,origin.tcpPose)>kRotationTrialDriftMm ||
       rotationDistanceDeg(measured,servoPRotation(origin.tcpPose))>kRotationTrialHardTravelDeg)
        return false;
    // Actual and controller target joint excursions, not just TCP angles.
    // Nominal CR3 range from hardware manual V1.8 Annex A is NOT the
    // installed controller's safety limit. Keep controller limits active.
    for(int i=0;i<6;++i) {
        const double nominal=i==2?155.0:360.0;
        if(std::abs(origin.jointPosition[i])>nominal-kRotationTrialJointExcursionDeg ||
           std::abs(s.jointPosition[i])>nominal ||
           std::abs(s.jointTarget[i])>nominal ||
           std::abs(s.jointPosition[i]-origin.jointPosition[i])>kRotationTrialJointExcursionDeg ||
           std::abs(s.jointTarget[i]-origin.jointPosition[i])>kRotationTrialJointExcursionDeg ||
           std::abs(s.jointVelocity[i])>kRotationTrialJointSpeedDegPerSec)return false;
    }
    return true;
}

// No physical translation in rotation trial. Use a continuous Euler branch
// of a matrix-derived fixed-axis rotation; its L1 step is an upper bound
// on the corresponding rotation distance. No wrap-around or stale backlog.
inline bool rotationTrialCandidateAllowed(const Pose6& candidate,const Pose6& last,
                                          const Pose6& clutch,const Pose6& origin,
                                          const Mat3& measured) {
    if(!rotationTrialPoseValid(candidate) || !rotationTrialPoseValid(last) ||
       !rotationTrialPoseValid(clutch) || !rotationTrialPoseValid(origin) ||
       !properRotation(measured))return false;
    for(int i=0;i<3;++i)if(candidate[i]!=clutch[i] || candidate[i]!=last[i])return false;
    if(servoPCartesianDistance(candidate,origin)>kRotationTrialDriftMm)return false;
    double step=0;
    for(int i=3;i<6;++i)step+=std::abs(candidate[i]-last[i]);
    const auto rotation=servoPRotation(candidate);
    constexpr double tolerance=1e-7;
    return step<=kRotationTrialStepDeg+tolerance &&
           rotationDistanceDeg(rotation,servoPRotation(last))<=kRotationTrialStepDeg+tolerance &&
           rotationDistanceDeg(rotation,servoPRotation(clutch))<=kRotationTrialTravelDeg+tolerance &&
           rotationDistanceDeg(rotation,servoPRotation(origin))<=kRotationTrialTravelDeg+tolerance &&
           rotationDistanceDeg(rotation,measured)<=kRotationTrialLeadDeg+tolerance;
}
} // namespace touchpanel::teleop
