#pragma once
#include "TeleopExecutionPolicy.h"
#include "RelativeServoP.h"
#include <cmath>
#include <cstdint>

namespace touchpanel::teleop {

// 8891 normally supplies robot_state at ~50Hz. A single 250ms age spike
// must not create new ServoP motion, but also need not immediately issue
// ResetRobot. This is a narrowly scoped grace window ONLY for a healthy,
// unchanged session; relay heartbeat, robot mode and error gates remain live.
// Beyond 400ms the existing RelayStateClient watchdog invalidates the feed.
inline constexpr int kServoPFeedbackGraceEndMs=400;
inline bool servoPRecoverableFeedbackGap(const RobotSnapshot& s,
                                         std::uint64_t session,
                                         int user,int tool) {
    return s.sessionEstablished && s.sessionEpoch==session &&
        s.relayStatusSeen && s.relayReady &&
        s.port29999Connected && s.port30003Connected && s.port30004Connected &&
        s.feedbackSeen && s.feedbackValid &&
        s.feedbackAgeMs>kContinuousRobotFeedbackAgeLimitMs &&
        s.feedbackAgeMs<=kServoPFeedbackGraceEndMs &&
        s.relayStatusAgeMs>=0 &&
        s.relayStatusAgeMs<=kContinuousRelayStatusAgeLimitMs &&
        s.userIndex==user && s.toolIndex==tool &&
        (s.robotMode==5 || s.robotMode==7) && s.enabled &&
        !s.error && !s.dragging && !s.jogging && !s.paused &&
        finiteSix(s.tcpPose) && finiteSix(s.tcpVelocity) &&
        finiteSix(s.jointVelocity);
}

// Back-pressure: compare the last *transmitted* target with the measured
// robot TCP separately from the *candidate*, which might be rejected.
// A candidate that is far ahead is not itself evidence the robot is unsafe.
// Do not keep issuing farther-ahead targets; allow only corrective targets
// that reduce measured lag. Never weaken the existing 25mm hard bound on
// an already transmitted target.
inline constexpr double kServoPTrackingSoftHoldMm=16.0;
inline constexpr double kServoPTrackingResumeMm=11.0;
inline constexpr double kServoPTrackingCorrectiveMarginMm=0.25;
inline constexpr int kServoPTrackingHoldTimeoutMs=1500;
enum class ServoPTrackingDecision { Send, Hold, Abort };
inline ServoPTrackingDecision servoPTrackingDecision(
    double acceptedLagMm,double candidateLagMm,
    bool alreadyHolding,int holdingMs) {
    if(!std::isfinite(acceptedLagMm) || !std::isfinite(candidateLagMm) ||
       acceptedLagMm<0.0 || candidateLagMm<0.0 ||
       acceptedLagMm>kServoPTrackingErrorMm ||
       (alreadyHolding && holdingMs>kServoPTrackingHoldTimeoutMs &&
        acceptedLagMm>kServoPTrackingResumeMm))
        return ServoPTrackingDecision::Abort;

    // During a hold, wait until the existing target has caught up before
    // resuming forward motion. Corrective motion may be allowed earlier.
    if(alreadyHolding && acceptedLagMm>kServoPTrackingResumeMm) {
        if(candidateLagMm<=kServoPTrackingErrorMm &&
           candidateLagMm+kServoPTrackingCorrectiveMarginMm<acceptedLagMm)
            return ServoPTrackingDecision::Send;
        return ServoPTrackingDecision::Hold;
    }
    return candidateLagMm<=kServoPTrackingSoftHoldMm
        ? ServoPTrackingDecision::Send : ServoPTrackingDecision::Hold;
}
} // namespace touchpanel::teleop
