#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace touchpanel {

// Immutable by-value copy of the last robot feedback. All age fields use the
// CLIENT's monotonic clock; do not compare relay steady_ns with client time.
struct RobotSnapshot {
    bool sessionEstablished = false;
    bool relayStatusSeen = false;
    bool feedbackSeen = false;
    bool relayReady = false;
    bool port29999Connected = false;
    bool port30003Connected = false;
    bool port30004Connected = false;
    bool feedbackValid = false;
    std::uint64_t sessionEpoch = 0;
    std::uint64_t stateSeq = 0;
    int feedbackAgeMs = -1;
    int relayStatusAgeMs = -1;

    int robotMode = -1;
    int userIndex = -1;
    int toolIndex = -1;
    bool enabled = false;
    bool dragging = false;
    bool running = false;
    bool error = false;
    bool jogging = false;
    bool paused = false;
    bool queuedMotion = false;
    std::array<double,6> tcpPose{};
    std::array<double,6> jointPosition{};
    std::array<double,6> jointVelocity{};
    std::array<double,6> tcpVelocity{};
    std::array<double,6> jointTarget{};
};

enum class GateReason {
    Ok,
    NoSession,
    NoRelayStatus,
    RelayStatusStale,
    RelayNotReady,
    NoRobotFeedback,
    RobotFeedbackStale,
    InvalidFeedback,
    RobotError,
    RobotDisabled,
    RobotDragging,
    RobotNotIdle,
    CoordinateMismatch,
    NoVerifiedCalibration
};

struct GateDecision {
    GateReason reason = GateReason::NoSession;
    bool canCaptureCalibration = false;
    bool canPreview = false;
    bool canSendRobotMotion = false; // NO physical motion path in this phase.
};

inline bool finiteSix(const std::array<double,6>& a) {
    for(double x:a)if(!std::isfinite(x))return false;
    return true;
}

inline GateDecision evaluateRobotGate(const RobotSnapshot& s,
                                     bool calibrationApplied=false,
                                     int calibrationUser=-1,
                                     int calibrationTool=-1,
                                     int maxFeedbackAgeMs=400,
                                     int maxRelayStatusAgeMs=750) {
    GateDecision result;
    if(!s.sessionEstablished) {result.reason=GateReason::NoSession;return result;}
    if(!s.relayStatusSeen) {result.reason=GateReason::NoRelayStatus;return result;}
    if(s.relayStatusAgeMs<0 || s.relayStatusAgeMs>maxRelayStatusAgeMs) {
        result.reason=GateReason::RelayStatusStale;return result;
    }
    if(!s.relayReady || !s.port29999Connected || !s.port30003Connected || !s.port30004Connected) {
        result.reason=GateReason::RelayNotReady;return result;
    }
    if(!s.feedbackSeen) {result.reason=GateReason::NoRobotFeedback;return result;}
    if(s.feedbackAgeMs<0 || s.feedbackAgeMs>maxFeedbackAgeMs) {
        result.reason=GateReason::RobotFeedbackStale;return result;
    }
    if(!s.feedbackValid || s.stateSeq==0 || s.sessionEpoch==0 ||
       s.userIndex<0 || s.toolIndex<0 ||
       !finiteSix(s.tcpPose) || !finiteSix(s.jointPosition) ||
       !finiteSix(s.jointVelocity) || !finiteSix(s.tcpVelocity) ||
       !finiteSix(s.jointTarget)) {
        result.reason=GateReason::InvalidFeedback;return result;
    }
    // Calibration samples may be taken during manually taught robot motion;
    // this permission is only to READ paired positions, not to move anything.
    result.canCaptureCalibration=true;
    if(s.error) {result.reason=GateReason::RobotError;return result;}
    if(!s.enabled) {result.reason=GateReason::RobotDisabled;return result;}
    if(s.dragging) {result.reason=GateReason::RobotDragging;return result;}
    if(s.robotMode!=5 || s.running || s.jogging || s.paused || s.queuedMotion) {
        result.reason=GateReason::RobotNotIdle;return result;
    }
    if(calibrationApplied && (s.userIndex!=calibrationUser || s.toolIndex!=calibrationTool)) {
        result.reason=GateReason::CoordinateMismatch;return result;
    }
    if(!calibrationApplied) {result.reason=GateReason::NoVerifiedCalibration;return result;}
    result.canPreview=true;
    result.reason=GateReason::Ok;
    // Intentionally *never* set canSendRobotMotion=true, even when all checks pass.
    return result;
}
} // namespace touchpanel
