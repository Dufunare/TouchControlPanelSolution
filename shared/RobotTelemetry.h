#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

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
    // Dobot RunQueuedCmd: algorithm queue execution flag. Does NOT by itself
    // establish pending motion or a non-idle physical robot state.
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
    bool canSendRobotMotion = false; // This legacy gate is preview-only; real modes have a separate policy.
};

inline const char* gateReasonChinese(GateReason reason) {
    switch(reason) {
        case GateReason::Ok:return "条件满足（仅预览）";
        case GateReason::NoSession:return "8891 未建立会话";
        case GateReason::NoRelayStatus:return "尚未收到中转站状态";
        case GateReason::RelayStatusStale:return "中转站状态已过期";
        case GateReason::RelayNotReady:return "中转站或机器人端口未就绪";
        case GateReason::NoRobotFeedback:return "尚未收到机器人反馈";
        case GateReason::RobotFeedbackStale:return "机器人反馈已过期";
        case GateReason::InvalidFeedback:return "机器人反馈字段无效";
        case GateReason::RobotError:return "机器人报告错误";
        case GateReason::RobotDisabled:return "机器人未使能";
        case GateReason::RobotDragging:return "机器人处于拖拽模式，请安全退出拖拽后再试";
        case GateReason::RobotNotIdle:return "机器人非空闲状态";
        case GateReason::CoordinateMismatch:return "User/Tool 与标定不一致";
        case GateReason::NoVerifiedCalibration:return "尚未在本次会话应用空间标定（只读预览）";
    }
    return "未知状态";
}

// Diagnostics only: show the exact flags responsible for the idle gate.
// This must NEVER replace evaluateRobotGate or authorize robot movement.
inline std::string idleGateDiagnostics(const RobotSnapshot& s) {
    const auto flag=[](bool b){return b?"yes":"no";};
    return std::string("Mode=")+std::to_string(s.robotMode)+
        " Running="+flag(s.running)+
        " Jog="+flag(s.jogging)+
        " Pause="+flag(s.paused)+
        " Queued="+flag(s.queuedMotion)+
        " Drag="+flag(s.dragging);
}

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
    // This gate is for read-only Dry Run ONLY. RunQueuedCmd (queuedMotion)
    // is defined by Dobot as the algorithm queue execution flag, not as a
    // count of pending robot motions. It may stay true with RobotMode=5 and
    // RunningStatus=0. Do not disqualify the preview on that flag alone.
    // This preview gate never grants physical motion. Experimental motion
    // uses TeleopExecutionPolicy instead of reusing canPreview as authorization.
    if(s.robotMode!=5 || s.running || s.jogging || s.paused) {
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
