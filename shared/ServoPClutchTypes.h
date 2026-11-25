#pragma once
#include "TeleopGeometry.h"

namespace touchpanel::teleop {

// Existing experimental command-space travel envelope; not a certified
// physical workspace or stopping distance. Do not raise it from UI feel.
inline constexpr double kServoPClutchTravelMm=60.0;
inline constexpr double kServoPMaxClutchTravelMm=180.0;

enum class ServoPPhase { NeedRelease, Ready, Active, WaitingForIdle };
enum class ServoPAction {
    None, Ready, Captured, Send, Released, TrackingHold, Invalid, RegripBlocked
};

struct ServoPTick {
    ServoPAction action=ServoPAction::None;
    Pose6 target{};
    // Original fixed-axis intent, before the independent command guard.
    Pose6 intent{};
    bool intentValid=false;
    // Only the live fixed-axis controller uses this release handshake.
    bool releaseStopRequired=false;
    double rawDistanceMm=0.0;
    double boundedDistanceMm=0.0;
    bool saturated=false;
    bool feedbackLimited=false;
    unsigned wallHoldAxisMask=0;
};

} // namespace touchpanel::teleop
