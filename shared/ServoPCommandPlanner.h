#pragma once
#include "ServoPWorkspace.h"
#include "TeleopGeometry.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace touchpanel::teleop {

// ServoP online reference and feedback safety are deliberately separate.
// This module consumes the latest already-mapped Touch goal and the last
// successfully LOCAL-TCP-written target. It holds no queued old goals.
// Firmware, real robot velocity/acceleration and stopping distance MUST be
// validated independently. These bounds are software test limits only.
inline constexpr double kServoPTrackingErrorMm=25.0;
inline constexpr double kServoPCommandLeadMm=20.0;
inline constexpr double kServoPCommandMaxJumpMm=20.0;
inline constexpr double kServoPCommandDeadbandMm=0.3;

enum class ServoPCommandDisposition { None, Send, TrackingHold, Invalid };
struct ServoPCommandPlan {
    ServoPCommandDisposition disposition=ServoPCommandDisposition::None;
    Pose6 target{};
    unsigned heldAxes=0;
    bool leadRestricted=false;
    bool jumpRestricted=false;
};
inline bool servoPFinitePose(const Pose6& p) {
    for(double x:p)if(!std::isfinite(x))return false;
    return true;
}
inline double servoPCartesianDistance(const Pose6& a,const Pose6& b) {
    return norm(Vec3{a[0]-b[0],a[1]-b[1],a[2]-b[2]});
}

// A deterministic command-space safety arbiter. CRITICAL: the goal is not
// regenerated from actual TCP. Feedback only restricts whether a command
// generated from the latest Touch goal may be transmitted. The existing
// 25mm last-sent/actual hard-stop is separately enforced by the caller.
inline ServoPCommandPlan planServoPCommand(const Pose6& latestTouchGoal,
                                          const Pose6& lastSent,
                                          const Pose6& actualTcp) {
    ServoPCommandPlan out;
    if(!servoPFinitePose(latestTouchGoal)||!servoPFinitePose(lastSent)||
       !servoPFinitePose(actualTcp)) {
        out.disposition=ServoPCommandDisposition::Invalid;
        return out;
    }
    if(checkServoPWorkspace(lastSent).code!=ServoPWorkspaceCode::Allowed) {
        out.disposition=ServoPCommandDisposition::Invalid;
        return out;
    }
    Pose6 next=latestTouchGoal;
    out.heldAxes=holdOutwardServoPAxes(next,lastSent);
    if(checkServoPWorkspace(next).code!=ServoPWorkspaceCode::Allowed) {
        out.disposition=ServoPCommandDisposition::Invalid;
        return out;
    }
    const Vec3 delta={next[0]-lastSent[0],
                      next[1]-lastSent[1],
                      next[2]-lastSent[2]};
    const double d=norm(delta);
    if(!std::isfinite(d)) {
        out.disposition=ServoPCommandDisposition::Invalid;
        return out;
    }
    if(d<=kServoPCommandDeadbandMm)return out;
    if(d>kServoPCommandMaxJumpMm) {
        const double t=kServoPCommandMaxJumpMm/d;
        for(int i=0;i<3;++i)next[i]=lastSent[i]+delta[i]*t;
        out.jumpRestricted=true;
    }
    // Restrict the *candidate* to the current tracking envelope. Do not
    // use actual TCP as an input to the operator's requested trajectory.
    const double candidateLag=servoPCartesianDistance(next,actualTcp);
    if(!std::isfinite(candidateLag)) {
        out.disposition=ServoPCommandDisposition::Invalid;
        return out;
    }
    if(candidateLag>kServoPCommandLeadMm) {
        out.leadRestricted=true;
        const double previousLag=servoPCartesianDistance(lastSent,actualTcp);
        if(previousLag>kServoPCommandLeadMm) {
            // Once the measured TCP has fallen behind, permit only true
            // corrective movement, never an outward target or an old goal.
            if(candidateLag>=previousLag-0.25) {
                out.disposition=ServoPCommandDisposition::TrackingHold;
                return out;
            }
        } else {
            // Furthest point ON THE SEGMENT lastSent -> candidate within
            // the feedback envelope. This avoids popping to actual+lead,
            // which could change direction and invalidate the Touch intent.
            const Vec3 e={lastSent[0]-actualTcp[0],
                          lastSent[1]-actualTcp[1],
                          lastSent[2]-actualTcp[2]};
            const Vec3 v={next[0]-lastSent[0],
                          next[1]-lastSent[1],
                          next[2]-lastSent[2]};
            const double a=dot(v,v);
            const double b=2.0*dot(e,v);
            const double c=dot(e,e)-
                kServoPCommandLeadMm*kServoPCommandLeadMm;
            const double discriminant=b*b-4.0*a*c;
            if(a<=0.0||!std::isfinite(discriminant)||discriminant<0.0) {
                out.disposition=ServoPCommandDisposition::Invalid;
                return out;
            }
            const double t=std::clamp((-b+std::sqrt(discriminant))/(2.0*a),
                                      0.0,1.0);
            for(int i=0;i<3;++i)next[i]=lastSent[i]+v[i]*t;
        }
    }
    // Corrective commands are allowed to reduce tracking lag, including
    // after the robot has moved unexpectedly (still below the hard stop).
    out.heldAxes|=holdOutwardServoPAxes(next,lastSent);
    const double advanced=servoPCartesianDistance(next,lastSent);
    if(!std::isfinite(advanced)||
       checkServoPWorkspace(next).code!=ServoPWorkspaceCode::Allowed) {
        out.disposition=ServoPCommandDisposition::Invalid;
        return out;
    }
    if(advanced<=kServoPCommandDeadbandMm) {
        out.disposition=out.leadRestricted
            ? ServoPCommandDisposition::TrackingHold
            : ServoPCommandDisposition::None;
        return out;
    }
    // Pose orientation comes from the captured robot RPY, not the actual
    // 8891 RPY at each tick; it must remain unchanged in this function.
    out.disposition=ServoPCommandDisposition::Send;
    out.target=next;
    return out;
}
} // namespace touchpanel::teleop
