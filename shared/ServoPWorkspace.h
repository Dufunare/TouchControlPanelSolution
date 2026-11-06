#pragma once
#include <array>
#include <cmath>

namespace touchpanel::teleop {

// These are the control panel's legacy experimental XYZ limits, not a
// certified robot workspace. A requested target must never be silently
// clamped onto a boundary.
inline constexpr std::array<double,3> kServoPWorkspaceMinimum{-350.0,-350.0,50.0};
inline constexpr std::array<double,3> kServoPWorkspaceMaximum{ 650.0, 350.0,500.0};

// Optional virtual-wall experiment: when a mapped POSITION request crosses
// one or more XYZ software boundaries, hold ONLY those axes at the last
// successfully transmitted target and leave other axes as requested.
// Never project TO the workspace boundary, change the physical workspace,
// override an invalid last target, or authorize movement on its own.
// Caller remains responsible for feedback freshness, speed, hard tracking,
// and final checkServoPWorkspace() before any actual send.
inline unsigned holdOutwardServoPAxes(std::array<double,6>& desired,
                                      const std::array<double,6>& lastSent) {
    unsigned mask=0;
    for(int axis=0;axis<3;++axis) {
        if(!std::isfinite(desired[axis])||!std::isfinite(lastSent[axis]))
            continue; // The final validator rejects nonfinite targets.
        const double lo=kServoPWorkspaceMinimum[axis];
        const double hi=kServoPWorkspaceMaximum[axis];
        if(desired[axis]<lo||desired[axis]>hi) {
            if(lastSent[axis]<lo||lastSent[axis]>hi)
                continue; // Never launder an unsafe last-sent position.
            desired[axis]=lastSent[axis];
            mask|=1U<<axis;
        }
    }
    return mask;
}

enum class ServoPWorkspaceCode { Allowed, NonFinite, XYZOutside, RotationOutside };

struct ServoPWorkspaceResult {
    ServoPWorkspaceCode code=ServoPWorkspaceCode::Allowed;
    int axis=-1;
    double value=0.0;
    double minimum=0.0;
    double maximum=0.0;
};

inline ServoPWorkspaceResult checkServoPWorkspace(const std::array<double,6>& target) {
    for(int i=0;i<6;++i) {
        if(!std::isfinite(target[i]))
            return {ServoPWorkspaceCode::NonFinite,i,target[i],0,0};
    }
    for(int i=0;i<3;++i) {
        if(target[i]<kServoPWorkspaceMinimum[i]||
           target[i]>kServoPWorkspaceMaximum[i])
            return {ServoPWorkspaceCode::XYZOutside,i,target[i],
                    kServoPWorkspaceMinimum[i],kServoPWorkspaceMaximum[i]};
    }
    for(int i=3;i<6;++i) {
        if(std::abs(target[i])>360.0)
            return {ServoPWorkspaceCode::RotationOutside,i,target[i],-360,360};
    }
    return {};
}
} // namespace touchpanel::teleop
