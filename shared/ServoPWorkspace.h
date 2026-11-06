#pragma once
#include <array>
#include <cmath>

namespace touchpanel::teleop {

// These are the control panel's legacy experimental XYZ limits, not a
// certified robot workspace. A requested target must never be silently
// clamped onto a boundary.
inline constexpr std::array<double,3> kServoPWorkspaceMinimum{-350.0,-350.0,50.0};
inline constexpr std::array<double,3> kServoPWorkspaceMaximum{ 650.0, 350.0,500.0};

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
