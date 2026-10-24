#pragma once
#include "TeleopGeometry.h"
#include <array>
#include <cmath>
namespace touchpanel::teleop {
// Read-only position joystick: one signed Touch axis per robot axis.
// Inputs 0..5 mean +X,+Y,+Z,-X,-Y,-Z, respectively.
struct ManualAxisMap {
    std::array<int,3> output{0,1,2};
    double scale=0.25;
    double deadzoneMm=1.5;
};
// Keep a 1:1 source-axis assignment when the operator changes a combo.
// Selecting an axis already used by another robot output swaps its previous
// mapping into that output, retaining the previous sign.
inline std::array<int,3> swapManualAxisSelection(
    const std::array<int,3>& previous,int changedRobotAxis,int selectedCode) {
    if(changedRobotAxis<0||changedRobotAxis>=3 ||
       selectedCode<0||selectedCode>5)return previous;
    std::array<int,3> next=previous;
    const int displaced=previous[changedRobotAxis];
    for(int other=0;other<3;++other) {
        if(other!=changedRobotAxis && previous[other]%3==selectedCode%3) {
            next[other]=displaced;
            break;
        }
    }
    next[changedRobotAxis]=selectedCode;
    return next;
}
inline bool validManualAxisMap(const ManualAxisMap& m) {
    if(!std::isfinite(m.scale)||m.scale<0.05||m.scale>2.0||
       !std::isfinite(m.deadzoneMm)||m.deadzoneMm<0||m.deadzoneMm>10)return false;
    bool used[3]={false,false,false};
    for(int code:m.output) {
        if(code<0||code>5||used[code%3])return false;
        used[code%3]=true;
    }
    return true;
}
inline bool manualPositionPreview(const ManualAxisMap& map,Vec3 touchDelta,
                                  const Pose6& robotAnchor,Pose6& target) {
    if(!validManualAxisMap(map)||!finite(touchDelta))return false;
    for(double v:robotAnchor)if(!std::isfinite(v))return false;
    target=robotAnchor; // Rotation remains locked.
    for(int axis=0;axis<3;++axis) {
        const int code=map.output[axis];
        const double input=touchDelta[code%3]*(code>=3?-1.0:1.0);
        // Sign-symmetric continuous deadband.
        const double adjusted=std::copysign(std::max(0.0,std::abs(input)-map.deadzoneMm),input);
        target[axis]=robotAnchor[axis]+map.scale*adjusted;
    }
    return true;
}
} // namespace touchpanel::teleop
