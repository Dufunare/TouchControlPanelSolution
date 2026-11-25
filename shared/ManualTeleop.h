#pragma once
#include "TeleopGeometry.h"
#include <array>
#include <cmath>
namespace touchpanel::teleop {
// Higher gain is opt-in for direct Cartesian ServoP. ServoJ retains its own
// conservative <=2x guard in the Qt entrypoint.
inline constexpr double kManualTeleopMaximumScale=4.0;
// Pure device-coordinate position mapping: one signed Touch axis per
// robot axis. This function produces intent only; it grants no motion.
// Inputs 0..5 mean +X,+Y,+Z,-X,-Y,-Z, respectively.
struct ManualAxisMap {
    std::array<int,3> output{0,5,1}; // master: X<-Touch X, Y<--Touch Z, Z<-Touch Y
    double scale=1.0;
    double deadzoneMm=0.3;
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
    if(!std::isfinite(m.scale)||m.scale<0.05||m.scale>kManualTeleopMaximumScale||
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

// Fixed-device-axis rotation preview, using the SAME signed permutation
// as translation. M may be a reflection; M*R*M^T still has determinant +1.
// Rotation is 1:1 in angle, independent of the translation scale.
// Z-Y-X RPY is the project's preview convention, not a verified firmware
// convention. No motion command is authorized by this pure math helper.
inline Mat3 manualAxisMatrix(const ManualAxisMap& map) {
    Mat3 m{};
    if(!validManualAxisMap(map))return m;
    for(int i=0;i<3;++i)
        m[3*i+map.output[i]%3]=map.output[i]>=3?-1.0:1.0;
    return m;
}
inline bool fixedAxisPosePreview(const ManualAxisMap& map,const Clutch& anchor,
                                 Vec3 touchPosition,Mat3 touchRotation,
                                 Vec3 referenceRpy,Pose6& target) {
    if(!anchor.valid || !validManualAxisMap(map) || !finite(referenceRpy) ||
       !properRotation(anchor.touchRotation) || !properRotation(touchRotation) ||
       !manualPositionPreview(map,sub(touchPosition,anchor.touchPosition),
                              anchor.robotPose,target))return false;
    const auto m=manualAxisMatrix(map);
    const auto delta=mul(touchRotation,transpose(anchor.touchRotation));
    const auto robotDelta=mul(mul(m,delta),transpose(m));
    const auto start=rpyToMatrix({anchor.robotPose[3],anchor.robotPose[4],
                                  anchor.robotPose[5]});
    const auto desired=mul(robotDelta,start);
    Vec3 rpy{};
    if(!matrixToRpy(desired,referenceRpy,rpy))return false;
    // Consider the second equivalent ZYX Euler representation as well.
    // Keep the branch closest to the preceding preview to avoid a 180deg
    // representational flip when the captured RPY uses the other branch.
    Vec3 other={rpy[0]+180.0,180.0-rpy[1],rpy[2]+180.0};
    for(int i=0;i<3;++i)other[i]=unwrapNear(other[i],referenceRpy[i]);
    if(norm(sub(other,referenceRpy))<norm(sub(rpy,referenceRpy)))rpy=other;
    if(!finite(rpy) || rotationDistanceDeg(rpyToMatrix(rpy),desired)>0.1)
        return false;
    for(int i=0;i<3;++i)target[i+3]=rpy[i];
    return true;
}
} // namespace touchpanel::teleop
