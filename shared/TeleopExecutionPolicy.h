#pragma once
#include "ManualTeleop.h"
#include "RobotTelemetry.h"
#include <algorithm>
#include <cmath>

namespace touchpanel::teleop {
struct StepProposal {
    Pose6 pose{};
    double distanceMm=0;
};
inline bool idleForExperimentalStep(const RobotSnapshot& s,int user,int tool) {
    const auto gate=evaluateRobotGate(s,true,user,tool,150,350);
    if(!gate.canPreview || !finiteSix(s.tcpVelocity) || !finiteSix(s.jointVelocity))
        return false;
    for(int i=0;i<3;++i)if(std::abs(s.tcpVelocity[i])>2.0)return false;
    for(double v:s.jointVelocity)if(std::abs(v)>1.0)return false;
    return true;
}
inline bool proposeSmallPositionStep(const RobotSnapshot& s,
                                     const Pose6& desired,
                                     const Pose6& clutchAnchor,
                                     int user,int tool,
                                     StepProposal& result) {
    if(!idleForExperimentalStep(s,user,tool)||!finiteSix(desired)||
       !finiteSix(clutchAnchor))return false;
    const Vec3 here={s.tcpPose[0],s.tcpPose[1],s.tcpPose[2]};
    const Vec3 wanted={desired[0],desired[1],desired[2]};
    const Vec3 anchor={clutchAnchor[0],clutchAnchor[1],clutchAnchor[2]};
    const Vec3 error=sub(wanted,here);
    const double len=norm(error);
    if(!std::isfinite(len)||len<0.4)return false;
    const Vec3 next=add(here,scale(error,std::min(len,1.0)/len));
    if(norm(sub(next,anchor))>15.0)return false;
    result.pose=s.tcpPose;
    for(int i=0;i<3;++i)result.pose[i]=next[i];
    result.distanceMm=norm(sub(next,here));
    return true;
}
inline bool stepArrived(const RobotSnapshot& s,const StepProposal& step,
                        int user,int tool) {
    if(!idleForExperimentalStep(s,user,tool)||!finiteSix(step.pose))return false;
    const Vec3 error={s.tcpPose[0]-step.pose[0],
                      s.tcpPose[1]-step.pose[1],
                      s.tcpPose[2]-step.pose[2]};
    return norm(error)<=0.6;
}
inline bool acceptableJointSolution(const RobotSnapshot& s,
                                    const std::array<double,6>& reference,
                                    const std::array<double,6>& solved) {
    if(!finiteSix(reference)||!finiteSix(solved)||!finiteSix(s.jointPosition)||
       !s.feedbackValid||s.feedbackAgeMs<0||s.feedbackAgeMs>150)return false;
    for(int i=0;i<6;++i) {
        if(std::abs(solved[i]-s.jointPosition[i])>3.0 ||
           std::abs(solved[i]-reference[i])>8.0)return false;
    }
    return true;
}
inline std::array<double,6> limitedServoTarget(const RobotSnapshot& s,
                                               const std::array<double,6>& solved) {
    std::array<double,6> result=s.jointPosition;
    for(int i=0;i<6;++i)
        result[i]+=std::clamp(solved[i]-s.jointPosition[i],-0.18,0.18);
    return result;
}
} // namespace touchpanel::teleop
