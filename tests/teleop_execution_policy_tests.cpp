#include "../shared/TeleopExecutionPolicy.h"
#include <cassert>
#include <iostream>
using namespace touchpanel;
using namespace touchpanel::teleop;
int main(){
    RobotSnapshot s;
    s.sessionEstablished=true;s.relayStatusSeen=true;s.feedbackSeen=true;
    s.relayReady=true;s.port29999Connected=true;s.port30003Connected=true;
    s.port30004Connected=true;s.feedbackValid=true;s.sessionEpoch=1;
    s.stateSeq=1;s.feedbackAgeMs=8;s.relayStatusAgeMs=8;
    s.robotMode=5;s.enabled=true;s.userIndex=0;s.toolIndex=0;
    s.tcpPose={100,100,300,170,0,-90};
    StepProposal p;
    auto desired=s.tcpPose;desired[1]+=9;
    assert(proposeSmallPositionStep(s,desired,s.tcpPose,0,0,p));
    assert(p.distanceMm<=1.0&&p.pose[1]==101);
    assert(p.pose[3]==170);
    s.running=true;assert(!proposeSmallPositionStep(s,desired,s.tcpPose,0,0,p));
    s.running=false;s.feedbackAgeMs=200;
    assert(!proposeSmallPositionStep(s,desired,s.tcpPose,0,0,p));
    s.feedbackAgeMs=8;s.tcpPose=p.pose;
    assert(stepArrived(s,p,0,0));
    s.queuedMotion=true;assert(stepArrived(s,p,0,0));
    const std::array<double,6> first={0,0,0,0,0,0};
    s.jointPosition=first;
    const std::array<double,6> solved={0.6,1,0,0,0,0};
    assert(acceptableJointSolution(s,first,solved));
    const auto target=limitedServoTarget(s,solved);
    assert(target[0]<=0.18+1e-10&&target[1]<=0.18+1e-10);
    auto bad=solved;bad[0]=40;
    assert(!acceptableJointSolution(s,first,bad));
    std::cout<<"Motion proposal and IK continuity tests passed\n";
}
