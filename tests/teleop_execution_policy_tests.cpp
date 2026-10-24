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
    // Regression: relay_status publishes at a 200ms heartbeat, whereas
    // actual robot_state feedback is much faster. At 501/511ms metadata
    // age with fresh motion feedback, the controller must not stop a
    // moving robot merely because one or two heartbeats were delayed.
    assert(continuousTelemetryAgesHealthy(s));
    s.relayStatusAgeMs=501;
    assert(continuousTelemetryAgesHealthy(s));
    s.relayStatusAgeMs=511;
    assert(continuousTelemetryAgesHealthy(s));
    s.relayStatusAgeMs=750;
    assert(continuousTelemetryAgesHealthy(s));
    s.relayStatusAgeMs=751;
    assert(!continuousTelemetryAgesHealthy(s));
    s.relayStatusAgeMs=8;
    s.feedbackAgeMs=251;
    assert(!continuousTelemetryAgesHealthy(s));
    s.feedbackAgeMs=8;
    s.relayReady=false;
    assert(!continuousTelemetryAgesHealthy(s));
    s.relayReady=true;
    s.port30003Connected=false;
    assert(!continuousTelemetryAgesHealthy(s));
    s.port30003Connected=true;
    s.sessionEstablished=false;
    assert(!continuousTelemetryAgesHealthy(s));
    s.sessionEstablished=true;
    StepProposal p;
    auto desired=s.tcpPose;desired[1]+=9;
    assert(proposeSmallPositionStep(s,desired,s.tcpPose,0,0,p));
    assert(p.distanceMm<=3.0&&p.pose[1]==103);
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
    assert(target[0]<=0.35+1e-10&&target[1]<=0.35+1e-10);
    auto bad=solved;bad[0]=40;
    assert(!acceptableJointSolution(s,first,bad));
    // A large physical Touch gesture must NOT force ResetRobot on the first
    // tick. Saturate the target at the verified clutch envelope instead.
    ClutchBound bound;
    Pose6 origin={100,200,300,-170,0,90};
    Pose6 fast=origin;fast[0]+=100;fast[1]+=40;fast[2]-=80;
    assert(boundClutchTarget(fast,origin,bound));
    assert(bound.saturated && bound.rawDistanceMm>100);
    const Vec3 limited={bound.pose[0]-origin[0],bound.pose[1]-origin[1],
                        bound.pose[2]-origin[2]};
    assert(std::abs(norm(limited)-kClutchTrialLimitMm)<1e-8);
    assert(bound.pose[3]==origin[3]&&bound.pose[4]==origin[4]);
    fast=origin;fast[1]+=5;
    assert(boundClutchTarget(fast,origin,bound));
    assert(!bound.saturated&&bound.pose==fast);
    fast=origin;fast[0]=std::numeric_limits<double>::quiet_NaN();
    assert(!boundClutchTarget(fast,origin,bound));
    fast=origin;fast[0]+=40;
    assert(!boundClutchTarget(fast,origin,bound,100.0));
    std::cout<<"Motion proposal and IK continuity tests passed\n";
}
