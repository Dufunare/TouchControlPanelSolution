#include "../shared/ServoPCommandPlanner.h"
#include "../shared/RelativeServoP.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace touchpanel::teleop;
static bool near(double a,double b,double tol=1e-7) {
    return std::abs(a-b)<=tol;
}
static void ready(RelativeServoP& c,Pose6 p,Vec3 h,
                  std::uint64_t start) {
    for(std::uint64_t k=start;k<start+3;++k)
        c.tick(false,h,p,true,k,ManualAxisMap{});
    assert(c.phase()==ServoPPhase::Ready);
    assert(c.tick(true,h,p,true,start+3,ManualAxisMap{}).action==
           ServoPAction::Captured);
}
int main() {
    const Pose6 origin={0,-200,200,175,1.0,17};
    Pose6 goal=origin;
    goal[0]=100;
    // New latest goal causes an immediate bounded target, not 2mm/tick.
    auto plan=planServoPCommand(goal,origin,origin);
    assert(plan.disposition==ServoPCommandDisposition::Send);
    assert(near(plan.target[0],20.0));
    assert(plan.jumpRestricted);
    assert(near(plan.target[3],175));
    const Pose6 sent=plan.target;
    // With unchanged feedback, DO NOT walk a queue of stale waypoints
    // beyond the feedback envelope.
    plan=planServoPCommand(goal,sent,origin);
    assert(plan.disposition==ServoPCommandDisposition::TrackingHold);
    // When the measured robot has moved, send a new current-goal segment.
    Pose6 actual=origin; actual[0]=10.0;
    plan=planServoPCommand(goal,sent,actual);
    assert(plan.disposition==ServoPCommandDisposition::Send);
    assert(near(plan.target[0],30.0));
    // Operator reverses: new goal takes priority immediately.
    Pose6 reverse=origin;
    plan=planServoPCommand(reverse,plan.target,actual);
    assert(plan.disposition==ServoPCommandDisposition::Send);
    assert(plan.target[0]<30.0);
    // A large outward Y goal cannot starve a legal tangent X.
    const Pose6 wallStart={12,-349.97,200,175,1,17};
    Pose6 wallGoal=wallStart;
    wallGoal[0]+=15;
    wallGoal[1]=-900;
    plan=planServoPCommand(wallGoal,wallStart,wallStart);
    assert(plan.disposition==ServoPCommandDisposition::Send);
    assert(plan.heldAxes & 2U);
    assert(near(plan.target[1],wallStart[1]));
    assert(plan.target[0]>wallStart[0]+10);
    assert(checkServoPWorkspace(plan.target).code==ServoPWorkspaceCode::Allowed);
    // Reject a nonfinite input and an already unsafe last-sent pose.
    Pose6 bad=goal; bad[1]=std::numeric_limits<double>::quiet_NaN();
    assert(planServoPCommand(bad,origin,origin).disposition==
           ServoPCommandDisposition::Invalid);
    bad=origin;bad[1]=-351;
    assert(planServoPCommand(goal,bad,origin).disposition==
           ServoPCommandDisposition::Invalid);

    RelativeServoP hand;
    assert(hand.setDirectResponseEnabled(true));
    assert(hand.setVirtualWallSlideEnabled(true));
    ManualAxisMap m;
    m.scale=2.0;
    const Vec3 clutch={5,10,20};
    ready(hand,origin,clutch,10);
    assert(!hand.setDirectResponseEnabled(false)); // cannot switch while active
    auto t=hand.tick(true,{15,10,20},origin,false,14,m);
    assert(t.action==ServoPAction::Send);
    assert(t.target[0]-origin[0]>19.0);
    assert(t.target[0]-origin[0]<=kServoPCommandLeadMm+1e-7);
    assert(hand.acknowledgeSentTarget(t.target));
    assert(hand.tick(true,{15,10,20},origin,false,15,m).action==
           ServoPAction::None); // stationary Touch, no stale-goal replay
    // Fresh Touch input moves right up to the safety-envelope boundary;
    // with no new actual TCP progress, further packets are withheld.
    t=hand.tick(true,{45,10,20},origin,false,16,m);
    assert(t.action==ServoPAction::Send);
    assert(near(t.target[0],20.0));
    assert(hand.acknowledgeSentTarget(t.target));
    t=hand.tick(true,{45,10,20},origin,false,17,m);
    assert(t.action==ServoPAction::TrackingHold);
    actual=origin;actual[0]=10;
    t=hand.tick(true,{45,10,20},actual,false,18,m);
    assert(t.action==ServoPAction::Send);
    assert(t.target[0]>hand.lastAcceptedTarget()[0]);
    assert(hand.acknowledgeSentTarget(t.target));
    // Release and move Touch to a physical limit: zero new packets.
    t=hand.tick(false,{200,10,20},actual,false,19,m);
    assert(t.action==ServoPAction::Released);
    assert(hand.tick(true,{200,10,20},actual,true,20,m).action==
           ServoPAction::None);
    Pose6 nextRobot=origin;nextRobot[0]=30;
    for(std::uint64_t k=21;k<24;++k)
        hand.tick(false,{0,0,0},nextRobot,true,k,m);
    assert(hand.phase()==ServoPPhase::Ready);
    assert(hand.tick(true,{0,0,0},nextRobot,true,24,m).action==
           ServoPAction::Captured);
    assert(hand.tick(true,{0,0,0},nextRobot,false,25,m).action==
           ServoPAction::None);
    assert(hand.anchorRobot()==nextRobot);
    assert(hand.anchorTouch()==(Vec3{0,0,0}));
    std::cout<<"ServoP latest-Touch planning and re-clutch passed\n";
}
