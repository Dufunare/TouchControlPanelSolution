#include "../shared/RelativeServoP.h"
#include "../shared/ServoPWorkspace.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace touchpanel::teleop;
static bool near(double a,double b,double eps=1e-9) {return std::abs(a-b)<=eps;}
static void arm(RelativeServoP& drive,const Pose6& robot,Vec3 touch,std::uint64_t first) {
    for(std::uint64_t i=first;i<first+3;++i)
        drive.tick(false,touch,robot,true,i,ManualAxisMap{});
    assert(drive.phase()==ServoPPhase::Ready);
}
int main() {
    RelativeServoP drive;
    ManualAxisMap mapping;
    assert((mapping.output==std::array<int,3>{0,5,1}));
    const Pose6 firstPose={-150,-310,300,-178,-0.5,-163};
    const Vec3 firstTouch={15,-65,-88};
    // Starting with Button1 held must never generate a motion command.
    for(std::uint64_t seq=1;seq<8;++seq)
        assert(drive.tick(true,firstTouch,firstPose,true,seq,mapping).action==ServoPAction::None);
    arm(drive,firstPose,firstTouch,8);
    auto got=drive.tick(true,firstTouch,firstPose,true,11,mapping);
    assert(got.action==ServoPAction::Captured);
    assert(drive.phase()==ServoPPhase::Active);
    // Moving the Touch zero mm generates no movement. The machine pose
    // stays at the robot's actual XYZ + actual RPY captured on the press.
    assert(drive.tick(true,firstTouch,firstPose,false,12,mapping).action==ServoPAction::None);
    got=drive.tick(true,{20,-60,-88},firstPose,false,13,mapping);
    assert(got.action==ServoPAction::Send);
    const Vec3 step={got.target[0]-firstPose[0],got.target[1]-firstPose[1],got.target[2]-firstPose[2]};
    assert(norm(step)<=kServoPMaxSetpointStepMm+1e-9);
    assert(step[0]>0&&step[2]>0&&near(step[1],0));
    assert(near(got.target[3],-178)&&near(got.target[4],-0.5)&&near(got.target[5],-163));
    // Relative X/Y/Z mapping uses original master: X, -Z, +Y.
    RelativeServoP directions;
    arm(directions,firstPose,firstTouch,100);
    directions.tick(true,firstTouch,firstPose,true,103,mapping);
    got=directions.tick(true,{15,-65,-98},firstPose,false,104,mapping);
    assert(got.action==ServoPAction::Send&&got.target[1]>firstPose[1]);
    got=directions.tick(true,{15,-75,-88},firstPose,false,105,mapping);
    assert(got.action==ServoPAction::Send&&got.target[2]<firstPose[2]);
    // Large stylus motion saturates the intended clutch target at 60mm,
    // but the actual sent target is also rate limited to 2mm per tick.
    got=drive.tick(true,{300,-65,-88},firstPose,false,14,mapping);
    assert(got.action==ServoPAction::Send&&got.saturated);
    assert(near(got.boundedDistanceMm,kServoPClutchTravelMm));
    // Release: discard the old clutch, and do not issue a command while
    // the operator relocates the stylus to a completely different place.
    got=drive.tick(false,{310,-65,-88},firstPose,false,15,mapping);
    assert(got.action==ServoPAction::Released);
    const Pose6 secondPose={-148,-305,302,176,1,-171};
    for(std::uint64_t seq=16;seq<40;++seq)
        assert(drive.tick(true,{500,100,80},secondPose,true,seq,mapping).action==ServoPAction::None);
    // Fresh release and 3 new idle state_seq observations are mandatory.
    assert(drive.tick(false,{500,100,80},secondPose,true,40,mapping).action==ServoPAction::None);
    assert(drive.tick(false,{500,100,80},secondPose,true,41,mapping).action==ServoPAction::None);
    assert(drive.tick(false,{500,100,80},secondPose,true,42,mapping).action==ServoPAction::Ready);
    got=drive.tick(true,{500,100,80},secondPose,true,43,mapping);
    assert(got.action==ServoPAction::Captured);
    assert(drive.anchorRobot()==secondPose); // not the first clutch anchor
    assert(drive.anchorTouch()==(Vec3{500,100,80}));
    assert(drive.tick(true,{500,100,80},secondPose,false,44,mapping).action==ServoPAction::None);
    got=drive.tick(true,{501,100,80},secondPose,false,45,mapping);
    assert(got.action==ServoPAction::Send);
    assert(near(got.target[0],secondPose[0]+0.7));
    for(int i=3;i<6;++i)assert(near(got.target[i],secondPose[i]));
    // A moving robot is never used as a fresh anchor.
    RelativeServoP blocked;
    arm(blocked,firstPose,firstTouch,1);
    assert(blocked.tick(true,firstTouch,firstPose,false,4,mapping).action==ServoPAction::None);
    assert(blocked.phase()==ServoPPhase::NeedRelease);
    assert(blocked.tick(true,firstTouch,firstPose,true,5,mapping).action==ServoPAction::None);
    // Corrupt Touch inputs fail closed rather than silently moving the robot.
    got=drive.tick(true,{std::numeric_limits<double>::quiet_NaN(),0,0},secondPose,false,46,mapping);
    assert(got.action==ServoPAction::Invalid);
    // Regression from real robot logs: Y=-349.79 is only 0.21mm inside
    // the legacy Y lower bound (-350). An outward request must be rejected
    // without losing the clutch or advancing an untransmitted setpoint.
    const Pose6 edgePose={-79.02,-349.79,385.78,172.05,2.41,17.69};
    const Vec3 edgeTouch={117.36,41.31,20.72};
    RelativeServoP edge;
    arm(edge,edgePose,edgeTouch,200);
    assert(edge.tick(true,edgeTouch,edgePose,true,203,mapping).action==
           ServoPAction::Captured);
    auto outside=edge.tick(true,{117.36,41.31,35.72},edgePose,false,204,mapping);
    assert(outside.action==ServoPAction::Send);
    auto check=checkServoPWorkspace(outside.target);
    assert(check.code==ServoPWorkspaceCode::XYZOutside);
    assert(check.axis==1&&near(check.minimum,-350.0));
    assert(outside.target[1]<-350.0);
    edge.rejectUnsentTarget();
    assert(edge.lastAcceptedTarget()==edgePose);
    // A second outward sample remains blocked, not an accumulated command.
    outside=edge.tick(true,{117.36,41.31,35.72},edgePose,false,205,mapping);
    assert(outside.action==ServoPAction::Send);
    assert(near(outside.target[1],edgePose[1]-kServoPMaxSetpointStepMm));
    edge.rejectUnsentTarget();
    // Reverse the Touch while STILL holding Button1; no restart or
    // ResetRobot required. The next candidate is safe and can commit.
    auto recovered=edge.tick(true,{117.36,41.31,12.72},edgePose,false,206,mapping);
    assert(recovered.action==ServoPAction::Send);
    assert(checkServoPWorkspace(recovered.target).code==ServoPWorkspaceCode::Allowed);
    assert(recovered.target[1]>edgePose[1]);
    assert(edge.acknowledgeSentTarget(recovered.target));
    assert(edge.lastAcceptedTarget()==recovered.target);
    const auto recoveredNext=edge.tick(true,{117.36,41.31,12.72},edgePose,false,207,mapping);
    assert(recoveredNext.action==ServoPAction::Send);
    assert(recoveredNext.target[1]>recovered.target[1]);
    assert(norm({recoveredNext.target[0]-recovered.target[0],
                 recoveredNext.target[1]-recovered.target[1],
                 recoveredNext.target[2]-recovered.target[2]})
           <=kServoPMaxSetpointStepMm+1e-9);
    assert(edge.acknowledgeSentTarget(recoveredNext.target));
    // An operator can opt into a larger *relative* gesture without changing
    // base coordinates or the separately enforced XYZ software workspace.
    RelativeServoP extended;
    assert(!extended.setClutchTravelLimitMm(181.0));
    assert(extended.setClutchTravelLimitMm(120.0));
    arm(extended,firstPose,firstTouch,300);
    assert(extended.tick(true,firstTouch,firstPose,true,303,mapping).action==
           ServoPAction::Captured);
    assert(!extended.setClutchTravelLimitMm(180.0)); // no live gain jump
    got=extended.tick(true,{300,-65,-88},firstPose,false,304,mapping);
    assert(got.saturated&&near(got.boundedDistanceMm,120.0));
    assert(norm({got.target[0]-firstPose[0],got.target[1]-firstPose[1],
                 got.target[2]-firstPose[2]})<=kServoPMaxSetpointStepMm+1e-9);
    // At 50ms, advance 50/33 of the nominal 2mm (not just 2mm).
    RelativeServoP delayed;
    arm(delayed,firstPose,firstTouch,400);
    assert(delayed.tick(true,firstTouch,firstPose,true,403,mapping).action==
           ServoPAction::Captured);
    got=delayed.tick(true,{100,-65,-88},firstPose,false,404,mapping,0.05);
    const double advanced=got.target[0]-firstPose[0];
    assert(got.action==ServoPAction::Send&&advanced>2.9&&advanced<3.1);
    delayed.rejectUnsentTarget();
    got=delayed.tick(true,{100,-65,-88},firstPose,false,405,mapping,0.5);
    assert(got.action==ServoPAction::Send);
    assert(got.target[0]-firstPose[0]<=4.0+1e-9); // no huge catch-up
    delayed.rejectUnsentTarget();
    // Re-grab at arbitrary Touch positions after hitting its physical edge.
    // The second press must anchor at the *current robot position*, not
    // chase a prior virtual absolute Touch position.
    RelativeServoP repeat;
    const Vec3 touchEdge{155.0,-30.0,40.0};
    arm(repeat,firstPose,touchEdge,500);
    assert(repeat.tick(true,touchEdge,firstPose,true,503,mapping).action==
           ServoPAction::Captured);
    got=repeat.tick(true,{158.0,-30.0,40.0},firstPose,false,504,mapping);
    assert(got.action==ServoPAction::Send);
    assert(repeat.acknowledgeSentTarget(got.target));
    assert(repeat.tick(false,{158.0,-30.0,40.0},firstPose,false,505,mapping).action==
           ServoPAction::Released);
    const Pose6 regrabPose={-149.0,-310.0,300.0,-178.0,-0.5,-163.0};
    const Vec3 touchCenter{0.0,0.0,0.0};
    arm(repeat,regrabPose,touchCenter,506);
    assert(repeat.tick(true,touchCenter,regrabPose,true,509,mapping).action==
           ServoPAction::Captured);
    assert(repeat.anchorRobot()==regrabPose);
    assert(repeat.anchorTouch()==touchCenter);
    assert(repeat.tick(true,touchCenter,regrabPose,false,510,mapping).action==
           ServoPAction::None);
    got=repeat.tick(true,{3.0,0.0,0.0},regrabPose,false,511,mapping);
    assert(got.action==ServoPAction::Send&&got.target[0]>regrabPose[0]);
    const Pose6 bad={-79.02,-349.79,385.78,172.05,2.41,
                     std::numeric_limits<double>::quiet_NaN()};
    assert(checkServoPWorkspace(bad).code==ServoPWorkspaceCode::NonFinite);
    assert(checkServoPWorkspace(edgePose).code==ServoPWorkspaceCode::Allowed);
    std::cout<<"Relative ServoP clutch: no-jump, reanchor, axes, RPY and bounds passed\n";
}
