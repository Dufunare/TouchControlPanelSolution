#include "../shared/RelativeServoP.h"
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
    std::cout<<"Relative ServoP clutch: no-jump, reanchor, axes, RPY and bounds passed\n";
}
