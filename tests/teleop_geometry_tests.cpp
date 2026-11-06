#include "../shared/TeleopGeometry.h"
#include <cassert>
#include <iostream>
using namespace touchpanel::teleop;
int main() {
 Mat3 axes=fixedAxisMap(); assert(properRotation(axes));
 assert(mul(axes,Vec3{0,1,0})==(Vec3{0,0,1}));
 assert(mul(axes,Vec3{0,0,1})==(Vec3{0,-1,0}));
 Clutch c; c.valid=true;c.touchPosition={10,20,30};c.touchRotation=identity();c.robotPose={250,0,200,0,0,0};
 Pose6 out{};Calibration cfg;
 assert(mapClutchedPose(cfg,c,c.touchPosition,c.touchRotation,out)&&out==c.robotPose);
 assert(mapClutchedPose(cfg,c,{20,20,30},identity(),out)&&std::abs(out[0]-260)<1e-9);
 assert(mapClutchedPose(cfg,c,c.touchPosition,rpyToMatrix({0,0,30}),out));
 assert(rotationDistanceDeg(rpyToMatrix({out[3],out[4],out[5]}),mul(mul(axes,rpyToMatrix({0,0,30})),transpose(axes)))<1e-5);
 Vec3 xyz{};assert(matrixToRpy(rpyToMatrix({22,-25,37}),{22,-25,37},xyz));
 assert(!matrixToRpy(rpyToMatrix({0,90,0}),{},xyz));
 std::array<Pair,3> points{{{{0,0,0},{0,0,0}},{{100,0,0},{200,0,0}},{{0,100,0},{0,200,0}}}};
 Calibration registered;assert(calibrateThreePairs(points,registered)&&registered.measured);
 assert(std::abs(registered.scale-2)<1e-8);
 points[2].touch={50,0,0};assert(!calibrateThreePairs(points,registered));
 Bounds b; b.configured=true;b.min={-300,-300,50};b.max={500,300,400};
 assert(insideWorkspace(b,{100,0,200,0,0,0}));
 assert(!insideWorkspace(b,{100,0,10,0,0,0}));
 JointLimits j; j.configured=true;j.lower.fill(-180);j.upper.fill(180);
 std::array<double,6> q{};assert(withinJointLimits(j,q,q));q[0]=200;assert(!withinJointLimits(j,q,{}));
 assert(smallStep({0,0,0,0,0,0},{1,0,0,1,0,0},2,2));
 assert(!smallStep({0,0,0,0,0,0},{8,0,0,0,0,0},2,2));
 std::cout<<"TeleopGeometry checks passed\n";
}
