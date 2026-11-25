#include "../shared/ManualTeleop.h"
#include <cassert>
#include <iostream>
using namespace touchpanel::teleop;
int main() {
    ManualAxisMap m;
    // Mirror the working master ServoP direction convention by default.
    assert((m.output==std::array<int,3>{0,5,1}));
    assert(m.scale==1.0&&m.deadzoneMm==0.3);
    Pose6 source={0,0,0,0,0,0};
    Pose6 initial{};
    assert(manualPositionPreview(m,{10,20,30},source,initial));
    assert(initial[0]==9.7 && initial[1]==-29.7 && initial[2]==19.7);
    m.output={0,1,2};
    m.scale=0.25;m.deadzoneMm=1.5;
    Pose6 base={100,200,300,10,20,30}, out{};
    assert(validManualAxisMap(m));
    assert(manualPositionPreview(m,{21,0,0},base,out));
    assert(out[0]==104.875);
    assert(out[3]==10&&out[4]==20&&out[5]==30);
    m.output={3,1,2};
    assert(manualPositionPreview(m,{21,0,0},base,out));
    assert(out[0]==95.125);
    m.output={0,3,2};
    assert(!validManualAxisMap(m));
    m.output={0,1,2};m.scale=4.0;
    assert(validManualAxisMap(m));
    assert(manualPositionPreview(m,{11.5,0,0},base,out));
    assert(out[0]==140.0); // 4*(11.5-1.5) beyond the Touch displacement
    m.scale=4.01;
    assert(!validManualAxisMap(m));
    m.scale=100;
    assert(!validManualAxisMap(m));

    // Regression for the Qt combo boxes: defaults MUST be accepted, and
    // choosing a conflicting Touch source must swap rather than reject.
    std::array<int,3> selection{0,1,2};
    selection=swapManualAxisSelection(selection,0,4); // Robot X <- -Touch Y
    assert((selection==std::array<int,3>{4,0,2}));
    m.output=selection;m.scale=0.25;
    assert(validManualAxisMap(m));
    selection=swapManualAxisSelection(selection,2,1); // Robot Z <- +Touch Y
    assert((selection==std::array<int,3>{2,0,1}));
    m.output=selection;
    assert(validManualAxisMap(m));
    selection=swapManualAxisSelection(selection,1,3); // Robot Y <- -Touch X
    assert((selection==std::array<int,3>{2,3,1}));
    m.output=selection;
    assert(validManualAxisMap(m));
    const auto unchanged=swapManualAxisSelection(selection,1,99);
    assert(unchanged==selection);
    // Flipping the sign of the same source axis must not disturb others.
    assert((swapManualAxisSelection(selection,1,0)==std::array<int,3>{2,0,1}));
    // Fixed-axis 6DoF preview, with the actual rotation matrix composed
    // in robot coordinates rather than subtracting Euler angles.
    ManualAxisMap sixMap;
    sixMap.scale=2.0;sixMap.deadzoneMm=0;
    Clutch anchor;anchor.valid=true;
    anchor.touchPosition={10,20,30};
    anchor.touchRotation=rpyToMatrix({15,5,-20});
    anchor.robotPose={100,-100,200,175,3,20};
    Pose6 sixPose{};
    const Vec3 refRpy={175,3,20};
    assert(fixedAxisPosePreview(sixMap,anchor,anchor.touchPosition,
        anchor.touchRotation,refRpy,sixPose));
    for(int i=0;i<6;++i)assert(std::abs(sixPose[i]-anchor.robotPose[i])<1e-5);
    const auto rotated=mul(rpyToMatrix({0,0,10}),anchor.touchRotation);
    assert(fixedAxisPosePreview(sixMap,anchor,add(anchor.touchPosition,{2,3,4}),
        rotated,refRpy,sixPose));
    assert(std::abs(sixPose[0]-104)<1e-6 && std::abs(sixPose[1]+108)<1e-6 &&
           std::abs(sixPose[2]-206)<1e-6);
    const auto baseRotation=rpyToMatrix(refRpy);
    auto mapped=rpyToMatrix({sixPose[3],sixPose[4],sixPose[5]});
    assert(rotationDistanceDeg(mapped,mul(rpyToMatrix({0,-10,0}),baseRotation))<1e-5);
    // Reflection of a polar axis is not the same as reflecting an axial
    // rotation vector; conjugation keeps a proper right-handed rotation.
    sixMap.output={0,1,5};
    assert(fixedAxisPosePreview(sixMap,anchor,anchor.touchPosition,
        rotated,refRpy,sixPose));
    mapped=rpyToMatrix({sixPose[3],sixPose[4],sixPose[5]});
    assert(rotationDistanceDeg(mapped,mul(rpyToMatrix({0,0,10}),baseRotation))<1e-5);
    assert(!fixedAxisPosePreview(sixMap,anchor,anchor.touchPosition,
        Mat3{},refRpy,sixPose));
    // Alternate Euler branch stays continuous around an already captured
    // valid non-principal pitch.
    anchor.robotPose={100,-100,200,10,120,20};
    const Vec3 otherRef={10,120,20};
    assert(fixedAxisPosePreview(sixMap,anchor,anchor.touchPosition,
        anchor.touchRotation,otherRef,sixPose));
    for(int i=0;i<6;++i)assert(std::abs(sixPose[i]-anchor.robotPose[i])<1e-5);
    std::cout<<"Manual mapping preview tests passed\n";
}
