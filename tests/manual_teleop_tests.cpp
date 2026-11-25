#include "../shared/ManualTeleop.h"
#include "../shared/OrientationConventionAudit.h"
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
    // Independent closed-form Hamilton quaternion for Rz*Ry*Rx.
    auto qzyx=[](Vec3 a) {
        const double x=a[0]*pi/360,y=a[1]*pi/360,z=a[2]*pi/360;
        const double cx=std::cos(x),sx=std::sin(x),cy=std::cos(y),sy=std::sin(y),
                     cz=std::cos(z),sz=std::sin(z);
        return Quaternion4{cz*cy*cx+sz*sy*sx,cz*cy*sx-sz*sy*cx,
                           cz*sy*cx+sz*cy*sx,sz*cy*cx-cz*sy*sx};
    };
    touchpanel::RobotSnapshot state;
    state.sessionEstablished=state.relayStatusSeen=state.feedbackSeen=true;
    state.relayReady=state.port29999Connected=state.port30003Connected=state.port30004Connected=true;
    state.feedbackValid=state.enabled=true;
    state.sessionEpoch=1;state.stateSeq=1;state.robotMode=5;
    state.feedbackAgeMs=state.relayStatusAgeMs=10;
    state.userIndex=state.toolIndex=0;
    state.actualQuaternionValid=true;
    OrientationConventionAudit audit;
    auto setPose=[&](Vec3 r) {
        ++state.stateSeq;
        state.tcpPose={100,-100,300,r[0],r[1],r[2]};
        state.actualQuaternion=qzyx(r);
    };
    setPose({15,20,30});
    assert(audit.capture(state)==OrientationCapture::Added);
    assert(audit.capture(state)==OrientationCapture::Duplicate);
    ++state.stateSeq;
    for(double& x:state.actualQuaternion)x=-x;
    assert(audit.capture(state)==OrientationCapture::Duplicate); // q == -q
    assert(!audit.report().unique);
    setPose({-35,18,55});
    assert(audit.capture(state)==OrientationCapture::Added);
    setPose({70,-28,-40});
    assert(audit.capture(state)==OrientationCapture::Added);
    assert(audit.report().count==3 && audit.report().unique && audit.supportsZyx(state));
    state.actualQuaternion={0,0,0,0};
    assert(audit.capture(state)==OrientationCapture::Invalid);
    assert(!audit.supportsZyx(state));
    setPose({-20,-25,70});state.toolIndex=1;
    assert(audit.capture(state)==OrientationCapture::Invalid);
    state.toolIndex=0;state.feedbackAgeMs=200;
    assert(audit.capture(state)==OrientationCapture::Invalid);
    state.feedbackAgeMs=10;state.sessionEpoch=2;
    assert(!audit.supportsZyx(state));
    assert(audit.capture(state)==OrientationCapture::Added);
    assert(audit.report().count==1 && !audit.report().unique);
    // Pure single-axis motion cannot disambiguate an Euler convention.
    audit.clear();
    for(Vec3 r:std::array<Vec3,3>{{{30,0,0},{0,35,0},{0,0,40}}}) {
        setPose(r);assert(audit.capture(state)==OrientationCapture::Added);
    }
    assert(!audit.report().unique);
    std::cout<<"Manual mapping preview tests passed\n";
}
