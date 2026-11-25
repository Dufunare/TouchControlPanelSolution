#include "../shared/FixedAxisServoP.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace touchpanel::teleop;
static bool near(double a,double b) {return std::abs(a-b)<1e-7;}

static void arm(FixedAxisServoP& c,const Pose6& robot,Vec3 hand,
                const ManualAxisMap& map,std::uint64_t first) {
    assert(c.tick(false,hand,robot,true,first,map).action==ServoPAction::None);
    assert(c.tick(false,hand,robot,true,first,map).action==ServoPAction::None);
    assert(c.tick(false,hand,robot,true,first+1,map).action==ServoPAction::None);
    assert(c.tick(false,hand,robot,true,first+2,map).action==ServoPAction::Ready);
    assert(c.tick(true,hand,robot,true,first+3,map).action==ServoPAction::Captured);
}
int main() {
    const Pose6 robot={10,-200,300,175,2,17};
    const Vec3 hand={100,-70,30};
    const Vec3 delta={1,2,3};
    std::array<int,3> perm{0,1,2};
    unsigned cases=0;
    // All 6 axis permutations and 8 sign choices, with an exact 2x
    // coordinate relationship in the unsaturated local command envelope.
    do {
        for(unsigned signs=0;signs<8;++signs) {
            FixedAxisServoP c;
            ManualAxisMap m;
            m.scale=2.0;m.deadzoneMm=0;
            for(int i=0;i<3;++i)m.output[i]=perm[i]+((signs&(1U<<i))?3:0);
            arm(c,robot,hand,m,10);
            assert(c.tick(true,hand,robot,false,14,m).action==ServoPAction::None);
            auto t=c.tick(true,add(hand,delta),robot,false,15,m);
            assert(t.intentValid && t.action==ServoPAction::Send);
            for(int i=0;i<3;++i) {
                const double expected=robot[i]+2*delta[perm[i]]*((signs&(1U<<i))?-1:1);
                assert(near(t.intent[i],expected));
                assert(near(t.target[i],expected));
            }
            for(int i=3;i<6;++i)assert(near(t.target[i],robot[i]));
            assert(c.acknowledgeSentTarget(t.target));
            ++cases;
        }
    } while(std::next_permutation(perm.begin(),perm.end()));
    assert(cases==48);

    FixedAxisServoP c;
    ManualAxisMap map;map.scale=2;map.deadzoneMm=0;
    arm(c,robot,hand,map,100);
    auto t=c.tick(true,add(hand,delta),robot,false,104,map);
    assert(t.action==ServoPAction::Send);
    assert(c.acknowledgeSentTarget(t.target));
    const Pose6 second={12,-205,306,175,2,17};
    const Vec3 relocated={-130,90,-60};
    assert(c.tick(false,relocated,second,false,105,map).action==ServoPAction::Released);
    assert(c.anchorRobot()==Pose6{});
    assert(c.anchorTouch()==Vec3{});
    // Early re-press is rejected once, not queued to capture later.
    assert(c.tick(true,relocated,second,false,106,map).action==ServoPAction::RegripBlocked);
    assert(c.tick(true,{100,100,100},second,true,107,map).action==ServoPAction::None);
    assert(c.phase()==ServoPPhase::WaitingForIdle);
    assert(c.tick(false,relocated,second,true,108,map).action==ServoPAction::None);
    assert(c.tick(false,relocated,second,true,109,map).action==ServoPAction::None);
    assert(c.tick(false,relocated,second,true,110,map).action==ServoPAction::Ready);
    assert(c.tick(true,relocated,second,true,111,map).action==ServoPAction::Captured);
    assert(c.anchorRobot()==second && c.anchorTouch()==relocated);
    assert(c.tick(true,relocated,second,false,112,map).action==ServoPAction::None);
    t=c.tick(true,add(relocated,{0,0,2}),second,false,113,map);
    assert(t.action==ServoPAction::Send && t.intentValid);
    assert(near(t.intent[0],second[0]) && near(t.intent[1],second[1]-4));
    assert(near(t.intent[2],second[2]));
    assert(c.acknowledgeSentTarget(t.target));
    auto changed=map;changed.scale=1;
    assert(c.tick(true,relocated,second,false,114,changed).action==ServoPAction::Invalid);

    // The same raw intent survives bounds; it is NOT the transmitted pose.
    FixedAxisServoP limited;
    arm(limited,robot,hand,map,200);
    t=limited.tick(true,add(hand,{100,0,0}),robot,false,204,map);
    assert(t.intentValid && near(t.intent[0],robot[0]+200));
    assert(t.saturated && t.action==ServoPAction::Send);
    assert(servoPCartesianDistance(t.target,robot)<=kServoPCommandLeadMm+1e-7);
    assert(limited.acknowledgeSentTarget(t.target));
    assert(limited.tick(true,add(hand,{100,0,0}),robot,false,205,map).action==
           ServoPAction::TrackingHold);
    FixedAxisServoP wall;
    Pose6 edge=robot;edge[1]=-349.97;
    arm(wall,edge,hand,map,300);
    t=wall.tick(true,add(hand,{0,0,10}),edge,false,304,map);
    assert(t.intentValid && (t.wallHoldAxisMask&2U) && t.action==ServoPAction::None);
    t=wall.tick(true,add(hand,{3,0,10}),edge,false,305,map);
    assert(t.action==ServoPAction::Send && (t.wallHoldAxisMask&2U));
    assert(near(t.target[0],edge[0]+6) && near(t.target[1],edge[1]));
    assert(checkServoPWorkspace(t.target).code==ServoPWorkspaceCode::Allowed);
    FixedAxisServoP invalid;
    Pose6 outside=robot;outside[1]=-413.58;
    assert(invalid.tick(false,hand,outside,true,400,map).action==ServoPAction::Invalid);
    std::cout<<"48 signed-axis maps, independent regrip and preserved command bounds passed\n";
}
