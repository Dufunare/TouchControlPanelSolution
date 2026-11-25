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
    t=c.tick(false,relocated,second,false,105,map);
    assert(t.action==ServoPAction::Released && t.releaseStopRequired);
    assert(c.releaseStopPending());
    assert(c.anchorRobot()==Pose6{});
    assert(c.anchorTouch()==Vec3{});
    // Early re-press and even fresh idle before ACK cannot rearm.
    assert(c.tick(true,relocated,second,false,106,map).action==ServoPAction::RegripBlocked);
    assert(c.tick(true,{100,100,100},second,true,107,map).action==ServoPAction::None);
    for(std::uint64_t seq=108;seq<=110;++seq)
        assert(c.tick(false,relocated,second,true,seq,map).action==ServoPAction::None);
    assert(c.phase()==ServoPPhase::WaitingForIdle);
    assert(c.acknowledgeReleaseStop(110));
    assert(!c.acknowledgeReleaseStop(110));
    // A rejected press can overlap stop confirmation but never move.
    assert(c.tick(true,relocated,second,true,111,map).action==ServoPAction::RegripBlocked);
    assert(c.tick(true,relocated,second,true,111,map).action==ServoPAction::None);
    assert(c.tick(true,relocated,second,true,112,map).action==ServoPAction::None);
    assert(c.tick(true,relocated,second,true,113,map).action==ServoPAction::None);
    assert(c.phase()==ServoPPhase::WaitingForIdle);
    // Once three distinct post-ACK idle states are observed, release
    // immediately makes READY; a fresh subsequent press captures.
    assert(c.tick(false,relocated,second,true,113,map).action==ServoPAction::Ready);
    assert(c.tick(true,relocated,second,true,114,map).action==ServoPAction::Captured);
    assert(c.anchorRobot()==second && c.anchorTouch()==relocated);
    assert(c.tick(true,relocated,second,false,115,map).action==ServoPAction::None);
    t=c.tick(true,add(relocated,{0,0,2}),second,false,116,map);
    assert(t.action==ServoPAction::Send && t.intentValid);
    assert(near(t.intent[0],second[0]) && near(t.intent[1],second[1]-4));
    assert(near(t.intent[2],second[2]));
    assert(c.acknowledgeSentTarget(t.target));
    auto changed=map;changed.scale=1;
    assert(c.tick(true,relocated,second,false,117,changed).action==ServoPAction::Invalid);

    // The same raw intent survives bounds; it is NOT the transmitted pose.
    FixedAxisServoP limited;
    arm(limited,robot,hand,map,200);
    t=limited.tick(true,add(hand,{100,0,0}),robot,false,204,map);
    assert(t.intentValid && near(t.intent[0],robot[0]+200));
    assert(t.action==ServoPAction::Send);
    assert(servoPCartesianDistance(t.target,robot)<=kServoPCommandLeadMm+1e-7);
    assert(limited.acknowledgeSentTarget(t.target));
    // Stationary hand produces NO catch-up command, even if the raw
    // anchored goal is 200mm away and measured TCP later moves.
    const Pose6 firstSent=t.target;
    assert(limited.tick(true,add(hand,{100,0,0}),robot,false,205,map).action==
           ServoPAction::None);
    assert(limited.tick(true,add(hand,{101,0,0}),robot,false,206,map).action==
           ServoPAction::TrackingHold);
    assert(limited.tick(true,add(hand,{101,0,0}),firstSent,false,207,map).action==
           ServoPAction::None);
    t=limited.tick(true,add(hand,{102,0,0}),firstSent,false,208,map);
    assert(t.action==ServoPAction::Send && near(t.target[0],firstSent[0]+2));
    assert(limited.acknowledgeSentTarget(t.target));
    t=limited.tick(true,add(hand,{101,0,0}),firstSent,false,209,map);
    assert(t.action==ServoPAction::Send && near(t.target[0],firstSent[0]));
    assert(limited.acknowledgeSentTarget(t.target));

    // No lost release edge when the button is repressed before a GUI tick.
    t=limited.tick(true,hand,firstSent,false,210,map,true);
    assert(t.action==ServoPAction::Released && t.releaseStopRequired);
    assert(limited.tick(true,hand,firstSent,true,211,map).action==
           ServoPAction::RegripBlocked);
    assert(limited.releaseStopPending());

    // A clutch with no transmitted motion must not request ResetRobot.
    FixedAxisServoP noMotion;
    arm(noMotion,robot,hand,map,600);
    t=noMotion.tick(false,hand,robot,true,604,map);
    assert(t.action==ServoPAction::Released && !t.releaseStopRequired);
    assert(!noMotion.releaseStopPending());
    assert(noMotion.tick(false,hand,robot,true,605,map).action==ServoPAction::None);
    assert(noMotion.tick(false,hand,robot,true,606,map).action==ServoPAction::None);
    assert(noMotion.tick(false,hand,robot,true,607,map).action==ServoPAction::Ready);

    // Same-direction fine movement crosses the existing duplicate filter.
    // A stationary sample discards the sub-deadband residual.
    FixedAxisServoP fine;
    auto unit=map;unit.scale=1;
    arm(fine,robot,hand,unit,700);
    assert(fine.tick(true,add(hand,{0.15,0,0}),robot,false,704,unit).action==ServoPAction::None);
    t=fine.tick(true,add(hand,{0.35,0,0}),robot,false,705,unit);
    assert(t.action==ServoPAction::Send && near(t.target[0],robot[0]+0.35));
    assert(fine.acknowledgeSentTarget(t.target));
    assert(fine.tick(true,add(hand,{0.5,0,0}),robot,false,706,unit).action==ServoPAction::None);
    assert(fine.tick(true,add(hand,{0.5,0,0}),robot,false,707,unit).action==ServoPAction::None);
    assert(fine.tick(true,add(hand,{0.7,0,0}),robot,false,708,unit).action==ServoPAction::None);
    // Regression: continue the SAME clutch beyond both the old 60mm
    // default and 180mm maximum. Model measured TCP following each
    // accepted setpoint; never change the existing per-command envelope.
    FixedAxisServoP longClutch;
    arm(longClutch,robot,hand,map,500);
    Pose6 followed=robot;
    for(unsigned step=1;step<=30;++step) {
        t=longClutch.tick(true,add(hand,{5.0*step,0,0}),followed,
                          false,503+step,map);
        assert(t.intentValid && t.action==ServoPAction::Send);
        assert(near(t.target[0],robot[0]+10.0*step));
        assert(servoPCartesianDistance(t.target,followed)<=kServoPCommandLeadMm+1e-7);
        assert(checkServoPWorkspace(t.target).code==ServoPWorkspaceCode::Allowed);
        assert(longClutch.acknowledgeSentTarget(t.target));
        followed=t.target;
    }
    assert(near(followed[0]-robot[0],300.0));
    assert(longClutch.anchorRobot()==robot);
    assert(longClutch.anchorTouch()==hand);
    assert(longClutch.phase()==ServoPPhase::Active);
    assert(longClutch.tick(false,hand,followed,false,534,map).action==
           ServoPAction::Released);

    FixedAxisServoP wall;
    Pose6 edge=robot;edge[1]=-349.97;
    arm(wall,edge,hand,map,300);
    t=wall.tick(true,add(hand,{0,0,10}),edge,false,304,map);
    assert(t.intentValid && (t.wallHoldAxisMask&2U) && t.action==ServoPAction::None);
    t=wall.tick(true,add(hand,{3,0,10}),edge,false,305,map);
    assert(t.action==ServoPAction::Send && !(t.wallHoldAxisMask&2U));
    assert(near(t.target[0],edge[0]+6) && near(t.target[1],edge[1]));
    assert(checkServoPWorkspace(t.target).code==ServoPWorkspaceCode::Allowed);
    assert(wall.acknowledgeSentTarget(t.target));
    // Reversing by 1mm moves inward immediately. It is not necessary
    // to backtrack the entire discarded outward hand displacement.
    t=wall.tick(true,add(hand,{3,0,9}),edge,false,306,map);
    assert(t.action==ServoPAction::Send && near(t.target[1],edge[1]+2));
    assert(checkServoPWorkspace(t.target).code==ServoPWorkspaceCode::Allowed);
    FixedAxisServoP invalid;
    Pose6 outside=robot;outside[1]=-413.58;
    assert(invalid.tick(false,hand,outside,true,400,map).action==ServoPAction::Invalid);
    std::cout<<"48 signed-axis maps, sync/no catch-up, release-stop ACK, regrip, continuous travel and preserved command bounds passed\n";
}
