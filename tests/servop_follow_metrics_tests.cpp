#include "../shared/ServoPFollowMetrics.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace touchpanel::teleop;
static bool near(double a,double b) {return std::abs(a-b)<1e-9;}
int main() {
    ServoPFollowMetrics m;
    Pose6 a={0,0,100,1,2,3};
    Pose6 b={3,4,100,1,2,3};
    Pose6 c={3,12,100,1,2,3};
    assert(!m.active());
    m.reset(a,100);
    m.observeActual(b,101);
    m.observeActual(c,101); // same 8891 state_seq must NOT double count
    assert(near(m.window().actualPathMm,5.0));
    m.observeActual(c,102);
    assert(near(m.window().actualPathMm,13.0));
    m.observeSent(a,b);
    m.observeSent(b,c);
    assert(m.window().sent==2);
    assert(near(m.window().setpointPathMm,13.0));
    m.noteLeadLimit();
    m.noteLeadLimit();
    m.noteTrackingHold();
    m.noteWorkspaceHold();
    assert(m.window().leadLimited==2);
    assert(m.window().trackingHeld==1);
    assert(m.window().workspaceHeld==1);
    // Clearing a 2s window preserves the previous robot pose & state_seq.
    m.clearWindow();
    assert(m.window().sent==0&&near(m.window().actualPathMm,0));
    Pose6 d={3,13,100,1,2,3};
    m.observeActual(d,103);
    assert(near(m.window().actualPathMm,1.0));
    Pose6 bad=d;bad[0]=std::numeric_limits<double>::quiet_NaN();
    m.observeActual(bad,104);
    assert(near(m.window().actualPathMm,1.0));
    m.observeSent(a,bad);
    assert(m.window().sent==0);
    std::cout<<"ServoP follow metrics tests passed\n";
}
