#include "../shared/ServoPFlowGuard.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace touchpanel;
using namespace touchpanel::teleop;

int main() {
    RobotSnapshot s;
    s.sessionEstablished=true;s.sessionEpoch=8;
    s.relayStatusSeen=true;s.relayReady=true;
    s.port29999Connected=true;s.port30003Connected=true;
    s.port30004Connected=true;
    s.feedbackSeen=true;s.feedbackValid=true;
    s.feedbackAgeMs=263;s.relayStatusAgeMs=386;
    s.userIndex=0;s.toolIndex=0;s.robotMode=7;s.enabled=true;
    assert(servoPRecoverableFeedbackGap(s,8,0,0));
    // Observed independent 269/280ms spikes receive the SAME no-send hold.
    s.feedbackAgeMs=269;assert(servoPRecoverableFeedbackGap(s,8,0,0));
    s.feedbackAgeMs=280;assert(servoPRecoverableFeedbackGap(s,8,0,0));
    // No immunity for sustained stale feedback, lost ports, mode changes,
    // errors, a new session or invalid feedback.
    s.feedbackAgeMs=401;assert(!servoPRecoverableFeedbackGap(s,8,0,0));
    s.feedbackAgeMs=263;
    s.port30003Connected=false;assert(!servoPRecoverableFeedbackGap(s,8,0,0));
    s.port30003Connected=true;
    s.error=true;assert(!servoPRecoverableFeedbackGap(s,8,0,0));
    s.error=false;
    s.sessionEpoch=9;assert(!servoPRecoverableFeedbackGap(s,8,0,0));
    s.sessionEpoch=8;
    s.feedbackValid=false;assert(!servoPRecoverableFeedbackGap(s,8,0,0));
    s.feedbackValid=true;s.feedbackAgeMs=249;
    assert(!servoPRecoverableFeedbackGap(s,8,0,0));

    // The observed 25.70mm was a *candidate* (not yet transmitted).
    // Reject only the new candidate; do not ResetRobot for that reason.
    assert(servoPTrackingDecision(12.0,25.70,false,0)==
           ServoPTrackingDecision::Hold);
    assert(servoPTrackingDecision(15.0,17.0,false,0)==
           ServoPTrackingDecision::Hold);
    assert(servoPTrackingDecision(17.0,15.5,true,250)==
           ServoPTrackingDecision::Send);
    assert(servoPTrackingDecision(12.0,15.0,true,250)==
           ServoPTrackingDecision::Hold); // hysteresis until <=11mm
    assert(servoPTrackingDecision(10.5,14.0,true,250)==
           ServoPTrackingDecision::Send);
    assert(servoPTrackingDecision(19.0,17.0,true,250)==
           ServoPTrackingDecision::Send); // user reverses direction
    assert(servoPTrackingDecision(17.0,18.0,true,250)==
           ServoPTrackingDecision::Hold);
    // With 20mm feedback lookahead, the old 16mm soft gate must not
    // stop every outgoing target. 25mm hard lag still ends the motion.
    assert(servoPTrackingDecision(10.0,20.0,false,0,20.0)==
           ServoPTrackingDecision::Send);
    assert(servoPTrackingDecision(10.0,21.0,false,0,20.0)==
           ServoPTrackingDecision::Hold);
    assert(servoPTrackingDecision(25.01,18.0,false,0,20.0)==
           ServoPTrackingDecision::Abort);
    assert(servoPTrackingDecision(10.0,12.0,false,0,21.0)==
           ServoPTrackingDecision::Abort); // invalid configuration
    assert(servoPTrackingDecision(9.0,9.5,true,600)==
           ServoPTrackingDecision::Send);
    assert(servoPTrackingDecision(22.0,24.0,true,1600)==
           ServoPTrackingDecision::Abort); // persistent lag
    assert(servoPTrackingDecision(25.01,20.0,false,0)==
           ServoPTrackingDecision::Abort); // actual transmitted target lag
    assert(servoPTrackingDecision(9.0,
           std::numeric_limits<double>::quiet_NaN(),false,0)==
           ServoPTrackingDecision::Abort);
    std::cout<<"ServoP feedback grace and tracking backpressure passed\n";
}
