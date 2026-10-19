#include "../shared/RobotTelemetry.h"
#include <cassert>
#include <iostream>
#include <string>

using namespace touchpanel;
static RobotSnapshot healthy() {
    RobotSnapshot s;
    s.sessionEstablished=true;s.sessionEpoch=3;
    s.relayStatusSeen=true;s.relayReady=true;
    s.port29999Connected=s.port30003Connected=s.port30004Connected=true;
    s.relayStatusAgeMs=20;s.feedbackAgeMs=20;s.feedbackSeen=s.feedbackValid=true;
    s.stateSeq=50;s.robotMode=5;s.enabled=true;s.userIndex=0;s.toolIndex=0;
    return s;
}
int main() {
    auto s=healthy();
    auto d=evaluateRobotGate(s,true,0,0);
    assert(d.reason==GateReason::Ok && d.canPreview && d.canCaptureCalibration);
    assert(!d.canSendRobotMotion);
    s.jogging=true;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RobotNotIdle);
    assert(idleGateDiagnostics(s).find("Jog=yes")!=std::string::npos);
    s.jogging=false;
    assert(std::string(gateReasonChinese(GateReason::RobotDragging)).find("拖拽")!=std::string::npos);
    assert(evaluateRobotGate(s).reason==GateReason::NoVerifiedCalibration);
    s.error=true;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RobotError);
    s.error=false;s.dragging=true;
    assert(!evaluateRobotGate(s,true,0,0).canPreview);
    s.dragging=false;s.robotMode=7;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RobotNotIdle);
    s.robotMode=5;s.enabled=false;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RobotDisabled);
    s.enabled=true;s.feedbackAgeMs=401;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RobotFeedbackStale);
    s.feedbackAgeMs=20;s.relayStatusAgeMs=751;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RelayStatusStale);
    s.relayStatusAgeMs=20;s.port30004Connected=false;
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::RelayNotReady);
    s.port30004Connected=true;s.jointPosition[1]=std::numeric_limits<double>::quiet_NaN();
    assert(evaluateRobotGate(s,true,0,0).reason==GateReason::InvalidFeedback);
    s.jointPosition[1]=0;
    assert(evaluateRobotGate(s,true,1,0).reason==GateReason::CoordinateMismatch);
    s.robotMode=6;s.dragging=true;
    assert(evaluateRobotGate(s,true,0,0).canCaptureCalibration);
    assert(!evaluateRobotGate(s,true,0,0).canSendRobotMotion);
    std::cout<<"Robot telemetry gate checks passed\n";
}
