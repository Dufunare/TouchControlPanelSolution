#include "../shared/CalibrationCaptureQuality.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace touchpanel::teleop;
int main(){
    const std::array<double,6> quiet={0,0,0,0,0,0};
    assert(robotStationaryForCapture(quiet,quiet));
    auto fast=quiet;fast[1]=2;assert(!robotStationaryForCapture(fast,quiet));
    fast=quiet;fast[0]=6;assert(!robotStationaryForCapture(quiet,fast));
    std::vector<Pair> frame(8,{{15,20,25},{300,50,180}});
    assert(captureWindowStable(frame));
    frame[4].touch[0]+=3;
    assert(!captureWindowStable(frame));
    frame[4].touch[0]-=3;
    const auto average=meanCapture(frame);
    assert(norm(sub(average.touch,frame[0].touch))<1e-8);
    assert(!independentPoint(frame,frame[0]));
    assert(independentPoint(frame,{{50,60,70},{400,80,210}}));
    FitReport result;
    result.calibration.touchToBase=identity();
    result.calibration.scale=1.0;
    result.translationMm={2,3,4};
    assert(registrationResidualMm(result,{{1,2,3},{3,5,7}})<1e-9);
    std::cout<<"Read-only calibration capture quality tests passed\n";
}
