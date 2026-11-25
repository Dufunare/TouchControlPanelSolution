#include "../shared/RelativeMotionCalibration.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace touchpanel::teleop;
int main() {
    const auto rotation=rpyToMatrix({14,-24,42});
    const double knownScale=1.2;
    const Vec3 gestures[]={
        {35,0,0},{0,40,0},{0,0,38},{-37,0,0},
        {0,-35,0},{0,0,-39},{28,31,17},{-32,29,-17},
        {36,-28,22},{-26,-22,31},{30,30,-30},{-35,24,28}};
    std::vector<Pair> points;
    for(const auto& p:gestures)points.push_back({p,scale(mul(rotation,p),knownScale)});
    const auto fit=fitRelativeDisplacements(points);
    assert(fit.accepted);
    assert(std::abs(fit.calibration.scale-knownScale)<0.001);
    assert(rotationDistanceDeg(fit.calibration.touchToBase,rotation)<0.05);
    assert(fit.holdoutMaxMm<0.01);
    const auto wrong=points;
    auto badHoldout=wrong;badHoldout.back().robot[0]+=70;
    assert(!fitRelativeDisplacements(badHoldout).accepted);
    auto tooFew=points;tooFew.resize(9);
    assert(!fitRelativeDisplacements(tooFew).accepted);
    auto small=points;small[0].touch={2,0,0};
    assert(!fitRelativeDisplacements(small).accepted);
    auto biased=points;
    for(auto& p:biased)p.robot[0]+=18;
    assert(!fitRelativeDisplacements(biased).accepted);
    std::cout<<"Relative displacement calibration and independent holdout passed\n";
}
