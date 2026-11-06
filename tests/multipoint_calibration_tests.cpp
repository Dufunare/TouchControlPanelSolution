#include "../shared/MultiPointCalibration.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace touchpanel::teleop;
int main() {
    const Mat3 trueRot=rpyToMatrix({17,-23,41});
    constexpr double trueScale=1.65;
    const Vec3 trueOffset={240,-110,310};
    const Vec3 points[]={{0,0,0},{100,0,0},{0,100,0},{0,0,100},
        {80,90,40},{-50,80,120},{100,-50,60},{-80,-50,-60},
        {40,20,150},{-20,120,-40},{-110,30,90},{70,-90,-70}};
    std::vector<Pair> pairs;
    for(std::size_t i=0;i<sizeof(points)/sizeof(points[0]);++i) {
        Vec3 mapped=add(scale(mul(trueRot,points[i]),trueScale),trueOffset);
        // Synthetic deterministic sub-millimetre measurements.
        mapped[0]+=0.3*std::sin(double(i));
        mapped[1]+=0.3*std::cos(double(i));
        mapped[2]+=0.2*std::sin(2.0*double(i));
        pairs.push_back({points[i],mapped});
    }
    const auto good=fitSimilarity(pairs);
    assert(good.accepted());
    assert(good.calibration.measured);
    assert(std::abs(good.calibration.scale-trueScale)<0.02);
    assert(rotationDistanceDeg(good.calibration.touchToBase,trueRot)<0.5);
    assert(norm(sub(good.translationMm,trueOffset))<1.0);
    assert(good.rmsMm<1.0 && good.maxResidualMm<1.5);
    assert(good.residualsMm.size()==pairs.size());
    auto partial=pairs;partial.resize(7);
    assert(fitSimilarity(partial).issue==FitIssue::TooFewPoints);
    auto outlier=pairs;outlier[5].robot[0]+=90.0;
    assert(fitSimilarity(outlier).issue==FitIssue::ExcessiveResidual);
    auto flat=pairs;for(auto& p:flat)p.touch[2]=0.0;
    assert(fitSimilarity(flat).issue==FitIssue::RankDeficient);
    auto line=pairs;for(std::size_t i=0;i<line.size();++i)line[i].touch={double(i)*15,0,0};
    assert(fitSimilarity(line).issue==FitIssue::RankDeficient);
    auto nan=pairs;nan[2].robot[1]=std::numeric_limits<double>::quiet_NaN();
    assert(fitSimilarity(nan).issue==FitIssue::NonFinite);
    auto small=pairs;for(auto& p:small)p.touch=scale(p.touch,0.01);
    assert(fitSimilarity(small).issue==FitIssue::InsufficientSpread);
    std::cout<<"Multi-point similarity and residual checks passed\n";
}
