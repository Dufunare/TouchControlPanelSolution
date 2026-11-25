#pragma once
// Relative displacement registration for independent physical workspaces.
// Robot movements used to obtain these samples are taught independently.
// The solver is mathematically read-only and never grants motion permission.
#include "MultiPointCalibration.h"

namespace touchpanel::teleop {
struct RelativeMotionFit {
    FitReport training;
    Calibration calibration{};
    double zeroOffsetRmsMm=0;
    double zeroOffsetMaxMm=0;
    double holdoutRmsMm=0;
    double holdoutMaxMm=0;
    bool accepted=false;
};
inline double predictedDisplacementError(const Calibration& calibration,const Pair& p) {
    const Vec3 expected=scale(mul(calibration.touchToBase,p.touch),calibration.scale);
    return norm(sub(expected,p.robot));
}
// Each pair is (Touch END - Touch START, robot TCP END - robot TCP START).
// The last two deliberate gestures must be independent validation points.
// We never fit or store an absolute translation.
inline RelativeMotionFit fitRelativeDisplacements(const std::vector<Pair>& gestures) {
    RelativeMotionFit report;
    if(gestures.size()<10 || gestures.size()>32)return report;
    for(const Pair& p:gestures) {
        if(!finite(p.touch)||!finite(p.robot)||
           norm(p.touch)<15.0||norm(p.robot)<15.0)return report;
    }
    const std::vector<Pair> training(gestures.begin(),gestures.end()-2);
    report.training=fitSimilarity(training);
    if(!report.training.accepted())return report;
    report.calibration=report.training.calibration;
    double sum=0;
    for(const Pair& p:training) {
        const double err=predictedDisplacementError(report.calibration,p);
        if(!std::isfinite(err))return report;
        sum+=err*err;
        report.zeroOffsetMaxMm=std::max(report.zeroOffsetMaxMm,err);
    }
    report.zeroOffsetRmsMm=std::sqrt(sum/training.size());
    sum=0;
    for(std::size_t i=gestures.size()-2;i<gestures.size();++i) {
        const double err=predictedDisplacementError(report.calibration,gestures[i]);
        if(!std::isfinite(err))return report;
        sum+=err*err;report.holdoutMaxMm=std::max(report.holdoutMaxMm,err);
    }
    report.holdoutRmsMm=std::sqrt(sum/2.0);
    // Disallow the free translation in similarity fitting from disguising
    // inconsistent relative directions or scaling.
    report.accepted=report.zeroOffsetRmsMm<=6.0 &&
        report.zeroOffsetMaxMm<=15.0 &&
        report.holdoutRmsMm<=8.0 &&
        report.holdoutMaxMm<=12.0 &&
        report.calibration.scale>=0.15&&report.calibration.scale<=6.0;
    if(!report.accepted)report.calibration.measured=false;
    return report;
}
} // namespace touchpanel::teleop
