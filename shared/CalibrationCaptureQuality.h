#pragma once
// Read-only calibration acquisition criteria. Never sends robot commands.
#include "MultiPointCalibration.h"
#include <array>
#include <vector>

namespace touchpanel::teleop {
inline bool robotStationaryForCapture(const std::array<double,6>& jointVelocity,
                                     const std::array<double,6>& tcpVelocity,
                                     double maxJointSpeedDegS=0.5,
                                     double maxTcpSpeedMmS=3.0) {
    for(double v:jointVelocity)if(!std::isfinite(v)||std::abs(v)>maxJointSpeedDegS)return false;
    // TCP linear speed is in the first 3 components.
    for(int i=0;i<3;++i)
        if(!std::isfinite(tcpVelocity[i])||std::abs(tcpVelocity[i])>maxTcpSpeedMmS)return false;
    return true;
}
inline bool captureWindowStable(const std::vector<Pair>& measurements,
                                double maxTouchSpreadMm=1.5,
                                double maxRobotSpreadMm=2.0) {
    if(measurements.size()<6)return false;
    const auto& first=measurements.front();
    if(!finite(first.touch)||!finite(first.robot))return false;
    for(const auto& p:measurements) {
        if(!finite(p.touch)||!finite(p.robot)||
           norm(sub(p.touch,first.touch))>maxTouchSpreadMm||
           norm(sub(p.robot,first.robot))>maxRobotSpreadMm)return false;
    }
    return true;
}
inline Pair meanCapture(const std::vector<Pair>& measurements) {
    Pair avg{};
    if(measurements.empty())return avg;
    for(const auto& p:measurements){
        avg.touch=add(avg.touch,p.touch);
        avg.robot=add(avg.robot,p.robot);
    }
    const double w=1.0/measurements.size();
    avg.touch=scale(avg.touch,w);
    avg.robot=scale(avg.robot,w);
    return avg;
}
inline bool independentPoint(const std::vector<Pair>& training,const Pair& candidate,
                             double minTouchMm=5,double minRobotMm=5) {
    if(!finite(candidate.touch)||!finite(candidate.robot))return false;
    for(const auto& p:training)
        if(norm(sub(p.touch,candidate.touch))<minTouchMm||
           norm(sub(p.robot,candidate.robot))<minRobotMm)return false;
    return true;
}
inline double registrationResidualMm(const FitReport& fit,const Pair& pair) {
    const Vec3 mapped=add(scale(mul(fit.calibration.touchToBase,pair.touch),
                                  fit.calibration.scale),fit.translationMm);
    return norm(sub(mapped,pair.robot));
}
}
