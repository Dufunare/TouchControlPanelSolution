#pragma once
// Least-squares similarity fit for preview-only Touch -> robot registration.
// No dependency on Eigen/Qt/robot hardware. Never authorizes physical motion.
#include "TeleopGeometry.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace touchpanel::teleop {

enum class FitIssue {
    None, TooFewPoints, TooManyPoints, NonFinite, InsufficientSpread,
    RankDeficient, InvalidScale, NumericalFailure, ExcessiveResidual
};

struct FitReport {
    FitIssue issue=FitIssue::TooFewPoints;
    Calibration calibration{};
    Vec3 translationMm{};
    double rmsMm=0.0;
    double maxResidualMm=0.0;
    double touchSpreadMm=0.0;
    std::size_t worstPoint=0;
    std::vector<double> residualsMm;
    bool accepted() const { return issue==FitIssue::None && calibration.measured; }
};

namespace detail {
inline void symmetricEigen3(double a[3][3], double values[3]) {
    for(int iteration=0;iteration<60;++iteration) {
        int p=0,q=1;
        for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)
            if(std::abs(a[i][j])>std::abs(a[p][q])){p=i;q=j;}
        if(std::abs(a[p][q])<1e-12)break;
        const double tau=(a[q][q]-a[p][p])/(2.0*a[p][q]);
        const double t=std::copysign(1.0,tau)/(std::abs(tau)+std::sqrt(1.0+tau*tau));
        const double c=1.0/std::sqrt(1.0+t*t),s=t*c;
        const double app=a[p][p],aqq=a[q][q],apq=a[p][q];
        for(int k=0;k<3;++k)if(k!=p&&k!=q){
            const double x=a[k][p],y=a[k][q];
            a[k][p]=a[p][k]=c*x-s*y;
            a[k][q]=a[q][k]=s*x+c*y;
        }
        a[p][p]=app-t*apq;a[q][q]=aqq+t*apq;
        a[p][q]=a[q][p]=0.0;
    }
    for(int i=0;i<3;++i)values[i]=a[i][i];
}
inline bool largestEigenQuaternion(double n[4][4], std::array<double,4>& q) {
    double vectors[4][4]{};
    for(int i=0;i<4;++i)vectors[i][i]=1.0;
    for(int iteration=0;iteration<96;++iteration) {
        int p=0,j=1;
        for(int i=0;i<4;++i)for(int k=i+1;k<4;++k)
            if(std::abs(n[i][k])>std::abs(n[p][j])){p=i;j=k;}
        if(std::abs(n[p][j])<1e-12)break;
        const double tau=(n[j][j]-n[p][p])/(2.0*n[p][j]);
        const double t=std::copysign(1.0,tau)/(std::abs(tau)+std::sqrt(1+tau*tau));
        const double c=1.0/std::sqrt(1+t*t),s=t*c;
        const double app=n[p][p],aqq=n[j][j],apq=n[p][j];
        for(int k=0;k<4;++k)if(k!=p&&k!=j) {
            const double x=n[k][p],y=n[k][j];
            n[k][p]=n[p][k]=c*x-s*y;
            n[k][j]=n[j][k]=s*x+c*y;
        }
        n[p][p]=app-t*apq;n[j][j]=aqq+t*apq;n[p][j]=n[j][p]=0;
        for(int k=0;k<4;++k) {
            const double x=vectors[k][p],y=vectors[k][j];
            vectors[k][p]=c*x-s*y;vectors[k][j]=s*x+c*y;
        }
    }
    int best=0;for(int i=1;i<4;++i)if(n[i][i]>n[best][best])best=i;
    for(int i=0;i<4;++i)q[i]=vectors[i][best];
    const double len=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(!std::isfinite(len)||len<1e-8)return false;
    for(double& item:q)item/=len;
    return true;
}
inline Mat3 quaternionMatrix(const std::array<double,4>& q) {
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    return {1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
            2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
            2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)};
}
} // namespace detail

inline FitReport fitSimilarity(const std::vector<Pair>& pairs,
                               std::size_t minimumPairs=8,
                               double maximumRmsMm=6.0,
                               double maximumPointErrorMm=15.0) {
    FitReport result;
    if(pairs.size()<minimumPairs || pairs.size()<6){result.issue=FitIssue::TooFewPoints;return result;}
    if(pairs.size()>64){result.issue=FitIssue::TooManyPoints;return result;}
    Vec3 touchMean{},robotMean{};
    for(const auto& p:pairs) {
        if(!finite(p.touch)||!finite(p.robot)){result.issue=FitIssue::NonFinite;return result;}
        touchMean=add(touchMean,p.touch);robotMean=add(robotMean,p.robot);
    }
    touchMean=scale(touchMean,1.0/pairs.size());
    robotMean=scale(robotMean,1.0/pairs.size());
    double scatter[3][3]{};
    double cov[3][3]{};
    double energy=0.0;
    for(const auto& p:pairs) {
        const Vec3 a=sub(p.touch,touchMean),b=sub(p.robot,robotMean);
        energy+=dot(a,a);
        for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
            scatter[i][j]+=a[i]*a[j];
            cov[i][j]+=a[i]*b[j];
        }
    }
    result.touchSpreadMm=std::sqrt(energy/pairs.size());
    if(!std::isfinite(energy)||result.touchSpreadMm<25.0) {
        result.issue=FitIssue::InsufficientSpread;return result;
    }
    double eigen[3]{};
    detail::symmetricEigen3(scatter,eigen);
    const auto range=std::minmax_element(eigen,eigen+3);
    if(*range.first<=0 || *range.first<0.003*(*range.second)){
        result.issue=FitIssue::RankDeficient;return result;
    }
    // Horn's quaternion formulation, S = sum touch_centered * robot_centered^T.
    const double sxx=cov[0][0],sxy=cov[0][1],sxz=cov[0][2];
    const double syx=cov[1][0],syy=cov[1][1],syz=cov[1][2];
    const double szx=cov[2][0],szy=cov[2][1],szz=cov[2][2];
    double n[4][4]={
        {sxx+syy+szz, syz-szy,     szx-sxz,     sxy-syx},
        {syz-szy,     sxx-syy-szz, sxy+syx,     szx+sxz},
        {szx-sxz,     sxy+syx,    -sxx+syy-szz, syz+szy},
        {sxy-syx,     szx+sxz,     syz+szy,    -sxx-syy+szz}
    };
    std::array<double,4> q{};
    if(!detail::largestEigenQuaternion(n,q)) {
        result.issue=FitIssue::NumericalFailure;return result;
    }
    const Mat3 rot=detail::quaternionMatrix(q);
    if(!properRotation(rot)){result.issue=FitIssue::NumericalFailure;return result;}
    double numerator=0;
    for(const auto& p:pairs) {
        numerator+=dot(mul(rot,sub(p.touch,touchMean)),sub(p.robot,robotMean));
    }
    const double scaleFactor=numerator/energy;
    if(!std::isfinite(scaleFactor)||scaleFactor<0.05||scaleFactor>20.0) {
        result.issue=FitIssue::InvalidScale;return result;
    }
    const Vec3 offset=sub(robotMean,scale(mul(rot,touchMean),scaleFactor));
    result.translationMm=offset;
    result.calibration.touchToBase=rot;
    result.calibration.scale=scaleFactor;
    double sumSquared=0;
    result.residualsMm.reserve(pairs.size());
    for(std::size_t i=0;i<pairs.size();++i) {
        const Vec3 prediction=add(scale(mul(rot,pairs[i].touch),scaleFactor),offset);
        const double error=norm(sub(prediction,pairs[i].robot));
        if(!std::isfinite(error)){result.issue=FitIssue::NumericalFailure;return result;}
        result.residualsMm.push_back(error);
        sumSquared+=error*error;
        if(error>result.maxResidualMm){result.maxResidualMm=error;result.worstPoint=i;}
    }
    result.rmsMm=std::sqrt(sumSquared/pairs.size());
    if(result.rmsMm>maximumRmsMm || result.maxResidualMm>maximumPointErrorMm) {
        result.issue=FitIssue::ExcessiveResidual;return result;
    }
    result.calibration.measured=true;
    result.issue=FitIssue::None;
    return result;
}
} // namespace touchpanel::teleop
