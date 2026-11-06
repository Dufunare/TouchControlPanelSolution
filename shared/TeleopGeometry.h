#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace touchpanel::teleop {
constexpr double pi = 3.14159265358979323846;
using Vec3 = std::array<double,3>;
using Mat3 = std::array<double,9>; // row-major
using Pose6 = std::array<double,6>; // X Y Z Rx Ry Rz (mm, degrees)
inline Mat3 identity() { return {1,0,0,0,1,0,0,0,1}; }
inline Vec3 add(Vec3 a,Vec3 b) { return {a[0]+b[0],a[1]+b[1],a[2]+b[2]}; }
inline Vec3 sub(Vec3 a,Vec3 b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
inline Vec3 scale(Vec3 a,double s) { return {a[0]*s,a[1]*s,a[2]*s}; }
inline double dot(Vec3 a,Vec3 b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline Vec3 cross(Vec3 a,Vec3 b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline double norm(Vec3 v) { return std::sqrt(dot(v,v)); }
inline bool finite(Vec3 v) { return std::isfinite(v[0])&&std::isfinite(v[1])&&std::isfinite(v[2]); }
inline bool finite(Mat3 m) { for(double v:m)if(!std::isfinite(v))return false;return true; }
inline Vec3 unit(Vec3 a) { return scale(a,1.0/norm(a)); }
inline Mat3 transpose(Mat3 a) { return {a[0],a[3],a[6],a[1],a[4],a[7],a[2],a[5],a[8]}; }
inline Mat3 mul(Mat3 a,Mat3 b) { Mat3 c{}; for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)c[i*3+j]+=a[i*3+k]*b[k*3+j];return c; }
inline Vec3 mul(Mat3 a,Vec3 v) { return {dot({a[0],a[1],a[2]},v),dot({a[3],a[4],a[5]},v),dot({a[6],a[7],a[8]},v)}; }
inline Mat3 columns(Vec3 x,Vec3 y,Vec3 z) { return {x[0],y[0],z[0],x[1],y[1],z[1],x[2],y[2],z[2]}; }
inline double det(Mat3 a) { return dot({a[0],a[3],a[6]},cross({a[1],a[4],a[7]},{a[2],a[5],a[8]})); }
inline bool properRotation(Mat3 a) {
    if(!finite(a)||std::abs(det(a)-1.0)>0.03)return false;
    Mat3 p=mul(transpose(a),a);Mat3 e=identity();
    for(int k=0;k<9;++k)if(std::abs(p[k]-e[k])>0.02)return false;
    return true;
}
inline Mat3 openHapticsRotation(const std::array<double,16>& t) {
    // HD_CURRENT_TRANSFORM is column-major, not an OpenGL row-major matrix.
    return {t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]};
}
inline Mat3 fixedAxisMap() { return {1,0,0,0,0,-1,0,1,0}; }
inline Mat3 rpyToMatrix(Vec3 degrees) {
    // Z-Y-X fixed-axis convention; validate the controller's convention on a simulator.
    double a=degrees[0]*pi/180.0,b=degrees[1]*pi/180.0,c=degrees[2]*pi/180.0;
    double sx=std::sin(a),cx=std::cos(a),sy=std::sin(b),cy=std::cos(b),sz=std::sin(c),cz=std::cos(c);
    return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
            sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,
            -sy,cy*sx,cy*cx};
}
inline double unwrapNear(double angle,double reference) {
    while(angle-reference>180.0)angle-=360.0;
    while(angle-reference< -180.0)angle+=360.0;
    return angle;
}
inline bool matrixToRpy(Mat3 r,Vec3 reference,Vec3& out) {
    if(!properRotation(r))return false;
    double pitch=std::asin(std::clamp(-r[6],-1.0,1.0));
    if(std::abs(std::cos(pitch))<0.035)return false; // reject gimbal singularity
    Vec3 xyz={std::atan2(r[7],r[8])*180.0/pi,pitch*180.0/pi,
              std::atan2(r[3],r[0])*180.0/pi};
    for(int i=0;i<3;++i)out[i]=unwrapNear(xyz[i],reference[i]);
    return finite(out);
}
inline double rotationDistanceDeg(Mat3 a,Mat3 b) {
    Mat3 relative=mul(a,transpose(b));
    return std::acos(std::clamp((relative[0]+relative[4]+relative[8]-1.0)/2.0,-1.0,1.0))*180.0/pi;
}
struct Pair { Vec3 touch; Vec3 robot; };
inline bool basis(Vec3 p0,Vec3 p1,Vec3 p2,Mat3& result) {
    Vec3 x=sub(p1,p0),other=sub(p2,p0);
    if(!finite(x)||!finite(other)||norm(x)<5.0)return false;
    x=unit(x);Vec3 y=sub(other,scale(x,dot(other,x)));
    if(norm(y)<5.0)return false;
    y=unit(y);Vec3 z=cross(x,y);
    result=columns(x,y,z);return properRotation(result);
}
struct Calibration { Mat3 touchToBase=fixedAxisMap(); double scale=1.0; bool measured=false; };
inline bool calibrateThreePairs(const std::array<Pair,3>& points,Calibration& out) {
    Mat3 h{},r{};
    if(!basis(points[0].touch,points[1].touch,points[2].touch,h)||
       !basis(points[0].robot,points[1].robot,points[2].robot,r))return false;
    Vec3 ht=sub(points[1].touch,points[0].touch),rt=sub(points[1].robot,points[0].robot);
    Vec3 ht2=sub(points[2].touch,points[0].touch),rt2=sub(points[2].robot,points[0].robot);
    double a=norm(rt)/norm(ht),b=norm(rt2)/norm(ht2);
    if(!std::isfinite(a)||!std::isfinite(b)||a<0.05||a>20.0||
       std::abs(a-b)>0.15*std::max<double>(a,b))return false;
    Calibration c;c.touchToBase=mul(r,transpose(h));c.scale=(a+b)/2.0;c.measured=true;
    if(!properRotation(c.touchToBase))return false;
    out=c;return true;
}
struct Clutch { Vec3 touchPosition{}; Mat3 touchRotation=identity(); Pose6 robotPose{}; bool valid=false; };
inline bool mapClutchedPose(const Calibration& calib,const Clutch& anchor,
                            Vec3 touchPosition,Mat3 touchRotation,Pose6& output) {
    if(!anchor.valid||!finite(touchPosition)||!properRotation(touchRotation)||
       !properRotation(anchor.touchRotation)||!properRotation(calib.touchToBase)||
       !std::isfinite(calib.scale)||calib.scale<=0.0)return false;
    Vec3 robotOrigin={anchor.robotPose[0],anchor.robotPose[1],anchor.robotPose[2]};
    Vec3 delta=scale(mul(calib.touchToBase,sub(touchPosition,anchor.touchPosition)),calib.scale);
    Vec3 p=add(robotOrigin,delta);
    Mat3 handDelta=mul(touchRotation,transpose(anchor.touchRotation));
    Mat3 worldDelta=mul(mul(calib.touchToBase,handDelta),transpose(calib.touchToBase));
    Mat3 r0=rpyToMatrix({anchor.robotPose[3],anchor.robotPose[4],anchor.robotPose[5]});
    Vec3 reference={anchor.robotPose[3],anchor.robotPose[4],anchor.robotPose[5]},rpy{};
    if(!matrixToRpy(mul(worldDelta,r0),reference,rpy))return false;
    output={p[0],p[1],p[2],rpy[0],rpy[1],rpy[2]};return true;
}
struct Bounds { Vec3 min{},max{}; bool configured=false; };
inline bool insideWorkspace(const Bounds& b,const Pose6& pose) {
    if(!b.configured)return false;
    for(int i=0;i<3;++i)if(!std::isfinite(pose[i])||pose[i]<b.min[i]||pose[i]>b.max[i])return false;
    return true;
}
struct JointLimits {
    std::array<double,6> lower{},upper{};
    double maxDeviationDeg=10.0;
    bool configured=false;
};
inline bool withinJointLimits(const JointLimits& limits,const std::array<double,6>& target,
                              const std::array<double,6>& actual) {
    if(!limits.configured||!std::isfinite(limits.maxDeviationDeg)||limits.maxDeviationDeg<=0)return false;
    for(int i=0;i<6;++i) {
        if(!std::isfinite(target[i])||!std::isfinite(actual[i])||
           limits.lower[i]>=limits.upper[i]||target[i]<limits.lower[i]||target[i]>limits.upper[i]||
           std::abs(target[i]-actual[i])>limits.maxDeviationDeg)return false;
    }
    return true;
}
inline bool smallStep(const Pose6& a,const Pose6& b,double maxMm,double maxDegrees) {
    Vec3 dp={a[0]-b[0],a[1]-b[1],a[2]-b[2]};
    Mat3 ra=rpyToMatrix({a[3],a[4],a[5]}),rb=rpyToMatrix({b[3],b[4],b[5]});
    return norm(dp)<=maxMm&&rotationDistanceDeg(ra,rb)<=maxDegrees;
}
} // namespace touchpanel::teleop
