#pragma once
#include "TeleopGeometry.h"
#include "RobotTelemetry.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

namespace touchpanel::teleop {
// Diagnostic tolerances ONLY, not physical angle/velocity/collision limits.
// V3 feedback table p.76: ActualQuaternion is [qw,qx,qy,qz].
using Quaternion4=std::array<double,4>;
inline bool quaternionToRotation(const Quaternion4& q,Mat3& result) {
    double n2=0.0;
    for(double x:q) {if(!std::isfinite(x))return false;n2+=x*x;}
    if(!std::isfinite(n2)||std::abs(n2-1.0)>0.02)return false;
    const double n=std::sqrt(n2);
    const double w=q[0]/n,x=q[1]/n,y=q[2]/n,z=q[3]/n;
    result={1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
            2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
            2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)};
    return properRotation(result);
}
inline constexpr std::array<std::string_view,6> kRotationOrders={
    "ZYX","ZXY","YZX","YXZ","XZY","XYZ"
};
inline Mat3 matrixForOrder(Vec3 angles,std::size_t order) {
    if(order>=kRotationOrders.size() || !finite(angles))return {};
    Mat3 result=identity();
    for(char axis:kRotationOrders[order]) {
        Vec3 one{};
        one[axis-'X']=angles[axis-'X'];
        result=mul(result,rpyToMatrix(one));
    }
    return result;
}
inline bool defaultFrameOrientation(const RobotSnapshot& s,Mat3& measured) {
    // The V3 table does not explicitly specify the quaternion frame.
    // Limit this diagnostic to default user/tool and report mismatch.
    return s.userIndex==0 && s.toolIndex==0 && s.actualQuaternionValid &&
           finiteSix(s.tcpPose) && quaternionToRotation(s.actualQuaternion,measured);
}
inline double feedbackZyxErrorDeg(const RobotSnapshot& s) {
    Mat3 measured{};
    if(!defaultFrameOrientation(s,measured))
        return std::numeric_limits<double>::infinity();
    return rotationDistanceDeg(rpyToMatrix({s.tcpPose[3],s.tcpPose[4],s.tcpPose[5]}),measured);
}
enum class OrientationCapture { Added, Duplicate, Invalid, Full };
struct OrientationEvidenceReport {
    unsigned count=0;
    std::array<double,6> worstError{};
    std::size_t bestOrder=0, runnerUp=1;
    bool unique=false;
};
class OrientationConventionAudit {
public:
    void clear() {*this=OrientationConventionAudit{};}
    std::uint64_t session() const {return m_session;}
    OrientationEvidenceReport report() const {
        auto out=OrientationEvidenceReport{};
        out.count=m_count;out.worstError=m_worst;
        for(std::size_t i=0;i<m_worst.size();++i)
            if(m_worst[i]<m_worst[out.bestOrder])out.bestOrder=i;
        out.runnerUp=out.bestOrder==0?1:0;
        for(std::size_t i=0;i<m_worst.size();++i)
            if(i!=out.bestOrder && m_worst[i]<m_worst[out.runnerUp])out.runnerUp=i;
        // Three genuinely distinct poses AND a distinguishable runner-up.
        // Single-axis poses alone may agree with multiple orders.
        out.unique=m_count>=3 && m_worst[out.bestOrder]<=0.5 &&
                   m_worst[out.runnerUp]>=2.0;
        return out;
    }
    OrientationCapture capture(const RobotSnapshot& s) {
        if(m_session && m_session!=s.sessionEpoch)clear();
        const auto gate=evaluateRobotGate(s,true,0,0,150,350);
        Mat3 measured{};
        if(!gate.canPreview || !defaultFrameOrientation(s,measured))
            return OrientationCapture::Invalid;
        for(double v:s.jointVelocity)if(std::abs(v)>1.0)return OrientationCapture::Invalid;
        for(int i=0;i<3;++i)if(std::abs(s.tcpVelocity[i])>2.0)return OrientationCapture::Invalid;
        if(s.stateSeq<=m_lastSeq)return OrientationCapture::Duplicate;
        m_session=s.sessionEpoch;m_lastSeq=s.stateSeq;
        // Compare even duplicate poses for mismatch; do not count them
        // toward diverse evidence. A bad sample cannot improve a report.
        const Vec3 a={s.tcpPose[3],s.tcpPose[4],s.tcpPose[5]};
        for(std::size_t i=0;i<m_worst.size();++i)
            m_worst[i]=std::max(m_worst[i],rotationDistanceDeg(matrixForOrder(a,i),measured));
        for(unsigned i=0;i<m_count;++i)
            if(rotationDistanceDeg(m_samples[i],measured)<5.0)return OrientationCapture::Duplicate;
        if(m_count>=m_samples.size())return OrientationCapture::Full;
        m_samples[m_count++]=measured;
        return OrientationCapture::Added;
    }
    // Manual evidence is optional for live following. A missing or
    // inconclusive audit is not a veto; an observed contradiction in
    // this session remains a veto. Live quaternion checks are separate.
    bool hasZyxMismatch(std::uint64_t session) const {
        return session!=0 && session==m_session && m_lastSeq!=0 &&
               m_worst[0]>0.5;
    }
    bool supportsZyx(const RobotSnapshot& s) const {
        const auto r=report();
        return r.unique && r.bestOrder==0 && m_session!=0 &&
               m_session==s.sessionEpoch && feedbackZyxErrorDeg(s)<=0.5 &&
               evaluateRobotGate(s,true,0,0,150,350).canPreview;
    }
private:
    std::uint64_t m_session=0,m_lastSeq=0;
    unsigned m_count=0;
    std::array<Mat3,32> m_samples{};
    std::array<double,6> m_worst{};
};
} // namespace touchpanel::teleop
