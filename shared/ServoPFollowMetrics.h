#pragma once
#include "ManualTeleop.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace touchpanel::teleop {

// Diagnostics only. Never authorizes motion, changes a setpoint, or bypasses
// the existing ServoP controller/workspace/telemetry stop policies.
// Observes distinct 8891 states to estimate actual TCP path speed and
// successful local TCP writes to estimate the transmitted-setpoint speed.
struct ServoPFollowWindow {
    double actualPathMm=0.0;
    double setpointPathMm=0.0;
    std::uint32_t sent=0;
    std::uint32_t leadLimited=0;
    std::uint32_t trackingHeld=0;
    std::uint32_t workspaceHeld=0;
    double latestMappedLagMm=0.0;
    double maxMappedLagMm=0.0;
    std::uint32_t mappedSamples=0;
};
class ServoPFollowMetrics {
public:
    void reset(const Pose6& actual,std::uint64_t stateSeq) {
        m_prevActual=actual;
        m_prevSeq=stateSeq;
        m_active=true;
        clearWindow();
    }
    void clearWindow() {m_window={};}
    bool active() const {return m_active;}
    const ServoPFollowWindow& window() const {return m_window;}
    void observeActual(const Pose6& actual,std::uint64_t stateSeq) {
        if(!m_active||stateSeq<=m_prevSeq||!finiteSixValues(actual))return;
        const Vec3 delta={actual[0]-m_prevActual[0],
                          actual[1]-m_prevActual[1],
                          actual[2]-m_prevActual[2]};
        const double d=norm(delta);
        if(std::isfinite(d))m_window.actualPathMm+=d;
        m_prevActual=actual;
        m_prevSeq=stateSeq;
    }
    void observeSent(const Pose6& from,const Pose6& to) {
        if(!m_active||!finiteSixValues(from)||!finiteSixValues(to))return;
        const Vec3 delta={to[0]-from[0],to[1]-from[1],to[2]-from[2]};
        const double d=norm(delta);
        if(!std::isfinite(d))return;
        m_window.setpointPathMm+=d;
        ++m_window.sent;
    }
    void observeMappedDemand(const Pose6& mapped,const Pose6& actual) {
        if(!m_active||!finiteSixValues(mapped)||!finiteSixValues(actual))return;
        const Vec3 d={mapped[0]-actual[0],mapped[1]-actual[1],mapped[2]-actual[2]};
        const double distance=norm(d);
        if(!std::isfinite(distance))return;
        m_window.latestMappedLagMm=distance;
        m_window.maxMappedLagMm=std::max(distance,m_window.maxMappedLagMm);
        ++m_window.mappedSamples;
    }
    void noteLeadLimit() {if(m_active)++m_window.leadLimited;}
    void noteTrackingHold() {if(m_active)++m_window.trackingHeld;}
    void noteWorkspaceHold() {if(m_active)++m_window.workspaceHeld;}
private:
    static bool finiteSixValues(const Pose6& p) {
        for(double v:p)if(!std::isfinite(v))return false;
        return true;
    }
    bool m_active=false;
    Pose6 m_prevActual{};
    std::uint64_t m_prevSeq=0;
    ServoPFollowWindow m_window{};
};
} // namespace touchpanel::teleop
