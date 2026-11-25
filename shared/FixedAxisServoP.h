#pragma once
#include "ManualTeleop.h"
#include "ServoPClutchTypes.h"
#include "ServoPCommandPlanner.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace touchpanel::teleop {

// One live control mode. Two independent fixed device coordinate systems.
// No calibration matrix, stylus rotation, GL display origin, old waypoint
// queue, rate integrator, or previous clutch origin participates in intent.
// The caller must enforce live controller/telemetry/stop gates. The command
// arbiter retains the existing experimental envelope, not a certified
// physical speed, collision, or stopping-distance guarantee.
class FixedAxisServoP {
public:
    void reset() {
        clearClutch();
        m_phase=ServoPPhase::NeedRelease;
        m_idleSeq=0;
        m_idleCount=0;
        m_repressReported=false;
    }
    ServoPPhase phase() const {return m_phase;}
    const Pose6& anchorRobot() const {return m_robot0;}
    const Vec3& anchorTouch() const {return m_touch0;}
    const Pose6& lastAcceptedTarget() const {return m_lastSent;}
    const ManualAxisMap& capturedMapping() const {return m_mapping;}

    bool acknowledgeSentTarget(const Pose6& target) {
        if(m_phase!=ServoPPhase::Active || !m_pending ||
           target!=m_candidate)return false;
        m_lastSent=target;
        m_pending=false;
        return true;
    }
    void rejectUnsentTarget() {m_pending=false;}

    ServoPTick tick(bool pressed,Vec3 touch,const Pose6& actual,
                    bool idle,std::uint64_t seq,const ManualAxisMap& mapping) {
        ServoPTick out;
        if(!finite(touch)||!validManualAxisMap(mapping)||seq==0||
           checkServoPWorkspace(actual).code!=ServoPWorkspaceCode::Allowed) {
            out.action=ServoPAction::Invalid;
            return out;
        }

        if(m_phase==ServoPPhase::Active && !pressed) {
            // Destroy all per-press coordinates and candidates immediately.
            // The physical robot may still finish the last transmitted
            // setpoint. A NEW press cannot capture until measured idle.
            clearClutch();
            m_phase=ServoPPhase::WaitingForIdle;
            m_idleCount=0;
            m_idleSeq=seq;
            m_repressReported=false;
            out.action=ServoPAction::Released;
            return out;
        }
        if(m_phase==ServoPPhase::Ready && !idle) {
            m_phase=ServoPPhase::NeedRelease;
            m_idleCount=0;
            m_idleSeq=seq;
        }
        if(m_phase==ServoPPhase::NeedRelease ||
           m_phase==ServoPPhase::WaitingForIdle) {
            if(pressed) {
                // A press made before readiness is NOT deferred and then
                // captured at a later hand position. Require a new edge.
                m_idleCount=0;
                m_idleSeq=seq;
                if(!m_repressReported) {
                    m_repressReported=true;
                    out.action=ServoPAction::RegripBlocked;
                }
                return out;
            }
            m_repressReported=false;
            if(!idle) {
                m_idleCount=0;
                m_idleSeq=seq;
                return out;
            }
            if(seq>m_idleSeq) {
                m_idleSeq=seq;
                ++m_idleCount;
            }
            if(m_idleCount>=3) {
                m_phase=ServoPPhase::Ready;
                out.action=ServoPAction::Ready;
            }
            return out;
        }
        if(m_phase==ServoPPhase::Ready) {
            if(!pressed)return out;
            m_touch0=touch;
            m_robot0=actual;
            m_mapping=mapping; // immutable throughout this press
            m_lastSent=actual;
            m_pending=false;
            m_phase=ServoPPhase::Active;
            out.action=ServoPAction::Captured;
            return out; // capture alone never sends a command
        }

        // Never reinterpret a held clutch using a new gain/axis assignment.
        if(mapping.output!=m_mapping.output || mapping.scale!=m_mapping.scale ||
           mapping.deadzoneMm!=m_mapping.deadzoneMm || m_pending) {
            out.action=ServoPAction::Invalid;
            return out;
        }
        Pose6 goal{};
        if(!manualPositionPreview(m_mapping,sub(touch,m_touch0),m_robot0,goal) ||
           !servoPFinitePose(goal)) {
            out.action=ServoPAction::Invalid;
            return out;
        }
        out.intent=goal;
        out.intentValid=true;
        out.rawDistanceMm=servoPCartesianDistance(goal,m_robot0);
        if(!std::isfinite(out.rawDistanceMm)) {
            out.action=ServoPAction::Invalid;
            return out;
        }

        // The clutch anchor defines an origin, not a spherical workspace.
        // Do not saturate the intent by distance from that origin. Apply
        // only the existing independent command/absolute-workspace guard.
        // Retain the raw intent for diagnostics; no change to the per-command
        // jump, feedback lead, hard tracking stop or hardware fault gates.
        const auto command=planServoPCommand(goal,m_lastSent,actual);
        out.wallHoldAxisMask|=command.heldAxes;
        out.feedbackLimited=command.leadRestricted;
        switch(command.disposition) {
        case ServoPCommandDisposition::Invalid:
            out.action=ServoPAction::Invalid;break;
        case ServoPCommandDisposition::TrackingHold:
            out.action=ServoPAction::TrackingHold;break;
        case ServoPCommandDisposition::Send:
            m_candidate=command.target;
            m_pending=true;
            out.target=m_candidate;
            out.action=ServoPAction::Send;break;
        case ServoPCommandDisposition::None:break;
        }
        return out;
    }

private:
    void clearClutch() {
        m_touch0={};
        m_robot0={};
        m_lastSent={};
        m_candidate={};
        m_mapping={};
        m_pending=false;
    }
    ServoPPhase m_phase=ServoPPhase::NeedRelease;
    Vec3 m_touch0{};
    Pose6 m_robot0{},m_lastSent{},m_candidate{};
    ManualAxisMap m_mapping{};
    bool m_pending=false;
    bool m_repressReported=false;
    std::uint64_t m_idleSeq=0;
    unsigned m_idleCount=0;
};

} // namespace touchpanel::teleop
