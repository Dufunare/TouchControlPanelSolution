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
        m_releaseStopPending=false;
    }
    ServoPPhase phase() const {return m_phase;}
    const Pose6& anchorRobot() const {return m_robot0;}
    const Vec3& anchorTouch() const {return m_touch0;}
    const Pose6& lastAcceptedTarget() const {return m_lastSent;}
    const ManualAxisMap& capturedMapping() const {return m_mapping;}
    bool releaseStopPending() const {return m_releaseStopPending;}
    // An ACK alone is not proof of a physical stop. Count only subsequent
    // distinct idle feedback, and never capture while an early press is held.
    bool acknowledgeReleaseStop(std::uint64_t observedSeq) {
        if(m_phase!=ServoPPhase::WaitingForIdle || !m_releaseStopPending ||
           observedSeq==0)return false;
        m_releaseStopPending=false;
        m_idleSeq=std::max(m_idleSeq,observedSeq);
        m_idleCount=0;
        return true;
    }

    bool acknowledgeSentTarget(const Pose6& target) {
        if(m_phase!=ServoPPhase::Active || !m_pending ||
           target!=m_candidate)return false;
        m_lastSent=target;
        m_sentDuringClutch=true;
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
            // Discard all unsent motion on release, then demand a stop
            // request/ACK if this clutch ever transmitted a motion target.
            // Never treat dropping the local goal as a physical stop.
            const bool sent=m_sentDuringClutch;
            clearClutch();
            m_releaseStopPending=sent;
            out.releaseStopRequired=sent;
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
                // A premature press cannot arm automatically later.
                if(!m_repressReported) {
                    m_repressReported=true;
                    out.action=ServoPAction::RegripBlocked;
                }
            } else {
                m_repressReported=false;
            }
            // During a release stop, feedback before the ACK is not
            // eligible. During initial arming, require Button1 up.
            if(m_releaseStopPending || !idle ||
               (m_phase==ServoPPhase::NeedRelease && pressed)) {
                m_idleCount=0;
                m_idleSeq=seq;
                return out;
            }
            // A rejected early press may overlap the physical stop. Count
            // the actual idle feedback now, but require a fresh deliberate
            // release and press. Do not impose another three samples once
            // the operator has released a rejected press after confirmation.
            if(seq>m_idleSeq) {
                m_idleSeq=seq;
                if(m_idleCount<3)++m_idleCount;
            }
            if(!pressed && m_idleCount>=3) {
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
            m_previousMapped=actual;
            m_smallResidual={};
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

        // Synchronization-first: consume only NEW operator displacement.
        // Always advance the input reference, even if the command is
        // clipped, blocked or later rejected. Never accumulate the unmet
        // absolute-position error into a waypoint to chase after the hand
        // stops. The original anchored goal remains diagnostic only.
        const Vec3 inputDelta={goal[0]-m_previousMapped[0],
                               goal[1]-m_previousMapped[1],
                               goal[2]-m_previousMapped[2]};
        m_previousMapped=goal;
        const double inputNorm=norm(inputDelta);
        if(!std::isfinite(inputNorm)) {
            m_smallResidual={};
            out.action=ServoPAction::Invalid;
            return out;
        }
        // Only retain sub-deadband quantization during uninterrupted
        // same-direction movement. A pause/reversal clears it. There is
        // never a backlog of envelope-limited or workspace-blocked motion.
        if(inputNorm<=1e-7) {
            m_smallResidual={};
            return out;
        }
        if(dot(inputDelta,m_smallResidual)<=0.0)m_smallResidual={};
        const Vec3 step={inputDelta[0]+m_smallResidual[0],
                         inputDelta[1]+m_smallResidual[1],
                         inputDelta[2]+m_smallResidual[2]};
        m_smallResidual={};
        Pose6 requested=m_lastSent;
        for(int i=0;i<3;++i)requested[i]+=step[i];
        const auto command=planServoPCommand(requested,m_lastSent,actual);
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
        case ServoPCommandDisposition::None:
            // Preserve only <0.3mm of quantization, never a clipped goal.
            if(!command.heldAxes && !command.leadRestricted &&
               !command.jumpRestricted && norm(step)<=kServoPCommandDeadbandMm)
                m_smallResidual=step;
            break;
        }
        return out;
    }

private:
    void clearClutch() {
        m_touch0={};
        m_robot0={};
        m_lastSent={};
        m_candidate={};
        m_previousMapped={};
        m_smallResidual={};
        m_mapping={};
        m_pending=false;
        m_sentDuringClutch=false;
    }
    ServoPPhase m_phase=ServoPPhase::NeedRelease;
    Vec3 m_touch0{};
    Pose6 m_robot0{},m_lastSent{},m_candidate{},m_previousMapped{};
    Vec3 m_smallResidual{};
    ManualAxisMap m_mapping{};
    bool m_pending=false;
    bool m_sentDuringClutch=false;
    bool m_releaseStopPending=false;
    bool m_repressReported=false;
    std::uint64_t m_idleSeq=0;
    unsigned m_idleCount=0;
};

} // namespace touchpanel::teleop
