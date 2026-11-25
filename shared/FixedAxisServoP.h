#pragma once
#include "ManualTeleop.h"
#include "ServoPClutchTypes.h"
#include "ServoPCommandPlanner.h"
#include "RotationTrial.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace touchpanel::teleop {

// One live control mode. Two independent fixed device coordinate systems.
// The normal path is master-style absolute position mapping plus duplicate
// filtering. An offset changes ONLY when the existing guard discards motion.
// One shared clutch for position and orientation; the controller may also
// select translation-only. No trajectory queue or unbounded catch-up.
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
        m_sixAxis=false;
    }
    bool configureSixAxis(const Pose6& origin) {
        if(m_phase!=ServoPPhase::NeedRelease || !servoPRotationPoseValid(origin))return false;
        m_sixAxis=true;
        return true;
    }
    bool sixAxisEnabled() const {return m_sixAxis;}
    // Only before any command from the current clutch was sent. A missed
    // snapshot cannot be taken as a release or allow a delayed press to
    // capture. Discard unsent anchors and demand a new release/idle gate.
    bool discardUnsentClutch(std::uint64_t seq) {
        if(m_sentDuringClutch || m_pending || m_releaseStopPending)return false;
        const bool sixAxis=m_sixAxis;
        reset();
        m_sixAxis=sixAxis;
        m_idleSeq=seq;
        return true;
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
        // Keep the direct position law. Absorb rejected/clipped displacement
        // once, at the point of acceptance, instead of replaying it later.
        if(!m_sixAxis || m_pendingPositionAdvance)
            for(int i=0;i<3;++i)m_commandOffset[i]=target[i]-m_rawGoal[i];
        if(m_sixAxis)m_rotationTouchRef=m_rotationPendingTouch;
        m_sentDuringClutch=true;
        m_pending=false;
        return true;
    }
    void rejectUnsentTarget() {
        if(m_pending) {
            for(int i=0;i<3;++i)m_commandOffset[i]=m_lastSent[i]-m_rawGoal[i];
            if(m_sixAxis)m_rotationTouchRef=m_rotationPendingTouch;
        }
        m_pending=false;
    }

    ServoPTick tick(bool pressed,Vec3 touch,const Pose6& actual,
                    bool idle,std::uint64_t seq,const ManualAxisMap& mapping,
                    bool releaseObserved=false,
                    const Mat3& touchRotation=Mat3{},
                    const Mat3& actualRotation=Mat3{}) {
        ServoPTick out;
        if(!finite(touch)||!validManualAxisMap(mapping)||seq==0||
           checkServoPWorkspace(actual).code!=ServoPWorkspaceCode::Allowed ||
           (m_sixAxis && (!properRotation(touchRotation) || !properRotation(actualRotation) ||
               !servoPRotationPoseValid(actual)))) {
            out.action=ServoPAction::Invalid;
            return out;
        }

        if(m_phase==ServoPPhase::Active && (!pressed || releaseObserved)) {
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
            m_rawGoal=actual;
            m_rotationTouchRef=touchRotation;
            m_commandOffset={};
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

        // Master-style direct position target, not a trajectory integrator.
        // The raw goal is independent of actual TCP. The offset absorbs only
        // previously clipped/held input, so a stationary hand never chases
        // the rejected portion of a prior large displacement.
        m_rawGoal=goal;
        Pose6 requested=goal;
        for(int i=0;i<3;++i)requested[i]+=m_commandOffset[i];
        if(m_sixAxis)for(int i=3;i<6;++i)requested[i]=m_lastSent[i];
        // Use the same 0.3mm duplicate filter and direct target on every
        // legal frame, then let the existing arbiter enforce its bounds.
        const auto command=planServoPCommand(requested,m_lastSent,actual);
        out.wallHoldAxisMask|=command.heldAxes;
        out.feedbackLimited=command.leadRestricted;
        if(m_sixAxis)return combineSixAxis(out,command,touchRotation,actual,actualRotation);
        switch(command.disposition) {
        case ServoPCommandDisposition::Invalid:
            out.action=ServoPAction::Invalid;break;
        case ServoPCommandDisposition::TrackingHold:
            for(int i=0;i<3;++i)m_commandOffset[i]=m_lastSent[i]-m_rawGoal[i];
            out.action=ServoPAction::TrackingHold;break;
        case ServoPCommandDisposition::Send:
            m_candidate=command.target;
            m_pending=true;
            out.target=m_candidate;
            out.action=ServoPAction::Send;break;
        case ServoPCommandDisposition::None:
            // A normal duplicate retains the master position reference.
            // Discard only suppressed axes / bounded motion, never replay
            // them after the operator stops.
            for(int i=0;i<3;++i)
                if(command.leadRestricted || command.jumpRestricted ||
                   (command.heldAxes & (1U<<i)))
                    m_commandOffset[i]=m_lastSent[i]-m_rawGoal[i];
            break;
        }
        return out;
    }

private:
    ServoPTick combineSixAxis(ServoPTick out,const ServoPCommandPlan& command,
                              const Mat3& touchRotation,const Pose6& actual,
                              const Mat3& measured) {
        if(command.disposition==ServoPCommandDisposition::Invalid) {
            out.action=ServoPAction::Invalid;return out;
        }
        auto discard=[&]() {
            for(int i=0;i<3;++i)m_commandOffset[i]=m_lastSent[i]-m_rawGoal[i];
            m_rotationTouchRef=touchRotation;
            out.feedbackLimited=true;
            out.action=ServoPAction::TrackingHold;
            return out;
        };
        if(command.disposition==ServoPCommandDisposition::TrackingHold)return discard();

        // Preserve the existing direct clutch-relative position planner.
        // Map relative stylus rotation into the very same ServoP target.
        const bool positionAdvance=command.disposition==ServoPCommandDisposition::Send;
        Pose6 next=positionAdvance?command.target:m_lastSent;
        if(!positionAdvance)for(int i=0;i<3;++i)
            if(command.leadRestricted || command.jumpRestricted ||
               (command.heldAxes & (1U<<i)))
                m_commandOffset[i]=m_lastSent[i]-m_rawGoal[i];

        Clutch ref;
        ref.valid=true;ref.touchRotation=m_rotationTouchRef;ref.robotPose=m_lastSent;
        Pose6 raw{};
        if(!fixedAxisPosePreview(m_mapping,ref,Vec3{},touchRotation,
                                 servoPRpy(m_lastSent),raw)) {
            out.action=ServoPAction::Invalid;return out;
        }
        for(int i=3;i<6;++i)out.intent[i]=raw[i];
        const bool rotationAdvance=rotationDistanceDeg(servoPRotation(raw),
            servoPRotation(m_lastSent))>=kServoPRotationDeadbandDeg;
        if(rotationAdvance) {
            double delta=0;
            for(int i=3;i<6;++i)delta+=std::abs(raw[i]-m_lastSent[i]);
            if(!std::isfinite(delta) || delta<=0) {
                out.action=ServoPAction::Invalid;return out;
            }
            // Retain the former angular step limit and rounding headroom.
            const double fraction=std::min(1.0,(kServoPRotationStepDeg-0.002)/delta);
            for(int i=3;i<6;++i)
                next[i]=std::round((m_lastSent[i]+fraction*(raw[i]-m_lastSent[i]))*1000.0)/1000.0;
        }
        if(next==m_lastSent)return out;
        // Neither channel can bypass the other channel's local guards.
        if(!servoP6DCandidateAllowed(next,m_lastSent,actual,measured))return discard();
        m_pendingPositionAdvance=positionAdvance;
        m_rotationPendingTouch=rotationAdvance?touchRotation:m_rotationTouchRef;
        m_candidate=next;m_pending=true;
        out.target=next;out.action=ServoPAction::Send;
        return out;
    }
    void clearClutch() {
        m_touch0={};
        m_robot0={};
        m_lastSent={};
        m_candidate={};
        m_rawGoal={};
        m_commandOffset={};
        m_mapping={};
        m_rotationTouchRef=identity();m_rotationPendingTouch=identity();
        m_pendingPositionAdvance=false;
        m_pending=false;
        m_sentDuringClutch=false;
    }
    ServoPPhase m_phase=ServoPPhase::NeedRelease;
    Vec3 m_touch0{};
    Pose6 m_robot0{},m_lastSent{},m_candidate{},m_rawGoal{};
    Vec3 m_commandOffset{};
    ManualAxisMap m_mapping{};
    bool m_sixAxis=false;
    bool m_pendingPositionAdvance=false;
    Mat3 m_rotationTouchRef=identity(),m_rotationPendingTouch=identity();
    bool m_pending=false;
    bool m_sentDuringClutch=false;
    bool m_releaseStopPending=false;
    bool m_repressReported=false;
    std::uint64_t m_idleSeq=0;
    unsigned m_idleCount=0;
};

} // namespace touchpanel::teleop
