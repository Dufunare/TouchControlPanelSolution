#pragma once
#include "ManualTeleop.h"
#include "ServoPWorkspace.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace touchpanel::teleop {

// Restores the field-proven 33ms ServoP Cartesian stream with explicit clutch
// semantics. Touch and robot do NOT share an absolute origin.
// These are conservative first-pass limits, not verified robot geometry.
inline constexpr double kServoPClutchTravelMm=60.0; // opt-in default
inline constexpr double kServoPMaxClutchTravelMm=180.0;
inline constexpr double kServoPNominalTickSeconds=0.033;
inline constexpr double kServoPMaxCatchupSeconds=0.066;
inline constexpr double kServoPMaxSetpointStepMm=2.0; // legacy nominal 33ms
// This bounds the requested *setpoint*, not the robot's physical speed.
inline constexpr double kServoPDefaultFollowSpeedMmPerSec=
    kServoPMaxSetpointStepMm/kServoPNominalTickSeconds;
inline constexpr double kServoPMaxSelectableFollowSpeedMmPerSec=120.0;
// Keep the latest Cartesian setpoint near current measured TCP. The remaining
// 25mm last-transmitted-target guard is independent and unchanged.
inline constexpr double kServoPFeedbackLeadLimitMm=12.0; // conservative default
inline constexpr double kServoPMaxSelectableFeedbackLeadMm=20.0;
// This is a feedback-relative software lookahead, not a robot collision
// boundary, certified velocity limit, or guarantee of smooth execution.
inline constexpr double kServoPTrackingErrorMm=25.0;
inline constexpr double kServoPCommandDeadbandMm=0.3;

enum class ServoPPhase { NeedRelease, Ready, Active, WaitingForIdle };
enum class ServoPAction { None, Ready, Captured, Send, Released, Invalid };

struct ServoPTick {
    ServoPAction action=ServoPAction::None;
    Pose6 target{};
    double rawDistanceMm=0.0;
    double boundedDistanceMm=0.0;
    bool saturated=false;
    bool feedbackLimited=false;
    unsigned wallHoldAxisMask=0;
};

// State-only clutch: no TCP/network access, no Qt, no movement authority.
// Each press captures the CURRENT Touch and CURRENT robot feedback pose.
// Release discards that pair. Moving the stylus while released never sends a
// target, and the next press cannot reuse the previous anchor.
class RelativeServoP {
public:
    void reset() {
        m_phase=ServoPPhase::NeedRelease;
        m_anchorTouch={};
        m_anchorRobot={};
        m_lastTarget={};
        m_pendingTarget={};
        m_hasPending=false;
        m_settleSeq=0;
        m_settleCount=0;
    }
    ServoPPhase phase() const {return m_phase;}
    double clutchTravelLimitMm() const {return m_clutchTravelLimitMm;}
    double followSpeedLimitMmPerSec() const {return m_followSpeedLimitMmPerSec;}
    double feedbackLeadLimitMm() const {return m_feedbackLeadLimitMm;}
    bool virtualWallSlideEnabled() const {return m_virtualWallSlideEnabled;}
    bool setVirtualWallSlideEnabled(bool enabled) {
        if(m_phase==ServoPPhase::Active||m_phase==ServoPPhase::WaitingForIdle)
            return false;
        m_virtualWallSlideEnabled=enabled;
        return true;
    }
    // Only configure while disarmed. Both clutch re-anchors and reset()
    // preserve this setting; no operator action changes it mid-clutch.
    bool setFeedbackLeadLimitMm(double leadMm) {
        if(!std::isfinite(leadMm)||leadMm<kServoPFeedbackLeadLimitMm||
           leadMm>kServoPMaxSelectableFeedbackLeadMm||
           m_phase==ServoPPhase::Active||m_phase==ServoPPhase::WaitingForIdle)
            return false;
        m_feedbackLeadLimitMm=leadMm;
        return true;
    }
    bool setFollowSpeedLimitMmPerSec(double speed) {
        if(!std::isfinite(speed)||speed<kServoPDefaultFollowSpeedMmPerSec-0.01||
           speed>kServoPMaxSelectableFollowSpeedMmPerSec||
           m_phase==ServoPPhase::Active||m_phase==ServoPPhase::WaitingForIdle)
            return false;
        m_followSpeedLimitMmPerSec=speed;
        return true;
    }
    // Configuration is permitted only while disarmed; reset() preserves it.
    // Enlarging the clutch envelope is not enlarging the robot workspace.
    bool setClutchTravelLimitMm(double mm) {
        if(!std::isfinite(mm)||mm<kServoPClutchTravelMm||
           mm>kServoPMaxClutchTravelMm||
           m_phase==ServoPPhase::Active||m_phase==ServoPPhase::WaitingForIdle)
            return false;
        m_clutchTravelLimitMm=mm;
        return true;
    }
    const Pose6& anchorRobot() const {return m_anchorRobot;}
    const Vec3& anchorTouch() const {return m_anchorTouch;}
    const Pose6& lastAcceptedTarget() const {return m_lastTarget;}

    // The previous implementation advanced m_lastTarget before attempting
    // the actual network send. A rejected workspace target then corrupted
    // the rate-limiter origin and made reversing the Touch unpredictable.
    // Only a successfully transmitted setpoint may advance this anchor.
    bool acknowledgeSentTarget(const Pose6& target) {
        if(m_phase!=ServoPPhase::Active || !m_hasPending ||
           m_pendingTarget!=target)return false;
        m_lastTarget=target;
        m_hasPending=false;
        return true;
    }
    void rejectUnsentTarget() {m_hasPending=false;}

    ServoPTick tick(bool pressed,Vec3 touch,const Pose6& robotPose,
                    bool robotIdle,std::uint64_t stateSeq,
                    const ManualAxisMap& mapping,
                    double elapsedSeconds=kServoPNominalTickSeconds) {
        ServoPTick result;
        if(!finite(touch)||!validManualAxisMap(mapping)||stateSeq==0||
           !std::isfinite(elapsedSeconds)||elapsedSeconds<=0.0) {
            result.action=ServoPAction::Invalid;return result;
        }
        for(double value:robotPose)if(!std::isfinite(value)) {
            result.action=ServoPAction::Invalid;return result;
        }
        // Before first clutch AND after releasing a clutch, require a
        // released button plus 3 distinct fresh idle snapshots. This
        // deliberately rejects a held Button1 while the robot is settling.
        if(m_phase==ServoPPhase::NeedRelease||
           m_phase==ServoPPhase::WaitingForIdle) {
            if(pressed || !robotIdle) {
                m_settleCount=0;
                return result;
            }
            if(m_settleSeq!=stateSeq) {
                m_settleSeq=stateSeq;
                ++m_settleCount;
            }
            if(m_settleCount>=3) {
                m_phase=ServoPPhase::Ready;
                m_settleCount=0;
                result.action=ServoPAction::Ready;
            }
            return result;
        }
        if(m_phase==ServoPPhase::Ready) {
            if(!pressed)return result;
            if(!robotIdle) {
                m_phase=ServoPPhase::NeedRelease;
                m_settleCount=0;
                return result;
            }
            m_anchorTouch=touch;
            m_anchorRobot=robotPose;
            m_lastTarget=robotPose;
            m_hasPending=false;
            m_phase=ServoPPhase::Active;
            result.action=ServoPAction::Captured;
            return result; // No command on press -> no coordinate jump.
        }
        if(!pressed) {
            m_hasPending=false;
            m_phase=ServoPPhase::WaitingForIdle;
            m_settleCount=0;
            m_settleSeq=stateSeq;
            result.action=ServoPAction::Released;
            return result; // Stop streaming. Never replay the last command.
        }
        Pose6 desired{};
        if(!manualPositionPreview(mapping,sub(touch,m_anchorTouch),
                                  m_anchorRobot,desired)) {
            result.action=ServoPAction::Invalid;return result;
        }
        const Vec3 displacement={desired[0]-m_anchorRobot[0],
                                 desired[1]-m_anchorRobot[1],
                                 desired[2]-m_anchorRobot[2]};
        const double distance=norm(displacement);
        if(!std::isfinite(distance)) {
            result.action=ServoPAction::Invalid;return result;
        }
        result.rawDistanceMm=distance;
        result.saturated=distance>m_clutchTravelLimitMm;
        if(result.saturated) {
            const auto ratio=m_clutchTravelLimitMm/distance;
            for(int i=0;i<3;++i)
                desired[i]=m_anchorRobot[i]+displacement[i]*ratio;
        }
        result.boundedDistanceMm=std::min(distance,m_clutchTravelLimitMm);
        // Feedback-referenced preview horizon: after a sudden Touch jump,
        // follow the CURRENT requested position by sending a nearby waypoint,
        // rather than slowly walking an old queued absolute target forward.
        // Do not treat the 8891 TCP pose as a reference when its age is stale:
        // the caller must have passed the independent telemetry gates first.
        const Vec3 fromActual={desired[0]-robotPose[0],
                               desired[1]-robotPose[1],
                               desired[2]-robotPose[2]};
        const double desiredLag=norm(fromActual);
        if(!std::isfinite(desiredLag)) {
            result.action=ServoPAction::Invalid;return result;
        }
        result.feedbackLimited=desiredLag>m_feedbackLeadLimitMm;
        if(result.feedbackLimited) {
            const double ratio=m_feedbackLeadLimitMm/desiredLag;
            for(int i=0;i<3;++i)
                desired[i]=robotPose[i]+fromActual[i]*ratio;
        }
        const Vec3 next={desired[0]-m_lastTarget[0],
                         desired[1]-m_lastTarget[1],
                         desired[2]-m_lastTarget[2]};
        const double toNext=norm(next);
        if(!std::isfinite(toNext)) {
            result.action=ServoPAction::Invalid;return result;
        }
        if(toNext<=kServoPCommandDeadbandMm)return result;
        // Preserve roughly the same target advance per SECOND when Qt's
        // actual 33ms tick runs late (e.g. 50ms). Never accumulate an
        // unbounded catch-up command after UI/network scheduling stalls.
        const double boundedSeconds=std::min(elapsedSeconds,kServoPMaxCatchupSeconds);
        const double maxStep=m_followSpeedLimitMmPerSec*boundedSeconds;
        if(toNext>maxStep) {
            for(int i=0;i<3;++i)
                desired[i]=m_lastTarget[i]+next[i]*maxStep/toNext;
        }
        if(m_virtualWallSlideEnabled) {
            result.wallHoldAxisMask=holdOutwardServoPAxes(desired,m_lastTarget);
            const Vec3 finalStep={desired[0]-m_lastTarget[0],
                                  desired[1]-m_lastTarget[1],
                                  desired[2]-m_lastTarget[2]};
            if(norm(finalStep)<=kServoPCommandDeadbandMm)return result;
        }
        // Orientation is the captured *actual* robot RPY. Never use the
        // original master code's hardcoded (150,0,90).
        for(int i=3;i<6;++i)desired[i]=m_anchorRobot[i];
        m_pendingTarget=desired;
        m_hasPending=true;
        result.target=desired;
        result.action=ServoPAction::Send;
        return result;
    }

private:
    ServoPPhase m_phase=ServoPPhase::NeedRelease;
    Vec3 m_anchorTouch{};
    Pose6 m_anchorRobot{},m_lastTarget{},m_pendingTarget{};
    bool m_hasPending=false;
    double m_clutchTravelLimitMm=kServoPClutchTravelMm;
    double m_followSpeedLimitMmPerSec=kServoPDefaultFollowSpeedMmPerSec;
    double m_feedbackLeadLimitMm=kServoPFeedbackLeadLimitMm;
    bool m_virtualWallSlideEnabled=false;
    std::uint64_t m_settleSeq=0;
    int m_settleCount=0;
};
} // namespace touchpanel::teleop
