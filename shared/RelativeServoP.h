#pragma once
#include "ManualTeleop.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace touchpanel::teleop {

// Restores the field-proven 33ms ServoP Cartesian stream with explicit clutch
// semantics. Touch and robot do NOT share an absolute origin.
// These are conservative first-pass limits, not verified robot geometry.
inline constexpr double kServoPClutchTravelMm=60.0;
inline constexpr double kServoPMaxSetpointStepMm=2.0;
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
        m_settleSeq=0;
        m_settleCount=0;
    }
    ServoPPhase phase() const {return m_phase;}
    const Pose6& anchorRobot() const {return m_anchorRobot;}
    const Vec3& anchorTouch() const {return m_anchorTouch;}

    ServoPTick tick(bool pressed,Vec3 touch,const Pose6& robotPose,
                    bool robotIdle,std::uint64_t stateSeq,
                    const ManualAxisMap& mapping) {
        ServoPTick result;
        if(!finite(touch)||!validManualAxisMap(mapping)||stateSeq==0) {
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
            m_phase=ServoPPhase::Active;
            result.action=ServoPAction::Captured;
            return result; // No command on press -> no coordinate jump.
        }
        if(!pressed) {
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
        result.saturated=distance>kServoPClutchTravelMm;
        if(result.saturated) {
            const auto ratio=kServoPClutchTravelMm/distance;
            for(int i=0;i<3;++i)
                desired[i]=m_anchorRobot[i]+displacement[i]*ratio;
        }
        result.boundedDistanceMm=std::min(distance,kServoPClutchTravelMm);
        const Vec3 next={desired[0]-m_lastTarget[0],
                         desired[1]-m_lastTarget[1],
                         desired[2]-m_lastTarget[2]};
        const double toNext=norm(next);
        if(!std::isfinite(toNext)) {
            result.action=ServoPAction::Invalid;return result;
        }
        if(toNext<=kServoPCommandDeadbandMm)return result;
        if(toNext>kServoPMaxSetpointStepMm) {
            for(int i=0;i<3;++i)
                desired[i]=m_lastTarget[i]+next[i]*kServoPMaxSetpointStepMm/toNext;
        }
        // Orientation is the captured *actual* robot RPY. Never use the
        // original master code's hardcoded (150,0,90).
        for(int i=3;i<6;++i)desired[i]=m_anchorRobot[i];
        m_lastTarget=desired;
        result.target=desired;
        result.action=ServoPAction::Send;
        return result;
    }

private:
    ServoPPhase m_phase=ServoPPhase::NeedRelease;
    Vec3 m_anchorTouch{};
    Pose6 m_anchorRobot{},m_lastTarget{};
    std::uint64_t m_settleSeq=0;
    int m_settleCount=0;
};
} // namespace touchpanel::teleop
