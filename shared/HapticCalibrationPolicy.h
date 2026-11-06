#pragma once
#include <cstdint>

namespace touchpanel::haptics {

// Pure state machine: exactly one HDAPI phase per scheduler callback.
// Calibration status checks and updates NEVER share a tick with a haptic frame.
enum class CalibrationAction { SampleFrame, CheckOnly, UpdateOnly };
enum class CalibrationCondition { Good, ManualInput, NeedsUpdate, Unsupported };

class CalibrationTickPolicy {
public:
    static constexpr std::uint32_t checkIntervalTicks=500;
    static constexpr std::uint32_t updateCooldownTicks=2000;

    CalibrationAction next() {
        ++ticks_;
        if(pendingUpdate_) {pendingUpdate_=false;return CalibrationAction::UpdateOnly;}
        if(recheck_||ticks_==1||(ticks_%checkIntervalTicks)==0) {
            recheck_=false;return CalibrationAction::CheckOnly;
        }
        return CalibrationAction::SampleFrame;
    }

    void checked(CalibrationCondition condition, bool canUpdate) {
        if(condition==CalibrationCondition::NeedsUpdate && canUpdate &&
           (lastUpdateTick_==0 || ticks_-lastUpdateTick_>=updateCooldownTicks)) {
            pendingUpdate_=true;
        }
    }

    void updated() {
        lastUpdateTick_=ticks_;
        recheck_=true;
        ++updateCount_;
    }

    void reset() {
        ticks_=0;lastUpdateTick_=0;updateCount_=0;
        pendingUpdate_=false;recheck_=false;
    }

    std::uint64_t ticks() const {return ticks_;}
    std::uint64_t updateCount() const {return updateCount_;}

private:
    std::uint64_t ticks_=0;
    std::uint64_t lastUpdateTick_=0;
    std::uint64_t updateCount_=0;
    bool pendingUpdate_=false;
    bool recheck_=false;
};
} // namespace touchpanel::haptics
