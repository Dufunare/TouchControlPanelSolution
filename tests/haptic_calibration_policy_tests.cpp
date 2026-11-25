#include "../shared/HapticCalibrationPolicy.h"
#include <cassert>
#include <iostream>
using namespace touchpanel::haptics;
int main() {
    CalibrationTickPolicy p;
    assert(p.next()==CalibrationAction::CheckOnly);
    p.checked(CalibrationCondition::NeedsUpdate,true);
    assert(p.next()==CalibrationAction::UpdateOnly);
    p.updated();
    assert(p.next()==CalibrationAction::CheckOnly);
    p.checked(CalibrationCondition::NeedsUpdate,true);
    for(int i=0;i<1500;++i) {
        const auto action=p.next();
        assert(action!=CalibrationAction::UpdateOnly);
        if(action==CalibrationAction::CheckOnly)
            p.checked(CalibrationCondition::NeedsUpdate,true);
    }
    bool updatedAgain=false;
    for(int i=0;i<1500;++i) {
        const auto action=p.next();
        if(action==CalibrationAction::CheckOnly)
            p.checked(CalibrationCondition::NeedsUpdate,true);
        if(action==CalibrationAction::UpdateOnly) {
            updatedAgain=true;p.updated();break;
        }
    }
    assert(updatedAgain&&p.updateCount()==2);
    p.reset();
    assert(p.next()==CalibrationAction::CheckOnly);
    p.checked(CalibrationCondition::ManualInput,true);
    for(int i=0;i<3000;++i) {
        const auto action=p.next();
        assert(action!=CalibrationAction::UpdateOnly);
        if(action==CalibrationAction::CheckOnly)
            p.checked(CalibrationCondition::ManualInput,true);
    }
    p.reset();
    assert(p.next()==CalibrationAction::CheckOnly);
    p.checked(CalibrationCondition::NeedsUpdate,false);
    for(int i=0;i<1000;++i)assert(p.next()!=CalibrationAction::UpdateOnly);
    std::cout<<"Haptic calibration tick isolation checks passed\n";
}
