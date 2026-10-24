#include "../shared/ManualTeleop.h"
#include <cassert>
#include <iostream>
using namespace touchpanel::teleop;
int main() {
    ManualAxisMap m;
    Pose6 base={100,200,300,10,20,30}, out{};
    assert(validManualAxisMap(m));
    assert(manualPositionPreview(m,{21,0,0},base,out));
    assert(out[0]==104.875);
    assert(out[3]==10&&out[4]==20&&out[5]==30);
    m.output={3,1,2};
    assert(manualPositionPreview(m,{21,0,0},base,out));
    assert(out[0]==95.125);
    m.output={0,3,2};
    assert(!validManualAxisMap(m));
    m.output={0,1,2};m.scale=100;
    assert(!validManualAxisMap(m));

    // Regression for the Qt combo boxes: defaults MUST be accepted, and
    // choosing a conflicting Touch source must swap rather than reject.
    std::array<int,3> selection{0,1,2};
    selection=swapManualAxisSelection(selection,0,4); // Robot X <- -Touch Y
    assert((selection==std::array<int,3>{4,0,2}));
    m.output=selection;m.scale=0.25;
    assert(validManualAxisMap(m));
    selection=swapManualAxisSelection(selection,2,1); // Robot Z <- +Touch Y
    assert((selection==std::array<int,3>{2,0,1}));
    m.output=selection;
    assert(validManualAxisMap(m));
    selection=swapManualAxisSelection(selection,1,3); // Robot Y <- -Touch X
    assert((selection==std::array<int,3>{2,3,1}));
    m.output=selection;
    assert(validManualAxisMap(m));
    const auto unchanged=swapManualAxisSelection(selection,1,99);
    assert(unchanged==selection);
    // Flipping the sign of the same source axis must not disturb others.
    assert((swapManualAxisSelection(selection,1,0)==std::array<int,3>{2,0,1}));
    std::cout<<"Manual mapping preview tests passed\n";
}
