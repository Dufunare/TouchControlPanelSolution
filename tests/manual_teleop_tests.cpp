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
    std::cout<<"Manual mapping preview tests passed\n";
}
