#include "../shared/GlPoseMath.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace touchpanel;
int main() {
    visualization::DisplayOrigin origin;
    const visualization::Point tip={0,-65.511,-88.114};
    visualization::Point point{};
    assert(visualization::stylusPoint(tip,teleop::identity(),
                                     {0,0,48},origin,point));
    assert(std::abs(point[1]+65.511)<1e-8 && std::abs(point[2]+40.114)<1e-8);
    assert(origin.capture(tip,true));
    assert(visualization::stylusPoint(tip,teleop::identity(),
                                     {0,0,48},origin,point));
    assert(point==(visualization::Point{0,0,48}));
    auto turn=teleop::rpyToMatrix({0,90,0});
    assert(visualization::stylusPoint(tip,turn,{0,0,40},origin,point));
    assert(std::abs(point[0]-40)<1e-8 && std::abs(point[1])<1e-8 &&
           std::abs(point[2])<1e-8);
    assert(visualization::stylusPoint(tip,turn,{0,0,0},origin,point));
    assert(point==(visualization::Point{0,0,0}));
    auto invalid=teleop::identity();
    invalid[0]=std::numeric_limits<double>::quiet_NaN();
    assert(!visualization::stylusPoint(tip,invalid,{0,0,1},origin,point));
    std::cout<<"GL 6DOF pose transform tests passed\n";
}
