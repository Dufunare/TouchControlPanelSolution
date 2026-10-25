#include "../shared/DisplayOrigin.h"
#include <cassert>
#include <limits>
#include <iostream>
using namespace touchpanel::visualization;
int main() {
    DisplayOrigin view;
    const Point dock{23.5,-17.25,95.0};
    assert(view.render(dock)==dock);
    assert(!view.capture({0,0,0},false));
    assert(!view.active());
    assert(view.capture(dock,true));
    assert(view.active());
    assert(view.render(dock)==(Point{0,0,0}));
    assert(view.render(Point{25.5,-12.25,92.0})==(Point{2,5,-3}));
    const Point invalid{0,0,std::numeric_limits<double>::infinity()};
    assert(!view.capture(invalid,true));
    assert(view.render(dock)==(Point{0,0,0}));
    view.clear();
    assert(!view.active());
    assert(view.render(dock)==dock);
    std::cout<<"GL presentation-only origin tests passed\n";
}
