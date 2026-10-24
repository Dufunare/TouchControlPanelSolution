#pragma once
#include "DisplayOrigin.h"
#include "TeleopGeometry.h"
#include <array>
namespace touchpanel::visualization {
using Mat3=touchpanel::teleop::Mat3;
using Vec3=touchpanel::teleop::Vec3;
// GL uses OpenHaptics' device frame unaltered. The visual stylus is drawn with
// its nib at local (0,0,0), body extending along +Z (configurable in view).
// Rotation is applied to local geometry and THEN position is translated.
inline bool stylusPoint(const Point& rawTip, Mat3 deviceOrientation,
                        const Vec3& localPoint, const DisplayOrigin& origin,
                        Point& worldPoint) {
    if(!finitePoint(rawTip) || !teleop::finite(localPoint) ||
       !teleop::properRotation(deviceOrientation))return false;
    const auto rotated=teleop::mul(deviceOrientation,localPoint);
    const auto tip=origin.render(rawTip);
    worldPoint={tip[0]+rotated[0],tip[1]+rotated[1],tip[2]+rotated[2]};
    return finitePoint(worldPoint);
}
} // namespace touchpanel::visualization
