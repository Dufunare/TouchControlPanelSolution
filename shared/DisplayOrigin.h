#pragma once
// Presentation-only local origin. NEVER use these coordinates for robot motion,
// device calibration, workspace limits or 8891 feedback registration.
#include <array>
#include <cmath>

namespace touchpanel::visualization {
using Point = std::array<double, 3>;
inline bool finitePoint(const Point& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}
class DisplayOrigin {
public:
    bool capture(const Point& point, bool sampleValid) {
        if (!sampleValid || !finitePoint(point)) return false;
        origin_=point;enabled_=true;return true;
    }
    void clear() { enabled_=false; origin_={}; }
    bool active() const { return enabled_; }
    Point render(const Point& raw) const {
        if (!finitePoint(raw)) return {};
        if (!enabled_) return raw;
        return {raw[0]-origin_[0],raw[1]-origin_[1],raw[2]-origin_[2]};
    }
private:
    Point origin_{};
    bool enabled_=false;
};
} // namespace touchpanel::visualization
