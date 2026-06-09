#pragma once

#include <cmath>

namespace energy_meter {

inline double normalize_rad(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a <= 0.0) {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

inline double shortest_rad(double from, double to) { return normalize_rad(to - from); }

inline double unwrap_rad(double prev, double raw) {
    const double d = shortest_rad(prev, raw);
    return prev + d;
}

} // namespace energy_meter
