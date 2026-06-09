#pragma once

#include <cmath>
#include <numbers>

namespace math_fuxk {
template <typename T>
struct SO2 {
public:
    explicit SO2(T&& v)
        : angle(v) {}

    friend SO2 operator-(const SO2& a, const SO2& b) {
        return SO2(std::remainder(a.angle - b.angle, 2.0 * std::numbers::pi));
    }

    friend SO2 operator+(const SO2& a, const T& delta) {
        return SO2(a.angle + static_cast<double>(delta));
    }

    explicit operator double() const { return angle; }

private:
    double angle; // [-π, π] rad
};
}; // namespace math_fuxk
