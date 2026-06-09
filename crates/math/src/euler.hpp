#pragma once
#include "groups/SO3.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>

namespace math_fuxk {

/// @brief ROS2 标准欧拉角
template <typename Value>
struct Ros2EulerRot {
    // rad
    Value roll, pitch, yaw;

    constexpr Ros2EulerRot(Value r, Value p, Value y) noexcept
        : roll(r)
        , pitch(p)
        , yaw(y) {}

    [[nodiscard]] constexpr auto rpy() const noexcept { return std::tuple{roll, pitch, yaw}; }

    [[nodiscard]] auto quat() const noexcept -> Eigen::Quaternion<Value> {
        return Eigen::Quaternion<Value>(
            Eigen::AngleAxis<Value>(yaw, Eigen::Matrix<Value, 3, 1>::UnitZ())
            * Eigen::AngleAxis<Value>(pitch, Eigen::Matrix<Value, 3, 1>::UnitY())
            * Eigen::AngleAxis<Value>(roll, Eigen::Matrix<Value, 3, 1>::UnitX()));
    }

    [[nodiscard]] auto matrix() const noexcept -> Eigen::Matrix3<Value> {
        return quat().toRotationMatrix();
    }

    [[nodiscard]] auto so3() const noexcept -> group::SO3<Value> {
        return group::SO3<Value>(quat().toRotationMatrix());
    }
};

// ===== 类型别名 =====
using Ros2EulerRotf = Ros2EulerRot<float>;
using Ros2EulerRotd = Ros2EulerRot<double>;

// ===== 工厂函数 =====
template <typename T>
[[nodiscard]] constexpr auto rpy(T roll, T pitch, T yaw) -> Ros2EulerRot<T> {
    return {roll, pitch, yaw};
}

template <typename T>
[[nodiscard]] constexpr auto rpy(const Eigen::Matrix3<T>& m) -> Ros2EulerRot<T> {
    T sin_pitch = std::clamp(-m(2, 0), T(-1), T(1));

    if (std::abs(sin_pitch) < T(0.9999999)) {
        return {
            std::atan2(m(2, 1), m(2, 2)), // roll
            std::asin(sin_pitch),         // pitch
            std::atan2(m(1, 0), m(0, 0))  // yaw
        };
    }
    // 万向锁
    return {T(0), sin_pitch > 0 ? T(M_PI_2) : T(-M_PI_2), std::atan2(-m(0, 1), m(1, 1))};
}

template <typename T>
[[nodiscard]] constexpr auto rpy(const Eigen::Quaternion<T>& q) -> Ros2EulerRot<T> {
    return rpy(q.toRotationMatrix());
}

template <typename T>
[[nodiscard]] auto rpy(const group::SO3<T>& q) -> Ros2EulerRot<T> {
    return rpy(q.R());
}

} // namespace math_fuxk
