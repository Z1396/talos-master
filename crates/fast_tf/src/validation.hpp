#pragma once

#include "matrix.hpp"
#include <cmath>
#include <concepts>
#include <expected>
#include <fmt/format.h>
#include <numbers>
#include <string>

namespace fast_tf {

/// @brief 验证变换矩阵的数值有效性
/// @return 成功返回 void，失败返回详细错误消息
template <std::floating_point T, typename From, typename To>
[[nodiscard]] std::expected<void, std::string>
    validate_transform(const TransformMatrix<T, From, To>& transform) noexcept;

/// @brief 检查四元数是否归一化
/// @param q 四元数
/// @param tolerance 容差（默认 0.01，参考 tf2）
template <std::floating_point T>
[[nodiscard]] bool is_quaternion_normalized(
    const Eigen::Quaternion<T>& q, T tolerance = static_cast<T>(0.01)) noexcept;

/// @brief 检查数值是否包含 NaN 或 Inf
template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(T value) noexcept;

template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(const Eigen::Matrix<T, 3, 1>& v) noexcept;

template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(const Eigen::Matrix<T, 3, 3>& m) noexcept;

template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(const Eigen::Quaternion<T>& q) noexcept;

/// @brief 格式化变换的 RPY 和平移分量（用于错误消息）
template <std::floating_point T, typename From, typename To>
[[nodiscard]] std::string
    format_transform_values(const TransformMatrix<T, From, To>& transform) noexcept;

template <std::floating_point T>
bool has_nan_or_inf(T value) noexcept {
    return std::isnan(value) || std::isinf(value);
}

template <std::floating_point T>
bool has_nan_or_inf(const Eigen::Matrix<T, 3, 1>& v) noexcept {
    for (int i = 0; i < 3; ++i) {
        if (has_nan_or_inf(v(i))) {
            return true;
        }
    }
    return false;
}

template <std::floating_point T>
bool has_nan_or_inf(const Eigen::Matrix<T, 3, 3>& m) noexcept {
    for (int i = 0; i < 9; ++i) {
        if (has_nan_or_inf(m(i))) {
            return true;
        }
    }
    return false;
}

template <std::floating_point T>
bool has_nan_or_inf(const Eigen::Quaternion<T>& q) noexcept {
    return has_nan_or_inf(q.x()) || has_nan_or_inf(q.y()) || has_nan_or_inf(q.z())
        || has_nan_or_inf(q.w());
}

template <std::floating_point T>
bool is_quaternion_normalized(const Eigen::Quaternion<T>& q, T tolerance) noexcept {
    const T norm = q.norm();
    return std::abs(norm - static_cast<T>(1)) <= tolerance;
}

template <std::floating_point T, typename From, typename To>
std::string format_transform_values(const TransformMatrix<T, From, To>& transform) noexcept {
    const auto euler = transform.euler_rot();
    const auto trans = transform.translation();

    constexpr T rad_to_deg = static_cast<T>(180) / std::numbers::pi_v<T>;

    return fmt::format(
        "roll={:.3f}\xC2\xB0, pitch={:.3f}\xC2\xB0, yaw={:.3f}\xC2\xB0, "
        "x={:.3f}m, y={:.3f}m, z={:.3f}m",
        euler.roll * rad_to_deg, euler.pitch * rad_to_deg, euler.yaw * rad_to_deg, trans.x(),
        trans.y(), trans.z());
}

template <std::floating_point T, typename From, typename To>
std::expected<void, std::string>
    validate_transform(const TransformMatrix<T, From, To>& transform) noexcept {
    const auto translation = transform.translation();
    const auto rotation    = transform.rotation();
    const auto quaternion  = transform.quaternion();

    if (has_nan_or_inf(translation.x())) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [translation.x is NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }
    if (has_nan_or_inf(translation.y())) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [translation.y is NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }
    if (has_nan_or_inf(translation.z())) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [translation.z is NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }

    if (has_nan_or_inf(rotation)) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [rotation matrix contains NaN or Inf]. Full values: "
                "{}",
                format_transform_values(transform)));
    }

    if (has_nan_or_inf(quaternion)) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [quaternion contains NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }

    if (!is_quaternion_normalized(quaternion, static_cast<T>(0.01))) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [quaternion not normalized, norm={:.6f}]. Full "
                "values: {}",
                quaternion.norm(), format_transform_values(transform)));
    }

    return {};
}

} // namespace fast_tf
