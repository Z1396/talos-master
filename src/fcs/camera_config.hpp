#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <fmt/core.h>
#include <optional>
#include <string>

#include <magic_enum.hpp>

// hikcamera types
#include "hik_camera.hpp"

namespace fcs {

using hikcamera::RotateType;

struct CameraProfileConfig {
    bool trigger_mode{false};
    bool invert_image{false};
    uint32_t exposure_time_us{3000}; // 3ms in microseconds
    double gain{16.7};
    RotateType rotate_angle{RotateType::None};
    std::optional<std::string> device_name{};
};

struct CameraConfig {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix{};
    Eigen::Matrix<double, 1, 5> distort_coefficient{};
    uint32_t width{1440};
    uint32_t height{1080};
    CameraProfileConfig profile{};
};

} // namespace fcs

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<fcs::RotateType> : formatter<std::string_view> {
    auto format(fcs::RotateType type, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(type), ctx);
    }
};

} // namespace fmt
