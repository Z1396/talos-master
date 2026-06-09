#pragma once

#include "calibration_types.hpp"
#include "camera_config.hpp"
#include "foxglove_config.hpp"
#include "toml_helper.hpp"

#include <cstdint>
#include <expected>
#include <string>

namespace fcs::calibration {

/// @brief Calibration board configuration
struct BoardConfig {
    BoardType type{BoardType::Chessboard};
    uint32_t width{11};       ///< Number of inner corners (width)
    uint32_t height{8};       ///< Number of inner corners (height)
    double square_size{0.02}; ///< Square size in meters
};

/// @brief ChArUco-specific configuration
struct CharucoConfig {
    double marker_size{0.015}; ///< ArUco marker size in meters
    ArucoDictionary dictionary{ArucoDictionary::DICT_6X6_250};
};

/// @brief Sample capture configuration
struct CaptureConfig {
    uint32_t min_samples{30};          ///< Minimum samples required for calibration
    uint32_t max_samples{100};         ///< Maximum samples to collect
    double min_angle_diff{15.0};       ///< Minimum angle difference between samples (degrees)
    double min_translation_diff{0.05}; ///< Minimum translation difference (meters)
    bool auto_capture{false};          ///< Automatic capture mode
    uint32_t capture_interval_ms{500}; ///< Auto capture interval (milliseconds)
};

/// @brief Intrinsic calibration flags
struct IntrinsicConfig {
    bool fix_aspect_ratio{false};    ///< Fix fx/fy ratio to 1
    bool fix_principal_point{false}; ///< Fix principal point to image center
    bool zero_tangent_dist{false};   ///< Set tangential distortion to zero

    /// @brief Convert to OpenCV calibration flags
    [[nodiscard]] int to_opencv_flags() const noexcept {
        int flags = 0;
        if (fix_aspect_ratio)
            flags |= cv::CALIB_FIX_ASPECT_RATIO;
        if (fix_principal_point)
            flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
        if (zero_tangent_dist)
            flags |= cv::CALIB_ZERO_TANGENT_DIST;
        return flags;
    }
};

/// @brief Hand-eye calibration configuration
struct HandeyeConfig {
    HandEyeMethod method{HandEyeMethod::Tsai};
};

/// @brief Foxglove visualization configuration
struct VisualizationConfig {
    bool show_corners{true};      ///< Show detected corners overlay
    bool show_reprojection{true}; ///< Show reprojection error visualization
    bool show_pose{true};         ///< Show board pose axes
};

/// @brief Output path configuration
struct OutputConfig {
    std::string intrinsic_path{"camera_intrinsic.toml"};
    std::string extrinsic_path{"camera_extrinsic.toml"};
};

/// @brief Input source configuration (hardware vs simulator)
struct InputConfig {
    bool daedalus{false}; ///< Use daedalus simulator instead of real hardware
};

/// @brief Main calibration configuration
struct CalibrationConfig {
    CalibrationMode mode{CalibrationMode::Intrinsic};
    CameraProfileConfig profile{};
    uint32_t width{};
    uint32_t height{};
    BoardConfig board{};
    CharucoConfig charuco{};
    CaptureConfig capture{};
    IntrinsicConfig intrinsic{};
    HandeyeConfig handeye{};
    VisualizationConfig visualization{};
    OutputConfig output{};
    FoxgloveConfig foxglove{};
    InputConfig input{}; ///< Input source (hardware/daedalus)

    /// @brief Load configuration from TOML file
    [[nodiscard]] static std::expected<CalibrationConfig, std::string>
        load_from_file(const std::string& path) noexcept {
        auto result = toml::parse_file(path);
        if (!result) {
            return std::unexpected(
                fmt::format("Failed to parse {}: {}", path, result.error().description()));
        }

        const auto& tbl = result.table();
        if (auto calib_tbl = tbl["calibration"].as_table()) {
            auto config = toml_helper::from_table<CalibrationConfig>(*calib_tbl);
            if (!config) {
                return std::unexpected(config.error());
            }
            if (config->foxglove.transport == FoxgloveTransport::Mcap
                && config->foxglove.mcap_path.empty()) {
                return std::unexpected(
                    "calibration.foxglove.mcap_path is required when "
                    "calibration.foxglove.transport=\"Mcap\"");
            }
            return std::move(*config);
        }

        return std::unexpected("Missing [calibration] section in config file");
    }
};

} // namespace fcs::calibration
