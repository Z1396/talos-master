#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <variant>
#include <vector>

#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include <fmt/core.h>
#include <magic_enum.hpp>
#include <numbers>

namespace fcs::L3 {

// ============================================================================
// Tracker Status
// ============================================================================

enum class TrackerStatus : uint8_t {
    Idle      = 0, // Not tracking, waiting for first detection
    Detecting = 1, // Short-term detection, awaiting confirmation
    Tracking  = 2, // Stable tracking
    TempLost  = 3, // Temporarily lost, using prediction
};

enum class FilterConvergenceStatus : uint8_t {
    Unknown    = 0,
    Converging = 1,
    Converged  = 2,
    Diverging  = 3,
};

struct FilterConvergenceState {
    FilterConvergenceStatus status{FilterConvergenceStatus::Unknown};
    double normalized_innovation_squared{std::numeric_limits<double>::infinity()};
    double max_covariance_diag{std::numeric_limits<double>::infinity()};
    int consecutive_converged_updates{0};
    int consecutive_diverged_updates{0};
};

// ============================================================================
// Robot Target State (11-dim model output)
// ============================================================================
struct RobotTargetState {
    // Center position (in odom frame)
    Eigen::Vector3d position{0, 0, 0}; // [xc, yc, z0]
    Eigen::Vector3d velocity{0, 0, 0}; // [vx, vy, vz]

    // Rotation
    double yaw{0};   // Armor 0 yaw angle
    double v_yaw{0}; // Angular velocity

    // Geometry
    double radius0{0.22}; // Armor 0,2 radius
    double radius1{0.22}; // Armor 1,3 radius
    double z1{0};         // z0 + h (armor 1,3 height)

    int armors_num{4};

    // EKF internal matrices (for visualization only)
    Eigen::MatrixXd P; // Posterior covariance (11×11)
    Eigen::MatrixXd K; // Kalman gain (11×4)
    Eigen::MatrixXd Q; // Process noise covariance (11×11)
    Eigen::MatrixXd R; // Measurement noise covariance (4×4)
    FilterConvergenceState convergence{};

    /// Calculate all armor poses [x, y, z, yaw]
    [[nodiscard]] std::array<Eigen::Vector4d, 4> armor_poses() const noexcept;
};

// ============================================================================
// Outpost Target State (7-dim model output)
// ============================================================================

struct OutpostTargetState {
    Eigen::Vector2d position{0, 0};    // [xc, yc]
    Eigen::Vector3d velocity{0, 0, 0}; // [vx, vy, vz]
    double yaw{0};
    double v_yaw{0};
    std::array<double, 3> z{0, 0, 0};  // z0, z1, z2

    static constexpr double radius  = 0.2765;
    static constexpr int armors_num = 3;

    // EKF internal matrices (for visualization only)
    Eigen::MatrixXd P; // Posterior covariance (7×7)
    Eigen::MatrixXd K; // Kalman gain (7×4)
    Eigen::MatrixXd Q; // Process noise covariance (7×7)
    Eigen::MatrixXd R; // Measurement noise covariance (4×4)
    FilterConvergenceState convergence{};

    /// Calculate all armor poses [x, y, z, yaw]
    [[nodiscard]] std::array<Eigen::Vector4d, 3> armor_poses() const noexcept;
};

// ============================================================================
// Unified Tracker Output
// ============================================================================

struct TrackerOutput {
    uint64_t timestamp_ns{0};
    TrackerStatus status{TrackerStatus::Idle};
    ArmorName target_name{ArmorName::Invalid};
    ArmorColor target_color{ArmorColor::Neutral};
    bool target_jumped{false};
    std::optional<int> last_armor_id{};
    double last_image_center_distance_px{std::numeric_limits<double>::infinity()};
    uint64_t last_observation_timestamp_ns{0};

    // Variant: empty, robot, or outpost
    std::variant<std::monostate, RobotTargetState, OutpostTargetState> state;

    [[nodiscard]] bool is_tracking() const noexcept {
        return status == TrackerStatus::Tracking || status == TrackerStatus::TempLost;
    }

    [[nodiscard]] bool is_robot() const noexcept {
        return std::holds_alternative<RobotTargetState>(state);
    }

    [[nodiscard]] bool is_outpost() const noexcept {
        return std::holds_alternative<OutpostTargetState>(state);
    }

    [[nodiscard]] const RobotTargetState* robot_state() const noexcept {
        return std::get_if<RobotTargetState>(&state);
    }

    [[nodiscard]] const OutpostTargetState* outpost_state() const noexcept {
        return std::get_if<OutpostTargetState>(&state);
    }
};

using TrackerOutputs = std::vector<TrackerOutput>;

// ============================================================================
// Channel Topics
// ============================================================================

// Channel topics are defined in fcs/core/channel_topics.hpp:
// - fcs::TrackerOutputChannelTopic (payload: TrackerOutputs)
// - fcs::SolverOutputChannelTopic

// Re-export for convenience
using ::fcs::TrackerOutputChannelTopic;

// ============================================================================
// Implementation
// ============================================================================

inline std::array<Eigen::Vector4d, 4> RobotTargetState::armor_poses() const noexcept {
    std::array<Eigen::Vector4d, 4> poses;
    constexpr double angle_step = 2.0 * std::numbers::pi / 4.0;

    for (int i = 0; i < 4; ++i) {
        const double armor_yaw = yaw + static_cast<double>(i) * angle_step;
        double r, z;

        if (i == 1 || i == 3) {
            r = radius1;
            z = z1;
        } else {
            r = radius0;
            z = position.z();
        }

        const double x = position.x() - r * std::cos(armor_yaw);
        const double y = position.y() - r * std::sin(armor_yaw);

        poses[i] = {x, y, z, armor_yaw};
    }

    return poses;
}

inline std::array<Eigen::Vector4d, 3> OutpostTargetState::armor_poses() const noexcept {
    std::array<Eigen::Vector4d, 3> poses;
    constexpr double angle_step = 2.0 * std::numbers::pi / 3.0;

    for (int i = 0; i < 3; ++i) {
        const double armor_yaw = yaw + static_cast<double>(i) * angle_step;
        const double x         = position.x() - radius * std::cos(armor_yaw);
        const double y         = position.y() - radius * std::sin(armor_yaw);

        poses[i] = {x, y, z[i], armor_yaw};
    }

    return poses;
}

} // namespace fcs::L3

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<fcs::L3::TrackerStatus> : formatter<std::string_view> {
    auto format(const fcs::L3::TrackerStatus s, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(s), ctx);
    }
};

} // namespace fmt
