#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <magic_enum.hpp>

namespace fcs::L3 {

// ============================================================================
// State Space Indices
// ============================================================================

/// Robot state space (11-dim): 4-armor target tracking
enum RoboState : uint8_t {
    XC,     // Center X position
    VX,     // X velocity
    YC,     // Center Y position
    VY,     // Y velocity
    Z0,     // Armor 0,2 Z position
    VZ,     // Z velocity
    YAW,    // Armor 0 yaw angle
    V_YAW,  // Angular velocity
    LOG_R0, // log(Armor 0,2 radius) — log-parameterized to guarantee r > 0
    LOG_R1, // log(Armor 1,3 radius) — log-parameterized to guarantee r > 0
    H,      // Z1 - Z0 (height difference)
    STATE_MAX,
};

/// Outpost state space (7-dim): 3-armor stationary target
enum OutpostState : uint8_t {
    O_XC,
    O_YC,
    O_YAW,
    O_VYAW,
    O_Z0,
    O_Z1,
    O_Z2,
    O_STATE_MAX,
};

/// Measurement space: [bearing_yaw, bearing_pitch, log_distance, armor_yaw]
enum Measure : uint8_t {
    ARMOR_YAW,
    ARMOR_PITCH,
    ARMOR_DISTANCE,
    ARMOR_YAW_ARMOR,
    MEASURE_MAX,
};

// ============================================================================
// Enum Label Helpers (magic_enum backed)
// ============================================================================

/// Get label for a single RoboState index
[[nodiscard]] inline std::string_view robo_state_label(int i) noexcept {
    return magic_enum::enum_name(static_cast<RoboState>(i));
}

/// Get label for a single OutpostState index
[[nodiscard]] inline std::string_view outpost_state_label(int i) noexcept {
    return magic_enum::enum_name(static_cast<OutpostState>(i));
}

/// Get label for a single Measure index
[[nodiscard]] inline std::string_view measure_label(int i) noexcept {
    return magic_enum::enum_name(static_cast<Measure>(i));
}

// ============================================================================
// Coordinate Transforms
// ============================================================================

/// Convert Cartesian to yaw/pitch/distance in ROS (REP-103) convention:
/// - yaw: rotation about +Z, positive CCW in XY plane: atan2(y, x)
/// - pitch: rotation about +Y, positive "down": atan2(-z, hypot(x, y))
/// - distance: Euclidean norm
[[nodiscard]] inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) noexcept {
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();

    const double distance   = xyz.norm();
    const double yaw        = std::atan2(y, x);
    const double horizontal = std::hypot(x, y); // More accurate and faster than sqrt(x*x + y*y)
    const double pitch      = std::atan2(-z, horizontal);

    return {yaw, pitch, distance};
}

/// Jacobian of xyz2ypd transformation: ∂(yaw, pitch, distance)/∂(x, y, z)
///
/// For spherical coordinate observation models, this Jacobian is used in the
/// chain rule to compute H = ∂h/∂x where h maps state to spherical measurements.
///
/// @param xyz Cartesian coordinates [x, y, z]
/// @return 3x3 Jacobian matrix
[[nodiscard]] inline Eigen::Matrix3d xyz2ypd_jacobian(const Eigen::Vector3d& xyz) noexcept {
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();

    const double r2_xy = x * x + y * y;
    const double r2    = r2_xy + z * z;
    const double r_xy  = std::sqrt(r2_xy);
    const double r     = std::sqrt(r2);

    // Handle singularities (at origin or on z-axis)
    if (r_xy < 1e-10 || r < 1e-10) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d J;

    // ∂yaw/∂(x,y,z) where yaw = atan2(y, x)
    J(0, 0) = -y / r2_xy;
    J(0, 1) = x / r2_xy;
    J(0, 2) = 0.0;

    // ∂pitch/∂(x,y,z) where pitch = atan2(-z, sqrt(x²+y²))
    // Using chain rule: d/dx[atan2(-z, r_xy)] = x*z / (r² * r_xy)
    const double denom = r2 * r_xy;
    J(1, 0)            = x * z / denom;
    J(1, 1)            = y * z / denom;
    J(1, 2)            = -r_xy / r2;

    // ∂distance/∂(x,y,z) where distance = sqrt(x²+y²+z²)
    J(2, 0) = x / r;
    J(2, 1) = y / r;
    J(2, 2) = z / r;

    return J;
}

/// Convert spherical to Cartesian: [yaw, pitch, distance] -> xyz
[[nodiscard]] inline Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd) noexcept {
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    const double cp = std::cos(pitch);
    const double x  = distance * cp * std::cos(yaw);
    const double y  = distance * cp * std::sin(yaw);
    const double z  = -distance * std::sin(pitch);

    return {x, y, z};
}

/// Jacobian of ypd2xyz transformation: ∂(x, y, z)/∂(yaw, pitch, distance)
[[nodiscard]] inline Eigen::Matrix3d ypd2xyz_jacobian(const Eigen::Vector3d& ypd) noexcept {
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    Eigen::Matrix3d J;
    // x = r cp cy
    J(0, 0) = -distance * cp * sy; // dx/dyaw
    J(0, 1) = -distance * sp * cy; // dx/dpitch
    J(0, 2) = cp * cy;             // dx/dr

    // y = r cp sy
    J(1, 0) = distance * cp * cy;  // dy/dyaw
    J(1, 1) = -distance * sp * sy; // dy/dpitch
    J(1, 2) = cp * sy;             // dy/dr

    // z = -r sp
    J(2, 0) = 0.0;            // dz/dyaw
    J(2, 1) = -distance * cp; // dz/dpitch
    J(2, 2) = -sp;            // dz/dr

    return J;
}

// ============================================================================
// Angle Utilities
// ============================================================================

/// Normalize angle to [-PI, PI)
[[nodiscard]] inline double normalize_rad(double a) noexcept {
    double r = std::remainder(a, 2.0 * std::numbers::pi);

    if (r <= -std::numbers::pi)
        r += 2.0 * std::numbers::pi;

    return r;
}

/// Compute shortest angle difference: to - from, result in (-PI, PI]
[[nodiscard]] inline double shortest_rad(double from, double to) noexcept {
    return normalize_rad(to - from);
}

/// Single argument version: normalize angle
[[nodiscard]] inline double shortest_rad(double angle) noexcept { return normalize_rad(angle); }

/// Unwrap angle based on previous value (continuity)
[[nodiscard]] inline double unwrap_rad(double prev, double raw) noexcept {
    const double d = shortest_rad(prev, raw);
    return prev + d;
}

/// Normalize angle to (-180, 180] degrees
[[nodiscard]] inline double normalize_deg(double a) noexcept {
    a = std::fmod(a + 180.0, 360.0);
    if (a <= 0.0) {
        a += 360.0;
    }
    return a - 180.0;
}

/// Compute shortest angle difference in degrees
[[nodiscard]] inline double shortest_deg(double from, double to) noexcept {
    return normalize_deg(to - from);
}

[[nodiscard]] inline double shortest_deg(double angle) noexcept { return normalize_deg(angle); }

/// Unwrap angle in degrees
[[nodiscard]] inline double unwrap_deg(double prev, double raw) noexcept {
    const double d = shortest_deg(prev, raw);
    return prev + d;
}

/// A plate is visible from the observation origin only when its yaw points into
/// the front-facing half-plane relative to the bearing from origin to plate.
[[nodiscard]] inline bool
    armor_face_visible_from_origin(double bearing_yaw, double armor_yaw) noexcept {
    if (!std::isfinite(bearing_yaw) || !std::isfinite(armor_yaw)) {
        return false;
    }
    constexpr double kMaxVisibleYawError = std::numbers::pi / 2.0;
    return std::abs(shortest_rad(bearing_yaw, armor_yaw)) < kMaxVisibleYawError;
}

template <typename VecZ>
[[nodiscard]] inline bool armor_measurement_visible_from_origin(const VecZ& z) noexcept {
    return armor_face_visible_from_origin(z[ARMOR_YAW], z[ARMOR_YAW_ARMOR]);
}

// ============================================================================
// Armor Position Calculation
// ============================================================================

/// Calculate all armor positions for a 4-armor robot target
/// @param target_center Center position [xc, yc, z0]
/// @param target_yaw Armor 0 yaw angle
/// @param radius0 Radius for armor 0,2
/// @param radius1 Radius for armor 1,3
/// @param z0 Z position for armor 0,2
/// @param z1 Z position for armor 1,3
/// @param armors_num Number of armors (3 or 4)
/// @return Vector of [x, y, z, yaw] for each armor
[[nodiscard]] inline std::vector<Eigen::Vector4d> get_robo_armor_poses(
    const Eigen::Vector3d& target_center, double target_yaw, double radius0, double radius1,
    double z0, double z1, size_t armors_num) noexcept {
    std::vector<Eigen::Vector4d> poses;
    poses.reserve(armors_num);

    const double angle_step = 2.0 * std::numbers::pi / static_cast<double>(armors_num);

    for (size_t i = 0; i < armors_num; ++i) {
        const double armor_yaw = normalize_rad(target_yaw + static_cast<double>(i) * angle_step);
        double radius;
        double armor_z;

        if (armors_num == 4 && (i == 1 || i == 3)) {
            radius  = radius1;
            armor_z = z1;
        } else {
            radius  = radius0;
            armor_z = z0;
        }

        const double armor_x = target_center.x() - radius * std::cos(armor_yaw);
        const double armor_y = target_center.y() - radius * std::sin(armor_yaw);

        poses.emplace_back(armor_x, armor_y, armor_z, armor_yaw);
    }

    return poses;
}

/// Calculate armor positions for outpost (3 armors, fixed radius)
[[nodiscard]] inline std::vector<Eigen::Vector4d> get_outpost_armor_poses(
    const Eigen::Vector2d& target_pos, double yaw, double radius, double z0, double z1,
    double z2) noexcept {
    constexpr size_t ARMORS_NUM = 3;
    constexpr double angle_step = 2.0 * std::numbers::pi / static_cast<double>(ARMORS_NUM);

    std::vector<Eigen::Vector4d> poses;
    poses.reserve(ARMORS_NUM);

    const std::array<double, 3> zs = {z0, z1, z2};

    for (size_t i = 0; i < ARMORS_NUM; ++i) {
        const double armor_yaw = (yaw + static_cast<double>(i) * angle_step);
        const double armor_x   = target_pos.x() - radius * std::cos(armor_yaw);
        const double armor_y   = target_pos.y() - radius * std::sin(armor_yaw);

        poses.emplace_back(armor_x, armor_y, zs[i], armor_yaw);
    }

    return poses;
}

/// Extract position from pose [x, y, z, yaw] -> [x, y, z]
[[nodiscard]] inline Eigen::Vector3d pose_to_position(const Eigen::Vector4d& pose) noexcept {
    return {pose.x(), pose.y(), pose.z()};
}

/// Extract positions from poses
[[nodiscard]] inline std::vector<Eigen::Vector3d>
    poses_to_positions(const std::vector<Eigen::Vector4d>& poses) noexcept {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(poses.size());
    for (const auto& pose : poses) {
        positions.emplace_back(pose.x(), pose.y(), pose.z());
    }
    return positions;
}

/// Calculate armor position from state vector
[[nodiscard]] inline Eigen::Vector3d
    state_to_armor_xyz(const double* x, int id, int armors_num) noexcept {
    const double angle_step = 2.0 * std::numbers::pi / static_cast<double>(armors_num);
    const double armor_yaw  = x[YAW] + static_cast<double>(id) * angle_step;

    double radius;
    double armor_z;

    if (armors_num == 4 && (id == 1 || id == 3)) {
        radius  = std::exp(x[LOG_R1]);
        armor_z = x[Z0] + x[H];
    } else {
        radius  = std::exp(x[LOG_R0]);
        armor_z = x[Z0];
    }

    const double armor_x = x[XC] - radius * std::cos(armor_yaw);
    const double armor_y = x[YC] - radius * std::sin(armor_yaw);

    return {armor_x, armor_y, armor_z};
}

} // namespace fcs::L3
