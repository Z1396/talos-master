#pragma once

// This header provides a type-safe, composable builder pattern for creating
// Foxglove scene entities with semantic styling.
//
// Design Philosophy:
// - Fluent interface: chain methods for readable code
// - Semantic defaults: colors, sizes from tactical palette
// - Convenience patterns: common operations in single function calls
//
// Usage Example:
// ```cpp
// using namespace fcs::visualization::builder;
// using namespace tactical;
//
// auto entity = EntityBuilder::create("l3", "target")
//     .position(1.0, 2.0, 3.0)
//     .color(L3::TRACKING_LOCKED)
//     .size(L3::TARGET_SIZE)
//     .sphere()
//     .build();
// ```

#include "frame.hpp"
#include "tactical_palette.hpp"

#include <Eigen/Core>
#include <cmath>
#include <fmt/core.h>
#include <foxglove/schemas.hpp>
#include <optional>
#include <string>
#include <utility.hpp>
#include <vector>

namespace fcs::visualization {

// ============================================================================
// UTILITY FUNCTIONS (inline — required by templates below)
// ============================================================================

/// @brief Create Vector3 from components
inline ::foxglove::schemas::Vector3 make_vector3(double x, double y, double z) noexcept {
    ::foxglove::schemas::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

/// @brief Create Vector3 from Eigen::Vector3d
inline ::foxglove::schemas::Vector3 make_vector3(const Eigen::Vector3d& vec) noexcept {
    return make_vector3(vec.x(), vec.y(), vec.z());
}

/// @brief Create Point3 from components
inline ::foxglove::schemas::Point3 make_point3(double x, double y, double z) noexcept {
    ::foxglove::schemas::Point3 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

/// @brief Create Point3 from Eigen::Vector3d
inline ::foxglove::schemas::Point3 make_point3(const Eigen::Vector3d& vec) noexcept {
    return make_point3(vec.x(), vec.y(), vec.z());
}

/// @brief Create Point3 from Vector3
inline ::foxglove::schemas::Point3 make_point3(const ::foxglove::schemas::Vector3& v) noexcept {
    return make_point3(v.x, v.y, v.z);
}

/// @brief Create Quaternion from components
inline ::foxglove::schemas::Quaternion
    make_quaternion(double x, double y, double z, double w) noexcept {
    ::foxglove::schemas::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

/// @brief Create Quaternion from Eigen::Quaterniond
inline ::foxglove::schemas::Quaternion make_quaternion(const Eigen::Quaterniond& q) noexcept {
    return make_quaternion(q.x(), q.y(), q.z(), q.w());
}

/// @brief Create identity quaternion
inline ::foxglove::schemas::Quaternion identity_quaternion() noexcept {
    return make_quaternion(0.0, 0.0, 0.0, 1.0);
}

/// @brief Create Color from components
inline ::foxglove::schemas::Color make_color(double r, double g, double b, double a) noexcept {
    ::foxglove::schemas::Color c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

/// @brief Create Pose from position and orientation
inline ::foxglove::schemas::Pose make_pose(
    std::optional<::foxglove::schemas::Vector3> position       = std::nullopt,
    std::optional<::foxglove::schemas::Quaternion> orientation = std::nullopt) noexcept {
    ::foxglove::schemas::Pose pose;
    pose.position    = position;
    pose.orientation = orientation;
    return pose;
}

/// @brief Create Pose from position with identity orientation
inline ::foxglove::schemas::Pose make_pose(const ::foxglove::schemas::Vector3& position) noexcept {
    return make_pose(position, identity_quaternion());
}

// ============================================================================
// ENTITY BUILDER
// ============================================================================

/// @brief Fluent builder for Foxglove SceneEntity
///
/// Provides a type-safe, composable interface for building scene entities
/// with semantic styling from the tactical palette.
///
/// Thread Safety: Not thread-safe. Create and use builders within a single thread.
///
/// Design Notes:
/// - SceneEntity does not have a top-level pose; each primitive has its own pose
/// - This builder maintains a "current_pose" that gets applied to primitives
/// - Call position()/orientation() before adding primitives to set their pose
class EntityBuilder {
public:
    /// @brief Create a new EntityBuilder with frame template parameter
    template <fast_tf::frame T>
    [[nodiscard]] static EntityBuilder create(std::string layer, std::string entity_id) {
        EntityBuilder builder;
        builder.entity_.id       = fmt::format("{}.{}", layer, entity_id);
        builder.entity_.frame_id = T::frame_id;
        builder.layer_           = std::move(layer);
        return builder;
    }

    // ========================================================================
    // POSITION & ORIENTATION
    // ========================================================================

    /// @brief Set position from components
    EntityBuilder& position(double x, double y, double z);
    /// @brief Set position from Eigen::Vector3d
    EntityBuilder& position(const Eigen::Vector3d& pos);
    /// @brief Set position from Vector3
    EntityBuilder& position(const ::foxglove::schemas::Vector3& pos);

    /// @brief Set orientation from quaternion components
    EntityBuilder& orientation(double qx, double qy, double qz, double qw);
    /// @brief Set orientation from Eigen::Quaterniond
    EntityBuilder& orientation(const Eigen::Quaterniond& q);
    /// @brief Set orientation from Quaternion
    EntityBuilder& orientation(const ::foxglove::schemas::Quaternion& q);
    /// @brief Set orientation from Euler angles (yaw, pitch, roll)
    EntityBuilder& orientation_euler(double yaw, double pitch, double roll);

    /// @brief Reset position to origin (for primitives at origin in current frame)
    EntityBuilder& reset_position();
    /// @brief Reset orientation to identity
    EntityBuilder& reset_orientation();

    // ========================================================================
    // VISUAL PROPERTIES
    // ========================================================================

    /// @brief Set color from components
    EntityBuilder& color(double r, double g, double b, double a = 1.0);
    /// @brief Set color from Color
    EntityBuilder& color(const ::foxglove::schemas::Color& c);

    /// @brief Set size (uniform for all axes)
    EntityBuilder& size(double s);
    /// @brief Set size from components
    EntityBuilder& size(double sx, double sy, double sz);
    /// @brief Set alpha (transparency) for current color
    EntityBuilder& alpha(double a);

    // ========================================================================
    // PRIMITIVES
    // ========================================================================

    /// @brief Add a sphere primitive with current pose, size, and color
    EntityBuilder& sphere();

    /// @brief Add a cube primitive with specified dimensions
    EntityBuilder& cube(double width, double height, double depth);

    /// @brief Add text label
    EntityBuilder& text(std::string content, double font_size = tactical::Text::SIZE_MEDIUM);

    /// @brief Add text label at offset from current position
    EntityBuilder& text_with_offset(
        std::string content, double offset_x, double offset_y, double offset_z,
        double font_size = tactical::Text::SIZE_MEDIUM);

    // ========================================================================
    // LINES & CYLINDERS
    // ========================================================================

    /// @brief Add a line strip connecting sequential points (0->1->2->...->N)
    EntityBuilder& line_strip(
        std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
        double thickness = 0.01);

    /// @brief Add individual line segments: 0->1, 2->3, 4->5, ...
    EntityBuilder& line_list(
        std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
        double thickness = 0.01);

    /// @brief Add a closed polygon: 0->1->...->N->0
    EntityBuilder& line_loop(
        std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
        double thickness = 0.01);

    /// @brief Add a cylinder primitive (usable as ring indicator)
    EntityBuilder& cylinder(double diameter, double height);

    // ========================================================================
    // PRIMITIVE MERGING (encapsulation-safe)
    // ========================================================================

    /// @brief Add a pre-built cube primitive directly
    EntityBuilder& add_cube(::foxglove::schemas::CubePrimitive cube);
    /// @brief Add a pre-built sphere primitive directly
    EntityBuilder& add_sphere(::foxglove::schemas::SpherePrimitive sphere);
    /// @brief Add a pre-built text primitive directly
    EntityBuilder& add_text(::foxglove::schemas::TextPrimitive text);
    /// @brief Add a pre-built arrow primitive directly
    EntityBuilder& add_arrow(::foxglove::schemas::ArrowPrimitive arrow);
    /// @brief Add a pre-built line primitive directly
    EntityBuilder& add_line(::foxglove::schemas::LinePrimitive line);
    /// @brief Add a pre-built cylinder primitive directly
    EntityBuilder& add_cylinder(::foxglove::schemas::CylinderPrimitive cyl);

    /// @brief Merge all primitives from another SceneEntity into this builder
    EntityBuilder& merge_primitives(const ::foxglove::schemas::SceneEntity& other);

    // ========================================================================
    // METADATA
    // ========================================================================

    /// @brief Add metadata key-value pair
    EntityBuilder& metadata(std::string key, std::string value);
    /// @brief Set timestamp for the entity
    EntityBuilder& timestamp(uint64_t ns);
    /// @brief Set entity lifetime (nanoseconds)
    EntityBuilder& lifetime(uint64_t ns);

    // ========================================================================
    // CLEAR & RETAIN
    // ========================================================================

    /// @brief Clear all accumulated primitives
    EntityBuilder& clear() noexcept;

    /// @brief Clear primitives but retain metadata
    EntityBuilder& retain() noexcept;

    // ========================================================================
    // BUILD
    // ========================================================================

    /// @brief Build and return the SceneEntity (moves builder)
    [[nodiscard]] ::foxglove::schemas::SceneEntity build() &&;
    /// @brief Build and return the SceneEntity (copies)
    [[nodiscard]] ::foxglove::schemas::SceneEntity build() const&;

private:
    EntityBuilder() = default;

    ::foxglove::schemas::SceneEntity entity_;
    std::string layer_;
    ::foxglove::schemas::Pose current_pose_{};
    std::optional<::foxglove::schemas::Color> current_color_;
    std::optional<::foxglove::schemas::Vector3> current_size_;
};

// ============================================================================
// CONVENIENCE PATTERNS
// ============================================================================

namespace patterns {

// ============================================================================
// L2: MEASUREMENT VISUALIZATION
// ============================================================================

/// @brief Create a ghost measurement sphere (L2 perception layer)
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity measurement_sphere(
    const Eigen::Vector3d& pos, double confidence, uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    const auto color = (confidence > 0.5) ? L2::MEASUREMENT_CONFIDENCE : L2::MEASUREMENT_GHOST;

    return EntityBuilder::create<Frame>("l2", "measurement")
        .position(pos)
        .size(L2::ARMOR_SIZE)
        .color(color)
        .sphere()
        .metadata("confidence", fmt::format("{:.3f}", confidence))
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create a measurement cube with full pose information
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity measurement_cube(
    const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation, const std::string& label,
    double confidence = 0.0) noexcept {
    using namespace tactical;

    return EntityBuilder::create<Frame>("l2", "measurement")
        .position(pos)
        .orientation(orientation)
        .cube(Geometry::ARMOR_THICKNESS, Geometry::ARMOR_HEIGHT_SMALL, Geometry::ARMOR_WIDTH)
        .text_with_offset(label, 0, -Text::SIZE_MEDIUM, 0, L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", confidence))
        .build();
}

// ============================================================================
// L3: TRACKER VISUALIZATION
// ============================================================================

/// @brief Create tracker target sphere with status-based coloring
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity tracker_target(
    const Eigen::Vector3d& pos, int status_int, uint64_t timestamp_ns,
    std::string entity_id = "target") noexcept {
    using namespace tactical;

    return EntityBuilder::create<Frame>("l3", std::move(entity_id))
        .position(pos)
        .size(L3::TARGET_SIZE)
        .color(tracker_status_color(status_int))
        .sphere()
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create predicted armor sphere
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity predicted_armor(
    const Eigen::Vector3d& pos, int armor_id, uint64_t timestamp_ns,
    std::string entity_id = {}) noexcept {
    using namespace tactical;

    if (entity_id.empty()) {
        entity_id = fmt::format("armor_{}", armor_id);
    }

    return EntityBuilder::create<Frame>("l3", std::move(entity_id))
        .position(pos)
        .size(L3::ARMOR_PLATE_SIZE)
        .color(L3::PREDICTION_CONTEXT)
        .sphere()
        .text_with_offset(
            fmt::format("pred_{}", armor_id), 0, 0, L3::LABEL_OFFSET_Z, Text::SIZE_MEDIUM)
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create robot center marker
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity
    robot_center(const Eigen::Vector3d& pos, int status_int) noexcept {
    using namespace tactical;

    return EntityBuilder::create<Frame>("l3", "robot_center")
        .position(pos)
        .size(L3::ROBOT_CENTER_SIZE)
        .color(tracker_status_color(status_int))
        .sphere()
        .build();
}

/// @brief Create outpost center marker
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity
    outpost_center(const Eigen::Vector3d& pos, int status_int) noexcept {
    using namespace tactical;

    return EntityBuilder::create<Frame>("l3", "outpost_center")
        .position(pos)
        .size(L3::OUTPOST_CENTER_SIZE)
        .color(tracker_status_color(status_int))
        .sphere()
        .build();
}

/// @brief Create armor cube with tilt
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity
    armor_cube(const Eigen::Vector3d& pos, double yaw, bool is_big_armor) noexcept {
    using namespace tactical;

    const Eigen::Quaterniond q =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(Geometry::ARMOR_TILT_ANGLE, Eigen::Vector3d::UnitY());

    return EntityBuilder::create<Frame>("l3", "armor_plate")
        .position(pos)
        .orientation(q)
        .cube(
            Geometry::ARMOR_THICKNESS,
            is_big_armor ? Geometry::ARMOR_HEIGHT_BIG : Geometry::ARMOR_HEIGHT_SMALL,
            Geometry::ARMOR_WIDTH)
        .build();
}

// ============================================================================
// L4: GIMBAL VISUALIZATION
// ============================================================================

/// @brief Create MPC trajectory point with temporal fade
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity mpc_trajectory_point(
    ::foxglove::schemas::Vector3&& pos, int temporal_distance, bool is_reference,
    uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    const auto base_color  = is_reference ? L4::MPC_REFERENCE : L4::MPC_PRESENT;
    const auto faded_color = temporal_fade(base_color, temporal_distance);

    const std::string type = is_reference ? "reference" : "optimized";

    return EntityBuilder::create<Frame>("l4", fmt::format("mpc_{}_{}", type, temporal_distance))
        .position(pos)
        .size(L4::TRAJECTORY_DOT)
        .color(faded_color)
        .sphere()
        .metadata("temporal_offset", std::to_string(temporal_distance))
        .metadata("type", type)
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create bullet trajectory (sequence of dots + connecting line)
std::vector<::foxglove::schemas::SceneEntity> bullet_trajectory(
    const std::vector<Eigen::Vector3d>& points, bool can_fire, uint64_t timestamp_ns) noexcept;

// ============================================================================
// VELOCITY VISUALIZATION
// ============================================================================

namespace Vel {
constexpr double PARALLEL_THRESHOLD = 1e-6;

/// @brief Create linear velocity arrow
inline std::optional<::foxglove::schemas::ArrowPrimitive> linear_arrow(
    const ::foxglove::schemas::Vector3& position, const Eigen::Vector3d& velocity) noexcept {
    using namespace tactical;

    const double length = velocity.norm();
    if (length <= Velocity::ARROW_MIN_LENGTH) {
        return std::nullopt;
    }

    ::foxglove::schemas::ArrowPrimitive arrow;
    arrow.pose = make_pose(position);

    const Eigen::Vector3d vel_dir = velocity.normalized();
    const Eigen::Vector3d default_dir(1, 0, 0);
    Eigen::Quaterniond q;

    if ((vel_dir + default_dir).norm() < PARALLEL_THRESHOLD) {
        // 180 around Z: cos(pi/2)=0, sin(pi/2)*(0,0,1)=(0,0,1)
        q = Eigen::Quaterniond(0, 0, 0, 1);
    } else {
        // Cross-product quaternion: q = (1 + a*b, a x b), then normalize
        // Equivalent to FromTwoVectors but without JacobiSVD template explosion
        const double d             = default_dir.dot(vel_dir);
        const Eigen::Vector3d axis = default_dir.cross(vel_dir);
        q = Eigen::Quaterniond(1.0 + d, axis.x(), axis.y(), axis.z()).normalized();
    }

    arrow.pose->orientation = make_quaternion(q);
    arrow.shaft_length      = length * Velocity::ARROW_SHAFT_RATIO;
    arrow.shaft_diameter    = Velocity::ARROW_SHAFT_DIAMETER;
    arrow.head_length       = length * Velocity::ARROW_HEAD_RATIO;
    arrow.head_diameter     = Velocity::ARROW_HEAD_DIAMETER;
    arrow.color             = Velocity::LINEAR;

    return arrow;
}

/// @brief Create angular velocity arrow (along Z axis)
inline std::optional<::foxglove::schemas::ArrowPrimitive>
    angular_arrow(const ::foxglove::schemas::Vector3& position, double v_yaw) noexcept {
    using namespace tactical;

    const double length = std::abs(v_yaw) / M_PI;
    if (length <= Velocity::ARROW_MIN_LENGTH) {
        return std::nullopt;
    }

    ::foxglove::schemas::ArrowPrimitive arrow;
    arrow.pose = make_pose(position);

    const double sign       = v_yaw >= 0 ? -1.0 : 1.0;
    arrow.pose->orientation = make_quaternion(0.0, sign * 0.7071, 0.0, 0.7071);
    arrow.shaft_length      = length * Velocity::ARROW_SHAFT_RATIO;
    arrow.shaft_diameter    = Velocity::ARROW_SHAFT_DIAMETER;
    arrow.head_length       = length * Velocity::ARROW_HEAD_RATIO;
    arrow.head_diameter     = Velocity::ARROW_HEAD_DIAMETER;
    arrow.color             = Velocity::ANGULAR;

    return arrow;
}
} // namespace Vel

/// @brief Create entity with velocity arrows
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity velocity_arrows(
    const Eigen::Vector3d& pos, const Eigen::Vector3d& linear_vel, double angular_vel_yaw,
    uint64_t timestamp_ns, std::string entity_id = "velocity") noexcept {
    using namespace tactical;

    const auto vec3_pos = make_vector3(pos);

    auto entity =
        EntityBuilder::create<Frame>("l3", std::move(entity_id)).timestamp(timestamp_ns).build();

    if (auto linear_arrow = Vel::linear_arrow(vec3_pos, linear_vel)) {
        entity.arrows.push_back(*linear_arrow);
    }

    if (auto angular_arrow = Vel::angular_arrow(vec3_pos, angular_vel_yaw)) {
        entity.arrows.push_back(*angular_arrow);
    }

    return entity;
}

// ============================================================================
// UNCERTAINTY VISUALIZATION
// ============================================================================

/// @brief Create uncertainty sphere for covariance visualization
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity
    uncertainty_sphere(const Eigen::Vector3d& pos, double scale, uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    const double scaled_size = L3::TARGET_SIZE * L3::UNCERTAINTY_SCALE * scale;

    return EntityBuilder::create<Frame>("l3", "uncertainty")
        .position(pos)
        .size(scaled_size)
        .color(L3::UNCERTAINTY_ORANGE)
        .sphere()
        .metadata("scale", fmt::format("{:.3f}", scale))
        .timestamp(timestamp_ns)
        .build();
}

} // namespace patterns

} // namespace fcs::visualization
