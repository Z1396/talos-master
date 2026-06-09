#include "scene_builder.hpp"

namespace fcs::visualization {

// ============================================================================
// ENTITY BUILDER — POSITION & ORIENTATION
// ============================================================================

EntityBuilder& EntityBuilder::position(double x, double y, double z) {
    current_pose_.position = make_vector3(x, y, z);
    return *this;
}

EntityBuilder& EntityBuilder::position(const Eigen::Vector3d& pos) {
    return position(pos.x(), pos.y(), pos.z());
}

EntityBuilder& EntityBuilder::position(const ::foxglove::schemas::Vector3& pos) {
    current_pose_.position = pos;
    return *this;
}

EntityBuilder& EntityBuilder::orientation(double qx, double qy, double qz, double qw) {
    current_pose_.orientation = make_quaternion(qx, qy, qz, qw);
    return *this;
}

EntityBuilder& EntityBuilder::orientation(const Eigen::Quaterniond& q) {
    return orientation(q.x(), q.y(), q.z(), q.w());
}

EntityBuilder& EntityBuilder::orientation(const ::foxglove::schemas::Quaternion& q) {
    current_pose_.orientation = q;
    return *this;
}

EntityBuilder& EntityBuilder::orientation_euler(double yaw, double pitch, double roll) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return orientation(q);
}

EntityBuilder& EntityBuilder::reset_position() {
    current_pose_.position = make_vector3(0, 0, 0);
    return *this;
}

EntityBuilder& EntityBuilder::reset_orientation() {
    current_pose_.orientation = identity_quaternion();
    return *this;
}

// ============================================================================
// ENTITY BUILDER — VISUAL PROPERTIES
// ============================================================================

EntityBuilder& EntityBuilder::color(double r, double g, double b, double a) {
    current_color_ = make_color(r, g, b, a);
    return *this;
}

EntityBuilder& EntityBuilder::color(const ::foxglove::schemas::Color& c) {
    current_color_ = c;
    return *this;
}

EntityBuilder& EntityBuilder::size(double s) {
    current_size_ = {s, s, s};
    return *this;
}

EntityBuilder& EntityBuilder::size(double sx, double sy, double sz) {
    current_size_ = {sx, sy, sz};
    return *this;
}

EntityBuilder& EntityBuilder::alpha(double a) {
    if (current_color_.has_value()) {
        current_color_->a = static_cast<float>(a);
    }
    return *this;
}

// ============================================================================
// ENTITY BUILDER — PRIMITIVES
// ============================================================================

EntityBuilder& EntityBuilder::sphere() {
    ::foxglove::schemas::SpherePrimitive prim;
    prim.pose  = current_pose_;
    prim.size  = current_size_.value_or(::foxglove::schemas::Vector3{0.1, 0.1, 0.1});
    prim.color = current_color_.value_or(tactical::L3::TRACKING_LOCKED);
    entity_.spheres.push_back(prim);
    return *this;
}

EntityBuilder& EntityBuilder::cube(double width, double height, double depth) {
    ::foxglove::schemas::CubePrimitive prim;
    prim.pose  = current_pose_;
    prim.size  = {width, height, depth};
    prim.color = current_color_.value_or(tactical::L2::MEASUREMENT_GHOST);
    entity_.cubes.push_back(prim);
    return *this;
}

EntityBuilder& EntityBuilder::text(std::string content, double font_size) {
    ::foxglove::schemas::TextPrimitive prim;
    prim.text      = std::move(content);
    prim.pose      = current_pose_;
    prim.font_size = font_size;
    prim.color     = current_color_.value_or(tactical::Text::PRIMARY);
    prim.billboard = tactical::Text::BILLBOARD_ENABLED;
    entity_.texts.push_back(prim);
    return *this;
}

EntityBuilder& EntityBuilder::text_with_offset(
    std::string content, double offset_x, double offset_y, double offset_z, double font_size) {
    ::foxglove::schemas::TextPrimitive prim;
    prim.text = std::move(content);

    auto text_pose = current_pose_;
    if (text_pose.position.has_value()) {
        text_pose.position->x += offset_x;
        text_pose.position->y += offset_y;
        text_pose.position->z += offset_z;
    }
    prim.pose      = text_pose;
    prim.font_size = font_size;
    prim.color     = current_color_.value_or(tactical::Text::PRIMARY);
    prim.billboard = tactical::Text::BILLBOARD_ENABLED;
    entity_.texts.push_back(prim);
    return *this;
}

// ============================================================================
// ENTITY BUILDER — LINES & CYLINDERS
// ============================================================================

EntityBuilder& EntityBuilder::line_strip(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    if (points.size() < 2)
        return *this;
    ::foxglove::schemas::LinePrimitive line;
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    line.pose      = current_pose_;
    line.thickness = thickness;
    line.points    = std::move(points);
    line.color     = color;
    entity_.lines.push_back(std::move(line));
    return *this;
}

EntityBuilder& EntityBuilder::line_list(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    if (points.size() < 4)
        return *this;
    ::foxglove::schemas::LinePrimitive line;
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    line.pose      = current_pose_;
    line.thickness = thickness;
    line.points    = std::move(points);
    line.color     = color;
    entity_.lines.push_back(std::move(line));
    return *this;
}

EntityBuilder& EntityBuilder::line_loop(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    if (points.size() < 3)
        return *this;
    ::foxglove::schemas::LinePrimitive line;
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    line.pose      = current_pose_;
    line.thickness = thickness;
    line.points    = std::move(points);
    line.color     = color;
    entity_.lines.push_back(std::move(line));
    return *this;
}

EntityBuilder& EntityBuilder::cylinder(double diameter, double height) {
    ::foxglove::schemas::CylinderPrimitive prim;
    prim.pose         = current_pose_;
    prim.size         = ::foxglove::schemas::Vector3{diameter, diameter, height};
    prim.color        = current_color_.value_or(tactical::L3::TRACKING_LOCKED);
    prim.bottom_scale = 1.0;
    prim.top_scale    = 1.0;
    entity_.cylinders.push_back(std::move(prim));
    return *this;
}

// ============================================================================
// ENTITY BUILDER — PRIMITIVE MERGING
// ============================================================================

EntityBuilder& EntityBuilder::add_cube(::foxglove::schemas::CubePrimitive cube) {
    entity_.cubes.push_back(std::move(cube));
    return *this;
}

EntityBuilder& EntityBuilder::add_sphere(::foxglove::schemas::SpherePrimitive sphere) {
    entity_.spheres.push_back(std::move(sphere));
    return *this;
}

EntityBuilder& EntityBuilder::add_text(::foxglove::schemas::TextPrimitive text) {
    entity_.texts.push_back(std::move(text));
    return *this;
}

EntityBuilder& EntityBuilder::add_arrow(::foxglove::schemas::ArrowPrimitive arrow) {
    entity_.arrows.push_back(std::move(arrow));
    return *this;
}

EntityBuilder& EntityBuilder::add_line(::foxglove::schemas::LinePrimitive line) {
    entity_.lines.push_back(std::move(line));
    return *this;
}

EntityBuilder& EntityBuilder::add_cylinder(::foxglove::schemas::CylinderPrimitive cyl) {
    entity_.cylinders.push_back(std::move(cyl));
    return *this;
}

EntityBuilder& EntityBuilder::merge_primitives(const ::foxglove::schemas::SceneEntity& other) {
    entity_.spheres.insert(entity_.spheres.end(), other.spheres.begin(), other.spheres.end());
    entity_.cubes.insert(entity_.cubes.end(), other.cubes.begin(), other.cubes.end());
    entity_.texts.insert(entity_.texts.end(), other.texts.begin(), other.texts.end());
    entity_.arrows.insert(entity_.arrows.end(), other.arrows.begin(), other.arrows.end());
    entity_.lines.insert(entity_.lines.end(), other.lines.begin(), other.lines.end());
    entity_.triangles.insert(
        entity_.triangles.end(), other.triangles.begin(), other.triangles.end());
    entity_.cylinders.insert(
        entity_.cylinders.end(), other.cylinders.begin(), other.cylinders.end());
    entity_.models.insert(entity_.models.end(), other.models.begin(), other.models.end());
    return *this;
}

// ============================================================================
// ENTITY BUILDER — METADATA
// ============================================================================

EntityBuilder& EntityBuilder::metadata(std::string key, std::string value) {
    ::foxglove::schemas::KeyValuePair kv;
    kv.key   = std::move(key);
    kv.value = std::move(value);
    entity_.metadata.push_back(kv);
    return *this;
}

EntityBuilder& EntityBuilder::timestamp(uint64_t ns) {
    entity_.timestamp = timestamp_from_ns(ns);
    return *this;
}

EntityBuilder& EntityBuilder::lifetime(uint64_t ns) {
    entity_.lifetime = ::foxglove::schemas::Duration{
        .sec  = static_cast<int32_t>(ns / 1000000000),
        .nsec = static_cast<uint32_t>(ns % 1000000000)};
    return *this;
}

// ============================================================================
// ENTITY BUILDER — CLEAR & RETAIN
// ============================================================================

EntityBuilder& EntityBuilder::clear() noexcept {
    entity_.spheres.clear();
    entity_.cubes.clear();
    entity_.texts.clear();
    entity_.arrows.clear();
    entity_.lines.clear();
    entity_.triangles.clear();
    entity_.cylinders.clear();
    entity_.models.clear();
    return *this;
}

EntityBuilder& EntityBuilder::retain() noexcept {
    clear();
    return *this;
}

// ============================================================================
// ENTITY BUILDER — BUILD
// ============================================================================

::foxglove::schemas::SceneEntity EntityBuilder::build() && {
    if (!entity_.timestamp.has_value()) {
        entity_.timestamp = timestamp_from_ns(0);
    }
    if (!entity_.lifetime.has_value()) {
        const auto ns = tactical::Temporal::ENTITY_LIFETIME_NS;

        entity_.lifetime = ::foxglove::schemas::Duration{
            .sec  = static_cast<int32_t>(ns / 1000000000),
            .nsec = static_cast<uint32_t>(ns % 1000000000)};
    }
    return std::move(entity_);
}

::foxglove::schemas::SceneEntity EntityBuilder::build() const& {
    ::foxglove::schemas::SceneEntity result = entity_;
    if (!result.timestamp.has_value()) {
        result.timestamp = timestamp_from_ns(0);
    }
    if (!result.lifetime.has_value()) {
        const auto ns = tactical::Temporal::ENTITY_LIFETIME_NS;

        result.lifetime = ::foxglove::schemas::Duration{
            .sec  = static_cast<int32_t>(ns / 1000000000),
            .nsec = static_cast<uint32_t>(ns % 1000000000)};
    }
    return result;
}

// ============================================================================
// PATTERNS — BULLET TRAJECTORY
// ============================================================================

namespace patterns {

std::vector<::foxglove::schemas::SceneEntity> bullet_trajectory(
    const std::vector<Eigen::Vector3d>& points, bool can_fire, uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    const auto color = can_fire ? L4::TRAJECTORY_FIRE : L4::TRAJECTORY_HOLD;

    std::vector<::foxglove::schemas::SceneEntity> entities;

    // Connect trajectory dots with a visible line strip
    if (points.size() >= 2) {
        std::vector<::foxglove::schemas::Point3> line_points;
        line_points.reserve(points.size());
        for (const auto& p : points) {
            line_points.push_back(make_point3(p));
        }
        entities.push_back(
            EntityBuilder::create<fast_tf::muzzle>("l4", "bullet_traj_line")
                .timestamp(timestamp_ns)
                .line_strip(std::move(line_points), color, L4::TRAJECTORY_LINE_THICKNESS)
                .build());
    }

    return entities;
}

} // namespace patterns

} // namespace fcs::visualization
