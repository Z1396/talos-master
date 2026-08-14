#include "scene_builder.hpp"
// 引入头文件，本文件是EntityBuilder类的实现cpp文件，hpp放声明，cpp写函数体

namespace fcs::visualization {
// 进入可视化模块命名空间

// ============================================================================
// ENTITY BUILDER — POSITION & ORIENTATION
// EntityBuilder 位置姿态相关成员函数实现
// ============================================================================

/// @brief 使用x/y/z三个double数值设置builder内部当前位姿的位置
/// @return *this 返回自身引用，支持链式调用 .a().b().c()
EntityBuilder& EntityBuilder::position(double x, double y, double z) {
    // 调用工具函数make_vector3，把三个double转为foxglove proto的Vector3对象，赋值给current_pose的position字段
    current_pose_.position = make_vector3(x, y, z);
    return *this;
}

/// @brief 重载position：传入Eigen::Vector3d向量设置位置
EntityBuilder& EntityBuilder::position(const Eigen::Vector3d& pos) {
    // 拆解Eigen向量x/y/z，调用上面double版本的position
    return position(pos.x(), pos.y(), pos.z());
}

/// @brief 重载position：直接传入foxglove的Vector3消息对象设置位置
EntityBuilder& EntityBuilder::position(const ::foxglove::schemas::Vector3& pos) {
    current_pose_.position = pos;
    return *this;
}

/// @brief 使用四元数四个分量qx,qy,qz,qw设置builder当前姿态
EntityBuilder& EntityBuilder::orientation(double qx, double qy, double qz, double qw) {
    current_pose_.orientation = make_quaternion(qx, qy, qz, qw);
    return *this;
}

/// @brief 重载orientation：传入Eigen四元数对象设置姿态
EntityBuilder& EntityBuilder::orientation(const Eigen::Quaterniond& q) {
    // 取出Eigen四元数的四个分量，调用上面基础版本orientation
    return orientation(q.x(), q.y(), q.z(), q.w());
}

/// @brief 重载orientation：直接传入foxglove proto四元数对象
EntityBuilder& EntityBuilder::orientation(const ::foxglove::schemas::Quaternion& q) {
    current_pose_.orientation = q;
    return *this;
}

/// @brief 使用欧拉角(yaw偏航Z，pitch俯仰Y，roll横滚X)设置姿态
EntityBuilder& EntityBuilder::orientation_euler(double yaw, double pitch, double roll) {
    // Eigen::AngleAxisd：构造绕指定轴旋转的旋转量
    // 矩阵乘法顺序从右往左：先roll，再pitch，最后yaw；机器人常用Z‑Y‑X欧拉角
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // 把生成好的四元数交给orientation函数
    return orientation(q);
}

/// @brief 重置当前位置到原点(0,0,0)
EntityBuilder& EntityBuilder::reset_position() {
    current_pose_.position = make_vector3(0, 0, 0);
    return *this;
}

/// @brief 重置姿态为单位四元数，清除所有旋转
EntityBuilder& EntityBuilder::reset_orientation() {
    current_pose_.orientation = identity_quaternion();
    return *this;
}

// ============================================================================
// ENTITY BUILDER — VISUAL PROPERTIES
// EntityBuilder 视觉属性：颜色、尺寸、透明度
// ============================================================================

/// @brief 设置当前绘图颜色 r g b a 0~1浮点数
EntityBuilder& EntityBuilder::color(double r, double g, double b, double a) {
    // make_color构造foxglove Color对象存入current_color_（std::optional）
    current_color_ = make_color(r, g, b, a);
    return *this;
}

/// @brief 重载color：直接传入foxglove Color对象
EntityBuilder& EntityBuilder::color(const ::foxglove::schemas::Color& c) {
    current_color_ = c;
    return *this;
}

/// @brief 设置统一尺寸，xyz三轴大小相同，球体用这个
EntityBuilder& EntityBuilder::size(double s) {
    current_size_ = {s, s, s};
    return *this;
}

/// @brief 设置三轴独立尺寸 sx,sy,sz，立方体用这个
EntityBuilder& EntityBuilder::size(double sx, double sy, double sz) {
    current_size_ = {sx, sy, sz};
    return *this;
}

/// @brief 修改当前颜色的透明度alpha，保留rgb不变
EntityBuilder& EntityBuilder::alpha(double a) {
    // 如果current_color_有值才修改；没有设置过颜色就什么都不做
    if (current_color_.has_value()) {
        // protobuf内部颜色a是float，把输入double强转
        current_color_->a = static_cast<float>(a);
    }
    return *this;
}

// ============================================================================
// ENTITY BUILDER — PRIMITIVES
// 基础图元实现：sphere球体、cube立方体、text文字
// ============================================================================

/// @brief 添加球体图元，使用builder内部缓存的current_pose、current_size_、current_color_
EntityBuilder& EntityBuilder::sphere() {
    // 构造Foxglove球体图元对象
    ::foxglove::schemas::SpherePrimitive prim;
    // 图元位姿 = builder当前位姿
    prim.pose  = current_pose_;
    // value_or：如果current_size_没有设置，就使用默认{0.1,0.1,0.1}
    prim.size  = current_size_.value_or(::foxglove::schemas::Vector3{0.1, 0.1, 0.1});
    // 如果没有设置颜色，回退到战术配色L3::TRACKING_LOCKED
    prim.color = current_color_.value_or(tactical::L3::TRACKING_LOCKED);
    // 将球体推入SceneEntity的spheres数组，一个实体可以拥有多个球体
    entity_.spheres.push_back(prim);
    return *this;
}

/// @brief 添加立方体图元，宽/高/深度由入参指定，不使用current_size_
EntityBuilder& EntityBuilder::cube(double width, double height, double depth) {
    ::foxglove::schemas::CubePrimitive prim;
    prim.pose  = current_pose_;
    // cube尺寸完全由函数参数传入，不读取builder内部current_size_
    prim.size  = {width, height, depth};
    prim.color = current_color_.value_or(tactical::L2::MEASUREMENT_GHOST);
    entity_.cubes.push_back(prim);
    return *this;
}

/// @brief 添加文字标签，位置使用builder当前current_pose
EntityBuilder& EntityBuilder::text(std::string content, double font_size) {
    ::foxglove::schemas::TextPrimitive prim;
    // std::move把字符串所有权转移，避免拷贝
    prim.text      = std::move(content);
    prim.pose      = current_pose_;
    prim.font_size = font_size;
    prim.color     = current_color_.value_or(tactical::Text::PRIMARY);
    // billboard=true：文字永远朝向相机，不会随模型旋转歪掉
    prim.billboard = tactical::Text::BILLBOARD_ENABLED;
    entity_.texts.push_back(prim);
    return *this;
}

/// @brief 在当前pose基础上增加偏移量生成文字
EntityBuilder& EntityBuilder::text_with_offset(
    std::string content, double offset_x, double offset_y, double offset_z, double font_size) {
    ::foxglove::schemas::TextPrimitive prim;
    prim.text = std::move(content);

    // 复制一份当前位姿，不要修改原始current_pose（后续图元不受偏移影响）
    auto text_pose = current_pose_;
    // 如果position存在有效值，叠加偏移
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
// 线条、圆柱体图元实现
// ============================================================================

/// @brief line_strip连续折线：点0→1→2→3依次相连
EntityBuilder& EntityBuilder::line_strip(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    // 少于2个点画不出线，直接返回，不做任何操作
    if (points.size() < 2)
        return *this;
    ::foxglove::schemas::LinePrimitive line;
    // 设置线条类型为LINE_STRIP连续折线
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    line.pose      = current_pose_;
    line.thickness = thickness;
    // std::move转移vector所有权，避免拷贝大数组
    line.points    = std::move(points);
    line.color     = color;
    entity_.lines.push_back(std::move(line));
    return *this;
}

/// @brief line_list线段列表：两两一组，0‑1，2‑3，4‑5独立线段
EntityBuilder& EntityBuilder::line_list(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    // LINE_LIST需要偶数个点，最少4个点构成2条线段
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

/// @brief line_loop闭合环线：0‑1‑2‑…‑N‑0，自动闭合多边形
EntityBuilder& EntityBuilder::line_loop(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    // 闭合多边形至少需要3个点
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

/// @brief 添加圆柱体图元，diameter直径，height高度
EntityBuilder& EntityBuilder::cylinder(double diameter, double height) {
    ::foxglove::schemas::CylinderPrimitive prim;
    prim.pose         = current_pose_;
    // x/y为直径，z为高度；圆柱xy截面是圆形，x=y=diameter
    prim.size         = ::foxglove::schemas::Vector3{diameter, diameter, height};
    prim.color        = current_color_.value_or(tactical::L3::TRACKING_LOCKED);
    // bottom_scale、top_scale=1.0，上下底面不缩放，普通圆柱；改小可以做圆锥
    prim.bottom_scale = 1.0;
    prim.top_scale    = 1.0;
    entity_.cylinders.push_back(std::move(prim));
    return *this;
}

// ============================================================================
// ENTITY BUILDER — PRIMITIVE MERGING
// 直接追加外部已经构造好的完整图元对象
// ============================================================================

/// @brief 直接把外部已经构造好的CubePrimitive加入当前实体
EntityBuilder& EntityBuilder::add_cube(::foxglove::schemas::CubePrimitive cube) {
    entity_.cubes.push_back(std::move(cube));
    return *this;
}

/// @brief 追加外部球体图元
EntityBuilder& EntityBuilder::add_sphere(::foxglove::schemas::SpherePrimitive sphere) {
    entity_.spheres.push_back(std::move(sphere));
    return *this;
}

/// @brief 追加外部文字图元
EntityBuilder& EntityBuilder::add_text(::foxglove::schemas::TextPrimitive text) {
    entity_.texts.push_back(std::move(text));
    return *this;
}

/// @brief 追加外部箭头图元ArrowPrimitive（速度箭头）
EntityBuilder& EntityBuilder::add_arrow(::foxglove::schemas::ArrowPrimitive arrow) {
    entity_.arrows.push_back(std::move(arrow));
    return *this;
}

/// @brief 追加外部线条图元
EntityBuilder& EntityBuilder::add_line(::foxglove::schemas::LinePrimitive line) {
    entity_.lines.push_back(std::move(line));
    return *this;
}

/// @brief 追加外部圆柱体图元
EntityBuilder& EntityBuilder::add_cylinder(::foxglove::schemas::CylinderPrimitive cyl) {
    entity_.cylinders.push_back(std::move(cyl));
    return *this;
}

/// @brief merge_primitives：把另一个SceneEntity里面所有图元全部拷贝合并进当前builder
EntityBuilder& EntityBuilder::merge_primitives(const ::foxglove::schemas::SceneEntity& other) {
    // insert(目标尾部, 源begin,源end)，把other每一类图元全部追加到entity_对应数组
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
// 元数据、时间戳、实体生命周期
// ============================================================================

/// @brief 添加键‑值元数据，Foxglove面板可以查看该实体附加信息
EntityBuilder& EntityBuilder::metadata(std::string key, std::string value) {
    ::foxglove::schemas::KeyValuePair kv;
    kv.key   = std::move(key);
    kv.value = std::move(value);
    entity_.metadata.push_back(kv);
    return *this;
}

/// @brief 设置实体时间戳；ns单位纳秒，调用timestamp_from_ns工具函数转成foxglove Timestamp消息
EntityBuilder& EntityBuilder::timestamp(uint64_t ns) {
    entity_.timestamp = timestamp_from_ns(ns);
    return *this;
}

/// @brief 设置实体存活时长，单位纳秒；超时Foxglove自动删除该实体，不用手动清除
EntityBuilder& EntityBuilder::lifetime(uint64_t ns) {
    ::foxglove::schemas::Duration duration;
    // 纳秒转秒：除以1e9得到秒部分；取模得到剩余纳秒部分
    duration.sec  = static_cast<int32_t>(ns / 1000000000);
    duration.nsec = static_cast<uint32_t>(ns % 1000000000);
    entity_.lifetime = duration;
    return *this;
}

// ============================================================================
// ENTITY BUILDER — CLEAR & RETAIN
// 清空图元
// ============================================================================

/// @brief clear：清空所有图元数组；id、frame_id、metadata、timestamp、lifetime保留不变
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

/// @brief retain：保留id/frame_id/元数据，清空绘制图元；本实现直接调用clear()
EntityBuilder& EntityBuilder::retain() noexcept {
    clear();
    return *this;
}

// ============================================================================
// ENTITY BUILDER — BUILD
// build()生成最终SceneEntity消息对象，两个重载：右值 && 和 const&左值
// ============================================================================

/// @brief build() && 右值版本：builder对象即将销毁，使用std::move零拷贝转移内部entity_
::foxglove::schemas::SceneEntity EntityBuilder::build() && {
    // 如果没有设置timestamp，填充0时间戳
    if (!entity_.timestamp.has_value()) {
        entity_.timestamp = timestamp_from_ns(0);
    }
    // 如果没有手动设置lifetime，使用战术配置里全局默认实体存活时间
    if (!entity_.lifetime.has_value()) {
        const auto ns = tactical::Temporal::ENTITY_LIFETIME_NS;

        entity_.lifetime = ::foxglove::schemas::Duration{
            .sec  = static_cast<int32_t>(ns / 1000000000),
            .nsec = static_cast<uint32_t>(ns % 1000000000)};
    }
    // 移动语义：把内部entity_所有权交给返回值，builder之后失效
    return std::move(entity_);
}

/// @brief build() const& 左值版本：复制一份entity返回；builder对象还可以继续复用
::foxglove::schemas::SceneEntity EntityBuilder::build() const& {
    // 拷贝内部entity_到result副本
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
// patterns命名空间下，弹丸轨迹可视化实现
// ============================================================================

namespace patterns {

/// @brief 生成弹丸轨迹：点序列，根据can_fire选择颜色，返回一组SceneEntity
std::vector<::foxglove::schemas::SceneEntity> bullet_trajectory(
    const std::vector<Eigen::Vector3d>& points, bool can_fire, uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    // can_fire=true允许发射，使用发射状态颜色；否则使用hold待命颜色
    const auto color = can_fire ? L4::TRAJECTORY_FIRE : L4::TRAJECTORY_HOLD;

    std::vector<::foxglove::schemas::SceneEntity> entities;

    // 点数量≥2才可以绘制折线
    if (points.size() >= 2) {
        std::vector<::foxglove::schemas::Point3> line_points;
        // 预分配内存，避免多次vector扩容
        line_points.reserve(points.size());
        // 遍历Eigen点，转为foxglove Point3存入数组
        for (const auto& p : points) {
            line_points.push_back(make_point3(p));
        }
        // 创建muzzle坐标系下l4层bullet_traj_line实体，生成line_strip折线，推入entities
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