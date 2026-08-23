#include "scene_builder.hpp"
// 引入头文件，hpp存放类声明，本cpp存放函数实现

namespace fcs::visualization {
// 打开fcs::visualization命名空间，隔离模块符号，避免命名冲突

// ============================================================================
// ENTITY BUILDER — POSITION & ORIENTATION
// EntityBuilder 位置姿态相关成员函数实现
// ============================================================================

/// @brief 使用x/y/z三个double数值设置builder内部当前位姿的位置
/// @return *this 返回自身引用，支持链式调用 .a().b().c()
EntityBuilder& EntityBuilder::position(double x, double y, double z) {
    // 将x y z组装为foxglove的Vector3，赋值给内部缓存位姿的position
    current_pose_.position = make_vector3(x, y, z);
    // 返回本对象引用，实现链式调用
    return *this;
}

/// @brief 重载position：传入Eigen::Vector3d向量设置位置
EntityBuilder& EntityBuilder::position(const Eigen::Vector3d& pos) {
    // 取出eigen向量的x、y、z，调用double版本的position函数
    return position(pos.x(), pos.y(), pos.z());
}

/// @brief 重载position：直接传入foxglove的Vector3消息对象设置位置
EntityBuilder& EntityBuilder::position(const ::foxglove::schemas::Vector3& pos) {
    // 直接把外部传入的Vector3赋值给内部位姿
    current_pose_.position = pos;
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 使用四元数四个分量qx,qy,qz,qw设置builder当前姿态
EntityBuilder& EntityBuilder::orientation(double qx, double qy, double qz, double qw) {
    // 工具函数组装foxglove四元数对象，存入内部位姿orientation
    current_pose_.orientation = make_quaternion(qx, qy, qz, qw);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 重载orientation：传入Eigen四元数对象设置姿态
EntityBuilder& EntityBuilder::orientation(const Eigen::Quaterniond& q) {
    // 读取eigen四元数四个分量，调用基础版本orientation
    return orientation(q.x(), q.y(), q.z(), q.w());
}

/// @brief 重载orientation：直接传入foxglove proto四元数对象
EntityBuilder& EntityBuilder::orientation(const ::foxglove::schemas::Quaternion& q) {
    // 外部四元数直接赋值给内部缓存姿态
    current_pose_.orientation = q;
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 使用欧拉角(yaw偏航Z，pitch俯仰Y，roll横滚X)设置姿态
EntityBuilder& EntityBuilder::orientation_euler(double yaw, double pitch, double roll) {
    // 绕Z轴旋转yaw角
    Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());
    // 绕Y轴旋转pitch角
    Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
    // 绕X轴旋转roll角
    Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
    // Z‑Y‑X顺序相乘得到最终四元数，矩阵乘法从右往左生效
    Eigen::Quaterniond q = rot_z * rot_y * rot_x;
    // 复用orientation接口，把四元数写入builder内部
    return orientation(q);
}

/// @brief 重置当前位置到原点(0,0,0)
EntityBuilder& EntityBuilder::reset_position() {
    // 构造零向量，覆盖内部position，回到坐标原点
    current_pose_.position = make_vector3(0, 0, 0);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 重置姿态为单位四元数，清除所有旋转
EntityBuilder& EntityBuilder::reset_orientation() {
    // 获取单位四元数（无旋转）覆盖内部姿态
    current_pose_.orientation = identity_quaternion();
    // 返回自身引用，支持链式调用
    return *this;
}

// ============================================================================
// ENTITY BUILDER — VISUAL PROPERTIES
// EntityBuilder 视觉属性：颜色、尺寸、透明度
// ============================================================================

/// @brief 设置当前绘图颜色 r g b a 0~1浮点数
EntityBuilder& EntityBuilder::color(double r, double g, double b, double a) {
    // 生成foxglove颜色对象存入std::optional可选缓存
    current_color_ = make_color(r, g, b, a);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 重载color：直接传入foxglove Color对象
EntityBuilder& EntityBuilder::color(const ::foxglove::schemas::Color& c) {
    // 外部颜色对象赋值到内部可选缓存
    current_color_ = c;
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 设置统一尺寸，xyz三轴大小相同，球体用这个
EntityBuilder& EntityBuilder::size(double s) {
    // x y z全部赋值s，存入内部optional尺寸缓存
    current_size_ = {s, s, s};
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 设置三轴独立尺寸 sx,sy,sz，立方体用这个
EntityBuilder& EntityBuilder::size(double sx, double sy, double sz) {
    // 三轴分别设置不同尺寸
    current_size_ = {sx, sy, sz};
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 修改当前颜色的透明度alpha，保留rgb不变
EntityBuilder& EntityBuilder::alpha(double a) {
    // 判断optional是否已经存放颜色
    if (current_color_.has_value()) {
        // proto颜色alpha是float，double强转float赋值
        current_color_->a = static_cast<float>(a);
    }
    // 返回自身引用，支持链式调用
    return *this;
}

// ============================================================================
// ENTITY BUILDER — PRIMITIVES
// 基础图元实现：sphere球体、cube立方体、text文字
// ============================================================================

/// @brief 添加球体图元，使用builder内部缓存的current_pose、current_size_、current_color_
EntityBuilder& EntityBuilder::sphere() {
    // 定义球体图元结构体
    ::foxglove::schemas::SpherePrimitive prim;
    // 使用builder当前缓存的位姿
    prim.pose  = current_pose_;
    // 如果没有设置size，使用默认0.1m的尺寸
    prim.size  = current_size_.value_or(::foxglove::schemas::Vector3{0.1, 0.1, 0.1});
    // 如果没有设置颜色，回退到预设战术颜色
    prim.color = current_color_.value_or(tactical::L3::TRACKING_LOCKED);
    // 将球体推入实体的spheres数组
    entity_.spheres.push_back(prim);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 添加立方体图元，宽/高/深度由入参指定，不使用current_size_
EntityBuilder& EntityBuilder::cube(double width, double height, double depth) {
    // 定义立方体图元结构体
    ::foxglove::schemas::CubePrimitive prim;
    // 使用builder当前缓存位姿
    prim.pose  = current_pose_;
    // 立方体尺寸完全由入参传入，不读取内部current_size_
    prim.size  = {width, height, depth};
    // 未设置颜色，使用战术回退色
    prim.color = current_color_.value_or(tactical::L2::MEASUREMENT_GHOST);
    // 将立方体推入实体cubes数组
    entity_.cubes.push_back(prim);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 添加文字标签，位置使用builder当前current_pose
EntityBuilder& EntityBuilder::text(std::string content, double font_size) {
    // 定义文字图元结构体
    ::foxglove::schemas::TextPrimitive prim;
    // std::move转移字符串所有权，避免拷贝
    prim.text      = std::move(content);
    // 使用builder当前缓存位姿
    prim.pose      = current_pose_;
    // 设置字体大小
    prim.font_size = font_size;
    // 没有设置颜色，使用文本默认战术颜色
    prim.color     = current_color_.value_or(tactical::Text::PRIMARY);
    // billboard开启，文字永远朝向相机，不会旋转歪掉
    prim.billboard = tactical::Text::BILLBOARD_ENABLED;
    // 将文字推入实体texts数组
    entity_.texts.push_back(prim);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 在当前pose基础上增加偏移量生成文字
EntityBuilder& EntityBuilder::text_with_offset(
    std::string content, double offset_x, double offset_y, double offset_z, double font_size) {
    // 定义文字图元结构体
    ::foxglove::schemas::TextPrimitive prim;
    // move转移字符串，避免拷贝
    prim.text = std::move(content);
    // 复制一份当前位姿副本，不修改builder原始current_pose_
    auto text_pose = current_pose_;
    // 判断position字段是否有效
    if (text_pose.position.has_value()) {
        // x方向叠加偏移
        text_pose.position->x += offset_x;
        // y方向叠加偏移
        text_pose.position->y += offset_y;
        // z方向叠加偏移
        text_pose.position->z += offset_z;
    }
    // 文字使用叠加偏移后的位姿副本
    prim.pose      = text_pose;
    // 设置字体大小
    prim.font_size = font_size;
    // 未设置颜色使用文本默认战术色
    prim.color     = current_color_.value_or(tactical::Text::PRIMARY);
    // 开启广告牌模式，面向相机
    prim.billboard = tactical::Text::BILLBOARD_ENABLED;
    // 将文字推入实体texts数组
    entity_.texts.push_back(prim);
    // 返回自身引用，支持链式调用
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
    // 折线至少2个点，不足直接返回不绘制
    if (points.size() < 2)
        return *this;
    // 定义线条图元结构体
    ::foxglove::schemas::LinePrimitive line;
    // 设置线条类型：连续折线
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    // 使用builder当前缓存位姿
    line.pose      = current_pose_;
    // 设置线条粗细
    line.thickness = thickness;
    // move转移vector，避免点数组拷贝
    line.points    = std::move(points);
    // 设置线条颜色
    line.color     = color;
    // 将线条移动推入实体lines数组
    entity_.lines.push_back(std::move(line));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief line_list线段列表：两两一组，0‑1，2‑3，4‑5独立线段
EntityBuilder& EntityBuilder::line_list(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    // line_list最少4个点，点数不足直接返回
    if (points.size() < 4)
        return *this;
    // 定义线条图元结构体
    ::foxglove::schemas::LinePrimitive line;
    // 设置线条类型：两两配对独立线段
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    // 使用builder当前缓存位姿
    line.pose      = current_pose_;
    // 设置线条粗细
    line.thickness = thickness;
    // move转移点数组，避免拷贝
    line.points    = std::move(points);
    // 设置线条颜色
    line.color     = color;
    // 移动推入实体线条数组
    entity_.lines.push_back(std::move(line));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief line_loop闭合环线：0‑1‑2‑…‑N‑0，自动闭合多边形
EntityBuilder& EntityBuilder::line_loop(
    std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
    double thickness) {
    // 闭合多边形至少3个点，点数不足直接返回
    if (points.size() < 3)
        return *this;
    // 定义线条图元结构体
    ::foxglove::schemas::LinePrimitive line;
    // 设置线条类型：自动首尾闭合环线
    line.type      = ::foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    // 使用builder当前缓存位姿
    line.pose      = current_pose_;
    // 设置线条粗细
    line.thickness = thickness;
    // move转移点数组
    line.points    = std::move(points);
    // 设置线条颜色
    line.color     = color;
    // 移动推入实体线条数组
    entity_.lines.push_back(std::move(line));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 添加圆柱体图元，diameter直径，height高度
EntityBuilder& EntityBuilder::cylinder(double diameter, double height) {
    // 定义圆柱体图元结构体
    ::foxglove::schemas::CylinderPrimitive prim;
    // 使用builder当前缓存位姿
    prim.pose         = current_pose_;
    // x=y截面直径，z为圆柱高度
    prim.size         = ::foxglove::schemas::Vector3{diameter, diameter, height};
    // 未设置颜色使用战术回退色
    prim.color        = current_color_.value_or(tactical::L3::TRACKING_LOCKED);
    // 底面缩放系数，1.0代表正常圆柱
    prim.bottom_scale = 1.0;
    // 顶面缩放系数，1.0代表正常圆柱
    prim.top_scale    = 1.0;
    // 移动推入实体cylinders数组
    entity_.cylinders.push_back(std::move(prim));
    // 返回自身引用，支持链式调用
    return *this;
}

// ============================================================================
// ENTITY BUILDER — PRIMITIVE MERGING
// 直接追加外部已经构造好的完整图元对象
// ============================================================================

/// @brief 直接把外部已经构造好的CubePrimitive加入当前实体
EntityBuilder& EntityBuilder::add_cube(::foxglove::schemas::CubePrimitive cube) {
    // move外部立方体对象，追加到实体cube数组
    entity_.cubes.push_back(std::move(cube));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 追加外部球体图元
EntityBuilder& EntityBuilder::add_sphere(::foxglove::schemas::SpherePrimitive sphere) {
    // move外部球体对象，追加到实体spheres数组
    entity_.spheres.push_back(std::move(sphere));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 追加外部文字图元
EntityBuilder& EntityBuilder::add_text(::foxglove::schemas::TextPrimitive text) {
    // move外部文字对象，追加到实体texts数组
    entity_.texts.push_back(std::move(text));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 追加外部箭头图元ArrowPrimitive（速度箭头）
EntityBuilder& EntityBuilder::add_arrow(::foxglove::schemas::ArrowPrimitive arrow) {
    // move外部箭头对象，追加到实体arrows数组
    entity_.arrows.push_back(std::move(arrow));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 追加外部线条图元
EntityBuilder& EntityBuilder::add_line(::foxglove::schemas::LinePrimitive line) {
    // move外部线条对象，追加到实体lines数组
    entity_.lines.push_back(std::move(line));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 追加外部圆柱体图元
EntityBuilder& EntityBuilder::add_cylinder(::foxglove::schemas::CylinderPrimitive cyl) {
    // move外部圆柱体对象，追加到实体cylinders数组
    entity_.cylinders.push_back(std::move(cyl));
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief merge_primitives：把另一个SceneEntity里面所有图元全部拷贝合并进当前builder
EntityBuilder& EntityBuilder::merge_primitives(const ::foxglove::schemas::SceneEntity& other) {
    // 将other球体数组全部插入当前实体球体尾部
    entity_.spheres.insert(entity_.spheres.end(), other.spheres.begin(), other.spheres.end());
    // 合并立方体数组
    entity_.cubes.insert(entity_.cubes.end(), other.cubes.begin(), other.cubes.end());
    // 合并文字数组
    entity_.texts.insert(entity_.texts.end(), other.texts.begin(), other.texts.end());
    // 合并箭头数组
    entity_.arrows.insert(entity_.arrows.end(), other.arrows.begin(), other.arrows.end());
    // 合并线条数组
    entity_.lines.insert(entity_.lines.end(), other.lines.begin(), other.lines.end());
    // 合并三角面片数组
    entity_.triangles.insert(entity_.triangles.end(), other.triangles.begin(), other.triangles.end());
    // 合并圆柱体数组
    entity_.cylinders.insert(entity_.cylinders.end(), other.cylinders.begin(), other.cylinders.end());
    // 合并外部3D模型数组
    entity_.models.insert(entity_.models.end(), other.models.begin(), other.models.end());
    // 返回自身引用，支持链式调用
    return *this;
}

// ============================================================================
// ENTITY BUILDER — METADATA
// 元数据、时间戳、实体生命周期
// ============================================================================

/// @brief 添加键‑值元数据，Foxglove面板可以查看该实体附加信息
EntityBuilder& EntityBuilder::metadata(std::string key, std::string value) {
    // 定义键值对元数据结构体
    ::foxglove::schemas::KeyValuePair kv;
    // move转移key字符串
    kv.key   = std::move(key);
    // move转移value字符串
    kv.value = std::move(value);
    // 将kv键值对推入实体metadata数组
    entity_.metadata.push_back(kv);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 设置实体时间戳；ns单位纳秒，调用timestamp_from_ns工具函数转成foxglove Timestamp消息
EntityBuilder& EntityBuilder::timestamp(uint64_t ns) {
    // 工具函数把纳秒转为foxglove sec/nsec时间戳，赋值给实体
    entity_.timestamp = timestamp_from_ns(ns);
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief 设置实体存活时长，单位纳秒；超时Foxglove自动删除该实体，不用手动清除
EntityBuilder& EntityBuilder::lifetime(uint64_t ns) {
    // 定义foxglove时长结构体，包含sec、nsec
    ::foxglove::schemas::Duration duration;
    // 纳秒除以1e9得到完整秒数，强转int32_t
    duration.sec  = static_cast<int32_t>(ns / 1000000000);
    // 取模得到剩余纳秒部分，强转uint32_t
    duration.nsec = static_cast<uint32_t>(ns % 1000000000);
    // 将时长赋值给实体lifetime字段
    entity_.lifetime = duration;
    // 返回自身引用，支持链式调用
    return *this;
}

// ============================================================================
// ENTITY BUILDER — CLEAR & RETAIN
// 清空图元
// ============================================================================

/// @brief clear：清空所有图元数组；id、frame_id、metadata、timestamp、lifetime保留不变
EntityBuilder& EntityBuilder::clear() noexcept {
    // 清空球体数组
    entity_.spheres.clear();
    // 清空立方体数组
    entity_.cubes.clear();
    // 清空文字数组
    entity_.texts.clear();
    // 清空箭头数组
    entity_.arrows.clear();
    // 清空线条数组
    entity_.lines.clear();
    // 清空三角面片数组
    entity_.triangles.clear();
    // 清空圆柱体数组
    entity_.cylinders.clear();
    // 清空外部3D模型数组
    entity_.models.clear();
    // 返回自身引用，支持链式调用
    return *this;
}

/// @brief retain：保留id/frame_id/元数据，清空绘制图元；本实现直接调用clear()
EntityBuilder& EntityBuilder::retain() noexcept {
    // 调用clear清空全部图元
    clear();
    // 返回自身引用，支持链式调用
    return *this;
}

// ============================================================================
// ENTITY BUILDER — BUILD
// build()生成最终SceneEntity消息对象，两个重载：右值 && 和 const&左值
// ============================================================================

/// @brief build() && 右值版本：builder对象即将销毁，使用std::move零拷贝转移内部entity_
::foxglove::schemas::SceneEntity EntityBuilder::build() && {
    // 判断实体是否没有设置时间戳
    if (!entity_.timestamp.has_value()) {
        // 填充0纳秒时间戳兜底
        entity_.timestamp = timestamp_from_ns(0);
    }
    // 判断实体是否没有设置生命周期
    if (!entity_.lifetime.has_value()) {
        // 读取战术配置全局默认实体存活纳秒
        const auto ns = tactical::Temporal::ENTITY_LIFETIME_NS;
        // 组装duration，sec为秒，nsec为剩余纳秒
        entity_.lifetime = ::foxglove::schemas::Duration{
            .sec  = static_cast<int32_t>(ns / 1000000000),
            .nsec = static_cast<uint32_t>(ns % 1000000000)};
    }
    // move转移内部entity_，builder对象之后失效
    return std::move(entity_);
}

/// @brief build() const& 左值版本：复制一份entity返回；builder对象还可以继续复用
::foxglove::schemas::SceneEntity EntityBuilder::build() const& {
    // 拷贝entity_到result副本，builder本体不受影响
    ::foxglove::schemas::SceneEntity result = entity_;
    // 如果副本没有时间戳
    if (!result.timestamp.has_value()) {
        // 填充0时间戳兜底
        result.timestamp = timestamp_from_ns(0);
    }
    // 如果副本没有生命周期
    if (!result.lifetime.has_value()) {
        // 读取全局战术默认存活时间
        const auto ns = tactical::Temporal::ENTITY_LIFETIME_NS;
        // 组装duration
        result.lifetime = ::foxglove::schemas::Duration{
            .sec  = static_cast<int32_t>(ns / 1000000000),
            .nsec = static_cast<uint32_t>(ns % 1000000000)};
    }
    // 返回拷贝出来的实体副本
    return result;
}

// ============================================================================
// PATTERNS — BULLET TRAJECTORY
// patterns命名空间下，弹丸轨迹可视化实现
// ============================================================================

namespace patterns {
// 打开patterns子命名空间

/// @brief 生成弹丸轨迹：点序列，根据can_fire选择颜色，返回一组SceneEntity
std::vector<::foxglove::schemas::SceneEntity> bullet_trajectory(
    const std::vector<Eigen::Vector3d>& points, bool can_fire, uint64_t timestamp_ns) noexcept {
    // 引入tactical命名空间，简化配色书写
    using namespace tactical;
    // 根据can_fire布尔选择轨迹颜色
    const auto color = can_fire ? L4::TRAJECTORY_FIRE : L4::TRAJECTORY_HOLD;
    // 创建容器存放生成好的可视化实体
    std::vector<::foxglove::schemas::SceneEntity> entities;
    // 轨迹点数量大于等于2才绘制折线
    if (points.size() >= 2) {
        // 定义foxglove格式点数组
        std::vector<::foxglove::schemas::Point3> line_points;
        // 预分配内存，避免vector多次扩容
        line_points.reserve(points.size());
        // 遍历每一个Eigen三维点
        for (const auto& p : points) {
            // 将Eigen向量转为foxglove Point3存入数组
            line_points.push_back(make_point3(p));
        }
        // 创建builder，生成轨迹实体，推入entities
        entities.push_back(
            EntityBuilder::create<fast_tf::muzzle>("l4", "bullet_traj_line")
                .timestamp(timestamp_ns)
                .line_strip(std::move(line_points), color, L4::TRAJECTORY_LINE_THICKNESS)
                .build());
    }
    // 返回生成完成的实体列表
    return entities;
}

} // namespace patterns
// 关闭patterns子命名空间

} // namespace fcs::visualization
// 关闭fcs::visualization命名空间
