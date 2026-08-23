#pragma once
// 头文件保护，防止多次include造成重复定义编译错误

// This header provides a type‑safe, composable builder pattern for creating
// Foxglove scene entities with semantic styling.
// 本头文件提供类型安全、可组合的建造者模式，用于构建Foxglove 3D场景实体，搭配语义化样式配色
//
// Design Philosophy:
// - Fluent interface: chain methods for readable code
//   流式接口：支持方法链式调用，代码可读性高
// - Semantic defaults: colors, sizes from tactical palette
//   语义化默认值：颜色、尺寸全部取自tactical_palette战术可视化配置表，不写魔法数字
// - Convenience patterns: common operations in single function calls
//   封装常用模板：高频可视化逻辑封装为简短函数直接调用
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
// 使用示例：链式调用构建一个球体3D图元，最后build()生成Foxglove的SceneEntity消息结构体

#include "frame.hpp"
// fast‑tf坐标系框架，fast_tf::frame模板，静态编译期坐标系标识
#include "tactical_palette.hpp"
// 可视化常量：颜色、尺寸、文字大小、透明度全部定义在此文件

#include <Eigen/Core>
// Eigen基础向量矩阵库
#include <cmath>
// 数学库 M_PI、三角函数
#include <fmt/core.h>
// fmt字符串格式化库，拼接id、元数据文本
#include <foxglove/schemas.hpp>
// Foxglove protobuf生成的消息定义：Vector3、Pose、CubePrimitive、SceneEntity等
#include <optional>
// std::optional，表达可选字段，值有效/无效
#include <string>
// std::string字符串
#include <utility.hpp>
// 项目内部工具库，移动语义、通用工具
#include <vector>
// std::vector容器，存储点集、图元数组

namespace fcs::visualization {
// fcs工程顶层命名空间，visualization可视化模块

// ============================================================================
// UTILITY FUNCTIONS (inline — required by templates below)
// 工具辅助函数，全部inline，头文件模板需要看到完整函数实现
// ============================================================================

/// @brief Create Vector3 from components
/// @brief 通过x/y/z三个数值构造Foxglove Vector3消息
inline ::foxglove::schemas::Vector3 make_vector3(double x, double y, double z) noexcept {
    ::foxglove::schemas::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

/// @brief Create Vector3 from Eigen::Vector3d
/// @brief 将Eigen三维向量转为Foxglove Vector3消息
inline ::foxglove::schemas::Vector3 make_vector3(const Eigen::Vector3d& vec) noexcept {
    return make_vector3(vec.x(), vec.y(), vec.z());
}

/// @brief Create Point3 from components
/// @brief 由x,y,z构造Foxglove Point3（用于线条点集，和Vector3结构体字段完全一样，proto中是两种不同类型）
inline ::foxglove::schemas::Point3 make_point3(double x, double y, double z) noexcept {
    ::foxglove::schemas::Point3 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

/// @brief Create Point3 from Eigen::Vector3d
/// @brief Eigen向量转Foxglove Point3
inline ::foxglove::schemas::Point3 make_point3(const Eigen::Vector3d& vec) noexcept {
    return make_point3(vec.x(), vec.y(), vec.z());
}

/// @brief Create Point3 from Vector3
/// @brief Foxglove Vector3 对象转 Point3 对象
inline ::foxglove::schemas::Point3 make_point3(const ::foxglove::schemas::Vector3& v) noexcept {
    return make_point3(v.x, v.y, v.z);
}

/// @brief Create Quaternion from components
/// @brief 四元数x y z w直接构造Foxglove四元数消息
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
/// @brief Eigen四元数转为Foxglove proto四元数
inline ::foxglove::schemas::Quaternion make_quaternion(const Eigen::Quaterniond& q) noexcept {
    return make_quaternion(q.x(), q.y(), q.z(), q.w());
}

/// @brief Create identity quaternion
/// @brief 返回单位四元数：无旋转 (0,0,0,1)
inline ::foxglove::schemas::Quaternion identity_quaternion() noexcept {
    return make_quaternion(0.0, 0.0, 0.0, 1.0);
}

/// @brief Create Color from components
/// @brief 构造Foxglove颜色，r/g/b/a 0~1浮点数
inline ::foxglove::schemas::Color make_color(double r, double g, double b, double a) noexcept {
    ::foxglove::schemas::Color c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

/// @brief Create Pose from position and orientation
/// @brief 构造位姿Pose，位置、姿态都是std::optional，可以不传
inline ::foxglove::schemas::Pose make_pose(
    std::optional<::foxglove::schemas::Vector3> position       = std::nullopt,
    std::optional<::foxglove::schemas::Quaternion> orientation = std::nullopt) noexcept {
    ::foxglove::schemas::Pose pose;
    pose.position    = position;
    pose.orientation = orientation;
    return pose;
}

/// @brief Create Pose from position with identity orientation
/// @brief 只传入位置，姿态使用单位四元数（无旋转）
inline ::foxglove::schemas::Pose make_pose(const ::foxglove::schemas::Vector3& position) noexcept {
    return make_pose(position, identity_quaternion());
}

// ============================================================================
// ENTITY BUILDER
// 场景实体建造者核心类 EntityBuilder
// ============================================================================

/// @brief Fluent builder for Foxglove SceneEntity
/// @brief Foxglove SceneEntity 的流式建造器
///
/// Provides a type‑safe, composable interface for building scene entities
/// with semantic styling from the tactical palette.
/// 提供类型安全、可组合接口，基于战术配色构建3D场景实体
///
/// Thread Safety: Not thread‑safe. Create and use builders within a single thread.
/// 线程安全：非线程安全，同一个builder只能在单线程创建使用
///
/// Design Notes:
/// - SceneEntity does not have a top‑level pose; each primitive has its own pose
///   Foxglove SceneEntity本身没有顶层位姿；**每一个图元(cube/sphere/text)各自携带pose**
/// - This builder maintains a "current_pose" that gets applied to primitives
///   builder内部保存current_pose，新增图元时把当前位姿赋值给图元
/// - Call position()/orientation() before adding primitives to set their pose
///   添加图元之前调用position()/orientation()，后续图元就会复用这套位姿
class EntityBuilder {
public:
    /// @brief Create a new EntityBuilder with frame template parameter
    /// @brief 静态工厂函数，模板参数T为fast_tf编译期坐标系
    template <fast_tf::frame T>
    [[nodiscard]] static EntityBuilder create(std::string layer, std::string entity_id) {
        EntityBuilder builder;
        // 拼接实体唯一ID：layer.entity_id，Foxglove依靠id做实体更新/覆盖
        builder.entity_.id       = fmt::format("{}.{}", layer, entity_id);
        // frame_id 设置为模板传入坐标系的字符串名称，fast_tf编译期获取
        builder.entity_.frame_id = T::frame_id;
        builder.layer_           = std::move(layer);
        return builder;
    }

    // ========================================================================
    // POSITION & ORIENTATION 位置与姿态接口
    // ========================================================================

    /// @brief Set position from components
    /// @brief 设置当前builder内部current_pose的位置，x/y/z数值
    EntityBuilder& position(double x, double y, double z);
    /// @brief Set position from Eigen::Vector3d
    /// @brief Eigen向量设置当前位置
    EntityBuilder& position(const Eigen::Vector3d& pos);
    /// @brief Set position from Vector3
    /// @brief Foxglove Vector3消息设置当前位置
    EntityBuilder& position(const ::foxglove::schemas::Vector3& pos);

    /// @brief Set orientation from quaternion components
    /// @brief 四元数四个分量设置当前姿态
    EntityBuilder& orientation(double qx, double qy, double qz, double qw);
    /// @brief Set orientation from Eigen::Quaterniond
    /// @brief Eigen四元数设置姿态
    EntityBuilder& orientation(const Eigen::Quaterniond& q);
    /// @brief Set orientation from Quaternion
    /// @brief Foxglove proto四元数设置姿态
    EntityBuilder& orientation(const ::foxglove::schemas::Quaternion& q);
    /// @brief Set orientation from Euler angles (yaw, pitch, roll)
    /// @brief 欧拉角(yaw偏航 pitch俯仰 roll横滚)设置姿态
    EntityBuilder& orientation_euler(double yaw, double pitch, double roll);

    /// @brief Reset position to origin (for primitives at origin in current frame)
    /// @brief 重置current_pose位置回到原点(0,0,0)
    EntityBuilder& reset_position();
    /// @brief Reset orientation to identity
    /// @brief 重置姿态为单位四元数，清除旋转
    EntityBuilder& reset_orientation();

    // ========================================================================
    // VISUAL PROPERTIES 视觉属性：颜色、大小、透明度
    // ========================================================================

    /// @brief Set color from components
    /// @brief 设置builder当前颜色 r g b a
    EntityBuilder& color(double r, double g, double b, double a = 1.0);
    /// @brief Set color from Color
    /// @brief 使用Foxglove Color对象设置当前颜色
    EntityBuilder& color(const ::foxglove::schemas::Color& c);

    /// @brief Set size (uniform for all axes)
    /// @brief 设置统一尺寸，xyz三轴大小相同（球体使用）
    EntityBuilder& size(double s);
    /// @brief Set size from components
    /// @brief 设置三轴独立尺寸 sx sy sz（立方体使用）
    EntityBuilder& size(double sx, double sy, double sz);
    /// @brief Set alpha (transparency) for current color
    /// @brief 修改当前颜色透明度alpha，保留rgb不变
    EntityBuilder& alpha(double a);

    // ========================================================================
    // PRIMITIVES 基础图元接口：球体、立方体、文字
    // ========================================================================

    /// @brief Add a sphere primitive with current pose, size, and color
    /// @brief 使用当前pose、size、color，向entity添加球体图元
    EntityBuilder& sphere();

    /// @brief Add a cube primitive with specified dimensions
    /// @brief 添加立方体，传入宽高深
    EntityBuilder& cube(double width, double height, double depth);

    /// @brief Add text label
    /// @brief 添加文字标签，使用当前pose位置
    EntityBuilder& text(std::string content, double font_size = tactical::Text::SIZE_MEDIUM);

    /// @brief Add text label at offset from current position
    /// @brief 在当前位置基础上增加偏移，添加文字
    EntityBuilder& text_with_offset(
        std::string content, double offset_x, double offset_y, double offset_z,
        double font_size = tactical::Text::SIZE_MEDIUM);

    // ========================================================================
    // LINES & CYLINDERS 线条、圆柱体图元
    // ========================================================================

    /// @brief Add a line strip connecting sequential points (0‑>1‑>2‑>...‑>N)
    /// @brief 连续线条：点0连点1，点1连点2，依次向后
    EntityBuilder& line_strip(
        std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
        double thickness = 0.01);

    /// @brief Add individual line segments: 0‑>1, 2‑>3, 4‑>5, ...
    /// @brief 线段列表：两两一组绘制线段；0‑1为一条，2‑3为第二条
    EntityBuilder& line_list(
        std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
        double thickness = 0.01);

    /// @brief Add a closed polygon: 0‑>1‑>...‑>N‑>0
    /// @brief 闭合环线：最后一个点连回第一个点，形成多边形
    EntityBuilder& line_loop(
        std::vector<::foxglove::schemas::Point3> points, const ::foxglove::schemas::Color& color,
        double thickness = 0.01);

    /// @brief Add a cylinder primitive (usable as ring indicator)
    /// @brief 添加圆柱体，可以用来画圆环标记
    EntityBuilder& cylinder(double diameter, double height);

    // ========================================================================
    // PRIMITIVE MERGING (encapsulation‑safe)
    // 直接追加外部已经构造好的完整图元对象
    // ========================================================================

    /// @brief Add a pre‑built cube primitive directly
    /// @brief 传入已经构造完成的CubePrimitive，直接加入场景实体
    EntityBuilder& add_cube(::foxglove::schemas::CubePrimitive cube);
    /// @brief Add a pre‑built sphere primitive directly
    EntityBuilder& add_sphere(::foxglove::schemas::SpherePrimitive sphere);
    /// @brief Add a pre‑built text primitive directly
    EntityBuilder& add_text(::foxglove::schemas::TextPrimitive text);
    /// @brief Add a pre‑built arrow primitive directly
    EntityBuilder& add_arrow(::foxglove::schemas::ArrowPrimitive arrow);
    /// @brief Add a pre‑built line primitive directly
    EntityBuilder& add_line(::foxglove::schemas::LinePrimitive line);
    /// @brief Add a pre‑built cylinder primitive directly
    EntityBuilder& add_cylinder(::foxglove::schemas::CylinderPrimitive cyl);

    /// @brief Merge all primitives from another SceneEntity into this builder
    /// @brief 将另一个SceneEntity里面全部图元合并到当前builder内部
    EntityBuilder& merge_primitives(const ::foxglove::schemas::SceneEntity& other);

    // ========================================================================
    // METADATA 元数据、时间戳、实体生命周期
    // ========================================================================

    /// @brief Add metadata key‑value pair
    /// @brief 添加键值对元数据，Foxglove可以查看该实体附加信息
    EntityBuilder& metadata(std::string key, std::string value);
    /// @brief Set timestamp for the entity
    /// @brief 设置该场景实体时间戳，单位纳秒
    EntityBuilder& timestamp(uint64_t ns);
    /// @brief Set entity lifetime (nanoseconds)
    /// @brief 设置实体存活时长；超时Foxglove自动删除该实体，不用手动清
    EntityBuilder& lifetime(uint64_t ns);

    // ========================================================================
    // CLEAR & RETAIN 清空图元
    // ========================================================================

    /// @brief Clear all accumulated primitives
    /// @brief 清空所有图元，元数据、id、frame_id全部清空
    EntityBuilder& clear() noexcept;

    /// @brief Clear primitives but retain metadata
    /// @brief 只清空绘制图元；保留id、frame_id、metadata、时间戳
    EntityBuilder& retain() noexcept;

    // ========================================================================
    // BUILD 生成最终SceneEntity消息对象
    // ========================================================================

    /// @brief Build and return the SceneEntity (moves builder)
    /// @brief 右值版本：build()之后builder对象不再可用，移动语义，零拷贝
    [[nodiscard]] ::foxglove::schemas::SceneEntity build() &&;
    /// @brief Build and return the SceneEntity (copies)
    /// @brief const左值版本：复制一份返回，builder还可以继续复用
    [[nodiscard]] ::foxglove::schemas::SceneEntity build() const&;

private:
    EntityBuilder() = default;
    // 私有构造函数，外部不能直接new，强制使用静态create<>工厂函数创建
    ::foxglove::schemas::SceneEntity entity_;
    // 保存正在构建的Foxglove场景实体消息
    std::string layer_;
    // 记录当前层级字符串 l2/l3/l4
    ::foxglove::schemas::Pose current_pose_{};
    // builder内部缓存当前位姿，新增图元会复制这套pose
    std::optional<::foxglove::schemas::Color> current_color_;
    // 当前设置的绘图颜色，std::optional未设置时为空
    std::optional<::foxglove::schemas::Vector3> current_size_;
    // 当前设置的尺寸大小
};

// ============================================================================
// CONVENIENCE PATTERNS 预制模板：封装各层常用可视化，直接调用即可
// ============================================================================

namespace patterns {
// patterns命名空间，存放预制好的可视化快捷模板函数

// ============================================================================
// L2: MEASUREMENT VISUALIZATION 感知层L2检测观测可视化
// ============================================================================

/// @brief Create a ghost measurement sphere (L2 perception layer)
/// @brief L2层观测球体；置信度不同显示不同颜色
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity measurement_sphere(
    const Eigen::Vector3d& pos, double confidence, uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    // 置信度大于0.5使用正常测量颜色；否则ghost伪观测颜色
    const auto color = (confidence > 0.5) ? L2::MEASUREMENT_CONFIDENCE : L2::MEASUREMENT_GHOST;

    return EntityBuilder::create<Frame>("l2", "measurement")
        .position(pos)
        .size(L2::ARMOR_SIZE)
        .color(color)
        .sphere()
        // 把置信度写入元数据，Foxglove可以查看
        .metadata("confidence", fmt::format("{:.3f}", confidence))
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create a measurement cube with full pose information
/// @brief 带完整姿态的观测立方体，模拟装甲板检测结果
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity measurement_cube(
    const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation, const std::string& label,
    double confidence = 0.0) noexcept {
    using namespace tactical;

    return EntityBuilder::create<Frame>("l2", "measurement")
        .position(pos)
        .orientation(orientation)
        // 装甲真实尺寸：厚度、小装甲高度、宽度
        .cube(Geometry::ARMOR_THICKNESS, Geometry::ARMOR_HEIGHT_SMALL, Geometry::ARMOR_WIDTH)
        // 文字向下偏移，贴在装甲板旁边
        .text_with_offset(label, 0, -Text::SIZE_MEDIUM, 0, L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", confidence))
        .build();
}

// ============================================================================
// L3: TRACKER VISUALIZATION 状态估计L3跟踪器可视化
// ============================================================================

/// @brief Create tracker target sphere with status‑based coloring
/// @brief 跟踪目标球体；根据跟踪状态status_int自动切换颜色
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity tracker_target(
    const Eigen::Vector3d& pos, int status_int, uint64_t timestamp_ns,
    std::string entity_id = "target") noexcept {
    using namespace tactical;

    return EntityBuilder::create<Frame>("l3", std::move(entity_id))
        .position(pos)
        .size(L3::TARGET_SIZE)
        // tracker_status_color 根据状态码返回对应颜色
        .color(tracker_status_color(status_int))
        .sphere()
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create predicted armor sphere
/// @brief 预测装甲点球体，显示滤波器预测出的装甲位置
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity predicted_armor(
    const Eigen::Vector3d& pos, int armor_id, uint64_t timestamp_ns,
    std::string entity_id = {}) noexcept {
    using namespace tactical;

    // 如果没有传入entity_id，自动生成armor_0 armor_1
    if (entity_id.empty()) {
        entity_id = fmt::format("armor_{}", armor_id);
    }

    return EntityBuilder::create<Frame>("l3", std::move(entity_id))
        .position(pos)
        .size(L3::ARMOR_PLATE_SIZE)
        .color(L3::PREDICTION_CONTEXT)
        .sphere()
        // z轴向上偏移，打上pred_0标签
        .text_with_offset(
            fmt::format("pred_{}", armor_id), 0, 0, L3::LABEL_OFFSET_Z, Text::SIZE_MEDIUM)
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create robot center marker
/// @brief 机器人中心标记球体
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
/// @brief 前哨站中心标记球体
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
/// @brief 生成带倾斜角度的装甲立方体；yaw水平面旋转，加上装甲向内倾斜角
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity
    armor_cube(const Eigen::Vector3d& pos, double yaw, bool is_big_armor) noexcept {
    using namespace tactical;

    // 姿态合成：先Y轴倾斜装甲板，再Z轴yaw水平旋转
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
// L4: GIMBAL VISUALIZATION L4层云台MPC规划可视化
// ============================================================================

/// @brief Create MPC trajectory point with temporal fade
/// @brief MPC轨迹点；temporal_distance时间距离越远颜色越淡
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity mpc_trajectory_point(
    ::foxglove::schemas::Vector3&& pos, int temporal_distance, bool is_reference,
    uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    // 参考轨迹 / 优化轨迹基础颜色
    const auto base_color  = is_reference ? L4::MPC_REFERENCE : L4::MPC_PRESENT;
    // temporal_fade：根据时间距离做颜色淡化
    const auto faded_color = temporal_fade(base_color, temporal_distance);

    const std::string type = is_reference ? "reference" : "optimized";

    return EntityBuilder::create<Frame>("l4", fmt::format("mpc_{}_{}", type, temporal_distance))
        .position(pos)
        .size(L4::TRAJECTORY_DOT)
        .color(faded_color)
        .sphere()
        // 元数据记录时间偏移、轨迹类型
        .metadata("temporal_offset", std::to_string(temporal_distance))
        .metadata("type", type)
        .timestamp(timestamp_ns)
        .build();
}

/// @brief Create bullet trajectory (sequence of dots + connecting line)
/// @brief 弹丸轨迹，返回一组SceneEntity：一串点+连线
std::vector<::foxglove::schemas::SceneEntity> bullet_trajectory(
    const std::vector<Eigen::Vector3d>& points, bool can_fire, uint64_t timestamp_ns) noexcept;

// ============================================================================
// VELOCITY VISUALIZATION 速度箭头可视化子模块
// ============================================================================

namespace Vel {
// Vel速度绘制子命名空间
constexpr double PARALLEL_THRESHOLD = 1e-6;
// 判断向量平行阈值

/// @brief Create linear velocity arrow
/// @brief 生成线速度箭头；速度太小返回std::nullopt，不绘制
inline std::optional<::foxglove::schemas::ArrowPrimitive> linear_arrow(
    const ::foxglove::schemas::Vector3& position, const Eigen::Vector3d& velocity) noexcept {
    using namespace tactical;

    const double length = velocity.norm();
    // 速度模长小于最小阈值，返回空，不渲染箭头
    if (length <= Velocity::ARROW_MIN_LENGTH) {
        return std::nullopt;
    }

    ::foxglove::schemas::ArrowPrimitive arrow;
    arrow.pose = make_pose(position);

    const Eigen::Vector3d vel_dir = velocity.normalized();
    // 默认箭头指向X轴正方向
    const Eigen::Vector3d default_dir(1, 0, 0);
    Eigen::Quaterniond q;

    // 特殊情况：速度方向与X轴反向（几乎‑X）
    if ((vel_dir + default_dir).norm() < PARALLEL_THRESHOLD) {
        // 绕Z轴旋转180度
        q = Eigen::Quaterniond(0, 0, 0, 1);
    } else {
        // 通用：两个向量之间生成旋转四元数，不使用FromTwoVectors，避免SVD模板膨胀编译变慢
        const double d             = default_dir.dot(vel_dir);
        const Eigen::Vector3d axis = default_dir.cross(vel_dir);
        q = Eigen::Quaterniond(1.0 + d, axis.x(), axis.y(), axis.z()).normalized();
    }

    arrow.pose->orientation = make_quaternion(q);
    // 箭杆长度、直径；箭头头部长度、直径，全部按速度模长缩放
    arrow.shaft_length      = length * Velocity::ARROW_SHAFT_RATIO;
    arrow.shaft_diameter    = Velocity::ARROW_SHAFT_DIAMETER;
    arrow.head_length       = length * Velocity::ARROW_HEAD_RATIO;
    arrow.head_diameter     = Velocity::ARROW_HEAD_DIAMETER;
    arrow.color             = Velocity::LINEAR;

    return arrow;
}

/// @brief Create angular velocity arrow (along Z axis)
/// @brief 生成yaw角速度箭头；正负控制旋转指示方向
inline std::optional<::foxglove::schemas::ArrowPrimitive>
    angular_arrow(const ::foxglove::schemas::Vector3& position, double v_yaw) noexcept {
    using namespace tactical;

    const double length = std::abs(v_yaw) / M_PI;
    // 角速度太小直接返回空，不绘制
    if (length <= Velocity::ARROW_MIN_LENGTH) {
        return std::nullopt;
    }

    ::foxglove::schemas::ArrowPrimitive arrow;
    arrow.pose = make_pose(position);

    // v_yaw正负，切换箭头朝向
    const double sign       = v_yaw >= 0 ? -1.0 : 1.0;
    // 固定四元数，把箭头指向Y轴方向
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
/// @brief 组合线速度箭头 + yaw角速度箭头，生成完整SceneEntity
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity velocity_arrows(
    const Eigen::Vector3d& pos, const Eigen::Vector3d& linear_vel, double angular_vel_yaw,
    uint64_t timestamp_ns, std::string entity_id = "velocity") noexcept {
    using namespace tactical;

    const auto vec3_pos = make_vector3(pos);

    // 先创建空实体
    auto entity =
        EntityBuilder::create<Frame>("l3", std::move(entity_id)).timestamp(timestamp_ns).build();

    // 如果线速度箭头有效，加入arrows数组
    if (auto linear_arrow = Vel::linear_arrow(vec3_pos, linear_vel)) {
        entity.arrows.push_back(*linear_arrow);
    }

    // 如果角速度箭头有效，加入arrows数组
    if (auto angular_arrow = Vel::angular_arrow(vec3_pos, angular_vel_yaw)) {
        entity.arrows.push_back(*angular_arrow);
    }

    return entity;
}

// ============================================================================
// UNCERTAINTY VISUALIZATION 滤波器协方差不确定性可视化
// ============================================================================

/// @brief Create uncertainty sphere for covariance visualization
/// @brief 不确定性球体，scale对应协方差大小，越大球体越大
template <fast_tf::frame Frame>
inline ::foxglove::schemas::SceneEntity
    uncertainty_sphere(const Eigen::Vector3d& pos, double scale, uint64_t timestamp_ns) noexcept {
    using namespace tactical;

    // 基础尺寸 × 全局不确定性缩放系数 × 当前scale
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