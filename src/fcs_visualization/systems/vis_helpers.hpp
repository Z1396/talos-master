#pragma once
// 头文件保护，防止该头文件被多次 include 重复编译

#include "L3_estimation/tracker/types.hpp"
// 导入L3层目标跟踪器的数据类型定义：TrackerOutput、RobotTargetState、OutpostTargetState等
#include "core/armor_types.hpp"
// 核心装甲类型：ArmorName、ArmorType、cls_to_armor_type 装甲类型转换函数
#include "scene_builder.hpp"
// Foxglove场景构建工具：viz::EntityBuilder，用于往场景里添加立方体、文字、图元
#include "tactical_palette.hpp"
// 可视化战术常量：装甲尺寸、倾斜角度、文字大小、颜色、偏移量等配置常量

#include <Eigen/Core>
// Eigen基础矩阵向量库，Vector3d等
#include <Eigen/Geometry>
// Eigen姿态四元数、旋转轴AngleAxisd
#include <fmt/format.h>
// fmt格式化库，用来拼接字符串，生成装甲编号文本
#include <optional>
// C++17 std::optional，用来表达“值存在/不存在”，替代裸指针表达无效数据

namespace fcs::L3::vis {
// fcs工程，L3状态估计层，vis可视化子命名空间

// Shorthand 别名简写，减少重复敲长命名空间
namespace viz = fcs::visualization;
namespace tac = visualization::tactical;

// ============================================================================
// Tracker Output Helper Functions - Tracker 输出辅助函数
// ============================================================================
//
// 这些函数从 TrackerOutput 中提取位置、速度等信息用于可视化。
// 原本位于 foxglove_systems_impl.hpp，现已移动到独立的 L3 模块。
// ============================================================================

/// @brief Get center position from tracker output as optional Vector3
/// @return 有效目标返回位置，无效状态返回 std::nullopt
[[nodiscard]]
// [[nodiscard]] 编译器属性：调用该函数如果丢弃返回值，编译器给出警告，防止漏处理返回的optional
inline std::optional<::foxglove::schemas::Vector3>
// inline：建议编译器函数内联，减少函数调用开销；返回optional，目标无效就没有位置
get_tracker_position(const TrackerOutput& output) noexcept {
// noexcept：承诺函数不会抛出C++异常，提升性能
    if (output.is_robot()) {
        // 判断跟踪输出的目标类型：是否为机器人目标
        const auto& state = *output.robot_state();
        // 获取机器人目标状态结构体，解引用得到state引用
        return viz::make_vector3(state.position.x(), state.position.y(), state.position.z());
        // 调用工具函数，把Eigen::Vector3d转为Foxglove协议的Vector3消息对象并返回
    }
    if (output.is_outpost()) {
        // 判断是否为前哨站目标
        const auto& state  = *output.outpost_state();
        // 获取前哨站目标状态
        const double avg_z = (state.z[0] + state.z[1] + state.z[2]) / 3.0;
        // 前哨站有3块装甲，取三块装甲z坐标平均值，作为前哨站中心高度
        return viz::make_vector3(state.position.x(), state.position.y(), avg_z);
    }
    return std::nullopt;
    // 既不是机器人也不是前哨站，返回空optional，表示没有可用位置
}

/// @brief Get velocity from tracker output
[[nodiscard]]
inline Eigen::Vector3d get_tracker_velocity(const TrackerOutput& output) noexcept {
    if (output.is_robot()) {
        return output.robot_state()->velocity;
        // 机器人目标：直接返回Eigen三维速度向量
    }
    if (output.is_outpost()) {
        return output.outpost_state()->velocity;
        // 前哨站目标：返回前哨站速度
    }
    return Eigen::Vector3d::Zero();
    // 无效目标，返回零向量作为兜底
}

/// @brief Get angular velocity from tracker output
[[nodiscard]]
inline double get_tracker_v_yaw(const TrackerOutput& output) noexcept {
    if (output.is_robot()) {
        return output.robot_state()->v_yaw;
        // 返回机器人yaw方向角速度（偏航角速度）
    }
    if (output.is_outpost()) {
        return output.outpost_state()->v_yaw;
        // 返回前哨站yaw角速度
    }
    return 0.0;
    // 无效目标兜底返回0
}

// ============================================================================
// Armor Visualization Helpers - 装甲板可视化辅助函数
// ============================================================================

/// @brief 创建单个装甲板立方体
/// @param armor_pose 装甲板位姿 [x, y, z, yaw]
/// @param tilt_angle 装甲板倾斜角（前哨站为负，机器人为正）
/// @param height 装甲板高度（大/小装甲板）
[[nodiscard]]
inline ::foxglove::schemas::CubePrimitive
make_armor_cube(const Eigen::Vector4d& armor_pose, double tilt_angle, double height) noexcept {
    // armor_pose:4维向量：x,y,z位置 + yaw偏航角；tilt_angle装甲俯仰倾斜；height装甲板高度
    const Eigen::Quaterniond q = Eigen::AngleAxisd(armor_pose[3], Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(tilt_angle, Eigen::Vector3d::UnitY());
    // 姿态合成：
    // 第一步绕Z轴旋转yaw：装甲在水平面旋转
    // 第二步绕Y轴旋转tilt_angle：装甲板向内/向外倾斜
    // 矩阵乘法顺序注意：右乘，先执行Y轴倾斜，再执行Z轴偏航

    ::foxglove::schemas::CubePrimitive cube;
    // 实例化Foxglove的立方体图元消息，用来在3D场景绘制一块装甲板
    cube.pose = viz::make_pose(
        viz::make_vector3(armor_pose[0], armor_pose[1], armor_pose[2]), viz::make_quaternion(q));
    // 组装pose：位置 + 四元数姿态，赋值给cube的位姿
    cube.size  = {tac::Geometry::ARMOR_THICKNESS, height, tac::Geometry::ARMOR_WIDTH};
    // 立方体三维尺寸：厚度、高度、宽度，全部取自tactical_palette配置常量
    cube.color = viz::make_color(0.75, 0.75, 0.75, 0.25);
    // RGBA颜色：浅灰色，alpha=0.25半透明，方便观察内部
    return cube;
    // 返回构造完成的立方体图元对象
}

/// @brief Add armor cubes to scene entity for outpost target
inline void
add_outpost_armor_cubes(viz::EntityBuilder& builder, const OutpostTargetState& state) noexcept {
// builder：场景构建器引用，往这个builder追加图元；state前哨站跟踪状态
    const auto armor_poses = state.armor_poses();
    // 获取前哨站三块装甲板的位姿数组
    static_assert(armor_poses.size() == 3);
    // 编译期静态断言：前哨站必须严格等于3块装甲，编译阶段就报错，不等到运行时

    for (const auto& armor_pose : armor_poses) {
        // 遍历每一块装甲位姿
        // 前哨站装甲板向下倾斜，使用小装甲板高度
        builder.add_cube(make_armor_cube(
            armor_pose, -tac::Geometry::ARMOR_TILT_ANGLE, tac::Geometry::ARMOR_HEIGHT_SMALL));
        // 倾斜角度取负值：装甲板向下倾斜；高度使用小装甲高度
        // make_armor_cube生成cube，直接交给builder添加到场景
    }
}

/// @brief Add armor cubes to scene entity for robot target
inline void add_robot_armor_cubes(
    viz::EntityBuilder& builder, const RobotTargetState& state, ArmorName target_name) noexcept {
// builder：场景构建器；state机器人跟踪状态；target_name机器人类型（英雄/步兵等）
    // 复用项目统一的装甲尺寸判断逻辑：英雄(One)、基地大装甲(BaseLarge) 为大装甲
    const bool is_big_armor = cls_to_armor_type(target_name) == ArmorType::Large;
    // cls_to_armor_type：转换机器人类型 → 装甲大小枚举；判断是否使用大装甲尺寸
    const auto armor_poses  = state.armor_poses();
    // 获取机器人4块装甲板位姿数组
    static_assert(armor_poses.size() == 4);
    // 编译期断言：机器人装甲固定4块，编译检查

    for (int i = 0; i < 4; i++) {
        // 循环遍历0~3号四块装甲
        // 机器人装甲板向上倾斜
        auto cube = make_armor_cube(
            armor_poses[i], tac::Geometry::ARMOR_TILT_ANGLE,
            is_big_armor ? tac::Geometry::ARMOR_HEIGHT_BIG : tac::Geometry::ARMOR_HEIGHT_SMALL);
        // 倾斜角度正数，装甲向外向上倾斜；三目运算符选择大/小装甲高度

        ::foxglove::schemas::TextPrimitive text;
        // Foxglove文本图元：在装甲上方打印装甲编号0/1/2/3
        text.text = fmt::format("{}", i);
        // fmt格式化，把装甲下标i转为字符串
        text.pose = cube.pose;
        // 文字初始位姿和装甲立方体保持一致
        if (text.pose->position.has_value()) {
            // 判断pose里面position字段是否有效（protobuf optional字段）
            text.pose->position->z += tac::L3::LABEL_OFFSET_Z;
            // z方向向上偏移一段距离，文字浮在装甲板上方，不会贴在装甲表面
        }
        text.font_size = tac::Text::SIZE_SMALL;
        // 设置文字字号，取自配置常量
        text.color     = tac::Text::PRIMARY;
        // 文字颜色
        text.billboard = tac::Text::BILLBOARD_ENABLED;
        // billboard：广告牌模式，文字永远朝向相机，不会随装甲旋转而歪掉

        builder.add_cube(cube);
        // 将装甲立方体加入场景
        builder.add_text(std::move(text));
        // std::move移动语义，把text所有权转移给builder，避免拷贝
    }
}

} // namespace fcs::L3::vis