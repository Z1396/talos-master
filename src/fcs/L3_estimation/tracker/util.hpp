/**
 * @file util.hpp
 * @brief 跟踪器核心工具函数与状态定义
 *
 * 本文件提供了跟踪器所需的共享定义和工具函数，包括：
 * - 状态空间索引定义（机器人、前哨站）
 * - 测量空间索引定义
 * - 坐标变换函数（Cartesian ↔ 球坐标）
 * - 角度处理工具（归一化、展开、差值）
 * - 装甲板位置计算
 *
 * 核心设计原则：
 * - **状态参数化**：使用对数半径参数化（log(r)），保证半径恒为正
 * - **角度归一化**：所有角度归一化到[-π, π)，避免多值性
 * - **坐标系统一**：使用ROS REP-103约定（yaw: atan2(y,x), pitch: atan2(-z, r_xy)）
 * - **零开销抽象**：使用enum和inline函数，编译期优化，无运行时开销
 *
 * 数学基础：
 * - 球坐标变换：[x,y,z] → [yaw, pitch, distance]
 * - Jacobian矩阵：用于EKF的线性化
 * - 角度展开：处理周期性，避免±π跳变
 *
 * 应用场景：
 * - EKF状态向量索引访问
 * - 测量模型坐标变换
 * - 装甲板几何计算
 */

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
// 状态空间索引定义
// ============================================================================

/**
 * @brief 机器人状态空间索引（11维）
 *
 * 状态向量：[xc, vx, yc, vy, z0, vz, yaw, v_yaw, log_r0, log_r1, h]
 *
 * 状态建模：
 * - **中心位置**：(xc, yc) 是装甲板中心在XY平面的投影
 * - **速度状态**：(vx, vy, vz) 是中心速度，使用恒定速度模型（CV）
 * - **高度结构**：z0是装甲板0和2的高度，h是装甲板1和3的相对高度（z1 = z0 + h）
 * - **旋转状态**：yaw是装甲板0的朝向角，v_yaw是角速度
 * - **半径参数化**：log_r0和log_r1是对数化半径，保证半径恒为正（r = exp(log_r)）
 *
 * 几何关系：
 * - 装甲板0：位于 (xc - r0*cos(yaw), yc - r0*sin(yaw), z0)
 * - 装甲板1：位于 (xc - r1*cos(yaw+π/2), yc - r1*sin(yaw+π/2), z0+h)
 * - 装甲板2：位于 (xc - r0*cos(yaw+π), yc - r0*sin(yaw+π), z0)
 * - 装甲板3：位于 (xc - r1*cos(yaw+3π/2), yc - r1*sin(yaw+3π/2), z0+h)
 *
 * @note 11维状态向量适用于4装甲板机器人目标跟踪
 */
enum RoboState : uint8_t {
    XC,        ///< 中心X坐标（米）
    VX,        ///< X方向速度（米/秒）
    YC,        ///< 中心Y坐标（米）
    VY,        ///< Y方向速度（米/秒）
    Z0,        ///< 装甲板0和2的Z坐标（米）
    VZ,        ///< Z方向速度（米/秒）
    YAW,       ///< 装甲板0的yaw角度（弧度）
    V_YAW,     ///< 角速度（弧度/秒）
    LOG_R0,    ///< 装甲板0和2的对数半径（log(米)），保证半径为正
    LOG_R1,    ///< 装甲板1和3的对数半径（log(米)）
    H,         ///< 高度差：z1 - z0（米），表示装甲板1和3的相对高度
    STATE_MAX, ///< 状态维度（枚举计数用）
};

/**
 * @brief 前哨站状态空间索引（7维）
 *
 * 状态向量：[xc, yc, yaw, v_yaw, z0, z1, z2]
 *
 * 状态建模：
 * - **固定半径**：前哨站为固定结构，半径已知（不估计）
 * - **固定速度**：前哨站静止，无速度状态
 * - **旋转状态**：只有yaw和v_yaw，表示旋转
 * - **独立高度**：三个装甲板高度独立估计
 *
 * @note 7维状态向量适用于3装甲板前哨站目标跟踪
 */
enum OutpostState : uint8_t {
    O_XC,        ///< 中心X坐标（米）
    O_YC,        ///< 中心Y坐标（米）
    O_YAW,       ///< 装甲板0的yaw角度（弧度）
    O_VYAW,      ///< 角速度（弧度/秒）
    O_Z0,        ///< 装甲板0的Z坐标（米）
    O_Z1,        ///< 装甲板1的Z坐标（米）
    O_Z2,        ///< 装甲板2的Z坐标（米）
    O_STATE_MAX, ///< 状态维度（枚举计数用）
};

/**
 * @brief 测量空间索引（4维）
 *
 * 测量向量：[yaw, pitch, log(distance), armor_yaw]
 *
 * 测量建模：
 * - **球坐标观测**：使用yaw/pitch/distance，避免距离对角度的数值耦合
 * - **对数距离**：log(distance)，提高近距离精度，避免距离主导
 * - **装甲板朝向**：armor_yaw是装甲板法向量的yaw角，用于可见性判断
 *
 * @note 4维测量向量适用于单装甲板观测
 */
enum Measure : uint8_t {
    ARMOR_YAW,       ///< 装甲板方位角（yaw）：atan2(y, x)（弧度）
    ARMOR_PITCH,     ///< 装甲板俯仰角（pitch）：atan2(-z, sqrt(x²+y²))（弧度）
    ARMOR_DISTANCE,  ///< 装甲板距离（对数）：log(sqrt(x²+y²+z²))（log(米)）
    ARMOR_YAW_ARMOR, ///< 装甲板朝向角：装甲板法向量的yaw（弧度）
    MEASURE_MAX,     ///< 测量维度（枚举计数用）
};

// ============================================================================
// 枚举标签辅助函数（基于magic_enum）
// ============================================================================

/**
 * @brief 获取RoboState索引的标签名称
 * @param i 状态索引
 * @return 状态标签（如"XC", "VX", "YAW"）
 * @note 使用magic_enum实现，零开销抽象
 */
[[nodiscard]] inline std::string_view robo_state_label(int i) noexcept {
    return magic_enum::enum_name(static_cast<RoboState>(i));
}

/**
 * @brief 获取OutpostState索引的标签名称
 * @param i 状态索引
 * @return 状态标签（如"O_XC", "O_YAW"）
 */
[[nodiscard]] inline std::string_view outpost_state_label(int i) noexcept {
    return magic_enum::enum_name(static_cast<OutpostState>(i));
}

/**
 * @brief 获取Measure索引的标签名称
 * @param i 测量索引
 * @return 测量标签（如"ARMOR_YAW", "ARMOR_PITCH"）
 */
[[nodiscard]] inline std::string_view measure_label(int i) noexcept {
    return magic_enum::enum_name(static_cast<Measure>(i));
}

// ============================================================================
// 坐标变换函数
// ============================================================================

/**
 * @brief Cartesian坐标转球坐标（ROS REP-103约定）
 *
 * 坐标系统义：
 * - **yaw**：绕+Z轴旋转，XY平面逆时针为正，范围[-π, π)
 * - **pitch**：绕+Y轴旋转，向下为正，范围[-π/2, π/2]
 * - **distance**：欧氏距离，范围[0, +∞)
 *
 * 变换公式：
 * - yaw = atan2(y, x)
 * - pitch = atan2(-z, sqrt(x²+y²))
 * - distance = sqrt(x²+y²+z²)
 *
 * @param xyz Cartesian坐标 [x, y, z]（米）
 * @return 球坐标 [yaw, pitch, distance]（弧度, 弧度, 米）
 *
 * @note 符合ROS REP-103标准，pitch向下为正
 * @note 使用std::hypot提升数值稳定性
 */
[[nodiscard]] inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) noexcept {
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();

    const double distance   = xyz.norm();
    const double yaw        = std::atan2(y, x);
    const double horizontal = std::hypot(x, y); // 比sqrt(x²+y²)更精确且快速
    const double pitch      = std::atan2(-z, horizontal);

    return {yaw, pitch, distance};
}

/**
 * @brief xyz2ypd变换的Jacobian矩阵：∂(yaw, pitch, distance)/∂(x, y, z)
 *
 * 数学推导：
 * - ∂yaw/∂x = -y/(x²+y²)
 * - ∂yaw/∂y = x/(x²+y²)
 * - ∂yaw/∂z = 0
 *
 * - ∂pitch/∂x = x*z/(r²*r_xy)
 * - ∂pitch/∂y = y*z/(r²*r_xy)
 * - ∂pitch/∂z = -r_xy/r²
 *
 * - ∂distance/∂x = x/r
 * - ∂distance/∂y = y/r
 * - ∂distance/∂z = z/r
 *
 * 应用场景：
 * - 用于EKF测量模型的线性化：H = ∂h/∂x
 * - 通过链式法则计算状态到测量的映射Jacobian
 *
 * @param xyz Cartesian坐标 [x, y, z]（米）
 * @return 3x3 Jacobian矩阵
 *
 * @note 奇异点处理：在原点或Z轴上返回单位矩阵（避免除零）
 */
[[nodiscard]] inline Eigen::Matrix3d xyz2ypd_jacobian(const Eigen::Vector3d& xyz) noexcept {
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();

    const double r2_xy = x * x + y * y;
    const double r2    = r2_xy + z * z;
    const double r_xy  = std::sqrt(r2_xy);
    const double r     = std::sqrt(r2);

    // 奇异点处理：原点或Z轴上返回单位矩阵
    if (r_xy < 1e-10 || r < 1e-10) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d J;

    // ∂yaw/∂(x,y,z)，其中yaw = atan2(y, x)
    J(0, 0) = -y / r2_xy;
    J(0, 1) = x / r2_xy;
    J(0, 2) = 0.0;

    // ∂pitch/∂(x,y,z)，其中pitch = atan2(-z, sqrt(x²+y²))
    // 使用链式法则：d/dx[atan2(-z, r_xy)] = x*z / (r² * r_xy)
    const double denom = r2 * r_xy;
    J(1, 0)            = x * z / denom;
    J(1, 1)            = y * z / denom;
    J(1, 2)            = -r_xy / r2;

    // ∂distance/∂(x,y,z)，其中distance = sqrt(x²+y²+z²)
    J(2, 0) = x / r;
    J(2, 1) = y / r;
    J(2, 2) = z / r;

    return J;
}

/**
 * @brief 球坐标转Cartesian坐标
 *
 * 变换公式：
 * - x = distance * cos(pitch) * cos(yaw)
 * - y = distance * cos(pitch) * sin(yaw)
 * - z = -distance * sin(pitch)
 *
 * @param ypd 球坐标 [yaw, pitch, distance]（弧度, 弧度, 米）
 * @return Cartesian坐标 [x, y, z]（米）
 */
[[nodiscard]] inline Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd) noexcept {
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    const double cp = std::cos(pitch);
    const double x  = distance * cp * std::cos(yaw);
    const double y  = distance * cp * std::sin(yaw);
    const double z  = -distance * std::sin(pitch); // 注意负号：pitch向下为正

    return {x, y, z};
}

/**
 * @brief ypd2xyz变换的Jacobian矩阵：∂(x, y, z)/∂(yaw, pitch, distance)
 *
 * 数学推导：
 * - x = r*cos(pitch)*cos(yaw)
 * - y = r*cos(pitch)*sin(yaw)
 * - z = -r*sin(pitch)
 *
 * @param ypd 球坐标 [yaw, pitch, distance]
 * @return 3x3 Jacobian矩阵
 */
[[nodiscard]] inline Eigen::Matrix3d ypd2xyz_jacobian(const Eigen::Vector3d& ypd) noexcept {
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    Eigen::Matrix3d J;
    // x = r * cp * cy
    J(0, 0) = -distance * cp * sy; // dx/dyaw
    J(0, 1) = -distance * sp * cy; // dx/dpitch
    J(0, 2) = cp * cy;             // dx/dr

    // y = r * cp * sy
    J(1, 0) = distance * cp * cy;  // dy/dyaw
    J(1, 1) = -distance * sp * sy; // dy/dpitch
    J(1, 2) = cp * sy;             // dy/dr

    // z = -r * sp
    J(2, 0) = 0.0;            // dz/dyaw
    J(2, 1) = -distance * cp; // dz/dpitch
    J(2, 2) = -sp;            // dz/dr

    return J;
}

// ============================================================================
// 角度处理工具
// ============================================================================

/**
 * @brief 归一化角度到[-π, π)
 *
 * 算法：使用std::remainder实现高精度归一化
 *
 * @param a 输入角度（弧度）
 * @return 归一化角度（弧度），范围[-π, π)
 *
 * @note 比手动减法更精确，避免累积误差
 */
[[nodiscard]] inline double normalize_rad(double a) noexcept {
    double r = std::remainder(a, 2.0 * std::numbers::pi);

    // 处理边界情况：remainder可能返回-π，需转换为π
    if (r <= -std::numbers::pi)
        r += 2.0 * std::numbers::pi;

    return r;
}

/**
 * @brief 计算最短角度差：to - from，结果在(-π, π]
 *
 * 应用场景：
 * - 计算角度误差（如新息）
 * - 避免角度跳变（如从179°到-179°差值应为2°而非-358°）
 *
 * @param from 起始角度（弧度）
 * @param to 目标角度（弧度）
 * @return 最短角度差（弧度）
 */
[[nodiscard]] inline double shortest_rad(double from, double to) noexcept {
    return normalize_rad(to - from);
}

/**
 * @brief 归一化角度（单参数版本）
 * @param angle 输入角度（弧度）
 * @return 归一化角度（弧度）
 */
[[nodiscard]] inline double shortest_rad(double angle) noexcept { return normalize_rad(angle); }

/**
 * @brief 角度展开（基于前值）
 *
 * 目的：保持角度连续性，避免±π跳变
 *
 * 应用场景：
 * - Sigma点变换中的角度展开
 * - 轨迹预测中的角度平滑
 *
 * @param prev 前一角度（弧度）
 * @param raw 当前原始角度（弧度）
 * @return 展开后的角度（弧度）
 *
 * @note 例如：prev=170°, raw=-170° → 返回190°
 */
[[nodiscard]] inline double unwrap_rad(double prev, double raw) noexcept {
    const double d = shortest_rad(prev, raw);
    return prev + d;
}

/**
 * @brief 归一化角度到(-180, 180]度
 *
 * @param a 输入角度（度）
 * @return 归一化角度（度）
 */
[[nodiscard]] inline double normalize_deg(double a) noexcept {
    a = std::fmod(a + 180.0, 360.0);
    if (a <= 0.0) {
        a += 360.0;
    }
    return a - 180.0;
}

/**
 * @brief 计算最短角度差（度）
 * @param from 起始角度（度）
 * @param to 目标角度（度）
 * @return 最短角度差（度）
 */
[[nodiscard]] inline double shortest_deg(double from, double to) noexcept {
    return normalize_deg(to - from);
}

/**
 * @brief 归一化角度（度）
 * @param angle 输入角度（度）
 * @return 归一化角度（度）
 */
[[nodiscard]] inline double shortest_deg(double angle) noexcept { return normalize_deg(angle); }

/**
 * @brief 角度展开（度）
 * @param prev 前一角度（度）
 * @param raw 当前原始角度（度）
 * @return 展开后的角度（度）
 */
[[nodiscard]] inline double unwrap_deg(double prev, double raw) noexcept {
    const double d = shortest_deg(prev, raw);
    return prev + d;
}

/**
 * @brief 判断装甲板是否可见（从原点观察）
 *
 * 可见性条件：
 * - 装甲板法向量必须指向观察者方向（夹角 < 90°）
 * - 即：|bearing_yaw - armor_yaw| < π/2
 *
 * 几何意义：
 * - bearing_yaw：从原点到装甲板的方位角
 * - armor_yaw：装甲板法向量的yaw角
 * - 若夹角过大，说明装甲板背对观察者，不可见
 *
 * @param bearing_yaw 装甲板方位角（弧度）
 * @param armor_yaw 装甲板朝向角（弧度）
 * @return true表示可见，false表示不可见
 *
 * @note 用于过滤不可见装甲板，减少误匹配
 */
[[nodiscard]] inline bool
    armor_face_visible_from_origin(double bearing_yaw, double armor_yaw) noexcept {
    if (!std::isfinite(bearing_yaw) || !std::isfinite(armor_yaw)) {
        return false;
    }
    constexpr double kMaxVisibleYawError = std::numbers::pi / 2.0;
    return std::abs(shortest_rad(bearing_yaw, armor_yaw)) < kMaxVisibleYawError;
}

/**
 * @brief 判断装甲板测量是否可见（模板化接口）
 *
 * @tparam VecZ 测量向量类型
 * @param z 测量向量 [yaw, pitch, distance, armor_yaw]
 * @return true表示可见
 */
template <typename VecZ>
[[nodiscard]] inline bool armor_measurement_visible_from_origin(const VecZ& z) noexcept {
    return armor_face_visible_from_origin(z[ARMOR_YAW], z[ARMOR_YAW_ARMOR]);
}

// ============================================================================
// 装甲板位置计算
// ============================================================================

/**
 * @brief 计算4装甲板机器人的所有装甲板位姿
 *
 * 几何关系：
 * - 装甲板均匀分布在圆周上（间隔90°）
 * - 装甲板0和2使用相同半径（r0）和高度（z0）
 * - 装甲板1和3使用相同半径（r1）和高度（z1）
 * - 装甲板朝向角：armor_yaw = target_yaw + i * (2π/armors_num)
 *
 * 坐标计算：
 * - armor_x = xc - radius * cos(armor_yaw)
 * - armor_y = yc - radius * sin(armor_yaw)
 * - armor_z = z0 或 z1
 *
 * @param target_center 中心位置 [xc, yc, z0]（米）
 * @param target_yaw 装甲板0的yaw角度（弧度）
 * @param radius0 装甲板0和2的半径（米）
 * @param radius1 装甲板1和3的半径（米）
 * @param z0 装甲板0和2的Z坐标（米）
 * @param z1 装甲板1和3的Z坐标（米）
 * @param armors_num 装甲板数量（通常为4）
 * @return 装甲板位姿向量，每个元素为[x, y, z, yaw]
 *
 * @note 支持可变装甲板数量（3或4）
 */
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

        // 根据装甲板ID选择半径和高度
        if (armors_num == 4 && (i == 1 || i == 3)) {
            radius  = radius1;
            armor_z = z1;
        } else {
            radius  = radius0;
            armor_z = z0;
        }

        // 计算装甲板位置（中心减去半径分量）
        const double armor_x = target_center.x() - radius * std::cos(armor_yaw);
        const double armor_y = target_center.y() - radius * std::sin(armor_yaw);

        poses.emplace_back(armor_x, armor_y, armor_z, armor_yaw);
    }

    return poses;
}

/**
 * @brief 计算前哨站的装甲板位姿（3装甲板，固定半径）
 *
 * 前哨站特点：
 * - 固定半径（不估计）
 * - 三个装甲板高度独立
 * - 间隔120°均匀分布
 *
 * @param target_pos 中心位置 [xc, yc]（米）
 * @param yaw 装甲板0的yaw角度（弧度）
 * @param radius 固定半径（米）
 * @param z0 装甲板0的Z坐标（米）
 * @param z1 装甲板1的Z坐标（米）
 * @param z2 装甲板2的Z坐标（米）
 * @return 装甲板位姿向量
 */
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

/**
 * @brief 从位姿提取位置 [x, y, z, yaw] → [x, y, z]
 * @param pose 位姿向量 [x, y, z, yaw]
 * @return 位置向量 [x, y, z]
 */
[[nodiscard]] inline Eigen::Vector3d pose_to_position(const Eigen::Vector4d& pose) noexcept {
    return {pose.x(), pose.y(), pose.z()};
}

/**
 * @brief 批量提取位置
 * @param poses 位姿向量列表
 * @return 位置向量列表
 */
[[nodiscard]] inline std::vector<Eigen::Vector3d>
    poses_to_positions(const std::vector<Eigen::Vector4d>& poses) noexcept {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(poses.size());
    for (const auto& pose : poses) {
        positions.emplace_back(pose.x(), pose.y(), pose.z());
    }
    return positions;
}

/**
 * @brief 从状态向量计算装甲板位置
 *
 * 核心算法：
 * - 从状态向量提取半径（exp(log_r)保证半径为正）
 * - 根据装甲板ID选择半径和高度
 * - 使用中心位置和朝向计算装甲板位置
 *
 * @param x 状态向量（11维）
 * @param id 装甲板ID（0-3）
 * @param armors_num 装甲板数量
 * @return 装甲板位置 [x, y, z]（米）
 *
 * @note 状态向量使用对数半径参数化，需exp()转换为实际半径
 */
[[nodiscard]] inline Eigen::Vector3d
    state_to_armor_xyz(const double* x, int id, int armors_num) noexcept {
    const double angle_step = 2.0 * std::numbers::pi / static_cast<double>(armors_num);
    const double armor_yaw  = x[YAW] + static_cast<double>(id) * angle_step;

    double radius;
    double armor_z;

    // 根据装甲板ID选择参数
    if (armors_num == 4 && (id == 1 || id == 3)) {
        radius  = std::exp(x[LOG_R1]); // 对数半径转换为实际半径
        armor_z = x[Z0] + x[H];        // 装甲板1和3的高度
    } else {
        radius  = std::exp(x[LOG_R0]); // 装甲板0和2的半径
        armor_z = x[Z0];               // 装甲板0和2的高度
    }

    // 计算装甲板位置
    const double armor_x = x[XC] - radius * std::cos(armor_yaw);
    const double armor_y = x[YC] - radius * std::sin(armor_yaw);

    return {armor_x, armor_y, armor_z};
}

} // namespace fcs::L3
