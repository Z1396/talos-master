#pragma once
// 头文件保护，防止多重包含引发重复定义

#include <cstddef>

namespace fcs::L3::ldm {

/**
 * @brief LDM（大能量机关）运动学模型噪声参数
 *
 * 单独拆分出参数结构体的设计目的：
 * 将纯参数与滤波器实现解耦；
 * NaiveLdmConfig 只需要引入本头文件，不会间接依赖沉重的 <groups/SEn3.hpp> 李群几何库；
 * 实现头文件依赖轻量化，减少编译依赖链、加快编译速度。
 */
struct LdmKinematicParams {
    using Scalar = double;  // 统一浮点精度类型，方便后续切换float/double

    /// 机体惯性角速度过程噪声标准差 (rad/s)
    /// 描述模型角速度随机扰动，表征大符转速变化的不确定性
    Scalar sigma_inert_omega = Scalar(50.0);
    /// 机体惯性加速度过程噪声标准差 (m/s²)
    Scalar sigma_inert_accel = Scalar(50.0);

    /// SO(3)旋转残差噪声（单位：弧度）
    /// 李群旋转观测残差噪声，对应姿态旋转分量不确定度
    Scalar sigma_rot_x = Scalar(0.5);
    Scalar sigma_rot_y = Scalar(0.5);
    Scalar sigma_rot_z = Scalar(1.0);

    // ========== 相机角度观测噪声（方位角观测） ==========
    /// 水平方位角(yaw)观测噪声标准差 rad
    Scalar sigma_r_bearing_yaw   = Scalar(5e-1);
    /// 俯仰角(pitch)观测噪声标准差 rad
    Scalar sigma_r_bearing_pitch = Scalar(2e-1);

    // ========== 距离观测噪声模型（距离噪声不是固定常数，和距离相关） ==========
    // 公式：
    // sigma_d = sigma_distance_min
    //         + k_distance_depth * abs(depth)
    //         + k_distance_planar * planar_offset
    // 距离观测标准差由三部分组成：固定基底噪声 + 深度相关项 + 平面偏移相关项

    /// 距离噪声最小基底(m)，近距离时噪声下限
    Scalar sigma_distance_min = Scalar(0.01);
    /// 距离噪声随深度增长系数
    Scalar k_distance_depth   = Scalar(0.5);
    /// 平面偏移相关噪声系数（装甲/叶片偏离光轴带来的距离误差）
    Scalar k_distance_planar  = Scalar(0.5);
};

} // namespace fcs::L3::ldm