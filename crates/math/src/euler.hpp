#pragma once
// 头文件保护宏，防止头文件重复包含导致重定义编译错误

// 李群SO3旋转矩阵封装头文件
#include "groups/SO3.hpp"

// Eigen 线性代数库基础矩阵、几何四元数、轴角头文件
#include <Eigen/Core>
#include <Eigen/Geometry>
// 标准元组，用于返回 r/p/y 三元组
#include <tuple>

namespace math_fuxk {

/// @brief ROS2 标准 Z-Y-X 欧拉角（RPY：roll-X 横滚、pitch-Y 俯仰、yaw-Z 偏航）
/// 旋转顺序：先绕X roll → 再绕Y pitch → 最后绕Z yaw，符合ROS2 TF2标准定义
template <typename Value>
struct Ros2EulerRot {
    // 单位：弧度 rad
    Value roll;  // 绕机体X轴 横滚角
    Value pitch; // 绕机体Y轴 俯仰角
    Value yaw;   // 绕机体Z轴 偏航角

    /**
     * @brief 构造函数，直接传入三个欧拉角弧度值
     * @param r roll 横滚
     * @param p pitch 俯仰
     * @param y yaw 偏航
     * noexcept 无抛出异常
     */
    constexpr Ros2EulerRot(Value r, Value p, Value y) noexcept
        : roll(r)
        , pitch(p)
        , yaw(y) {}

    /**
     * @brief 将RPY封装为std::tuple三元组返回 (roll, pitch, yaw)
     * @return std::tuple<Value, Value, Value> 有序三元组
     * constexpr 编译期可计算，无副作用
     * [[nodiscard]] 强制接收返回值，防止丢弃计算结果
     */
    [[nodiscard]] constexpr auto rpy() const noexcept { return std::tuple{roll, pitch, yaw}; }

    /**
     * @brief 将Z-Y-X欧拉角转换为 Eigen 单位四元数
     * 旋转顺序：Rz(yaw) * Ry(pitch) * Rx(roll) 右乘局部旋转，严格匹配ROS2 TF2
     * @return Eigen::Quaternion<Value> 单位四元数
     */
    [[nodiscard]] auto quat() const noexcept -> Eigen::Quaternion<Value> {
        return Eigen::Quaternion<Value>(
            // 第三步：绕Z轴偏航 yaw
            Eigen::AngleAxis<Value>(yaw, Eigen::Matrix<Value, 3, 1>::UnitZ())
            // 第二步：绕Y轴俯仰 pitch
            * Eigen::AngleAxis<Value>(pitch, Eigen::Matrix<Value, 3, 1>::UnitY())
            // 第一步：绕X轴横滚 roll
            * Eigen::AngleAxis<Value>(roll, Eigen::Matrix<Value, 3, 1>::UnitX()));
    }

    /**
     * @brief 欧拉角转换3阶旋转矩阵 SO(3) 正交矩阵
     * @return Eigen::Matrix3<Value> 3×3旋转矩阵
     */
    [[nodiscard]] auto matrix() const noexcept -> Eigen::Matrix3<Value> {
        // 先转四元数，再转为旋转矩阵
        return quat().toRotationMatrix();
    }

    /**
     * @brief 转换为自研李群SO3封装类型（用于李代数扰动、微分、优化）
     * @return group::SO3<Value> SO3旋转流形封装
     */
    [[nodiscard]] auto so3() const noexcept -> group::SO3<Value> {
        // 四元数转旋转矩阵，构造SO3实例
        return group::SO3<Value>(quat().toRotationMatrix());
    }
};

// ===== 类型别名，简化浮点/双精度使用 =====
// 单精度浮点 ROS2 RPY 欧拉角
using Ros2EulerRotf = Ros2EulerRot<float>;
// 双精度浮点 ROS2 RPY 欧拉角
using Ros2EulerRotd = Ros2EulerRot<double>;

// ===== 工厂构造函数，统一创建 Ros2EulerRot 对象 =====
/**
 * @brief 直接传入三个弧度欧拉角，创建RPY欧拉角结构体
 * @tparam T 数值类型 float/double
 * @param roll 横滚角 rad
 * @param pitch 俯仰角 rad
 * @param yaw 偏航角 rad
 * @return Ros2EulerRot<T> 欧拉角实例
 */
template <typename T>
[[nodiscard]] constexpr auto rpy(T roll, T pitch, T yaw) -> Ros2EulerRot<T> {
    return {roll, pitch, yaw};
}

/**
 * @brief 从3阶旋转矩阵反解 ROS2 Z-Y-X RPY欧拉角
 * 处理万向锁（pitch=±90°）奇异点分支
 * @tparam T 数值类型 float/double
 * @param m 输入3×3正交旋转矩阵
 * @return Ros2EulerRot<T> 分解后的RPY欧拉角
 */
template <typename T>
[[nodiscard]] constexpr auto rpy(const Eigen::Matrix3<T>& m) -> Ros2EulerRot<T> {
    // 计算 sin(pitch) = -R[2,0]，限制取值范围 [-1, 1] 防止数值溢出
    T sin_pitch = std::clamp(-m(2, 0), T(-1), T(1));

    // 非万向锁区域：|sin(pitch)| < 0.9999999 等价 pitch ≠ ±π/2
    if (std::abs(sin_pitch) < T(0.9999999)) {
        return {
            std::atan2(m(2, 1), m(2, 2)), // roll = atan2(R[2,1], R[2,2])
            std::asin(sin_pitch),         // pitch = arcsin(-R[2,0])
            std::atan2(m(1, 0), m(0, 0))  // yaw = atan2(R[1,0], R[0,0])
        };
    }
    // 万向锁奇异分支：pitch=±90°，roll自由度丢失置0，仅保留yaw
    return {
        T(0),                                   // roll 置0
        sin_pitch > 0 ? T(M_PI_2) : T(-M_PI_2), // pitch = ±π/2
        std::atan2(-m(0, 1), m(1, 1))           // 仅保留yaw分量
    };
}

/**
 * @brief 从Eigen四元数反解RPY欧拉角
 * @tparam T 数值类型
 * @param q 单位四元数
 * @return Ros2EulerRot<T> 欧拉角
 */
template <typename T>
[[nodiscard]] constexpr auto rpy(const Eigen::Quaternion<T>& q) -> Ros2EulerRot<T> {
    // 四元数转旋转矩阵，复用矩阵分解逻辑
    return rpy(q.toRotationMatrix());
}

/**
 * @brief 从自研SO3李群对象反解RPY欧拉角
 * @tparam T 数值类型
 * @param q SO3旋转封装
 * @return Ros2EulerRot<T> 欧拉角
 */
template <typename T>
[[nodiscard]] auto rpy(const group::SO3<T>& q) -> Ros2EulerRot<T> {
    // 提取SO3内部旋转矩阵，复用矩阵分解函数
    return rpy(q.R());
}

} // namespace math_fuxk