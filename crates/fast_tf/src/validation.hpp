// 头文件保护宏，防止头文件被多次重复包含
#pragma once

// 引入自定义矩阵/变换矩阵模板定义头文件，包含 TransformMatrix、Eigen 相关封装
#include "matrix.hpp"
// C++标准数学库，提供 std::isnan、std::isinf、三角函数、绝对值等数学函数
#include <cmath>
// C++20 概念约束，std::floating_point 用于限定浮点类型（float/double/long double）
#include <concepts>
// C++23 标准预期错误处理，std::expected<T, Err> 替代异常的返回式错误
#include <expected>
// fmt 格式化库，用于快速拼接日志、错误字符串
#include <fmt/format.h>
// C++标准数学常量，π 圆周率常量 std::numbers::pi_v<T>
#include <numbers>
// 标准字符串类型，存放错误信息、格式化后的变换文本
#include <string>

// 快速坐标变换库顶层命名空间，隔离所有tf相关代码
namespace fast_tf {

/**
 * @brief 校验坐标变换矩阵数值合法性，检测非法数值、非归一化四元数
 * @tparam T 浮点数值类型，受概念约束仅支持浮点 float/double
 * @tparam From 源坐标系标签类型（空标记类，仅用于模板区分坐标系）
 * @tparam To 目标坐标系标签类型
 * @param transform 源帧到目标帧的齐次变换矩阵对象
 * @return std::expected<void, std::string> 校验正常返回空成功；校验失败返回携带详细描述的错误字符串
 * noexcept 函数不会抛出C++异常
 * [[nodiscard]] 强制调用方接收返回值，禁止忽略校验结果
 */
template <std::floating_point T, typename From, typename To>
[[nodiscard]] std::expected<void, std::string>
    validate_transform(const TransformMatrix<T, From, To>& transform) noexcept;

/**
 * @brief 校验四元数是否单位归一化（刚体旋转要求模长≈1）
 * @tparam T 浮点类型
 * @param q Eigen四元数对象 w,x,y,z
 * @param tolerance 归一化允许误差容限，默认0.01，参考ROS tf2标准阈值
 * @return 归一化返回true，模长偏离1超过容限返回false
 */
template <std::floating_point T>
[[nodiscard]] bool is_quaternion_normalized(
    const Eigen::Quaternion<T>& q, T tolerance = static_cast<T>(0.01)) noexcept;

/**
 * @brief 标量浮点数值非法检测：判断是否存在NaN（无效值）/Inf（无穷大）
 * @tparam T 浮点类型
 * @param value 单个浮点数
 * @return 存在NaN/Inf返回true，正常数值返回false
 */
template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(T value) noexcept;

/**
 * @brief 三维向量非法数值检测，遍历3个分量判断是否含NaN/Inf
 * @tparam T 浮点类型
 * @param v Eigen三维列向量 [x,y,z]
 * @return 任意分量非法则返回true
 */
template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(const Eigen::Matrix<T, 3, 1>& v) noexcept;

/**
 * @brief 3×3旋转矩阵非法数值检测，遍历全部9个矩阵元素
 * @tparam T 浮点类型
 * @param m 3阶方阵（旋转矩阵）
 * @return 任意元素存在NaN/Inf返回true
 */
template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(const Eigen::Matrix<T, 3, 3>& m) noexcept;

/**
 * @brief 四元数四分量非法数值检测，遍历w/x/y/z
 * @tparam T 浮点类型
 * @param q Eigen四元数
 * @return 任意分量非法返回true
 */
template <std::floating_point T>
[[nodiscard]] bool has_nan_or_inf(const Eigen::Quaternion<T>& q) noexcept;

/**
 * @brief 将变换矩阵解析为可读字符串，输出欧拉角（角度制）+三轴平移，用于错误日志打印
 * @tparam T 浮点类型
 * @tparam From 源坐标系标签
 * @tparam To 目标坐标系标签
 * @param transform 待格式化的变换矩阵
 * @return 拼接好的人类可读字符串，如 roll=0.000°, pitch=... x=0.123m
 */
template <std::floating_point T, typename From, typename To>
[[nodiscard]] std::string
    format_transform_values(const TransformMatrix<T, From, To>& transform) noexcept;

// ===================== 模板函数实现部分 =====================

/**
 * @brief 单个浮点标量判NaN/Inf实现
 */
template <std::floating_point T>
bool has_nan_or_inf(T value) noexcept {
    // std::isnan：判断是否无效数字；std::isinf：判断正负无穷
    return std::isnan(value) || std::isinf(value);
}

/**
 * @brief 三维向量遍历检测非法值实现
 */
template <std::floating_point T>
bool has_nan_or_inf(const Eigen::Matrix<T, 3, 1>& v) noexcept {
    // 循环遍历向量x/y/z三个分量
    for (int i = 0; i < 3; ++i) {
        // 任意分量非法直接返回true，提前终止循环
        if (has_nan_or_inf(v(i))) {
            return true;
        }
    }
    // 全部分量数值正常
    return false;
}

/**
 * @brief 3×3矩阵遍历9个元素检测非法值实现
 */
template <std::floating_point T>
bool has_nan_or_inf(const Eigen::Matrix<T, 3, 3>& m) noexcept {
    // Eigen矩阵一维索引遍历全部9个元素
    for (int i = 0; i < 9; ++i) {
        if (has_nan_or_inf(m(i))) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 四元数四个分量检测非法值实现
 */
template <std::floating_point T>
bool has_nan_or_inf(const Eigen::Quaternion<T>& q) noexcept {
    // 分别检测w、x、y、z四个分量，任一非法返回true
    return has_nan_or_inf(q.x()) || has_nan_or_inf(q.y()) || has_nan_or_inf(q.z())
        || has_nan_or_inf(q.w());
}

/**
 * @brief 四元数归一化检测实现
 */
template <std::floating_point T>
bool is_quaternion_normalized(const Eigen::Quaternion<T>& q, T tolerance) noexcept {
    // 计算四元数模长 norm = sqrt(w²+x²+y²+z²)
    const T norm = q.norm();
    // 模长与1的差值绝对值小于容差，判定为归一化合法
    return std::abs(norm - static_cast<T>(1)) <= tolerance;
}

/**
 * @brief 变换矩阵格式化字符串实现
 */
template <std::floating_point T, typename From, typename To>
std::string format_transform_values(const TransformMatrix<T, From, To>& transform) noexcept {
    // 从变换矩阵提取欧拉角（roll/pitch/yaw，弧度）
    const auto euler = transform.euler_rot();
    // 从变换矩阵提取平移向量 x/y/z（米）
    const auto trans = transform.translation();

    // 弧度转角度换算系数 180/π
    constexpr T rad_to_deg = static_cast<T>(180) / std::numbers::pi_v<T>;

    // fmt 格式化拼接字符串，欧拉角转角度，保留3位小数；平移单位米保留3位小数
    return fmt::format(
        "roll={:.3f}\xC2\xB0, pitch={:.3f}\xC2\xB0, yaw={:.3f}\xC2\xB0, "
        "x={:.3f}m, y={:.3f}m, z={:.3f}m",
        euler.roll * rad_to_deg, euler.pitch * rad_to_deg, euler.yaw * rad_to_deg, trans.x(),
        trans.y(), trans.z());
}

/**
 * @brief 完整变换矩阵校验逻辑实现
 */
template <std::floating_point T, typename From, typename To>
std::expected<void, std::string>
    validate_transform(const TransformMatrix<T, From, To>& transform) noexcept {
    // 分离提取平移向量、旋转矩阵、旋转四元数
    const auto translation = transform.translation();
    const auto rotation    = transform.rotation();
    const auto quaternion  = transform.quaternion();

    // 分步校验：平移X分量非法
    if (has_nan_or_inf(translation.x())) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [translation.x is NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }
    // 平移Y分量非法
    if (has_nan_or_inf(translation.y())) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [translation.y is NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }
    // 平移Z分量非法
    if (has_nan_or_inf(translation.z())) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [translation.z is NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }

    // 旋转矩阵存在非法数值
    if (has_nan_or_inf(rotation)) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [rotation matrix contains NaN or Inf]. Full values: "
                "{}",
                format_transform_values(transform)));
    }

    // 四元数分量存在非法数值
    if (has_nan_or_inf(quaternion)) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [quaternion contains NaN or Inf]. Full values: {}",
                format_transform_values(transform)));
    }

    // 四元数未归一化（刚体旋转必须单位四元数）
    if (!is_quaternion_normalized(quaternion, static_cast<T>(0.01))) {
        return std::unexpected(
            fmt::format(
                "Transform validation failed: [quaternion not normalized, norm={:.6f}]. Full "
                "values: {}",
                quaternion.norm(), format_transform_values(transform)));
    }

    // 全部校验通过，返回成功空值
    return {};
}

} // namespace fast_tf_driver