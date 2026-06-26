// 头文件保护，防止重复包含引发重定义编译错误
#pragma once

// 标准数学库：浮点数运算、三角函数
#include <cmath>
// C++20 标准数学常量：圆周率 π
#include <numbers>

/**
 * @namespace fcs::core::math
 * @brief 框架底层通用数学工具命名空间
 * 封装机器人火控高频使用的角度归一化、坐标转换工具函数，全部无异常、内联高性能，适配实时调度系统
 */
namespace fcs::core::math {

/**
 * @brief 将任意角度弧度值归一化到区间 [-π, π]
 * @param angle 输入弧度值（可超大正数/超大负数）
 * @return 归一化后弧度，范围 [-π, π]
 * @ [[nodiscard]] 强制接收返回值，禁止丢弃计算结果
 * @ noexcept 函数无任何抛出异常，嵌入式硬实时系统安全
 *
 * 应用场景：云台偏航角、目标相对偏航角角度差计算，避免 350° 与 -10° 判定为巨大差值
 */
[[nodiscard]] inline double normalize_angle(double angle) noexcept {
    // std::remainder(a, b)：求 a 相对于 b 的带符号余数，输出范围 (-b/2, b/2)
    // 2π 为一个完整圆周周期，初步归一化到 (-π, π)
    double r = std::remainder(angle, 2.0 * std::numbers::pi);

    // remainder 边界特殊情况：极少数场景会得到 r = -π，不在 [-π, π] 闭区间内，补偿 +2π 修正
    if (r <= -std::numbers::pi)
        r += 2.0 * std::numbers::pi;

    return r;
}

/**
 * @brief 笛卡尔三维坐标 XYZ 转换为 偏航Yaw / 俯仰Pitch / 直线距离Distance
 * @param xyz 相机/世界系三维笛卡尔坐标 Eigen::Vector3d
 * @return Eigen::Vector3d {yaw, pitch, distance}
 * @ noexcept 无异常，实时系统友好
 *
 * 坐标系约定（机器人视觉标准）：
 * X：镜头向前（光轴）
 * Y：相机右方
 * Z：相机下方
 * Yaw：绕Z轴水平旋转角，atan2(y, x)，范围 [-π, π]
 * Pitch：竖直俯仰角，向下为正，atan2(-z, 水平距离)
 * Distance：三维空间原点到目标直线欧氏距离
 */
[[nodiscard]] inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) noexcept {
    // 提取三维坐标分量
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();

    // 三维空间总距离：sqrt(x²+y²+z²)
    const double distance   = xyz.norm();
    // 水平偏航角：X-Y平面内，X轴到目标向量夹角，左右水平旋转
    const double yaw        = std::atan2(y, x);
    // 水平平面模长：sqrt(x² + y²)，使用std::hypot避免溢出、精度更高
    const double horizontal = std::hypot(x, y);
    // 俯仰角：竖直方向，负Z保证向下为正角度
    const double pitch      = std::atan2(-z, horizontal);

    // 返回三元组 {yaw, pitch, distance}
    return {yaw, pitch, distance};
}

} // namespace fcs::core::math