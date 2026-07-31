/**
 * @file types.hpp
 * @brief 能量机关角度处理工具函数
 *
 * 本文件提供了能量机关跟踪所需的角度归一化和解包裹工具函数。
 * 这些函数用于处理能量机关叶片的角度测量值，确保角度值在合理的范围内，
 * 并正确处理角度跳变（如从π到-π的跳变）。
 *
 * 主要功能：
 * - 角度归一化：将任意角度映射到[-π, π)区间
 * - 最短角度差计算：计算两个角度之间的最短差值
 * - 角度解包裹：连续跟踪角度变化，避免跳变
 */

#pragma once

#include <cmath>

namespace energy_meter {

/**
 * @brief 将角度归一化到[-π, π)区间
 *
 * 该函数将任意角度值归一化到标准区间[-π, π)，用于处理角度测量的周期性。
 * 例如：normalize_rad(3π) = π, normalize_rad(-3π/2) = π/2
 *
 * @param a 待归一化的角度值（单位：弧度）
 * @return 归一化后的角度值，范围在[-π, π)
 *
 * @note 该函数使用标准数学库的fmod函数，保证数值稳定性
 *
 * @warning 对于非常大的角度值（如>1e6），可能存在精度损失
 */
inline double normalize_rad(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a <= 0.0) {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

/**
 * @brief 计算从from到to的最短角度差
 *
 * 该函数计算两个角度之间的最短角度差，结果在[-π, π]区间内。
 * 这对于判断能量机关叶片的旋转方向和角度变化量至关重要。
 *
 * @param from 起始角度（单位：弧度）
 * @param to 目标角度（单位：弧度）
 * @return 从from到to的最短角度差（单位：弧度），范围在[-π, π]
 *
 * @note 正值表示逆时针旋转，负值表示顺时针旋转
 */
inline double shortest_rad(double from, double to) { return normalize_rad(to - from); }

/**
 * @brief 角度解包裹，跟踪连续角度变化
 *
 * 该函数用于连续跟踪能量机关叶片的角度变化，避免角度在边界处跳变。
 * 例如，当角度从π/2连续变化到3π/2时，若不解包裹会变为π/2→-π/2，
 * 解包裹后保持连续性。
 *
 * @param prev 上一时刻的角度值（已解包裹）
 * @param raw 当前时刻的原始角度测量值（可能在[-π, π]区间）
 * @return 解包裹后的当前角度值，保持与prev的连续性
 *
 * @note 该函数假设角度变化率不超过π/frame，否则可能解包裹失败
 *
 * @warning 若角度变化过快（超过π/帧），可能导致错误的解包裹结果
 */
inline double unwrap_rad(double prev, double raw) {
    const double d = shortest_rad(prev, raw);
    return prev + d;
}

} // namespace energy_meter
