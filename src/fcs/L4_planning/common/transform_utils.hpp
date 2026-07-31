/**
 * @file transform_utils.hpp
 * @brief L4规划层坐标变换查找工具
 *
 * 本文件提供坐标变换查找的工具函数，用于：
 * - 从坐标缓冲区查找云台和枪口坐标系变换
 * - 提供类型安全的错误处理
 * - 简化客户端代码
 *
 * 设计理念：
 * - 使用expected<T, string>表达可能失败的操作
 * - 使用fast_tf的强类型坐标系，防止误用
 * - 提供详细的错误诊断信息
 *
 * 使用场景：
 * - L4系统的run()函数开始时查找变换矩阵
 * - 为Aimer::aim()提供坐标系参数
 *
 * 性能考虑：
 * - 查找操作是高频操作（250Hz），需要快速
 * - 使用inline函数避免函数调用开销
 * - 变换矩阵是预计算的，查找只是索引操作
 */

#pragma once

#include "frame.hpp"

#include <cstdint>
#include <expected>
#include <fmt/format.h>

namespace fcs::L4 {

// ============================================================================
// 坐标变换查找结果
// ============================================================================

/**
 * @brief 云台和枪口变换矩阵查找结果
 *
 * 包含查询时刻的完整坐标系变换信息。
 * 这些变换矩阵用于将目标位置从世界坐标系转换到云台/枪口坐标系。
 *
 * 坐标系说明：
 * - odom：里程计坐标系（世界坐标系）
 * - gimbal_pitch：云台pitch坐标系（绕pitch轴旋转后的坐标系）
 * - muzzle：枪口坐标系（考虑云台和枪管安装误差）
 */
struct TransformLookupResult {
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch> gimbal;  ///< odom→gimbal_pitch变换矩阵
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::muzzle> muzzle;        ///< odom→muzzle变换矩阵
    uint64_t timestamp_ns;                                                    ///< 查询时间戳（纳秒）
};

// ============================================================================
// 坐标变换查找工具
// ============================================================================

/**
 * @brief 同时查找云台和枪口变换矩阵
 *
 * 从坐标缓冲区查找指定时刻的云台和枪口坐标系变换。
 * 使用lookup_clamped，如果查询时间超出缓冲区范围，会自动钳制到最近的有效时间。
 *
 * @param tf_buffer 坐标系缓冲区
 * @param current_ns 查询时间戳（纳秒）
 * @return 成功时返回TransformLookupResult，失败时返回错误信息
 *
 * @note 使用lookup_clamped避免时间戳越界错误
 * @note 错误信息包含详细的诊断信息（如时间范围、请求时间等）
 *
 * 使用示例：
 * @code
 * auto xf = lookup_gimbal_muzzle_transforms(*tf_buffer, current_ns);
 * if (!xf) {
 *     SPDLOG_WARN("变换查找失败: {}", xf.error());
 *     return;
 * }
 * const auto& [gimbal, muzzle, ts] = *xf;
 * // 使用gimbal和muzzle进行后续计算
 * @endcode
 */
[[nodiscard]] inline std::expected<TransformLookupResult, std::string>
    lookup_gimbal_muzzle_transforms(
        const fast_tf::CoordinateSystem& tf_buffer, uint64_t current_ns) noexcept {
    using namespace fast_tf;

    // 查找枪口变换（先查找，因为枪口变换依赖云台变换）
    auto muzzle_tf = lookup_clamped<odom, muzzle>(tf_buffer, current_ns);
    if (!muzzle_tf) {
        return std::unexpected(muzzle_tf.error());
    }

    // 查找云台变换
    auto gimbal_tf = lookup_clamped<odom, gimbal_pitch>(tf_buffer, current_ns);
    if (!gimbal_tf) {
        return std::unexpected(gimbal_tf.error());
    }

    return TransformLookupResult{
        .gimbal       = *gimbal_tf,
        .muzzle       = *muzzle_tf,
        .timestamp_ns = current_ns,
    };
}

/**
 * @brief 仅查找云台变换矩阵
 *
 * 当只需要云台变换而不需要枪口变换时使用，减少不必要的计算。
 *
 * @param tf_buffer 坐标系缓冲区
 * @param current_ns 查询时间戳（纳秒）
 * @return 成功时返回变换矩阵，失败时返回错误信息
 *
 * @note 使用场景：某些可视化或诊断功能只需要云台位置
 */
[[nodiscard]] inline std::expected<
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>, std::string>
    lookup_gimbal_transform(
        const fast_tf::CoordinateSystem& tf_buffer, uint64_t current_ns) noexcept {
    using namespace fast_tf;

    auto gimbal_tf = lookup_clamped<odom, gimbal_pitch>(tf_buffer, current_ns);
    if (!gimbal_tf) {
        return std::unexpected(gimbal_tf.error());
    }

    return *gimbal_tf;
}

} // namespace fcs::L4
