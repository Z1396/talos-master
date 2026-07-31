/**
 * @file manager.hpp
 * @brief 多目标跟踪管理器 - 统一管理所有(name, color)组合的tracker实例
 *
 * @details
 * 本文件实现了TrackerManager类，负责：
 * - 预分配并管理所有可能的(name, color)组合的tracker实例
 * - 批量处理测量数据，按目标身份路由到对应tracker
 * - 提供多种查询接口（active/all/status/name过滤）
 * - 实现C++20 ranges过滤器，支持链式调用
 *
 * 设计模式：
 * - Manager模式：集中管理多个tracker实例
 * - Flyweight模式：共享配置，避免重复存储
 * - 非拷贝非移动：持有unique_ptr，语义明确
 *
 * 核心算法：
 * - 数据关联：按(name, color)分组测量数据
 * - 时间同步：计算帧间时间间隔dt
 * - 状态机：统一处理Idle/Detecting/Tracking/TempLost状态转换
 *
 * @see TrackerNew
 * @see TargetKey
 * @see ArmorMeasurementBatch
 * @see TrackerOutputs
 */

#pragma once

#include "L3_estimation/tracker/new_tracker.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "core/target_key.hpp"
#include "core/types.hpp"
#include <functional>
#include <memory>
#include <ranges>
#include <unordered_map>
#include <utility>
#include <vector>

#include <magic_enum.hpp>

namespace fcs::L3 {

// ============================================================================
// TrackerManager - 多目标跟踪管理器
// ============================================================================

/**
 * @class TrackerManager
 * @brief 多目标跟踪管理器，管理所有(name, color)组合的tracker实例
 *
 * @details
 * 核心职责：
 * 1. 在构造时预分配所有可能的tracker实例（避免运行时动态创建）
 * 2. 批量处理测量数据，自动路由到对应tracker
 * 3. 维护每个tracker的时间戳，计算准确的dt
 * 4. 提供丰富的查询接口和C++20 ranges过滤器
 *
 * 内存管理：
 * - 使用unordered_map<TargetKey, unique_ptr<TrackerNew>>存储所有tracker
 * - 使用unordered_map<TargetKey, uint64_t>存储上次更新时间戳
 * - 非拷贝非移动，RAII管理生命周期
 *
 * 性能特性：
 * - 预分配避免运行时内存分配
 * - 批量处理减少通道访问次数
 * - 哈希查找O(1)复杂度
 *
 * @warning 所有tracker共享同一个配置对象，修改会影响所有实例
 */
class TrackerManager {
public:
    /**
     * @brief 构造函数 - 预分配所有(name, color)组合的tracker实例
     *
     * @details
     * 遍历所有有效的ArmorName和ArmorColor组合，为每个组合创建一个TrackerNew实例。
     * 这确保运行时无需动态创建tracker，避免内存分配延迟。
     *
     * 初始化过程：
     * 1. 使用magic_enum遍历所有枚举值（编译期确定）
     * 2. 跳过无效值：ArmorName::Invalid, ArmorColor::Neutral
     * 3. 创建TrackerNew实例，共享配置对象
     * 4. 初始化时间戳为0（表示未更新）
     *
     * @param config Tracker配置对象（const引用，内部会拷贝）
     *
     * @note 复杂度：O(N×M)，其中N为ArmorName数量，M为ArmorColor数量
     * @note 典型配置：8种ArmorName × 3种Color = 24个tracker实例
     *
     * @warning 构造函数必须noexcept，避免抛出异常
     */
    explicit TrackerManager(const TrackerConfig& config) {
        // 使用magic_enum编译期遍历所有枚举值（避免手写列表）
        const auto all_names  = magic_enum::enum_values<ArmorName>();
        const auto all_colors = magic_enum::enum_values<ArmorColor>();

        // 双重循环：为每个(name, color)组合创建tracker
        for (const auto name : all_names) {
            // 跳过无效目标名称
            if (name == ArmorName::Invalid) {
                continue;
            }

            for (const auto color : all_colors) {
                // 跳过中性颜色（不是有效敌方目标）
                if (color == ArmorColor::Neutral) {
                    continue;
                }

                // 构造目标身份key
                const core::TargetKey key{name, color};

                // 创建tracker实例，共享配置对象
                // 注意：所有tracker共享同一份config拷贝
                auto tracker            = std::make_unique<TrackerNew>(config);
                trackers_[key]          = std::move(tracker);
                last_timestamp_ns_[key] = 0; // 初始化时间戳为0（表示未更新）
            }
        }
    }

    // 禁用拷贝和移动语义（持有unique_ptr，语义明确）
    TrackerManager(const TrackerManager&)            = delete;
    TrackerManager& operator=(const TrackerManager&) = delete;
    TrackerManager(TrackerManager&&)                 = delete;
    TrackerManager& operator=(TrackerManager&&)      = delete;

    // ========================================================================
    // Update API - 批量更新与测量数据路由
    // ========================================================================

    /**
     * @brief 批量更新所有tracker - 核心接口，处理完整的预测-更新循环
     *
     * @details
     * 这是TrackerManager的主要入口，实现完整的跟踪流程：
     *
     * 算法流程：
     * 1. 数据分组：按(name, color)对测量数据进行分组
     * 2. 时间同步：计算每个tracker的dt（帧间时间间隔）
     * 3. 预测阶段：对所有非Idle tracker执行predict(dt)
     * 4. 更新阶段：根据是否有测量数据，调用first_meet/update
     * 5. 输出收集：返回所有非Idle tracker的输出
     *
     * 状态机处理：
     * - Idle + 有测量 -> first_meet() -> Detecting
     * - Idle + 无测量 -> 保持Idle
     * - 非Idle + 有测量 -> update() -> 保持/升级状态
     * - 非Idle + 无测量 -> update(empty) -> 可能进入TempLost
     *
     * 数据关联策略：
     * - 简单哈希路由：按目标身份(name, color)分组
     * - 不处理跨目标关联（如ID切换）
     * - 每个tracker独立处理自己的测量数据
     *
     * @param measurements 装甲板测量数据批量（可能包含多个目标的测量）
     * @return TrackerOutputs 所有非Idle状态tracker的输出列表
     *
     * @note 复杂度：O(N)，其中N为tracker总数（典型值24）
     * @note 线程安全：无共享状态，可并发读取
     *
     * @warning 必须每帧调用，即使没有测量数据（用于状态机推进）
     */
    [[nodiscard]] TrackerOutputs update_all(const ArmorMeasurementBatch& measurements) {
        TrackerOutputs updated_outputs;

        // ----------------------------------------------------------------------
        // 步骤1：按(name, color)对测量数据进行分组
        // ----------------------------------------------------------------------
        // 使用哈希表分组，避免多次遍历
        std::unordered_map<core::TargetKey, ArmorMeasurementBatch, core::TargetKeyHash> grouped;
        for (const auto& meas : measurements.measurements) {
            const core::TargetKey key{meas.name, meas.color};
            grouped[key].measurements.push_back(meas);
        }

        // 为每个分组设置时间戳和帧ID（从原始batch继承）
        for (auto& [key, batch] : grouped) {
            batch.timestamp_ns = measurements.timestamp_ns;
            batch.frame_id     = measurements.frame_id;
        }

        // ----------------------------------------------------------------------
        // 步骤2-4：遍历所有tracker，执行预测-更新循环
        // ----------------------------------------------------------------------
        for (auto& [key, tracker] : trackers_) {
            uint64_t& last_ts = last_timestamp_ns_[key];

            // 计算时间间隔dt（纳秒转秒）
            double dt = 0.0;
            if (measurements.timestamp_ns > last_ts && last_ts != 0) {
                dt = static_cast<double>(measurements.timestamp_ns - last_ts) / 1e9;
            }
            last_ts = measurements.timestamp_ns;

            // 预测阶段（时间更新）：仅对非Idle tracker执行
            // 注意：TempLost状态也需要预测，用于超时检测
            if (tracker->status() != TrackerStatus::Idle) {
                tracker->predict(dt);
            }

            // 更新阶段（测量更新或丢失处理）
            auto it = grouped.find(key);
            if (it != grouped.end()) {
                // 情况A：有测量数据
                if (tracker->status() == TrackerStatus::Idle) {
                    // Idle状态首次观测到目标
                    (void)tracker->first_meet(it->second);
                } else {
                    // 已有跟踪目标，执行测量更新
                    (void)tracker->update(it->second);
                }
            } else {
                // 情况B：无测量数据（丢失或未观测到）
                // 构造空batch，让state_machine处理"未找到"逻辑
                ArmorMeasurementBatch empty_batch;
                empty_batch.timestamp_ns = measurements.timestamp_ns;
                empty_batch.frame_id     = measurements.frame_id;
                (void)tracker->update(empty_batch);
            }

            // 收集输出：仅返回非Idle状态的tracker（语义：有效跟踪目标）
            if (tracker->status() != TrackerStatus::Idle) {
                auto output         = tracker->get_output();
                output.timestamp_ns = measurements.timestamp_ns;
                updated_outputs.push_back(std::move(output));
            }
        }

        return updated_outputs;
    }

    // ========================================================================
    // Query APIs - 查询接口，返回物化向量用于调度器集成
    // ========================================================================

    /**
     * @brief 查询活跃tracker输出（Tracking或TempLost状态）
     *
     * @details
     * 返回所有正在跟踪或暂时丢失的tracker输出。
     * 这通常用于L4规划层的目标选择。
     *
     * @return TrackerOutputs 活跃tracker的输出列表
     *
     * @note 复杂度：O(N)，需要遍历所有tracker
     */
    [[nodiscard]] TrackerOutputs active_outputs() const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            const auto output = tracker->get_output();
            if (output.is_tracking()) {
                outputs.push_back(output);
            }
        }

        return outputs;
    }

    /**
     * @brief 查询所有tracker输出（包括Idle状态）
     *
     * @details
     * 返回所有tracker的输出，用于调试或可视化。
     * 注意：包含大量Idle状态的tracker，实际使用价值有限。
     *
     * @return TrackerOutputs 所有tracker的输出列表
     *
     * @note 主要用于调试和Foxglove可视化
     */
    [[nodiscard]] TrackerOutputs all_outputs() const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            outputs.push_back(tracker->get_output());
        }

        return outputs;
    }

    /**
     * @brief 按状态过滤tracker输出
     *
     * @param status 目标状态（Idle/Detecting/Tracking/TempLost）
     * @return TrackerOutputs 匹配状态的tracker输出列表
     *
     * @note 常用于查找所有TempLost状态的目标（用于超时检测）
     */
    [[nodiscard]] TrackerOutputs outputs_with_status(TrackerStatus status) const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            const auto output = tracker->get_output();
            if (output.status == status) {
                outputs.push_back(output);
            }
        }

        return outputs;
    }

    /**
     * @brief 按名称过滤tracker输出（忽略颜色）
     *
     * @details
     * 返回所有指定名称的tracker输出，不论颜色。
     * 例如：查找所有"英雄"机器人（不论红蓝方）。
     *
     * @param name 目标名称（如Hero, Infantry等）
     * @return TrackerOutputs 匹配名称的所有tracker输出
     *
     * @note 用于跨颜色目标选择（如优先攻击英雄）
     */
    [[nodiscard]] TrackerOutputs outputs_with_name(ArmorName name) const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            if (key.name == name) {
                outputs.push_back(tracker->get_output());
            }
        }

        return outputs;
    }

    // ========================================================================
    // Direct Access API - 直接访问接口
    // ========================================================================

    /**
     * @brief 直接访问单个tracker实例（用于调试或可视化）
     *
     * @details
     * 返回指定(name, color)的tracker引用。
     * 典型用途：
     * - Foxglove可视化：读取tracker内部状态
     * - 单元测试：验证特定tracker的状态
     * - 性能分析：监控单个tracker的收敛情况
     *
     * @param name 目标名称
     * @param color 目标颜色
     * @return std::optional<std::reference_wrapper<TrackerNew>> tracker引用，失败返回nullopt
     *
     * @note 返回reference_wrapper避免拷贝unique_ptr
     * @warning 不要长期持有引用，tracker可能被重置
     */
    [[nodiscard]] std::optional<std::reference_wrapper<TrackerNew>>
        get_tracker(ArmorName name, ArmorColor color) noexcept {
        const core::TargetKey key{name, color};
        auto it = trackers_.find(key);
        if (it == trackers_.end()) {
            return std::nullopt;
        }
        return std::ref(*it->second);
    }

    /// @brief const重载版本
    [[nodiscard]] std::optional<std::reference_wrapper<const TrackerNew>>
        get_tracker(ArmorName name, ArmorColor color) const noexcept {
        const core::TargetKey key{name, color};
        auto it = trackers_.find(key);
        if (it == trackers_.end()) {
            return std::nullopt;
        }
        return std::ref(*it->second);
    }

private:
    std::unordered_map<core::TargetKey, std::unique_ptr<TrackerNew>, core::TargetKeyHash>
        trackers_;          ///< tracker实例映射
    std::unordered_map<core::TargetKey, uint64_t, core::TargetKeyHash>
        last_timestamp_ns_; ///< 上次更新时间戳（纳秒）
};

// ============================================================================
// Pipeable Filter Utilities for C++20 Ranges - C++20 ranges可管道过滤器
// ============================================================================

/**
 * @namespace filters
 * @brief C++20 ranges过滤器工具集，支持链式调用
 *
 * @details
 * 使用示例：
 * @code
 * auto outputs = manager->all_outputs();
 * auto active_hero = outputs | filters::is_tracking | filters::with_name(ArmorName::Hero);
 * @endcode
 *
 * 优势：
 * - 编译期类型安全
 * - 惰性求值，避免中间拷贝
 * - 可组合，支持链式调用
 */
namespace filters {

/**
 * @brief 过滤活跃tracker（Tracking或TempLost状态）
 *
 * @details
 * 典型用法：筛选所有正在跟踪的目标
 * @code
 * auto active = outputs | filters::is_tracking;
 * @endcode
 */
inline constexpr auto is_tracking =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.is_tracking(); });

/**
 * @brief 按状态过滤tracker
 *
 * @param status 目标状态
 * @return 过滤器视图
 *
 * @example
 * @code
 * auto temp_lost = outputs | filters::with_status(TrackerStatus::TempLost);
 * @endcode
 */
[[nodiscard]] inline auto with_status(TrackerStatus status) {
    return std::views::filter(
        [status](const TrackerOutput& output) noexcept { return output.status == status; });
}

/**
 * @brief 按名称过滤tracker
 *
 * @param name 目标名称
 * @return 过滤器视图
 */
[[nodiscard]] inline auto with_name(ArmorName name) {
    return std::views::filter(
        [name](const TrackerOutput& output) noexcept { return output.target_name == name; });
}

/**
 * @brief 按颜色过滤tracker
 *
 * @param color 目标颜色
 * @return 过滤器视图
 */
[[nodiscard]] inline auto with_color(ArmorColor color) {
    return std::views::filter(
        [color](const TrackerOutput& output) noexcept { return output.target_color == color; });
}

/**
 * @brief 按(name, color)组合过滤tracker
 *
 * @param name 目标名称
 * @param color 目标颜色
 * @return 过滤器视图
 *
 * @example
 * @code
 * auto red_hero = outputs | filters::with_target(ArmorName::Hero, ArmorColor::Red);
 * @endcode
 */
[[nodiscard]] inline auto with_target(ArmorName name, ArmorColor color) {
    return std::views::filter([name, color](const TrackerOutput& output) noexcept {
        return output.target_name == name && output.target_color == color;
    });
}

/**
 * @brief 过滤机器人目标（排除前哨站）
 *
 * @details
 * 前哨站（Outpost）具有特殊的运动模型，需单独处理。
 */
inline constexpr auto is_robot =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.is_robot(); });

/**
 * @brief 过滤前哨站目标
 */
inline constexpr auto is_outpost =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.is_outpost(); });

/**
 * @brief 过滤发生跳变的目标（用于调试）
 *
 * @details
 * "跳变"指tracker观测到非零装甲板ID，表示目标旋转超过预期。
 */
inline constexpr auto has_jumped =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.target_jumped; });

} // namespace filters

} // namespace fcs::L3
