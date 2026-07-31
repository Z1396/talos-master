/**
 * @file tracker_systems.cpp
 * @brief Tracker系统注册模块 - 将多目标跟踪系统集成到Talos调度器
 *
 * @details
 * 本文件负责将TrackerManager注册到调度器中，作为L3估计层的核心系统。
 * 主要功能：
 * - 以固定频率(250Hz)运行tracker系统
 * - 从L2感知层接收装甲板测量数据批量
 * - 通过TrackerManager管理所有(name, color)组合的tracker实例
 * - 输出所有tracker的状态估计结果给L4规划层
 *
 * 数据流向：
 * L2_perception -> ArmorMeasurementBatch -> TrackerManager -> TrackerOutputs -> L4_planning
 *
 * @see TrackerManager
 * @see TrackerConfig
 * @see ArmorMeasurementBatch
 * @see TrackerOutputs
 */

#include "L3_estimation/tracker_systems.hpp"

#include "L3_estimation/manager.hpp"
#include "L3_estimation/tracker/config.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include "scheduler/scheduler.hpp"

#include <memory>
#include <vector>

namespace fcs::L3 {

/**
 * @brief 注册Tracker系统到调度器
 *
 * @details
 * 将TrackerManager封装为调度器系统，配置执行策略：
 * - 执行策略：fixed_rate<250>，固定频率250Hz
 * - 线程模型：独占线程，定频触发，通知下游
 * - 初始化：延迟构造，首次运行时创建TrackerManager
 *
 * 系统生命周期：
 * 1. 首次运行时创建TrackerManager实例（延迟初始化）
 * 2. 每帧从输入通道读取ArmorMeasurementBatch
 * 3. 调用manager->update_all()更新所有tracker
 * 4. 将结果写入输出通道供L4规划层使用
 *
 * @param scheduler Talos调度器引用
 * @param config Tracker配置对象（移动语义，配置后不可修改）
 *
 * @note 性能优化点：
 * - 延迟初始化避免启动时阻塞
 * - 使用std::move转移config所有权
 * - 批量处理减少通道访问次数
 *
 * @warning 配置传入后不可修改，确保参数在调用前验证完毕
 */
void register_tracker_systems(talos::Scheduler& scheduler, TrackerConfig&& config) {
    // 将配置对象作为资源插入World，供所有系统只读访问
    scheduler.world().insert_resource(std::move(config));

    // 注册armor_tracker系统，固定频率250Hz
    scheduler.add_system<talos::fixed_rate<250>>(
        "armor_tracker",
        // Lambda捕获：延迟构造的TrackerManager unique_ptr
        [manager = std::unique_ptr<TrackerManager>{}](
            talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,  ///< 输入：装甲板测量批量（SPMC只读）
            talos::spmc_mut<TrackerOutputs, TrackerOutputChannelTopic> tracker_out, ///< 输出：tracker结果批量（SPMC只写）
            talos::res<TrackerConfig> config  ///< 配置资源（只读，版本追踪）
        ) mutable {
            // 延迟初始化：首次运行时创建TrackerManager
            // 好处：避免构造函数阻塞调度器启动
            if (!manager) {
                manager = std::make_unique<TrackerManager>(*config);
            }

            // 从输入通道读取测量数据批量
            // 如果无数据（通道为空或上游未生产），直接返回
            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            // 核心逻辑：更新所有tracker实例
            // - 测量数据按(name, color)路由到对应tracker
            // - 执行预测-更新循环（predict-update）
            // - 收集所有非Idle状态tracker的输出
            // 注意：输出包含所有非Idle tracker，目标选择由L4层完成
            auto outputs = manager->update_all(*batch);

            // 将结果写入输出通道，供L4规划层消费
            tracker_out.write(std::move(outputs));
        });
}

} // namespace fcs::L3
