/**
 * @file ldm_naive_systems.cpp
 * @brief LDM Naive跟踪系统注册实现
 *
 * ## 文件功能概述
 * 本文件实现了LDM（Landing Device Marker）朴素跟踪系统的调度器注册。
 * 将LdmTracker封装为固定频率系统（250Hz），从感知层读取位姿测量，
 * 运行Invariant EKF滤波，并向下游输出状态估计结果。
 *
 * ## 系统架构位置
 * - 层级：L3估计层（Estimation Layer）
 * - 输入：L2感知层的LdmMeasurement（位姿测量）
 * - 输出：LdmState（SE2(3)状态、预测位置、跟踪状态）
 * - 下游：L4规划层（Aimer、Gimbal Planner）
 *
 * ## 调度策略
 * - 执行策略：fixed_rate<250>（固定频率250Hz）
 * - 触发方式：独立线程定时触发，不依赖上游
 * - 频率选择：250Hz平衡了计算开销与跟踪精度
 *
 * ## 数据流
 * 1. 从通道读取LdmMeasurement（位姿测量）
 * 2. 提取位姿观测（R_world_body, p_world_body）
 * 3. 调用LdmTracker::update()执行EKF预测和更新
 * 4. 获取LdmState并通过通道发布
 *
 * ## 关键设计
 * - 延迟初始化：tracker在首次执行时构造（通过std::optional）
 * - 优雅降级：位姿缺失时允许纯预测步骤
 * - 精度标记：根据感知层accurate标志标记输出状态
 *
 * @author Talos Team
 * @date 2024
 */

#include "L3_estimation/ldm_naive/ldm_naive_systems.hpp"

#include "L2_perception/ldm/types.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "core/channel_topics.hpp"
#include "core/time.hpp"
#include "ldm_tracker.hpp"
#include "scheduler/scheduler.hpp"

#include <Eigen/Core>
#include <optional>
#include <spdlog/spdlog.h>

namespace fcs::L3::ldm {

/**
 * @brief 注册LDM朴素跟踪系统到调度器
 *
 * 创建固定频率系统（250Hz），封装LdmTracker：
 * - 输入通道：LdmMeasurementChannelTopic（L2感知层输出）
 * - 输出通道：LdmState（L4规划层输入）
 * - 执行策略：fixed_rate（独立线程，定时触发）
 *
 * ## 系统生命周期
 * - 初始化：首次执行时构造LdmTracker（延迟初始化）
 * - 运行：循环执行预测-更新步骤
 * - 清理：由调度器管理（RAII）
 *
 * ## 错误处理
 * - 位姿缺失：允许纯预测步骤（不更新）
 * - 状态不可用：记录TRACE日志，不发布输出
 *
 * @param scheduler 调度器引用
 * @param config 配置参数（模型参数、跟踪阈值等）
 *
 * @note 使用fixed_rate确保稳定的控制频率，不依赖上游触发
 * @warning 需要确保LdmMeasurementChannelTopic已正确注册
 */
void register_naive_ldm_systems(talos::Scheduler& scheduler, const NaiveLdmConfig& config) {
    // 插入配置资源（只读，所有系统共享）
    scheduler.world().insert_resource(config);

    // 注册固定频率系统（250Hz）
    scheduler.add_system<talos::fixed_rate<250>>(
        "l3_ldm_tracker",  // 系统名称（用于调试和可视化）
        [tracker = std::optional<LdmTracker>{}](  // 延迟初始化：首次执行时构造
            talos::res<NaiveLdmConfig> cfg,       // 配置资源（只读）
            talos::spmc<fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> meas_in,  // 输入通道
            talos::publish<LdmState> state_out) mutable {                                    // 输出通道
            // 延迟初始化：首次执行时构造tracker
            if (!tracker.has_value()) {
                tracker.emplace(*cfg);
            }

            // 读取感知层测量（非阻塞，可能为空）
            const auto meas       = meas_in.read();
            const uint64_t now_ns = fcs::clock::now_ns();  // 当前时间戳
            uint64_t update_ns    = now_ns;

            // 准备位姿测量（可能缺失）
            std::optional<LdmKinematic::PoseMeasurement> pose_measurement;

            if (meas.has_value()) {
                update_ns = meas->timestamp_ns;  // 使用测量时间戳
                if (meas->transform_odom.has_value()) {
                    // 提取位姿：旋转矩阵 + 位置向量
                    const auto& pose = *meas->transform_odom;
                    pose_measurement = LdmKinematic::PoseMeasurement{
                        .R_world_body = pose.rotation(),
                        .p_world_body = pose.translation(),
                    };
                } else {
                    // SPDLOG_ERROR(
                    //     "LdmMeasurement(t={}, frame={}, depth_quality={}) has no transform_odom:
                    //     " "PnP pose estimation failed or bearing-only mode", meas->timestamp_ns,
                    //     meas->frame_id, meas->depth_quality);
                    // Allow predict-only step when pose is unavailable.
                    // 位姿缺失：允许纯预测步骤（PnP失败或仅方位模式）
                }
                // 执行EKF预测和更新
                tracker->update(update_ns, pose_measurement);
            }

            // 获取输出状态
            const auto state = tracker->get_output();
            if (state.has_value()) {
                LdmState output = *state;
                // 继承感知层的精度标记
                if (meas.has_value() && meas->accurate) {
                    output.accurate = true;
                }
                state_out.write(std::move(output));
            } else {
                SPDLOG_TRACE("LdmState unavailable: {}", state.error());
            }
        });
}

} // namespace fcs::L3::ldm
