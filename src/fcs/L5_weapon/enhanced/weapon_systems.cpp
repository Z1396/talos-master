/**
 * @file weapon_systems.cpp
 * @brief L5武器层增强版系统注册实现
 *
 * 本文件实现了增强版武器系统的核心逻辑,包括:
 * - 参考轨迹采样
 * - 开火门判断
 * - 控制意图分发处理
 * - 系统注册到调度器
 *
 * 核心思想:
 * 将复杂的武器控制逻辑分解为多个小函数,每个函数职责单一,
 * 通过std::visit实现意图类型的穷尽处理。
 */

#include "L5_weapon/enhanced/weapon_systems.hpp"

#include "L4_planning/common/transform_utils.hpp"
#include "L4_planning/control_intent.hpp"
#include "L5_weapon/config.hpp"
#include "L5_weapon/enhanced/trajectory_optimizer.hpp"
#include "L5_weapon/fire_control.hpp"
#include "L5_weapon/fire_decision.hpp"
#include "core/channel_topics.hpp"
#include "core/math/normalize.hpp"
#include "core/time.hpp"
#include "scheduler/scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace fcs::L5 {

namespace {
/// std::visit辅助类,用于穷尽处理variant的所有分支
template <typename... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

using ReferenceTrajectory = core::trajectory::ReferenceTrajectory;
using fcs::core::math::normalize_angle;

/**
 * @brief 将时间延迟量化为离散步数
 *
 * 与trajectory_optimizer.cpp中的函数相同,用于轨迹索引对齐
 */
[[nodiscard]] int quantize_reference_age_steps(double reference_age_s, double dt_s) noexcept {
    if (!(reference_age_s > 0.0) || !(dt_s > 0.0)) {
        return 0;
    }
    return std::max(0, static_cast<int>(std::lround(reference_age_s / dt_s)));
}

/**
 * @brief 从参考轨迹中采样瞄准点(通用实现)
 *
 * 核心算法:
 * 1. 计算时间延迟对应的轨迹索引偏移
 * 2. 提取中心索引的状态(位置+速度)
 * 3. 组装为AimPoint结构
 *
 * @param trajectory 参考轨迹
 * @param trajectory_cfg 轨迹配置
 * @param trajectory_timestamp_ns 轨迹生成时间
 * @param command_timestamp_ns 当前时间
 *
 * @return 成功返回瞄准点,失败返回nullopt
 */
[[nodiscard]] std::optional<ReferenceTrajectory::AimPoint> sample_reference_trajectory(
    const ReferenceTrajectory& trajectory, const L4::ReferenceTrajectoryConfig& trajectory_cfg,
    uint64_t trajectory_timestamp_ns, uint64_t command_timestamp_ns) noexcept {
    // 检查轨迹数据有效性
    const int horizon = trajectory.horizon();
    if (horizon <= 0 || trajectory.distances.size() < static_cast<size_t>(horizon)
        || trajectory.time_of_flights.size() < static_cast<size_t>(horizon)) {
        return std::nullopt;
    }

    // 计算时间延迟
    const double reference_age_s = static_cast<double>(
                                       static_cast<int64_t>(command_timestamp_ns)
                                       - static_cast<int64_t>(trajectory_timestamp_ns))
                                 * 1e-9;

    // 将时间延迟转换为索引偏移
    const int shift_steps =
        quantize_reference_age_steps(std::max(reference_age_s, 0.0), trajectory_cfg.dt);

    // 计算采样索引
    const int start_index       = std::clamp(shift_steps, 0, horizon - 1);
    const int available_horizon = horizon - start_index;
    const int center_index =
        std::clamp(trajectory_cfg.horizon_back, 0, std::max(available_horizon - 1, 0));
    const int sample_index = start_index + center_index;

    // 提取瞄准点(应用yaw原点偏移)
    return ReferenceTrajectory::AimPoint{
        .yaw      = normalize_angle(trajectory.state(0, sample_index) + trajectory.yaw_origin),
        .pitch    = trajectory.state(2, sample_index),
        .distance = trajectory.distances[static_cast<size_t>(sample_index)],
    };
}

/**
 * @brief 应用开火门判断(核心逻辑)
 *
 * 计算当前云台姿态与目标点的误差,判断是否在射击窗口内。
 *
 * @param cmd 待处理的武器指令
 * @param fire_cfg 开火决策配置
 * @param cur_yaw 当前云台yaw角度
 * @param cur_pitch 当前云台pitch角度
 * @param target 目标瞄准点
 *
 * @return 应用开火门后的武器指令(包含fire标志和误差数据)
 *
 * @note pitch符号约定:cur_pitch取负值是因为坐标系定义不同
 */
[[nodiscard]] WeaponCommand fire_gate(
    WeaponCommand cmd, const FireDecisionConfig& fire_cfg, double cur_yaw, double cur_pitch,
    const ReferenceTrajectory::AimPoint& target) noexcept {
    // 调用开火决策函数(注意pitch符号约定)
    auto result =
        is_on_target(fire_cfg, cur_yaw, -cur_pitch, target.yaw, target.pitch, target.distance);

    // 填充开火判断结果
    cmd.fire                 = result.fire;
    cmd.yaw_error            = result.yaw_error;
    cmd.pitch_error          = result.pitch_error;
    cmd.shooting_range_yaw   = result.shooting_range_yaw;
    cmd.shooting_range_pitch = result.shooting_range_pitch;
    cmd.ref_yaw              = target.yaw;
    cmd.ref_pitch            = target.pitch;
    return cmd;
}

/**
 * @brief 构建Track指令的降级透传指令
 *
 * 当MPC优化失败时,直接使用参考轨迹的中心点作为目标。
 * 这是一个安全的降级策略,保证系统总能输出控制指令。
 *
 * @param track L4的Track指令
 * @param command_timestamp_ns 当前时间
 *
 * @return 降级后的武器指令(速度和加速度为0)
 */
[[nodiscard]] WeaponCommand
    track_fallback(const L4::TrackCommand& track, uint64_t command_timestamp_ns) noexcept {
    // 提取参考轨迹的中心瞄准点
    const auto aim = track.control_trajectory.center_aim_point();

    // 构建透传指令
    WeaponCommand cmd;
    cmd.timestamp_ns      = command_timestamp_ns;
    cmd.plan_timestamp_ns = track.timestamp_ns;
    cmd.plan_yaw          = aim.yaw;
    cmd.plan_pitch        = aim.pitch;
    cmd.plan_distance     = aim.distance;
    cmd.yaw               = aim.yaw;
    cmd.pitch             = aim.pitch;
    cmd.distance          = aim.distance;
    return cmd;
}
} // namespace

/**
 * @brief 从Track指令的fire_trajectory采样开火瞄准点
 *
 * 这是公开接口,调用内部的sample_reference_trajectory实现。
 */
std::optional<ReferenceTrajectory::AimPoint> sample_fire_trajectory(
    const L4::TrackCommand& track, const L4::ReferenceTrajectoryConfig& trajectory_cfg,
    uint64_t command_timestamp_ns) noexcept {
    return sample_reference_trajectory(
        track.fire_trajectory, trajectory_cfg, track.timestamp_ns, command_timestamp_ns);
}

/**
 * @brief 对Track指令应用开火门判断(公开接口)
 *
 * 这是公开接口,调用内部的fire_gate实现。
 */
WeaponCommand apply_track_fire_gate(
    WeaponCommand cmd, const L4::TrackCommand& track,
    const L4::ReferenceTrajectoryConfig& trajectory_cfg, const FireDecisionConfig& fire_cfg,
    double current_yaw, double current_pitch) noexcept {
    // 从fire_trajectory采样开火瞄准点
    const auto fire_target = sample_fire_trajectory(track, trajectory_cfg, cmd.timestamp_ns);

    // 如果采样失败,禁止开火
    if (!fire_target) {
        cmd.fire = false;
        return cmd;
    }

    // 应用开火门判断
    return fire_gate(cmd, fire_cfg, current_yaw, current_pitch, *fire_target);
}

/**
 * @brief 注册增强版武器系统到调度器(核心实现)
 *
 * 这是整个L5武器层的入口函数,负责:
 * 1. 初始化轨迹优化器(创建TinyMpcTrajectoryOptimizer实例)
 * 2. 将武器控制逻辑注册为fixed_rate系统(250Hz)
 * 3. 处理三种控制意图:Track、Shot、Hold
 *
 * 系统架构:
 * - 输入通道:ControlIntent(L4层的控制意图)
 * - 资源访问:CoordinateSystem(获取当前云台姿态)
 * - 输出通道:WeaponCommand(发送给硬件层)
 *
 * 工作流程(每4ms):
 * 1. 读取控制意图通道(非阻塞)
 * 2. 根据意图类型分支处理:
 *    - Track:MPC优化 → 开火门判断 → 写入武器指令
 *    - Shot:透传目标 → 开火门判断 → 写入武器指令
 *    - Hold:写入空指令(distance=-1) → 云台保持当前位置
 * 3. 错误处理:
 *    - MPC优化失败 → 使用降级透传指令
 *    - 云台姿态查询失败 → 记录错误日志
 *
 * @param scheduler 调度器实例
 * @param config L5配置(移动语义,避免拷贝)
 */
void register_enhanced_weapon_system(talos::Scheduler& scheduler, L5Config&& config) {
    // 步骤1:获取L4轨迹配置
    const auto trajectory_cfg = scheduler.world().get_res<L4::L4Config>()->reference_trajectory;

    // 步骤2:将L5配置插入资源容器(其他系统可访问)
    scheduler.world().insert_resource(config);

    // 步骤3:创建轨迹优化器实例(使用shared_ptr避免拷贝)
    auto optimizer =
        std::make_shared<TinyMpcTrajectoryOptimizer>(config.mpc_weapon, trajectory_cfg);

    // 步骤4:注册武器控制系统(fixed_rate@250Hz)
    scheduler.add_system<talos::fixed_rate<250>>(
        "enhanced_weapon_control",
        [optimizer, trajectory_cfg, fire_cfg = config.fire_decision](
            talos::spmc<fcs::L4::ControlIntent, ControlIntentChannelTopic> intent_in,
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            talos::spmc_mut<WeaponCommand, WeaponCommandChannelTopic> weapon_out) mutable {
            // 读取控制意图(非阻塞)
            auto intent = intent_in.read();
            if (!intent) {
                return; // 无新数据,直接返回
            }

            const uint64_t now = fcs::clock::now_ns();

            // 使用std::visit穷尽处理所有意图类型
            std::visit(
                overloaded{
                    // 分支1:Track意图(持续跟踪)
                    [&](const L4::TrackCommand& cmd) {
                        // 步骤1:执行MPC优化
                        auto opt         = optimizer->optimize(cmd, now);
                        WeaponCommand wc = opt ? *opt : track_fallback(cmd, now);

                        // 步骤2:记录降级原因(如果优化失败)
                        if (!opt) {
                            wc.degradation_reason = opt.error();
                            SPDLOG_WARN("MPC optimization failed: {}", opt.error());
                        }

                        // 步骤3:获取当前云台姿态
                        auto tf = L4::lookup_gimbal_transform(*tf_buffer, now);
                        if (tf) {
                            // 步骤4:应用开火门判断
                            auto [r, p, y] = tf->euler_rot().rpy();
                            wc = apply_track_fire_gate(wc, cmd, trajectory_cfg, fire_cfg, y, p);
                        }

                        // 步骤5:写入武器指令通道
                        weapon_out.write(wc);
                    },

                    // 分支2:Shot意图(单次打击)
                    [&](const L4::ShotCommand& cmd) {
                        // 步骤1:构建透传指令(无MPC优化)
                        auto wc = optimizer->passthrough(cmd, now);

                        // 步骤2:传递降级原因(如果有)
                        if (cmd.degradation_reason) {
                            wc.degradation_reason = *cmd.degradation_reason;
                        }

                        // 步骤3:获取当前云台姿态
                        auto tf = L4::lookup_gimbal_transform(*tf_buffer, now);
                        if (tf) {
                            // 步骤4:应用开火门判断
                            auto [r, p, y] = tf->euler_rot().rpy();
                            const ReferenceTrajectory::AimPoint target{
                                .yaw      = cmd.yaw,
                                .pitch    = cmd.pitch,
                                .distance = cmd.distance,
                            };
                            wc = fire_gate(wc, fire_cfg, y, p, target);
                        } else {
                            // 步骤5:云台姿态查询失败,记录错误
                            SPDLOG_ERROR("Gimbal transform lookup failed: {}", tf.error());
                        }

                        // 步骤6:写入武器指令通道
                        weapon_out.write(wc);
                    },

                    // 分支3:Hold意图(保持姿态)
                    [&](const L4::HoldCommand&) {
                        // 不发送瞄准指令,云台保持当前位置
                        auto wc         = WeaponCommand{};
                        wc.timestamp_ns = now;
                        wc.distance     = -1.0; // 标记为无效目标
                        weapon_out.write(wc);
                    },
                },
                *intent);
        });
}

} // namespace fcs::L5
