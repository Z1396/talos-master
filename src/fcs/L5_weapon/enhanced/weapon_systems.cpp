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
/// std::visit helper for variant exhaustiveness.
template <typename... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

using ReferenceTrajectory = core::trajectory::ReferenceTrajectory;
using fcs::core::math::normalize_angle;

[[nodiscard]] int quantize_reference_age_steps(double reference_age_s, double dt_s) noexcept {
    if (!(reference_age_s > 0.0) || !(dt_s > 0.0)) {
        return 0;
    }
    return std::max(0, static_cast<int>(std::lround(reference_age_s / dt_s)));
}

[[nodiscard]] std::optional<ReferenceTrajectory::AimPoint> sample_reference_trajectory(
    const ReferenceTrajectory& trajectory, const L4::ReferenceTrajectoryConfig& trajectory_cfg,
    uint64_t trajectory_timestamp_ns, uint64_t command_timestamp_ns) noexcept {
    const int horizon = trajectory.horizon();
    if (horizon <= 0 || trajectory.distances.size() < static_cast<size_t>(horizon)
        || trajectory.time_of_flights.size() < static_cast<size_t>(horizon)) {
        return std::nullopt;
    }

    const double reference_age_s = static_cast<double>(
                                       static_cast<int64_t>(command_timestamp_ns)
                                       - static_cast<int64_t>(trajectory_timestamp_ns))
                                 * 1e-9;
    const int shift_steps =
        quantize_reference_age_steps(std::max(reference_age_s, 0.0), trajectory_cfg.dt);
    const int start_index       = std::clamp(shift_steps, 0, horizon - 1);
    const int available_horizon = horizon - start_index;
    const int center_index =
        std::clamp(trajectory_cfg.horizon_back, 0, std::max(available_horizon - 1, 0));
    const int sample_index = start_index + center_index;

    return ReferenceTrajectory::AimPoint{
        .yaw      = normalize_angle(trajectory.state(0, sample_index) + trajectory.yaw_origin),
        .pitch    = trajectory.state(2, sample_index),
        .distance = trajectory.distances[static_cast<size_t>(sample_index)],
    };
}

/// Apply fire gate: compute is_on_target and stamp it into the command.
[[nodiscard]] WeaponCommand fire_gate(
    WeaponCommand cmd, const FireDecisionConfig& fire_cfg, double cur_yaw, double cur_pitch,
    const ReferenceTrajectory::AimPoint& target) noexcept {
    auto result =
        is_on_target(fire_cfg, cur_yaw, -cur_pitch, target.yaw, target.pitch, target.distance);
    cmd.fire                 = result.fire;
    cmd.yaw_error            = result.yaw_error;
    cmd.pitch_error          = result.pitch_error;
    cmd.shooting_range_yaw   = result.shooting_range_yaw;
    cmd.shooting_range_pitch = result.shooting_range_pitch;
    cmd.ref_yaw              = target.yaw;
    cmd.ref_pitch            = target.pitch;
    return cmd;
}

/// Build a fallback passthrough command from a TrackCommand's trajectory center.
[[nodiscard]] WeaponCommand
    track_fallback(const L4::TrackCommand& track, uint64_t command_timestamp_ns) noexcept {
    const auto aim = track.control_trajectory.center_aim_point();
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

std::optional<ReferenceTrajectory::AimPoint> sample_fire_trajectory(
    const L4::TrackCommand& track, const L4::ReferenceTrajectoryConfig& trajectory_cfg,
    uint64_t command_timestamp_ns) noexcept {
    return sample_reference_trajectory(
        track.fire_trajectory, trajectory_cfg, track.timestamp_ns, command_timestamp_ns);
}

WeaponCommand apply_track_fire_gate(
    WeaponCommand cmd, const L4::TrackCommand& track,
    const L4::ReferenceTrajectoryConfig& trajectory_cfg, const FireDecisionConfig& fire_cfg,
    double current_yaw, double current_pitch) noexcept {
    const auto fire_target = sample_fire_trajectory(track, trajectory_cfg, cmd.timestamp_ns);
    if (!fire_target) {
        cmd.fire = false;
        return cmd;
    }
    return fire_gate(cmd, fire_cfg, current_yaw, current_pitch, *fire_target);
}

void register_enhanced_weapon_system(talos::Scheduler& scheduler, L5Config&& config) {
    const auto trajectory_cfg = scheduler.world().get_res<L4::L4Config>()->reference_trajectory;
    scheduler.world().insert_resource(config);
    auto optimizer =
        std::make_shared<TinyMpcTrajectoryOptimizer>(config.mpc_weapon, trajectory_cfg);

    scheduler.add_system<talos::fixed_rate<250>>(
        "enhanced_weapon_control",
        [optimizer, trajectory_cfg, fire_cfg = config.fire_decision](
            talos::spmc<fcs::L4::ControlIntent, ControlIntentChannelTopic> intent_in,
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            talos::spmc_mut<WeaponCommand, WeaponCommandChannelTopic> weapon_out) mutable {
            auto intent = intent_in.read();
            if (!intent) {
                return;
            }

            const uint64_t now = fcs::clock::now_ns();

            std::visit(
                overloaded{
                    [&](const L4::TrackCommand& cmd) {
                        auto opt         = optimizer->optimize(cmd, now);
                        WeaponCommand wc = opt ? *opt : track_fallback(cmd, now);
                        if (!opt) {
                            wc.degradation_reason = opt.error();
                            SPDLOG_WARN("MPC optimization failed: {}", opt.error());
                        }
                        auto tf = L4::lookup_gimbal_transform(*tf_buffer, now);
                        if (tf) {
                            auto [r, p, y] = tf->euler_rot().rpy();
                            wc = apply_track_fire_gate(wc, cmd, trajectory_cfg, fire_cfg, y, p);
                        }
                        weapon_out.write(wc);
                    },
                    [&](const L4::ShotCommand& cmd) {
                        auto wc = optimizer->passthrough(cmd, now);
                        if (cmd.degradation_reason) {
                            wc.degradation_reason = *cmd.degradation_reason;
                        }
                        auto tf = L4::lookup_gimbal_transform(*tf_buffer, now);
                        if (tf) {
                            auto [r, p, y] = tf->euler_rot().rpy();
                            const ReferenceTrajectory::AimPoint target{
                                .yaw      = cmd.yaw,
                                .pitch    = cmd.pitch,
                                .distance = cmd.distance,
                            };
                            wc = fire_gate(wc, fire_cfg, y, p, target);
                        } else {
                            SPDLOG_ERROR("Gimbal transform lookup failed: {}", tf.error());
                        }
                        weapon_out.write(wc);
                    },
                    [&](const L4::HoldCommand&) {
                        // No target — don't emit a weapon command.
                        // Gimbal stays at current position.
                        auto wc         = WeaponCommand{};
                        wc.timestamp_ns = now;
                        wc.distance     = -1.0;
                        weapon_out.write(wc);
                    },
                },
                *intent);
        });
}

} // namespace fcs::L5
