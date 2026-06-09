#include "L3_estimation/energy_meter/energy_meter_systems.hpp"

#include "L2_perception/rune/types.hpp"
#include "L3_estimation/energy_meter/types.hpp"
#include "L3_estimation/energy_meter_solver/energy_meter_tracker.hpp"
#include "L3_estimation/energy_meter_solver/types.hpp"
#include "L3_estimation/energy_meter_solver/voter.hpp"
#include "core/channel_topics.hpp"
#include "core/time.hpp"
#include "scheduler/scheduler.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <variant>

#include <spdlog/spdlog.h>

namespace fcs::energy_meter {

namespace {

// Build state from a tracker of any model type.
// Both models share indices [XC=0, YC=1, ZC=2, YAW=3, THETA=4].
template <typename ModelT>
EnergyMeterState build_state_from_tracker(
    const ::energy_meter::Tracker<ModelT>& tracker, uint64_t timestamp_ns, int tracking_frames,
    bool is_big, bool obs_valid) {
    const auto& x = tracker.get_state();

    const double roll   = x(ModelT::THETA);
    const double yaw    = x(ModelT::YAW);
    const double radius = tracker.getRadius();

    const bool is_tracking =
        (tracker.state == ::energy_meter::Tracker<ModelT>::TRACKING
         || tracker.state == ::energy_meter::Tracker<ModelT>::TEMP_LOST);

    const double blade_roll =
        roll + static_cast<double>(tracker.getCurrentBladeId()) * 2.0 * M_PI / 5.0;
    const double sr = std::sin(blade_roll);
    const double cr = std::cos(blade_roll);
    Eigen::Vector3d blade_local(0.0, -radius * sr, radius * cr);

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const Eigen::Vector3d blade_offset(
        blade_local.x() * cy - blade_local.y() * sy, blade_local.x() * sy + blade_local.y() * cy,
        blade_local.z());

    const Eigen::Vector3d r_center(x(ModelT::XC), x(ModelT::YC), x(ModelT::ZC));

    const Eigen::Quaterniond yaw_quat(std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2));
    const Eigen::Quaterniond roll_quat(
        std::cos(blade_roll / 2), std::sin(blade_roll / 2), 0.0, 0.0);

    EnergyMeterState out;
    out.timestamp_ns   = timestamp_ns;
    out.tracker_state  = static_cast<TrackerState>(tracker.state);
    out.tracking_valid = is_tracking;
    out.r_center_odom  = r_center;
    out.radius         = radius;
    out.yaw            = yaw;
    out.pitch          = 0.0;
    out.roll           = roll;
    out.blade_id       = tracker.getCurrentBladeId();
    out.direction      = tracker.getDirection();
    out.is_big_rune    = is_big;
    out.position       = r_center + blade_offset;
    out.quat           = yaw_quat * roll_quat;

    out.obs_valid = obs_valid;

    if constexpr (std::is_same_v<ModelT, ::energy_meter::BigRuneModel>) {
        out.t           = x(ModelT::TAU);
        out.a           = x(ModelT::A);
        out.omega       = x(ModelT::W);
        out.b           = ::energy_meter::to_b(out.a);
        out.model_valid = is_tracking && tracking_frames > 5;
    } else {
        out.t           = 0.0;
        out.a           = 0.0;
        out.omega       = ::energy_meter::FIXED_RUNE_SPEED;
        out.b           = ::energy_meter::FIXED_RUNE_SPEED;
        out.model_valid = is_tracking;
    }

    return out;
}

EnergyMeterState build_state(
    const ::energy_meter::AnyTracker& tracker_var, uint64_t timestamp_ns, int tracking_frames,
    bool is_big, bool obs_valid) {
    return std::visit(
        [&](const auto& tk) -> EnergyMeterState {
            using T = std::decay_t<decltype(tk)>;
            if constexpr (std::is_same_v<T, std::monostate>) {
                return EnergyMeterState{};
            } else {
                return build_state_from_tracker(
                    tk, timestamp_ns, tracking_frames, is_big, obs_valid);
            }
        },
        tracker_var);
}

} // namespace

void register_energy_meter_systems(talos::Scheduler& scheduler, EnergyMeterL3Config&& config) {
    scheduler.world().insert_resource(std::move(config));

    scheduler.world().insert_resource(std::make_shared<::energy_meter::AnyTracker>());

    scheduler.add_system<talos::pool_compute>(
        "l3_energy_meter",
        [last_timestamp_ns = uint64_t{0}, tracking_frames = int{0},
         voter = ::energy_meter::Voter{}](
            talos::res<EnergyMeterL3Config> cfg,
            talos::res_mut<std::shared_ptr<::energy_meter::AnyTracker>> tracker_ptr,
            talos::subscribe<fcs::rune::RuneObservation, RuneObservationChannelTopic> obs_in,
            talos::publish<EnergyMeterState, EnergyMeterStateChannelTopic> state_out) mutable {
            auto obs = obs_in.read();

            const bool obs_valid = obs && obs->valid && !obs->target_positions_odom.empty()
                                && !obs->target_quats_odom.empty();

            const uint64_t frame_ns = obs ? obs->timestamp_ns : fcs::clock::now_ns();
            double dt               = 0.0;
            if (last_timestamp_ns > 0 && frame_ns > last_timestamp_ns) {
                dt = static_cast<double>(frame_ns - last_timestamp_ns) * 1e-9;
            }

            auto& tracker_var = **tracker_ptr;

            // IDLE 状态且无有效观测，直接返回
            bool is_idle = false;
            std::visit(
                [&](const auto& tk) {
                    using T = std::decay_t<decltype(tk)>;
                    if constexpr (std::is_same_v<T, std::monostate>) {
                        is_idle = true;
                    } else {
                        is_idle = (tk.state == T::IDLE);
                    }
                },
                tracker_var);

            if (is_idle && !obs_valid) {
                return;
            }

            last_timestamp_ns = frame_ns;

            // Reset on large gap
            if (dt > cfg->reset_vote_time) {
                **tracker_ptr     = ::energy_meter::AnyTracker{};
                last_timestamp_ns = 0;
                tracking_frames   = 0;
                voter.reset();
                SPDLOG_WARN("energy_meter: large gap {:.1f}s, resetting", dt);
                return;
            }

            // ── No valid observation: advance state machine, publish lost state ──
            if (!obs_valid) {
                std::visit(
                    [&](auto& tk) {
                        using T = std::decay_t<decltype(tk)>;
                        if constexpr (!std::is_same_v<T, std::monostate>) {
                            if (dt > 0.0) {
                                tk.predict(dt);
                                tk.update(Eigen::Vector3d::Zero(), {}, {});
                            }
                        }
                    },
                    tracker_var);

                if (voter.determined() && !std::holds_alternative<std::monostate>(tracker_var)) {
                    const bool is_tracking = std::visit(
                        [&](const auto& tk) -> bool {
                            using T = std::decay_t<decltype(tk)>;
                            if constexpr (std::is_same_v<T, std::monostate>) {
                                return false;
                            } else {
                                return tk.state == T::TRACKING || tk.state == T::TEMP_LOST;
                            }
                        },
                        tracker_var);

                    tracking_frames = is_tracking ? tracking_frames + 1 : 0;
                    state_out.write(build_state(
                        tracker_var, frame_ns, tracking_frames, voter.is_big(), obs_valid));
                }
                return;
            }

            // ── Big/small rune detection ──
            if (!voter.determined()) {
                if (const auto decision = voter.update(obs->target_positions_odom.size())) {
                    SPDLOG_INFO(
                        "靶数投票决策: {}, big={} small={}, frames={}",
                        decision->is_big ? "大符" : "小符", decision->big_votes,
                        decision->small_votes, decision->frames);

                    if (decision->is_big) {
                        tracker_var = ::energy_meter::BigTracker{};
                    } else {
                        tracker_var = ::energy_meter::SmallTracker{};
                    }
                }
            }

            if (std::holds_alternative<std::monostate>(tracker_var)) {
                return;
            }

            // ── EKF tracker step ──
            std::visit(
                [&](auto& tk) {
                    using T = std::decay_t<decltype(tk)>;
                    if constexpr (!std::is_same_v<T, std::monostate>) {
                        typename T::Params p;
                        p.lost_thres          = cfg->lost_thres;
                        p.tracking_thres      = cfg->tracking_thres;
                        p.matcher_gate        = cfg->matcher_gate;
                        p.blade_unlock_frames = cfg->blade_unlock_frames;
                        if constexpr (std::is_same_v<T, ::energy_meter::BigTracker>) {
                            p.model_params = cfg->big_model;
                        } else {
                            p.model_params = cfg->small_model;
                        }
                        tk.set_params(p);

                        if (!tk.has_ekf()) {
                            tk.first_meet_u(
                                obs->r_center_odom.translation(), obs->target_positions_odom[0],
                                obs->target_quats_odom[0]);
                        }

                        const double observed_radius =
                            (obs->target_positions_odom[0] - obs->r_center_odom.translation())
                                .norm();
                        tk.setRadius(observed_radius);

                        if (dt > 0.0) {
                            tk.step(
                                dt, obs->r_center_odom.translation(), obs->target_positions_odom,
                                obs->target_quats_odom);
                        } else {
                            tk.update(
                                obs->r_center_odom.translation(), obs->target_positions_odom,
                                obs->target_quats_odom);
                        }
                    }
                },
                tracker_var);

            // ── Output ──
            const bool is_tracking = std::visit(
                [&](const auto& tk) -> bool {
                    using T = std::decay_t<decltype(tk)>;
                    if constexpr (std::is_same_v<T, std::monostate>) {
                        return false;
                    } else {
                        return tk.state == T::TRACKING || tk.state == T::TEMP_LOST;
                    }
                },
                tracker_var);

            tracking_frames = is_tracking ? tracking_frames + 1 : 0;
            state_out.write(build_state(
                tracker_var, obs->timestamp_ns, tracking_frames, voter.is_big(), obs_valid));
        });
}

} // namespace fcs::energy_meter
