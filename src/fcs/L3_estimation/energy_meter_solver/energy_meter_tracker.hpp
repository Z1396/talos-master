#pragma once

#include "L3_estimation/energy_meter_solver/motion_model.hpp"

#include "L3_estimation/tracker/extended_kalman_filter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace energy_meter {

// ============================================================================
// Templated Tracker — works with any model (BigRuneModel, SmallRuneModel)
// ============================================================================

template <typename ModelT>
struct Tracker {
    using Model = ModelT;
    using JetX  = ceres::Jet<double, Model::NX>;

    using PredictFunc = std::function<void(const JetX*, JetX*)>;
    using MeasureFunc = std::function<void(const JetX*, JetX*)>;
    using EKF         = ExtendedKalmanFilter<Model::NX, Model::NZ, PredictFunc, MeasureFunc>;

    struct Params {
        int lost_thres{10};
        int tracking_thres{3};
        double matcher_gate{25.0};
        int blade_unlock_frames{10};
        typename Model::Params model_params{};
    } params;

    enum State : uint8_t {
        IDLE,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } state{IDLE};

    Tracker() = default;

    explicit Tracker(const Params& p) { set_params(p); }

    void set_params(const Params& p) {
        params        = p;
        model_.params = params.model_params;
    }

    // ── EKF access ──

    bool has_ekf() const { return ekf_.has_value(); }

    void reset_ekf() { ekf_.reset(); }

    auto get_state() const -> const typename Model::VecX& {
        static typename Model::VecX zero = Model::VecX::Zero();
        return ekf_ ? ekf_->X() : zero;
    }

    auto getRadius() const -> double { return model_.params.radius; }

    void setRadius(double radius) { model_.params.radius = radius; }

    auto getCurrentBladeId() const -> int { return armor_id_; }
    auto getDirection() const -> int { return dir_; }

    void reset_blade_lock() {
        blade_locked_      = false;
        blade_lost_frames_ = 0;
    }

    // ── Lifecycle ──

    bool first_meet_u(
        const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& target_quat);

    void predict(double dt);

    bool update(
        const Eigen::Vector3d& rcenter_position,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats);

    void step(
        double dt, const Eigen::Vector3d& rcenter_position,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats);

private:
    void state_machine(bool found);
    void init_ekf(const typename Model::VecX& x0, const typename Model::MatXX& P0);

    Model model_{};
    std::optional<EKF> ekf_;

    // Direction: +1 CW, -1 ACW
    int dir_{1};
    double prev_observed_roll_{0.0};
    int dir_votes_pos_{0};
    int dir_votes_neg_{0};
    int dir_detect_count_{0};
    bool dir_locked_{false};
    static constexpr int DIR_DETECT_FRAMES = 20;

    int detecting_count_{0};
    int lost_count_{0};

    bool blade_locked_{false};
    int blade_lost_frames_{0};

    double dt_{0.0};
    int armor_id_{0};
    std::optional<typename Model::VecX> last_state_{};
};

// ============================================================================
// Tracker variant — used by energy_meter_systems after voter decides
// ============================================================================

using SmallTracker = Tracker<SmallRuneModel>;
using BigTracker   = Tracker<BigRuneModel>;
using AnyTracker   = std::variant<std::monostate, SmallTracker, BigTracker>;

} // namespace energy_meter
