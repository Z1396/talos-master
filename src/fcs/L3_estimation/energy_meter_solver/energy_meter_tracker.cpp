#include "L3_estimation/energy_meter_solver/energy_meter_tracker.hpp"

#include "L3_estimation/energy_meter_solver/types.hpp"
#include "euler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace energy_meter {

// ============================================================================
// Template implementation — works for both BigRuneModel and SmallRuneModel
// ============================================================================

template <typename ModelT>
void Tracker<ModelT>::init_ekf(const typename Model::VecX& x0, const typename Model::MatXX& P0) {
    auto f = [this](const JetX* x, JetX* xp) {
        Model::template predict_state<JetX>(x, JetX(dt_), dir_, xp);
    };
    auto h = [this](const JetX* x, JetX* z) {
        Model::template measure_state<JetX>(x, armor_id_, z);
    };
    auto q = [this]() -> typename Model::MatXX { return model_.Q(dt_); };
    auto r = [this](const typename Model::VecZ& z) -> typename Model::MatZZ { return model_.R(z); };

    ekf_.emplace(f, h, q, r, P0);
    ekf_->setState(x0);
}

template <typename ModelT>
bool Tracker<ModelT>::update(
    const Eigen::Vector3d& rcenter_position, const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats) {

    if (!ekf_ || target_positions.empty() || target_quats.empty()) {
        state_machine(false);
        return false;
    }

    using VecZ = typename Model::VecZ;

    const auto& pos          = rcenter_position;
    const auto& x_pre        = ekf_->X();
    constexpr int armors_num = ARMORS_NUM;

    VecZ best_meas   = VecZ::Zero();
    double best_cost = std::numeric_limits<double>::max();
    int best_id      = -1;

    auto match_observation = [&](const VecZ& meas, int id_start, int id_end) {
        for (int id = id_start; id < id_end; ++id) {
            VecZ z_pred    = model_.h(x_pre, id);
            VecZ nu        = meas - z_pred;
            nu[ARMOR_ROLL] = normalize_rad(nu[ARMOR_ROLL]);

            const auto R_diag = model_.R_diag(z_pred);
            const double d2   = (nu.array().square() / R_diag.array()).sum();

            constexpr double kMaxRollResidual = M_PI / static_cast<double>(ARMORS_NUM);
            if (std::isfinite(d2) && d2 < params.matcher_gate && d2 < best_cost
                && std::abs(nu[ARMOR_ROLL]) < kMaxRollResidual) {
                best_cost = d2;
                best_id   = id;
                best_meas = meas;
            }
        }
    };

    for (const auto& target_quat : target_quats) {
        const auto euler              = math_fuxk::rpy(target_quat);
        const auto [roll, pitch, yaw] = euler.rpy();

        VecZ meas;
        meas[CENTER_X]   = pos.x();
        meas[CENTER_Y]   = pos.y();
        meas[CENTER_Z]   = pos.z();
        meas[RUNE_YAW]   = yaw;
        meas[ARMOR_ROLL] = roll;

        if (blade_locked_) {
            match_observation(meas, armor_id_, armor_id_ + 1);
        } else {
            match_observation(meas, 0, armors_num);
        }
    }

    if (blade_locked_ && best_id < 0) {
        blade_lost_frames_++;
        if (blade_lost_frames_ >= params.blade_unlock_frames) {
            blade_locked_      = false;
            blade_lost_frames_ = 0;
            SPDLOG_INFO(
                "energy_meter: blade lock released after {} frames", params.blade_unlock_frames);

            best_cost = std::numeric_limits<double>::max();
            best_id   = -1;
            for (const auto& target_quat : target_quats) {
                const auto euler              = math_fuxk::rpy(target_quat);
                const auto [roll, pitch, yaw] = euler.rpy();

                VecZ meas;
                meas[CENTER_X]   = pos.x();
                meas[CENTER_Y]   = pos.y();
                meas[CENTER_Z]   = pos.z();
                meas[RUNE_YAW]   = yaw;
                meas[ARMOR_ROLL] = roll;

                match_observation(meas, 0, armors_num);
            }
        }
    } else if (blade_locked_ && best_id >= 0) {
        blade_lost_frames_ = 0;
    }

    if (best_id < 0) {
        state_machine(false);
        return false;
    }

    state_machine(true);

    if (state == TRACKING && !blade_locked_) {
        blade_locked_      = true;
        blade_lost_frames_ = 0;
        SPDLOG_INFO("energy_meter: blade locked to id={}", best_id);
    }

    const double blade_offset =
        static_cast<double>(best_id) * 2.0 * M_PI / static_cast<double>(ARMORS_NUM);
    const double observed_base_roll =
        unwrap_rad(x_pre[Model::State::THETA], best_meas[ARMOR_ROLL] - blade_offset);

    VecZ z;
    z[CENTER_X]   = pos.x();
    z[CENTER_Y]   = pos.y();
    z[CENTER_Z]   = pos.z();
    z[RUNE_YAW]   = best_meas[RUNE_YAW];
    z[ARMOR_ROLL] = observed_base_roll + blade_offset;

    armor_id_ = best_id;
    ekf_->update(z);

    // Clamp physical parameters to valid range (no sigmoid guard)
    if constexpr (std::is_same_v<ModelT, BigRuneModel>) {
        auto x      = ekf_->X();
        x(Model::A) = std::clamp(x(Model::A), BigRuneModel::A_LOWER, BigRuneModel::A_UPPER);
        x(Model::W) = std::clamp(x(Model::W), BigRuneModel::W_LOWER, BigRuneModel::W_UPPER);
        ekf_->setState(x);
    }

    // Direction detection from raw observations
    if (!dir_locked_) {
        if (dir_detect_count_ > 0) {
            const double droll         = normalize_rad(observed_base_roll - prev_observed_roll_);
            constexpr double kMinDelta = 0.005;
            if (droll > kMinDelta) {
                dir_votes_pos_++;
            } else if (droll < -kMinDelta) {
                dir_votes_neg_++;
            }
            if (dir_votes_pos_ != dir_votes_neg_) {
                dir_ = (dir_votes_pos_ > dir_votes_neg_) ? 1 : -1;
            }
        }
        prev_observed_roll_ = observed_base_roll;
        dir_detect_count_++;

        if (dir_detect_count_ >= DIR_DETECT_FRAMES) {
            dir_        = (dir_votes_pos_ >= dir_votes_neg_) ? 1 : -1;
            dir_locked_ = true;
            SPDLOG_INFO(
                "energy_meter: direction locked: dir={}, votes +{}/-{}", dir_, dir_votes_pos_,
                dir_votes_neg_);
        }
    }

    return true;
}

template <typename ModelT>
void Tracker<ModelT>::predict(double dt) {
    if (!ekf_) {
        return;
    }
    dt_ = std::max(0.0, dt);
    ekf_->predict();
}

template <typename ModelT>
void Tracker<ModelT>::step(
    double dt, const Eigen::Vector3d& rcenter_position,
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats) {
    predict(dt);
    update(rcenter_position, target_positions, target_quats);
}

// ============================================================================
// first_meet_u — BigRuneModel specialization (8D state with TAU, A, W)
// ============================================================================

template <>
bool Tracker<BigRuneModel>::first_meet_u(
    const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_quat) {

    const auto euler              = math_fuxk::rpy(target_quat);
    const auto [roll, pitch, yaw] = euler.rpy();

    const double observed_radius = (target_pos - r_center).norm();
    model_.params.radius         = observed_radius;

    BigRuneModel::VecX x0   = BigRuneModel::VecX::Zero();
    x0(BigRuneModel::XC)    = r_center.x();
    x0(BigRuneModel::YC)    = r_center.y();
    x0(BigRuneModel::ZC)    = r_center.z();
    x0(BigRuneModel::YAW)   = yaw;
    x0(BigRuneModel::THETA) = roll;
    if (last_state_) {
        x0(BigRuneModel::TAU) = (*last_state_)(BigRuneModel::TAU);
        x0(BigRuneModel::A)   = (*last_state_)(BigRuneModel::A);
        x0(BigRuneModel::W)   = (*last_state_)(BigRuneModel::W);
        last_state_.reset();
    } else {
        x0(BigRuneModel::TAU) = 0.0;
        x0(BigRuneModel::A)   = 0.9125; // midpoint of [0.780, 1.045]
        x0(BigRuneModel::W)   = 1.942;  // midpoint of [1.884, 2.000]
    }

    BigRuneModel::MatXX P0                   = BigRuneModel::MatXX::Identity() * 0.1;
    P0(BigRuneModel::A, BigRuneModel::A)     = 0.08;
    P0(BigRuneModel::W, BigRuneModel::W)     = 0.05;
    P0(BigRuneModel::TAU, BigRuneModel::TAU) = 4.0;

    init_ekf(x0, P0);

    dir_                = 1;
    prev_observed_roll_ = roll;
    dir_detect_count_   = 0;
    dir_votes_pos_      = 0;
    dir_votes_neg_      = 0;
    dir_locked_         = false;

    reset_blade_lock();

    state_machine(true);
    return true;
}

// ============================================================================
// first_meet_u — SmallRuneModel specialization (5D state, constant speed)
// ============================================================================

template <>
bool Tracker<SmallRuneModel>::first_meet_u(
    const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_quat) {

    const auto euler              = math_fuxk::rpy(target_quat);
    const auto [roll, pitch, yaw] = euler.rpy();

    const double observed_radius = (target_pos - r_center).norm();
    model_.params.radius         = observed_radius;

    SmallRuneModel::VecX x0   = SmallRuneModel::VecX::Zero();
    x0(SmallRuneModel::XC)    = r_center.x();
    x0(SmallRuneModel::YC)    = r_center.y();
    x0(SmallRuneModel::ZC)    = r_center.z();
    x0(SmallRuneModel::YAW)   = yaw;
    x0(SmallRuneModel::THETA) = roll;

    // P0: moderate position uncertainty, low θ uncertainty
    SmallRuneModel::MatXX P0                         = SmallRuneModel::MatXX::Identity() * 0.1;
    P0(SmallRuneModel::THETA, SmallRuneModel::THETA) = 1.0;

    init_ekf(x0, P0);

    dir_                = 1;
    prev_observed_roll_ = roll;
    dir_detect_count_   = 0;
    dir_votes_pos_      = 0;
    dir_votes_neg_      = 0;
    dir_locked_         = false;

    reset_blade_lock();

    state_machine(true);
    return true;
}

template <typename ModelT>
void Tracker<ModelT>::state_machine(bool found) {
    switch (state) {
    case IDLE:
        if (found) {
            ++detecting_count_;
            state = DETECTING;
        } else {
            if (ekf_)
                last_state_.emplace(ekf_->X());
            ekf_.reset();
            detecting_count_ = 0;
        }
        break;

    case DETECTING:
        if (found) {
            ++detecting_count_;
            if (detecting_count_ > params.tracking_thres) {
                state            = TRACKING;
                detecting_count_ = 0;
            }
        } else {
            detecting_count_ = 0;
            if (ekf_)
                last_state_.emplace(ekf_->X());
            ekf_.reset();
            state = IDLE;
        }
        break;

    case TRACKING:
        if (!found) {
            state       = TEMP_LOST;
            lost_count_ = 1;
        }
        break;

    case TEMP_LOST:
        if (found) {
            lost_count_ = 0;
            state       = TRACKING;
        } else {
            ++lost_count_;
            if (lost_count_ > params.lost_thres) {
                lost_count_      = 0;
                detecting_count_ = 0;
                if (ekf_)
                    last_state_.emplace(ekf_->X());
                ekf_.reset();
                reset_blade_lock();
                state = IDLE;
            }
        }
        break;
    }
}

// Explicit instantiation for both model types
template struct Tracker<BigRuneModel>;
template struct Tracker<SmallRuneModel>;

} // namespace energy_meter
