// Copyright Chen Jun 2023. Licensed under the MIT License.
// Copyright xinyang 2021.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// std
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
// Eigen
#include <Eigen/Dense>
// ceres
#include <ceres/jet.h>

// Extended Kalman Filter with auto differentiation
//
// CERES JET TYPE PROVENANCE (VALUE SEMANTICS - NO MANUAL MEMORY MANAGEMENT)
//
// This file uses ceres::Jet<double, N> for automatic differentiation.
//
// WHAT IS ceres::Jet?
//   ceres::Jet<T, N> is a VALUE TYPE containing:
//     - T a;           // Scalar value (e.g., double)
//     - T v[N];        // Derivatives (gradient vector)
//
// OWNERSHIP MODEL:
//   - Stack allocation only (no heap)
//   - Value semantics (no pointers)
//   - Automatic cleanup when function returns
//   - NO ownership transfer issues
//
// EXAMPLE (see predict() function below):
//   ceres::Jet<double, N_X> x_e_jet[N_X];  // ← Stack array, automatic storage
//   f(x_e_jet, x_p_jet);                    // ← User function with autodiff
//   // Extract derivatives from x_p_jet[i].v
//
// This is RAII-compliant - no manual memory management needed~ ♪
//
// N_X: state vector dimension
// N_Z: measurement vector dimension
// PredicFunc: process nonlinear vector function
// MeasureFunc: observation nonlinear vector function
template <int N_X, int N_Z, class PredicFunc, class MeasureFunc>
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;

    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;

    using UpdateQFunc = std::function<MatrixXX()>;
    using UpdateRFunc = std::function<MatrixZZ(const MatrixZ1& z)>;

    enum class ConvergenceStatus : uint8_t {
        Unknown,
        Converging,
        Converged,
        Diverging,
    };

    struct ConvergenceCriteria {
        double converged_nis{2.0 * static_cast<double>(N_Z)};
        double diverged_nis{25.0 * static_cast<double>(N_Z)};
        double converged_max_covariance_diag{1e6};
        double diverged_max_covariance_diag{1e12};
        double min_covariance_diag{-1e-9};
        int converged_updates{3};
        int diverged_updates{2};
    };

    explicit ExtendedKalmanFilter(
        const PredicFunc& f, const MeasureFunc& h, const UpdateQFunc& u_q, const UpdateRFunc& u_r,
        const MatrixXX& P0, ConvergenceCriteria convergence_criteria = {}) noexcept
        : f(f)
        , h(h)
        , update_Q(u_q)
        , update_R(u_r)
        , P_post(P0)
        , convergence_criteria_(convergence_criteria) {
        F = MatrixXX::Zero();
        H = MatrixZX::Zero();
        update_covariance_stat_(P_post);
    }

    // Set the initial state
    void setState(const MatrixX1& x0) noexcept { x_post = x0; }

    // Read-only access for visualization
    [[nodiscard]] const MatrixX1& X() const noexcept { return x_post; }
    [[nodiscard]] const MatrixXX& P() const noexcept { return P_post; }
    [[nodiscard]] const MatrixXZ& K_gain() const noexcept { return K_; }
    [[nodiscard]] const MatrixXX& Q_mat() const noexcept { return Q; }
    [[nodiscard]] const MatrixZZ& R_mat() const noexcept { return R; }
    [[nodiscard]] ConvergenceStatus convergence_status() const noexcept {
        return convergence_status_;
    }
    [[nodiscard]] bool converging() const noexcept {
        return convergence_status_ == ConvergenceStatus::Converging;
    }
    [[nodiscard]] bool converged() const noexcept {
        return convergence_status_ == ConvergenceStatus::Converged;
    }
    [[nodiscard]] bool diverging() const noexcept {
        return convergence_status_ == ConvergenceStatus::Diverging;
    }
    [[nodiscard]] double normalized_innovation_squared() const noexcept {
        return normalized_innovation_squared_;
    }
    [[nodiscard]] double max_covariance_diag() const noexcept { return max_covariance_diag_; }
    [[nodiscard]] int consecutive_converged_updates() const noexcept {
        return consecutive_converged_updates_;
    }
    [[nodiscard]] int consecutive_diverged_updates() const noexcept {
        return consecutive_diverged_updates_;
    }
    [[nodiscard]] const ConvergenceCriteria& convergence_criteria() const noexcept {
        return convergence_criteria_;
    }

    void setConvergenceCriteria(ConvergenceCriteria criteria) noexcept {
        convergence_criteria_ = criteria;
        resetConvergenceStatus();
    }

    void resetConvergenceStatus() noexcept {
        convergence_status_            = ConvergenceStatus::Unknown;
        consecutive_converged_updates_ = 0;
        consecutive_diverged_updates_  = 0;
        normalized_innovation_squared_ = std::numeric_limits<double>::infinity();
        max_covariance_diag_           = std::numeric_limits<double>::infinity();
    }

    void setPredictFunc(const PredicFunc& f) noexcept { this->f = f; }

    void setMeasureFunc(const MeasureFunc& h) noexcept { this->h = h; }

    // Compute a predicted state
    MatrixX1 predict() noexcept {
        ceres::Jet<double, N_X> x_e_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_e_jet[i].a    = x_post[i];
            x_e_jet[i].v[i] = 1.;
            // a 对自己的偏导数为 1.
        }
        ceres::Jet<double, N_X> x_p_jet[N_X];
        f(x_e_jet, x_p_jet);

        for (int i = 0; i < N_X; ++i) {
            x_pri[i]              = x_p_jet[i].a;
            F.block(i, 0, 1, N_X) = x_p_jet[i].v.transpose();
        }

        Q      = update_Q();
        P_pri  = F * P_post * F.transpose() + Q;
        x_post = x_pri;
        update_prediction_health_();

        return x_pri;
    }

    // Update the estimated state based on measurement
    MatrixX1 update(const MatrixZ1& z) noexcept {
        ceres::Jet<double, N_X> x_p_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            x_p_jet[i].a    = x_pri[i];
            x_p_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> z_p_jet[N_Z];
        h(x_p_jet, z_p_jet);

        MatrixZ1 z_pri;
        for (int i = 0; i < N_Z; i++) {
            z_pri[i]              = z_p_jet[i].a;
            H.block(i, 0, 1, N_X) = z_p_jet[i].v.transpose();
        }

        R                 = update_R(z);
        const MatrixZ1 nu = z - z_pri;
        const MatrixZZ S  = H * P_pri * H.transpose() + R;
        Eigen::LDLT<MatrixZZ> ldlt(S);
        if (!S.allFinite() || ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
            K_                             = MatrixXZ::Zero();
            normalized_innovation_squared_ = std::numeric_limits<double>::infinity();
            mark_immediate_divergence_();
            return x_post;
        }

        const MatrixZ1 solved_nu = ldlt.solve(nu);
        const MatrixZX gain_t    = ldlt.solve(H * P_pri.transpose());
        if (!solved_nu.allFinite() || !gain_t.allFinite()) {
            K_                             = MatrixXZ::Zero();
            normalized_innovation_squared_ = std::numeric_limits<double>::infinity();
            mark_immediate_divergence_();
            return x_post;
        }

        normalized_innovation_squared_ = std::max(0.0, static_cast<double>(nu.dot(solved_nu)));
        K_                             = gain_t.transpose();
        x_post                         = x_post + K_ * nu;
        // Joseph form: guarantees P_post stays symmetric positive-definite
        const MatrixXX IKH = MatrixXX::Identity() - K_ * H;
        P_post             = IKH * P_pri * IKH.transpose() + K_ * R * K_.transpose();
        update_convergence_status_();
        return x_post;
    }

    // Zero out cross-covariance between state indices i and j
    void decorrelate(int i, int j) noexcept {
        P_post(i, j) = 0.0;
        P_post(j, i) = 0.0;
        update_covariance_stat_(P_post);
    }

private:
    void update_prediction_health_() noexcept {
        update_covariance_stat_(P_pri);
        if (!x_pri.allFinite() || !P_pri.allFinite()
            || min_covariance_diag(P_pri) < convergence_criteria_.min_covariance_diag) {
            mark_immediate_divergence_();
            return;
        }

        if (max_covariance_diag_ > convergence_criteria_.diverged_max_covariance_diag) {
            register_divergence_sample_();
            return;
        }

        if (convergence_status_ == ConvergenceStatus::Unknown) {
            convergence_status_ = ConvergenceStatus::Converging;
        }
        if (convergence_status_ == ConvergenceStatus::Converged
            && max_covariance_diag_ > convergence_criteria_.converged_max_covariance_diag) {
            convergence_status_            = ConvergenceStatus::Converging;
            consecutive_converged_updates_ = 0;
        }
    }

    void update_convergence_status_() noexcept {
        update_covariance_stat_(P_post);
        if (!std::isfinite(normalized_innovation_squared_) || !x_post.allFinite()
            || !P_post.allFinite()
            || min_covariance_diag(P_post) < convergence_criteria_.min_covariance_diag) {
            mark_immediate_divergence_();
            return;
        }

        const bool covariance_diverging =
            max_covariance_diag_ > convergence_criteria_.diverged_max_covariance_diag;
        const bool innovation_diverging =
            normalized_innovation_squared_ > convergence_criteria_.diverged_nis;
        if (covariance_diverging || innovation_diverging) {
            register_divergence_sample_();
            return;
        }

        consecutive_diverged_updates_ = 0;
        const bool covariance_converged =
            max_covariance_diag_ <= convergence_criteria_.converged_max_covariance_diag;
        const bool innovation_converged =
            normalized_innovation_squared_ <= convergence_criteria_.converged_nis;
        if (covariance_converged && innovation_converged) {
            ++consecutive_converged_updates_;
            convergence_status_ = (consecutive_converged_updates_
                                   >= std::max(1, convergence_criteria_.converged_updates))
                                    ? ConvergenceStatus::Converged
                                    : ConvergenceStatus::Converging;
            return;
        }

        consecutive_converged_updates_ = 0;
        convergence_status_            = ConvergenceStatus::Converging;
    }

    void register_divergence_sample_() noexcept {
        consecutive_converged_updates_ = 0;
        ++consecutive_diverged_updates_;
        convergence_status_ =
            (consecutive_diverged_updates_ >= std::max(1, convergence_criteria_.diverged_updates))
                ? ConvergenceStatus::Diverging
                : ConvergenceStatus::Converging;
    }

    void mark_immediate_divergence_() noexcept {
        consecutive_converged_updates_ = 0;
        consecutive_diverged_updates_  = std::max(1, convergence_criteria_.diverged_updates);
        convergence_status_            = ConvergenceStatus::Diverging;
    }

    void update_covariance_stat_(const MatrixXX& P) noexcept {
        max_covariance_diag_ = max_abs_covariance_diag(P);
    }

    static double max_abs_covariance_diag(const MatrixXX& P) noexcept {
        double max_diag = 0.0;
        for (int i = 0; i < N_X; ++i) {
            const double diag = P(i, i);
            if (!std::isfinite(diag)) {
                return std::numeric_limits<double>::infinity();
            }
            max_diag = std::max(max_diag, std::abs(diag));
        }
        return max_diag;
    }

    static double min_covariance_diag(const MatrixXX& P) noexcept {
        double min_diag = std::numeric_limits<double>::infinity();
        for (int i = 0; i < N_X; ++i) {
            const double diag = P(i, i);
            if (!std::isfinite(diag)) {
                return -std::numeric_limits<double>::infinity();
            }
            min_diag = std::min(min_diag, diag);
        }
        return min_diag;
    }

    // Process nonlinear vector function
    PredicFunc f;
    MatrixXX F;
    // Observation nonlinear vector function
    MeasureFunc h;
    MatrixZX H;
    // Process noise covariance matrix
    UpdateQFunc update_Q;
    MatrixXX Q;
    // Measurement noise covariance matrix
    UpdateRFunc update_R;
    MatrixZZ R;

    // Priori error estimate covariance matrix
    MatrixXX P_pri;
    // Posteriori error estimate covariance matrix
    MatrixXX P_post;

    // Kalman gain
    MatrixXZ K_;

    // Priori state
    MatrixX1 x_pri;
    // Posteriori state
    MatrixX1 x_post;

    ConvergenceCriteria convergence_criteria_{};
    ConvergenceStatus convergence_status_{ConvergenceStatus::Unknown};
    int consecutive_converged_updates_{0};
    int consecutive_diverged_updates_{0};
    double normalized_innovation_squared_{std::numeric_limits<double>::infinity()};
    double max_covariance_diag_{std::numeric_limits<double>::infinity()};
};
