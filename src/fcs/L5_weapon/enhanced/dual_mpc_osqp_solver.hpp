#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <limits>
#include <string>

namespace fcs::L5 {

/// Dependency-free drop-in gold oracle for the original trajectory-smoothing QP,
/// restricted to the core constraint used by the planner: acceleration box bounds.
///
/// This class intentionally mirrors the public shape of DualSmallMpcSolver, but it is
/// an offline benchmark oracle, not a real-time solver.
///
/// It solves, independently for yaw and pitch:
///
///     minimize_u  0.5 * sum_k q_pos * (pos_k(u) - ref_pos_k)^2
///               + 0.5 * sum_k q_vel * (vel_k(u) - ref_vel_k)^2
///               + 0.5 * sum_k r     * u_k^2
///
///     subject to  -max_acc <= u_k <= max_acc.
///
/// Dynamics are condensed out exactly, so the QP variable is only the acceleration
/// sequence u[0:N-2]. The default dynamics match the original DualSmallMpcSolver:
///
///     pos[k+1] = pos[k] + dt * vel[k]
///     vel[k+1] = vel[k] + dt * acc[k]
///
/// Important limitations:
///   - enable_state_bound is accepted for API compatibility but is NOT enforced here.
///     If state bounds are essential to the gold standard, use a real QP solver.
///   - rho is accepted for API compatibility but is NOT used. rho is an ADMM parameter,
///     not part of the original trajectory optimization objective.
///   - This oracle uses projected-gradient iterations plus KKT residual verification.
class DualMpcOsqpSolver {
public:
    struct AxisConfig {
        float q_pos{9e6f};
        float q_vel{0.0f};
        float r{1.0f};
        float max_acc{50.0f};
        float state_min{-std::numeric_limits<float>::infinity()};
        float state_max{std::numeric_limits<float>::infinity()};
        bool enable_state_bound{false};
    };

    static constexpr int kAxisCount       = 2;
    static constexpr int kYawAxis         = 0;
    static constexpr int kPitchAxis       = 1;
    static constexpr int kMaxHorizon      = 101;
    static constexpr int kMaxInputHorizon = kMaxHorizon - 1;

    DualMpcOsqpSolver() = default;

    DualMpcOsqpSolver(const DualMpcOsqpSolver&)                = delete;
    DualMpcOsqpSolver& operator=(const DualMpcOsqpSolver&)     = delete;
    DualMpcOsqpSolver(DualMpcOsqpSolver&&) noexcept            = default;
    DualMpcOsqpSolver& operator=(DualMpcOsqpSolver&&) noexcept = default;
    ~DualMpcOsqpSolver()                                       = default;

    [[nodiscard]] static std::expected<DualMpcOsqpSolver, std::string>
        create(float dt, int horizon, float rho, const AxisConfig& yaw, const AxisConfig& pitch) {
        if (dt <= 0.0f || !std::isfinite(dt)) {
            return std::unexpected("dt must be positive and finite");
        }
        if (horizon < 2) {
            return std::unexpected("horizon must be >= 2");
        }
        if (horizon > kMaxHorizon) {
            return std::unexpected(
                "horizon exceeds fixed-capacity limit of " + std::to_string(kMaxHorizon));
        }
        if (rho <= 0.0f || !std::isfinite(rho)) {
            return std::unexpected("rho must be positive and finite");
        }

        auto validate_axis = [](const AxisConfig& cfg,
                                const char* name) -> std::expected<void, std::string> {
            if (cfg.q_pos < 0.0f || cfg.q_vel < 0.0f) {
                return std::unexpected(std::string{name} + ": Q weights must be nonnegative");
            }
            if (cfg.r < 0.0f || !std::isfinite(cfg.r)) {
                return std::unexpected(
                    std::string{name} + ": R weight must be nonnegative and finite");
            }
            if (cfg.max_acc < 0.0f || !std::isfinite(cfg.max_acc)) {
                return std::unexpected(
                    std::string{name} + ": max_acc must be nonnegative and finite");
            }
            return {};
        };
        if (auto ok = validate_axis(yaw, "yaw"); !ok) {
            return std::unexpected(ok.error());
        }
        if (auto ok = validate_axis(pitch, "pitch"); !ok) {
            return std::unexpected(ok.error());
        }

        DualMpcOsqpSolver s;
        s.N_                = horizon;
        s.last_state_index_ = horizon - 1;
        s.last_input_index_ = horizon - 2;
        s.rho_              = rho; // API parity only. Not used.

        s.a00_   = 1.0;
        s.a01_   = static_cast<double>(dt);
        s.a10_   = 0.0;
        s.a11_   = 1.0;
        s.b0_    = 0.0;
        s.b1_    = static_cast<double>(dt);
        s.f_pos_ = 0.0;
        s.f_vel_ = 0.0;

        s.init_axis(kYawAxis, yaw);
        s.init_axis(kPitchAxis, pitch);
        s.reset_workspace();
        return s;
    }

    void set_settings(float abs_tol, int max_iter) noexcept {
        abs_tol_  = static_cast<double>(abs_tol);
        max_iter_ = max_iter;
    }

    void set_x0(int axis, double pos, double vel) noexcept {
        x0_pos_(axis) = pos;
        x0_vel_(axis) = vel;
    }

    void set_x0(double yaw_pos, double yaw_vel, double pitch_pos, double pitch_vel) noexcept {
        set_x0(kYawAxis, yaw_pos, yaw_vel);
        set_x0(kPitchAxis, pitch_pos, pitch_vel);
    }

    void set_ref_col(int axis, int k, float pos, float vel) noexcept {
        ref_pos_(axis, k) = static_cast<double>(pos);
        ref_vel_(axis, k) = static_cast<double>(vel);
    }

    void set_ref_col(
        int k, float yaw_pos, float yaw_vel, float pitch_pos, float pitch_vel) noexcept {
        set_ref_col(kYawAxis, k, yaw_pos, yaw_vel);
        set_ref_col(kPitchAxis, k, pitch_pos, pitch_vel);
    }

    void set_reference(
        const Eigen::Ref<const Eigen::MatrixXf>& yaw_ref,
        const Eigen::Ref<const Eigen::MatrixXf>& pitch_ref) noexcept {
        for (int k = 0; k < N_; ++k) {
            ref_pos_(kYawAxis, k)   = static_cast<double>(yaw_ref(0, k));
            ref_vel_(kYawAxis, k)   = static_cast<double>(yaw_ref(1, k));
            ref_pos_(kPitchAxis, k) = static_cast<double>(pitch_ref(0, k));
            ref_vel_(kPitchAxis, k) = static_cast<double>(pitch_ref(1, k));
        }
    }

    bool solve() noexcept {
        converged_ = false;
        converged_axis_.fill(false);
        iters_ = 0;

        const bool yaw_ok   = solve_axis(kYawAxis);
        const bool pitch_ok = solve_axis(kPitchAxis);

        converged_axis_[kYawAxis]   = yaw_ok;
        converged_axis_[kPitchAxis] = pitch_ok;
        converged_                  = yaw_ok && pitch_ok;
        iters_                      = std::max(axis_iters_[kYawAxis], axis_iters_[kPitchAxis]);
        return converged_;
    }

    [[nodiscard]] float state(int axis, int dim, int k) const noexcept {
        return static_cast<float>(dim == 0 ? sol_pos_(axis, k) : sol_vel_(axis, k));
    }

    [[nodiscard]] float input(int axis, int k) const noexcept {
        return static_cast<float>(sol_acc_(axis, k));
    }

    [[nodiscard]] bool converged() const noexcept { return converged_; }
    [[nodiscard]] bool converged(int axis) const noexcept { return converged_axis_[axis]; }
    [[nodiscard]] int iterations() const noexcept { return iters_; }
    [[nodiscard]] int horizon() const noexcept { return N_; }

    [[nodiscard]] double objective(int axis) const noexcept { return objective_(axis); }
    [[nodiscard]] double dynamics_violation(int axis) const noexcept {
        return dyn_violation_(axis);
    }
    [[nodiscard]] double input_bound_violation(int axis) const noexcept {
        return input_violation_(axis);
    }
    [[nodiscard]] double kkt_violation(int axis) const noexcept { return kkt_violation_(axis); }

    /// Always zero because this box-QP oracle intentionally does not enforce state bounds.
    [[nodiscard]] double state_bound_violation(int) const noexcept { return 0.0; }

    [[nodiscard]] bool state_bounds_ignored(int axis) const noexcept {
        return enable_state_bound_[axis];
    }

private:
    using LaneD     = Eigen::Array<double, kAxisCount, 1>;
    using StateMatD = Eigen::Array<double, kAxisCount, kMaxHorizon>;
    using InputMatD = Eigen::Array<double, kAxisCount, kMaxInputHorizon>;

    [[nodiscard]] int num_inputs() const noexcept { return N_ - 1; }

    static double clamp_scalar(double v, double lo, double hi) noexcept {
        return std::min(std::max(v, lo), hi);
    }

    static Eigen::VectorXd clamp_vec(const Eigen::VectorXd& v, double lo, double hi) noexcept {
        return v.array().min(hi).max(lo).matrix();
    }

    double a00_{1.0};
    double a01_{0.0};
    double a10_{0.0};
    double a11_{1.0};
    double b0_{0.0};
    double b1_{0.0};
    double f_pos_{0.0};
    double f_vel_{0.0};
    double rho_{1.0};

    LaneD q_pos_{LaneD::Zero()};
    LaneD q_vel_{LaneD::Zero()};
    LaneD r_val_{LaneD::Zero()};
    LaneD u_min_{LaneD::Zero()};
    LaneD u_max_{LaneD::Zero()};
    std::array<bool, kAxisCount> enable_state_bound_{};

    int N_{0};
    int last_state_index_{0};
    int last_input_index_{0};

    LaneD x0_pos_{LaneD::Zero()};
    LaneD x0_vel_{LaneD::Zero()};
    StateMatD ref_pos_{StateMatD::Zero()};
    StateMatD ref_vel_{StateMatD::Zero()};
    StateMatD sol_pos_{StateMatD::Zero()};
    StateMatD sol_vel_{StateMatD::Zero()};
    InputMatD sol_acc_{InputMatD::Zero()};

    LaneD objective_{LaneD::Zero()};
    LaneD dyn_violation_{LaneD::Zero()};
    LaneD input_violation_{LaneD::Zero()};
    LaneD kkt_violation_{LaneD::Zero()};

    double abs_tol_{1e-9};
    int max_iter_{200000};

    bool converged_{false};
    std::array<bool, kAxisCount> converged_axis_{};
    std::array<int, kAxisCount> axis_iters_{};
    int iters_{0};

    void init_axis(int axis, const AxisConfig& cfg) noexcept {
        q_pos_(axis)              = static_cast<double>(cfg.q_pos);
        q_vel_(axis)              = static_cast<double>(cfg.q_vel);
        r_val_(axis)              = static_cast<double>(cfg.r);
        u_min_(axis)              = -static_cast<double>(cfg.max_acc);
        u_max_(axis)              = static_cast<double>(cfg.max_acc);
        enable_state_bound_[axis] = cfg.enable_state_bound;
    }

    void reset_workspace() noexcept {
        x0_pos_.setZero();
        x0_vel_.setZero();
        ref_pos_.setZero();
        ref_vel_.setZero();
        sol_pos_.setZero();
        sol_vel_.setZero();
        sol_acc_.setZero();
        objective_.setZero();
        dyn_violation_.setZero();
        input_violation_.setZero();
        kkt_violation_.setZero();
        converged_axis_.fill(false);
        axis_iters_.fill(0);
        converged_ = false;
        iters_     = 0;
    }

    void build_condensed_problem(
        int axis, Eigen::MatrixXd& H, Eigen::VectorXd& h, Eigen::VectorXd& base_pos,
        Eigen::VectorXd& base_vel, Eigen::MatrixXd& Gp, Eigen::MatrixXd& Gv) const {
        const int M = num_inputs();
        base_pos    = Eigen::VectorXd::Zero(N_);
        base_vel    = Eigen::VectorXd::Zero(N_);
        Gp          = Eigen::MatrixXd::Zero(N_, M);
        Gv          = Eigen::MatrixXd::Zero(N_, M);

        base_pos(0) = x0_pos_(axis);
        base_vel(0) = x0_vel_(axis);

        for (int k = 0; k < M; ++k) {
            base_pos(k + 1) = a00_ * base_pos(k) + a01_ * base_vel(k) + f_pos_;
            base_vel(k + 1) = a10_ * base_pos(k) + a11_ * base_vel(k) + f_vel_;

            Gp.row(k + 1) = a00_ * Gp.row(k) + a01_ * Gv.row(k);
            Gv.row(k + 1) = a10_ * Gp.row(k) + a11_ * Gv.row(k);
            if (b0_ != 0.0) {
                Gp(k + 1, k) += b0_;
            }
            Gv(k + 1, k) += b1_;
        }

        Eigen::VectorXd rpos(N_);
        Eigen::VectorXd rvel(N_);
        for (int k = 0; k < N_; ++k) {
            rpos(k) = ref_pos_(axis, k);
            rvel(k) = ref_vel_(axis, k);
        }

        H = Eigen::MatrixXd::Zero(M, M);
        h = Eigen::VectorXd::Zero(M);
        if (q_pos_(axis) != 0.0) {
            H.noalias() += q_pos_(axis) * (Gp.transpose() * Gp);
            h.noalias() += q_pos_(axis) * (Gp.transpose() * (base_pos - rpos));
        }
        if (q_vel_(axis) != 0.0) {
            H.noalias() += q_vel_(axis) * (Gv.transpose() * Gv);
            h.noalias() += q_vel_(axis) * (Gv.transpose() * (base_vel - rvel));
        }
        if (r_val_(axis) != 0.0) {
            H.diagonal().array() += r_val_(axis);
        }
    }

    static double projected_gradient_kkt_residual(
        const Eigen::VectorXd& u, const Eigen::VectorXd& grad, double lo, double hi) noexcept {
        double max_res             = 0.0;
        constexpr double kBoundTol = 1e-10;
        for (int i = 0; i < u.size(); ++i) {
            double res = 0.0;
            if (u(i) <= lo + kBoundTol) {
                // lower-bound active: stationarity requires grad >= 0.
                res = std::max(0.0, -grad(i));
            } else if (u(i) >= hi - kBoundTol) {
                // upper-bound active: stationarity requires grad <= 0.
                res = std::max(0.0, grad(i));
            } else {
                res = std::abs(grad(i));
            }
            max_res = std::max(max_res, res);
        }
        return max_res;
    }

    [[nodiscard]] bool solve_box_qp_projected_gradient(
        const Eigen::MatrixXd& H, const Eigen::VectorXd& h, double lo, double hi,
        Eigen::VectorXd& u, int& iterations, double& kkt) const noexcept {
        const int M = static_cast<int>(h.size());
        if (M == 0) {
            u.resize(0);
            iterations = 0;
            kkt        = 0.0;
            return true;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H);
        if (eig.info() != Eigen::Success) {
            return false;
        }
        const double lmax = std::max(1e-18, eig.eigenvalues().maxCoeff());
        const double step = 1.0 / lmax;

        // Start from clipped unconstrained Newton step if possible, otherwise zero.
        Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
        if (ldlt.info() == Eigen::Success) {
            u = clamp_vec(ldlt.solve(-h), lo, hi);
        } else {
            u = Eigen::VectorXd::Zero(M);
            u = clamp_vec(u, lo, hi);
        }
        if (!u.allFinite()) {
            u = Eigen::VectorXd::Zero(M);
            u = clamp_vec(u, lo, hi);
        }

        Eigen::VectorXd y = u;
        double t          = 1.0;
        double last_obj   = std::numeric_limits<double>::infinity();

        for (iterations = 1; iterations <= max_iter_; ++iterations) {
            const Eigen::VectorXd grad_y = H * y + h;
            Eigen::VectorXd u_next       = clamp_vec(y - step * grad_y, lo, hi);

            const Eigen::VectorXd grad_next = H * u_next + h;
            kkt              = projected_gradient_kkt_residual(u_next, grad_next, lo, hi);
            const double obj = 0.5 * u_next.dot(H * u_next) + h.dot(u_next);

            if (kkt <= abs_tol_) {
                u = u_next;
                return true;
            }

            // Monotone FISTA restart. If acceleration overshoots, fall back to plain PG.
            if (obj > last_obj) {
                y = u;
                t = 1.0;
                continue;
            }

            const double t_next = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * t * t));
            y                   = u_next + ((t - 1.0) / t_next) * (u_next - u);
            y                   = clamp_vec(y, lo, hi);
            u                   = u_next;
            t                   = t_next;
            last_obj            = obj;
        }

        const Eigen::VectorXd grad = H * u + h;
        kkt                        = projected_gradient_kkt_residual(u, grad, lo, hi);
        return kkt <= std::max(abs_tol_, 1e-7);
    }

    [[nodiscard]] bool solve_axis(int axis) noexcept {
        Eigen::MatrixXd H;
        Eigen::VectorXd h;
        Eigen::VectorXd base_pos;
        Eigen::VectorXd base_vel;
        Eigen::MatrixXd Gp;
        Eigen::MatrixXd Gv;
        build_condensed_problem(axis, H, h, base_pos, base_vel, Gp, Gv);

        Eigen::VectorXd u;
        int iterations = 0;
        double kkt     = std::numeric_limits<double>::infinity();
        const bool ok =
            solve_box_qp_projected_gradient(H, h, u_min_(axis), u_max_(axis), u, iterations, kkt);

        axis_iters_[axis]    = iterations;
        kkt_violation_(axis) = kkt;
        if (!ok || u.size() != num_inputs() || !u.allFinite()) {
            return false;
        }

        const Eigen::VectorXd pos = base_pos + Gp * u;
        const Eigen::VectorXd vel = base_vel + Gv * u;
        for (int k = 0; k < N_; ++k) {
            sol_pos_(axis, k) = pos(k);
            sol_vel_(axis, k) = vel(k);
        }
        for (int k = 0; k < num_inputs(); ++k) {
            sol_acc_(axis, k) = clamp_scalar(u(k), u_min_(axis), u_max_(axis));
        }

        compute_diagnostics(axis);
        return true;
    }

    void compute_diagnostics(int axis) noexcept {
        double obj = 0.0;
        for (int k = 0; k < N_; ++k) {
            const double ep = sol_pos_(axis, k) - ref_pos_(axis, k);
            const double ev = sol_vel_(axis, k) - ref_vel_(axis, k);
            obj += 0.5 * q_pos_(axis) * ep * ep;
            obj += 0.5 * q_vel_(axis) * ev * ev;
        }
        for (int k = 0; k < num_inputs(); ++k) {
            const double u = sol_acc_(axis, k);
            obj += 0.5 * r_val_(axis) * u * u;
        }
        objective_(axis) = obj;

        double dyn_max = 0.0;
        for (int k = 0; k < num_inputs(); ++k) {
            const double pred_pos = a00_ * sol_pos_(axis, k) + a01_ * sol_vel_(axis, k)
                                  + b0_ * sol_acc_(axis, k) + f_pos_;
            const double pred_vel = a10_ * sol_pos_(axis, k) + a11_ * sol_vel_(axis, k)
                                  + b1_ * sol_acc_(axis, k) + f_vel_;
            dyn_max = std::max(dyn_max, std::abs(sol_pos_(axis, k + 1) - pred_pos));
            dyn_max = std::max(dyn_max, std::abs(sol_vel_(axis, k + 1) - pred_vel));
        }
        dyn_violation_(axis) = dyn_max;

        double input_vio = 0.0;
        for (int k = 0; k < num_inputs(); ++k) {
            input_vio = std::max(input_vio, u_min_(axis) - sol_acc_(axis, k));
            input_vio = std::max(input_vio, sol_acc_(axis, k) - u_max_(axis));
        }
        input_violation_(axis) = std::max(0.0, input_vio);
    }
};

// Optional benchmark-side alias. Do not enable this in production next to DualSmallMpcSolver.
// using DualSmallMpcSolver = DualMpcOsqpSolver;

} // namespace fcs::L5
