// Rigorous dual-axis MPC benchmark: 2x TinyMPC (double/dynamic) vs
// DualSmallMpcSolver (float/fixed/batched).
//
// Correctness target:
//   Only compare the six center outputs actually consumed by production:
//
//     yaw
//     pitch
//     yaw_vel
//     pitch_vel
//     yaw_accel
//     pitch_accel
//
// Build: cmake --build build --target bench_mpc
// Run:   ./build/bin/bench_mpc

#include "dual_small_mpc_solver.hpp"
#include "tiny_api.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace fcs::L5;

struct TinySolverDeleter {
    void operator()(TinySolver* s) const noexcept {
        if (!s)
            return;
        auto d = [](auto* p) { delete p; };
        d(s->work);
        d(s->settings);
        d(s->cache);
        d(s->solution);
        d(s);
    }
};
using TinySolverPtr = std::unique_ptr<TinySolver, TinySolverDeleter>;

[[maybe_unused]] static inline void compiler_barrier() noexcept { asm volatile("" ::: "memory"); }

template <typename T>
[[maybe_unused]] static inline void do_not_optimize(const T& val) noexcept {
    asm volatile("" : : "r"(&val) : "memory");
}

using Clock     = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

[[nodiscard]] static inline double elapsed_us(TimePoint a, TimePoint b) noexcept {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

static constexpr int kHorizonAhead = 50;
static constexpr int kHorizonBack  = 50;
static constexpr int kHorizon      = kHorizonAhead + kHorizonBack + 1;
static constexpr double kDt        = 0.01;
static constexpr double kRho       = 1.0;
static constexpr double kAbsTol    = 1e-3;
static constexpr int kWarmup       = 200;
static constexpr int kTrials       = 1000;

struct AxisParams {
    double q_pos, q_vel, r, max_acc;
};

static constexpr AxisParams kYaw   = {9e6, 0.0, 1.0, 50.0};
static constexpr AxisParams kPitch = {9e6, 0.0, 1.0, 100.0};

using RefTrajectory = Eigen::MatrixXd;

[[nodiscard]] static RefTrajectory ref_sinusoid(int N, double freq, double amp) {
    Eigen::MatrixXd ref(2, N);
    const double omega = 2.0 * std::numbers::pi_v<double> * freq;
    for (int i = 0; i < N; ++i) {
        const double t = static_cast<double>(i - kHorizonBack) * kDt;
        ref(0, i)      = amp * std::sin(omega * t);
        ref(1, i)      = amp * omega * std::cos(omega * t);
    }
    return ref;
}

// Clean chirp: position and velocity are derived from the same shifted phase.
// This avoids testing against an internally inconsistent reference.
[[nodiscard]] static RefTrajectory ref_chirp(int N, double amp) {
    Eigen::MatrixXd ref(2, N);
    const double f0 = 0.5;
    const double f1 = 3.0;
    const double T  = static_cast<double>(std::max(N - 1, 1)) * kDt;

    for (int i = 0; i < N; ++i) {
        const double tau  = static_cast<double>(i) * kDt;
        const double freq = f0 + (f1 - f0) * tau / T;
        const double phase =
            2.0 * std::numbers::pi_v<double> * (f0 * tau + 0.5 * (f1 - f0) / T * tau * tau);

        ref(0, i) = amp * std::sin(phase);
        ref(1, i) = amp * std::cos(phase) * 2.0 * std::numbers::pi_v<double> * freq;
    }

    return ref;
}

[[nodiscard]] static RefTrajectory ref_noisy_sinusoid(int N, unsigned seed, double amp) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, 0.01f);
    auto ref = ref_sinusoid(N, 1.0, amp);
    for (int i = 0; i < N; ++i) {
        ref(0, i) += static_cast<double>(noise(rng));
        ref(1, i) += static_cast<double>(noise(rng)) * 10.0;
    }
    return ref;
}

[[nodiscard]] static RefTrajectory ref_zero(int N) { return Eigen::MatrixXd::Zero(2, N); }

[[nodiscard]] static RefTrajectory ref_step(int N, double pos, double vel = 0.0) {
    Eigen::MatrixXd ref(2, N);
    ref.row(0).setConstant(pos);
    ref.row(1).setConstant(vel);
    return ref;
}

[[nodiscard]] static RefTrajectory ref_ramp(int N, double start, double slope) {
    Eigen::MatrixXd ref(2, N);
    for (int i = 0; i < N; ++i) {
        const double t = static_cast<double>(i - kHorizonBack) * kDt;
        ref(0, i)      = start + slope * t;
        ref(1, i)      = slope;
    }
    return ref;
}

struct TinyPairSetup {
    TinySolverPtr yaw;
    TinySolverPtr pitch;

    TinyPairSetup() {
        yaw   = make(false);
        pitch = make(true);
    }

private:
    [[nodiscard]] static TinySolverPtr make(bool is_pitch) {
        const AxisParams& p     = is_pitch ? kPitch : kYaw;
        const Eigen::Matrix2d A = (Eigen::Matrix2d{} << 1.0, kDt, 0.0, 1.0).finished();
        const Eigen::Matrix<double, 2, 1> B =
            (Eigen::Matrix<double, 2, 1>{} << 0.0, kDt).finished();
        const Eigen::Vector2d f             = Eigen::Vector2d::Zero();
        const Eigen::Matrix2d Q             = Eigen::Vector2d{p.q_pos, p.q_vel}.asDiagonal();
        const Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Constant(p.r);

        TinySolver* raw = nullptr;
        tiny_setup(&raw, A, B, f, Q, R, kRho, 2, 1, kHorizon, 0);
        TinySolverPtr solver(raw);

        auto* s                        = solver.get();
        s->settings->abs_pri_tol       = kAbsTol;
        s->settings->abs_dua_tol       = kAbsTol;
        s->settings->check_termination = 1;
        s->settings->en_input_bound    = 1;
        s->settings->en_state_bound    = is_pitch ? 1 : 0;

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);

        if (is_pitch) {
            x_min.row(0).setConstant(-0.698);
            x_max.row(0).setConstant(0.698);
        }

        tiny_set_bound_constraints(
            s, x_min, x_max, Eigen::MatrixXd::Constant(1, kHorizon - 1, -p.max_acc),
            Eigen::MatrixXd::Constant(1, kHorizon - 1, p.max_acc));

        return solver;
    }
};

struct DualSetup {
    DualSmallMpcSolver solver;

    DualSetup() {
        DualSmallMpcSolver::AxisConfig yaw_cfg;
        yaw_cfg.q_pos   = static_cast<float>(kYaw.q_pos);
        yaw_cfg.q_vel   = static_cast<float>(kYaw.q_vel);
        yaw_cfg.r       = static_cast<float>(kYaw.r);
        yaw_cfg.max_acc = static_cast<float>(kYaw.max_acc);

        DualSmallMpcSolver::AxisConfig pitch_cfg;
        pitch_cfg.q_pos              = static_cast<float>(kPitch.q_pos);
        pitch_cfg.q_vel              = static_cast<float>(kPitch.q_vel);
        pitch_cfg.r                  = static_cast<float>(kPitch.r);
        pitch_cfg.max_acc            = static_cast<float>(kPitch.max_acc);
        pitch_cfg.enable_state_bound = true;
        pitch_cfg.state_min          = -0.698f;
        pitch_cfg.state_max          = 0.698f;

        auto result = DualSmallMpcSolver::create(
            static_cast<float>(kDt), kHorizon, static_cast<float>(kRho), yaw_cfg, pitch_cfg);
        assert(result.has_value());

        solver = std::move(*result);
        solver.set_settings(static_cast<float>(kAbsTol), 1000);
    }
};

struct Stats {
    double min, p50, p95, max;
    double mean, stddev;
    int converged_count;
};

[[nodiscard]] static Stats compute_stats(std::vector<double> samples, int converged) {
    assert(!samples.empty());

    std::sort(samples.begin(), samples.end());

    double sum = 0.0;
    for (double s : samples)
        sum += s;

    const double mean = sum / static_cast<double>(samples.size());

    double sq_sum = 0.0;
    for (double s : samples) {
        const double d = s - mean;
        sq_sum += d * d;
    }

    const size_t p95_index = std::min(
        samples.size() - 1, static_cast<size_t>(static_cast<double>(samples.size()) * 0.95));

    return {
        samples.front(),
        samples[samples.size() / 2],
        samples[p95_index],
        samples.back(),
        mean,
        std::sqrt(sq_sum / static_cast<double>(samples.size())),
        converged,
    };
}

static void print_stats(const char* label, const Stats& s, int total) {
    std::cout << "  " << std::left << std::setw(24) << label << " ";
    std::cout << std::fixed << std::setprecision(2) << "p50=" << std::setw(8) << s.p50 << "us"
              << " mean=" << std::setw(8) << s.mean << "us"
              << " p95=" << std::setw(8) << s.p95 << "us"
              << " min=" << std::setw(8) << s.min << "us"
              << " max=" << std::setw(8) << s.max << "us"
              << " conv=" << s.converged_count << "/" << total << "\n";
}

struct TimingResult {
    Stats stats;
    int iterations;
};

[[nodiscard]] static TimingResult bench_tiny_pair_solve(
    TinyPairSetup& setup, const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref,
    int max_iter, int trials) {
    auto* yaw                 = setup.yaw.get();
    auto* pitch               = setup.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;

    for (int w = 0; w < kWarmup; ++w) {
        tiny_set_x0(yaw, yaw_ref.col(0));
        tiny_set_x0(pitch, pitch_ref.col(0));
        yaw->work->Xref   = yaw_ref;
        pitch->work->Xref = pitch_ref;
        tiny_solve(yaw);
        tiny_solve(pitch);
    }

    std::vector<double> times;
    times.reserve(trials);

    int converged      = 0;
    int max_iterations = 0;

    for (int i = 0; i < trials; ++i) {
        tiny_set_x0(yaw, yaw_ref.col(0));
        tiny_set_x0(pitch, pitch_ref.col(0));
        yaw->work->Xref   = yaw_ref;
        pitch->work->Xref = pitch_ref;

        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        const int yaw_status   = tiny_solve(yaw);
        const int pitch_status = tiny_solve(pitch);

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        do_not_optimize(yaw_status);
        do_not_optimize(pitch_status);

        if (yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
            && pitch->solution->solved == 1) {
            ++converged;
        }

        max_iterations =
            std::max(max_iterations, std::max(yaw->solution->iter, pitch->solution->iter));
        times.push_back(elapsed_us(t0, t1));
    }

    return {compute_stats(std::move(times), converged), max_iterations};
}

[[nodiscard]] static TimingResult bench_dual_solve(
    DualSetup& setup, const Eigen::MatrixXd& yaw_ref_d, const Eigen::MatrixXd& pitch_ref_d,
    int max_iter, int trials) {
    setup.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    const Eigen::MatrixXf yaw_ref   = yaw_ref_d.cast<float>();
    const Eigen::MatrixXf pitch_ref = pitch_ref_d.cast<float>();

    for (int w = 0; w < kWarmup; ++w) {
        setup.solver.set_x0(yaw_ref_d(0, 0), yaw_ref_d(1, 0), pitch_ref_d(0, 0), pitch_ref_d(1, 0));
        setup.solver.set_reference(yaw_ref, pitch_ref);
        setup.solver.solve();
    }

    std::vector<double> times;
    times.reserve(trials);

    int converged      = 0;
    int max_iterations = 0;

    for (int i = 0; i < trials; ++i) {
        setup.solver.set_x0(yaw_ref_d(0, 0), yaw_ref_d(1, 0), pitch_ref_d(0, 0), pitch_ref_d(1, 0));
        setup.solver.set_reference(yaw_ref, pitch_ref);

        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        const bool ok = setup.solver.solve();

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        do_not_optimize(ok);

        if (ok)
            ++converged;

        max_iterations = std::max(max_iterations, setup.solver.iterations());
        times.push_back(elapsed_us(t0, t1));
    }

    return {compute_stats(std::move(times), converged), max_iterations};
}

[[nodiscard]] static Stats bench_tiny_pair_pipeline(
    TinyPairSetup& setup, const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref,
    int max_iter, int trials) {
    auto* yaw                 = setup.yaw.get();
    auto* pitch               = setup.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;

    std::vector<double> times;
    times.reserve(trials);

    int converged = 0;

    for (int i = 0; i < trials; ++i) {
        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        tiny_set_x0(yaw, yaw_ref.col(0));
        tiny_set_x0(pitch, pitch_ref.col(0));
        yaw->work->Xref        = yaw_ref;
        pitch->work->Xref      = pitch_ref;
        const int yaw_status   = tiny_solve(yaw);
        const int pitch_status = tiny_solve(pitch);

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        if (yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
            && pitch->solution->solved == 1) {
            ++converged;
        }

        times.push_back(elapsed_us(t0, t1));
    }

    return compute_stats(std::move(times), converged);
}

[[nodiscard]] static Stats bench_dual_pipeline(
    DualSetup& setup, const Eigen::MatrixXd& yaw_ref_d, const Eigen::MatrixXd& pitch_ref_d,
    int max_iter, int trials) {
    setup.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    const Eigen::MatrixXf yaw_ref   = yaw_ref_d.cast<float>();
    const Eigen::MatrixXf pitch_ref = pitch_ref_d.cast<float>();

    std::vector<double> times;
    times.reserve(trials);

    int converged = 0;

    for (int i = 0; i < trials; ++i) {
        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        setup.solver.set_x0(yaw_ref_d(0, 0), yaw_ref_d(1, 0), pitch_ref_d(0, 0), pitch_ref_d(1, 0));
        setup.solver.set_reference(yaw_ref, pitch_ref);
        const bool ok = setup.solver.solve();

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        if (ok)
            ++converged;

        times.push_back(elapsed_us(t0, t1));
    }

    return compute_stats(std::move(times), converged);
}

// ================================================================================================
// Center output consistency: this is the correctness metric that matches production usage.
// ================================================================================================

struct CenterOutput {
    double yaw;
    double pitch;
    double yaw_vel;
    double pitch_vel;
    double yaw_accel;
    double pitch_accel;
};

struct CenterOutputReport {
    CenterOutput tiny;
    CenterOutput dual;
    CenterOutput abs_err;
    CenterOutput norm_err;

    double max_abs_err;
    double max_norm_err;

    int center_index;
    int control_index;

    bool tiny_converged;
    bool dual_converged;
    bool pass;
};

[[nodiscard]] static double max_center_output_component(const CenterOutput& v) {
    return std::max({
        v.yaw,
        v.pitch,
        v.yaw_vel,
        v.pitch_vel,
        v.yaw_accel,
        v.pitch_accel,
    });
}

[[nodiscard]] static CenterOutput abs_diff(const CenterOutput& a, const CenterOutput& b) {
    return {
        std::abs(a.yaw - b.yaw),
        std::abs(a.pitch - b.pitch),
        std::abs(a.yaw_vel - b.yaw_vel),
        std::abs(a.pitch_vel - b.pitch_vel),
        std::abs(a.yaw_accel - b.yaw_accel),
        std::abs(a.pitch_accel - b.pitch_accel),
    };
}

[[nodiscard]] static CenterOutput normalized_error(const CenterOutput& e) {
    // These are engineering scales, not solver scales.
    // Tune them if your actuator/sensor output resolution is tighter or looser.
    constexpr double kYawScale        = 1.0;
    constexpr double kPitchScale      = 1.0;
    constexpr double kYawVelScale     = 10.0;
    constexpr double kPitchVelScale   = 10.0;
    constexpr double kYawAccelScale   = kYaw.max_acc;
    constexpr double kPitchAccelScale = kPitch.max_acc;

    return {
        e.yaw / kYawScale,
        e.pitch / kPitchScale,
        e.yaw_vel / kYawVelScale,
        e.pitch_vel / kPitchVelScale,
        e.yaw_accel / kYawAccelScale,
        e.pitch_accel / kPitchAccelScale,
    };
}

[[nodiscard]] static CenterOutputReport compare_center_output(
    const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref, int max_iter) {
    TinyPairSetup tiny;
    DualSetup dual;

    auto* yaw                 = tiny.yaw.get();
    auto* pitch               = tiny.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;
    dual.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    tiny_set_x0(yaw, yaw_ref.col(0));
    tiny_set_x0(pitch, pitch_ref.col(0));
    yaw->work->Xref        = yaw_ref;
    pitch->work->Xref      = pitch_ref;
    const int yaw_status   = tiny_solve(yaw);
    const int pitch_status = tiny_solve(pitch);

    dual.solver.set_x0(yaw_ref(0, 0), yaw_ref(1, 0), pitch_ref(0, 0), pitch_ref(1, 0));
    dual.solver.set_reference(yaw_ref.cast<float>(), pitch_ref.cast<float>());
    const bool dual_ok = dual.solver.solve();

    // Mirror production optimize() semantics:
    //   center_index  = clamp(horizon_back, 0, effective_horizon - 1)
    //   control_index = clamp(center_index, 0, effective_horizon - 2)
    //
    // In this benchmark, effective_horizon == kHorizon.
    const int effective_horizon = kHorizon;
    const int center_index      = std::clamp(kHorizonBack, 0, effective_horizon - 1);
    const int control_index     = std::clamp(center_index, 0, effective_horizon - 2);

    const CenterOutput tiny_out{
        yaw->solution->x(0, center_index),  pitch->solution->x(0, center_index),
        yaw->solution->x(1, center_index),  pitch->solution->x(1, center_index),
        yaw->solution->u(0, control_index), pitch->solution->u(0, control_index),
    };

    const CenterOutput dual_out{
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 0, center_index)),
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 0, center_index)),
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 1, center_index)),
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 1, center_index)),
        static_cast<double>(dual.solver.input(DualSmallMpcSolver::kYawAxis, control_index)),
        static_cast<double>(dual.solver.input(DualSmallMpcSolver::kPitchAxis, control_index)),
    };

    const CenterOutput err      = abs_diff(tiny_out, dual_out);
    const CenterOutput norm_err = normalized_error(err);

    const double max_abs  = max_center_output_component(err);
    const double max_norm = max_center_output_component(norm_err);

    const bool tiny_ok = yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
                      && pitch->solution->solved == 1;

    // Suggested correctness tolerances.
    //
    // These are intentionally focused on the exported command values, not the full horizon.
    // Tune accel tolerance to actuator command resolution.
    constexpr double kPosTol   = 1e-3;
    constexpr double kVelTol   = 1e-2;
    constexpr double kAccelTol = 1e-1;
    constexpr double kNormTol  = 1e-3;

    const bool pass = tiny_ok && dual_ok && err.yaw <= kPosTol && err.pitch <= kPosTol
                   && err.yaw_vel <= kVelTol && err.pitch_vel <= kVelTol
                   && err.yaw_accel <= kAccelTol && err.pitch_accel <= kAccelTol
                   && max_norm <= kNormTol;

    return {
        tiny_out,     dual_out,      err,     norm_err, max_abs, max_norm,
        center_index, control_index, tiny_ok, dual_ok,  pass,
    };
}

static void print_center_output_summary_header() {
    std::cout << "  " << std::left << std::setw(16) << "Scenario" << std::setw(12) << "yaw"
              << std::setw(12) << "pitch" << std::setw(12) << "yaw_vel" << std::setw(12)
              << "pit_vel" << std::setw(12) << "yaw_acc" << std::setw(12) << "pit_acc"
              << std::setw(12) << "max_norm" << std::setw(8) << "Pass"
              << "\n";
    std::cout << "  " << std::string(108, '-') << "\n";
}

static void print_center_output_summary_row(const char* name, const CenterOutputReport& r) {
    std::cout << "  " << std::left << std::setw(16) << name << std::scientific
              << std::setprecision(2) << std::setw(12) << r.abs_err.yaw << std::setw(12)
              << r.abs_err.pitch << std::setw(12) << r.abs_err.yaw_vel << std::setw(12)
              << r.abs_err.pitch_vel << std::setw(12) << r.abs_err.yaw_accel << std::setw(12)
              << r.abs_err.pitch_accel << std::setw(12) << r.max_norm_err << std::setw(8)
              << (r.pass ? "Y" : "N") << "\n";
}

static void print_center_output_report(const char* label, const CenterOutputReport& r) {
    std::cout << "  " << label << "\n";
    std::cout << "  center_index=" << r.center_index << " control_index=" << r.control_index
              << " tiny_ok=" << (r.tiny_converged ? "Y" : "N")
              << " dual_ok=" << (r.dual_converged ? "Y" : "N") << " pass=" << (r.pass ? "Y" : "N")
              << "\n";

    auto print_row = [](const char* name, double tiny, double dual, double err, double norm) {
        std::cout << "    " << std::left << std::setw(12) << name << " tiny=" << std::scientific
                  << std::setprecision(6) << std::setw(14) << tiny << " dual=" << std::setw(14)
                  << dual << " abs_err=" << std::setw(14) << err << " norm_err=" << norm << "\n";
    };

    print_row("yaw", r.tiny.yaw, r.dual.yaw, r.abs_err.yaw, r.norm_err.yaw);
    print_row("pitch", r.tiny.pitch, r.dual.pitch, r.abs_err.pitch, r.norm_err.pitch);
    print_row("yaw_vel", r.tiny.yaw_vel, r.dual.yaw_vel, r.abs_err.yaw_vel, r.norm_err.yaw_vel);
    print_row(
        "pitch_vel", r.tiny.pitch_vel, r.dual.pitch_vel, r.abs_err.pitch_vel, r.norm_err.pitch_vel);
    print_row(
        "yaw_accel", r.tiny.yaw_accel, r.dual.yaw_accel, r.abs_err.yaw_accel, r.norm_err.yaw_accel);
    print_row(
        "pitch_accel", r.tiny.pitch_accel, r.dual.pitch_accel, r.abs_err.pitch_accel,
        r.norm_err.pitch_accel);

    std::cout << "    max_abs_err=" << std::scientific << std::setprecision(6) << r.max_abs_err
              << " max_norm_err=" << r.max_norm_err << "\n";
}

// ================================================================================================
// Full horizon debug accuracy. This is intentionally not the main correctness metric.
// ================================================================================================

struct FullHorizonDebugReport {
    double max_abs_state_err;
    double rmse_state;
    double max_abs_input_err;
    bool tiny_converged;
    bool dual_converged;
};

[[nodiscard]] static FullHorizonDebugReport compare_full_horizon_debug(
    const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref, int max_iter) {
    TinyPairSetup tiny;
    DualSetup dual;

    auto* yaw                 = tiny.yaw.get();
    auto* pitch               = tiny.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;
    dual.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    tiny_set_x0(yaw, yaw_ref.col(0));
    tiny_set_x0(pitch, pitch_ref.col(0));
    yaw->work->Xref        = yaw_ref;
    pitch->work->Xref      = pitch_ref;
    const int yaw_status   = tiny_solve(yaw);
    const int pitch_status = tiny_solve(pitch);

    dual.solver.set_x0(yaw_ref(0, 0), yaw_ref(1, 0), pitch_ref(0, 0), pitch_ref(1, 0));
    dual.solver.set_reference(yaw_ref.cast<float>(), pitch_ref.cast<float>());
    const bool dual_ok = dual.solver.solve();

    double max_state = 0.0;
    double sum_sq    = 0.0;
    double max_input = 0.0;

    for (int k = 0; k < kHorizon; ++k) {
        const double yaw_pos_err = std::abs(
            yaw->solution->x(0, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 0, k)));
        const double yaw_vel_err = std::abs(
            yaw->solution->x(1, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 1, k)));
        const double pitch_pos_err = std::abs(
            pitch->solution->x(0, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 0, k)));
        const double pitch_vel_err = std::abs(
            pitch->solution->x(1, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 1, k)));

        max_state = std::max({max_state, yaw_pos_err, yaw_vel_err, pitch_pos_err, pitch_vel_err});
        sum_sq += yaw_pos_err * yaw_pos_err + yaw_vel_err * yaw_vel_err;
        sum_sq += pitch_pos_err * pitch_pos_err + pitch_vel_err * pitch_vel_err;
    }

    for (int k = 0; k < kHorizon - 1; ++k) {
        const double yaw_input_err = std::abs(
            yaw->solution->u(0, k)
            - static_cast<double>(dual.solver.input(DualSmallMpcSolver::kYawAxis, k)));
        const double pitch_input_err = std::abs(
            pitch->solution->u(0, k)
            - static_cast<double>(dual.solver.input(DualSmallMpcSolver::kPitchAxis, k)));

        max_input = std::max({max_input, yaw_input_err, pitch_input_err});
    }

    return {
        max_state,
        std::sqrt(sum_sq / static_cast<double>(4 * kHorizon)),
        max_input,
        yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
            && pitch->solution->solved == 1,
        dual_ok,
    };
}

static void print_section(const char* title) {
    std::cout << "\n" << std::string(78, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(78, '=') << "\n";
}

int main() {
    std::cout << std::unitbuf;

    const auto yaw_ref_sin     = ref_sinusoid(kHorizon, 1.0, 0.3);
    const auto pitch_ref_sin   = ref_sinusoid(kHorizon, 0.8, 0.2);
    const auto yaw_ref_chirp   = ref_chirp(kHorizon, 0.3);
    const auto pitch_ref_noisy = ref_noisy_sinusoid(kHorizon, 42, 0.2);
    const auto yaw_ref_zero    = ref_zero(kHorizon);
    const auto pitch_ref_zero  = ref_zero(kHorizon);
    const auto yaw_ref_step    = ref_step(kHorizon, 0.3);
    const auto pitch_ref_step  = ref_step(kHorizon, 0.6);
    const auto yaw_ref_ramp    = ref_ramp(kHorizon, 0.0, 0.8);
    const auto pitch_ref_bound = ref_step(kHorizon, 0.68);

    print_section("Dual-Axis MPC Benchmark");
    std::cout << "  Horizon: " << kHorizon << " (ahead=" << kHorizonAhead
              << " back=" << kHorizonBack << ")\n";
    std::cout << "  dt=" << kDt << " rho=" << kRho << " tol=" << kAbsTol << "\n";
    std::cout << "  Trials=" << kTrials << " Warmup=" << kWarmup << "\n";

    {
        print_section("Solve-Only Timing");

        TinyPairSetup tiny;
        DualSetup dual;

        const auto tiny_t = bench_tiny_pair_solve(tiny, yaw_ref_sin, pitch_ref_sin, 10, kTrials);
        const auto dual_t = bench_dual_solve(dual, yaw_ref_sin, pitch_ref_sin, 10, kTrials);

        print_stats("2x TinyMPC", tiny_t.stats, kTrials);
        print_stats("DualSmallMpcSolver", dual_t.stats, kTrials);

        std::cout << "  tiny_max_iter_seen=" << tiny_t.iterations
                  << " dual_max_iter_seen=" << dual_t.iterations << "\n";

        std::cout << "  speedup=" << std::fixed << std::setprecision(2)
                  << tiny_t.stats.p50 / std::max(dual_t.stats.p50, 0.001) << "x\n";
    }

    {
        print_section("Full Pipeline Timing");

        TinyPairSetup tiny;
        DualSetup dual;

        const auto tiny_t = bench_tiny_pair_pipeline(tiny, yaw_ref_sin, pitch_ref_sin, 10, kTrials);
        const auto dual_t = bench_dual_pipeline(dual, yaw_ref_sin, pitch_ref_sin, 10, kTrials);

        print_stats("2x TinyMPC", tiny_t, kTrials);
        print_stats("DualSmallMpcSolver", dual_t, kTrials);

        std::cout << "  speedup=" << std::fixed << std::setprecision(2)
                  << tiny_t.p50 / std::max(dual_t.p50, 0.001) << "x\n";
    }

    struct Scenario {
        const char* name;
        const Eigen::MatrixXd& yaw_ref;
        const Eigen::MatrixXd& pitch_ref;
    };

    const Scenario scenarios[] = {
        {    "sin/sin",   yaw_ref_sin,   pitch_ref_sin},
        {"chirp/noisy", yaw_ref_chirp, pitch_ref_noisy},
        {  "zero/zero",  yaw_ref_zero,  pitch_ref_zero},
        { "step/bound",  yaw_ref_step, pitch_ref_bound},
        {  "ramp/step",  yaw_ref_ramp,  pitch_ref_step},
    };

    {
        print_section("Scenario Timing Sweep");

        std::cout << "  " << std::left << std::setw(16) << "Scenario" << std::setw(14) << "Tiny p50"
                  << std::setw(14) << "Dual p50" << std::setw(10) << "Speedup"
                  << "\n";
        std::cout << "  " << std::string(54, '-') << "\n";

        for (const auto& scenario : scenarios) {
            TinyPairSetup tiny;
            DualSetup dual;

            const auto tiny_t =
                bench_tiny_pair_solve(tiny, scenario.yaw_ref, scenario.pitch_ref, 10, kTrials);
            const auto dual_t =
                bench_dual_solve(dual, scenario.yaw_ref, scenario.pitch_ref, 10, kTrials);

            std::cout << "  " << std::left << std::setw(16) << scenario.name << std::fixed
                      << std::setprecision(2) << std::setw(14) << tiny_t.stats.p50 << std::setw(14)
                      << dual_t.stats.p50 << std::setw(10)
                      << tiny_t.stats.p50 / std::max(dual_t.stats.p50, 0.001) << "\n";
        }
    }

    {
        print_section("Center Output Consistency");

        bool all_pass = true;

        print_center_output_summary_header();

        for (const auto& scenario : scenarios) {
            const auto report = compare_center_output(scenario.yaw_ref, scenario.pitch_ref, 100);

            print_center_output_summary_row(scenario.name, report);
            all_pass = all_pass && report.pass;
        }

        std::cout << "  overall_center_output_consistency=" << (all_pass ? "PASS" : "FAIL") << "\n";

        std::cout << "\n";
        const auto detailed = compare_center_output(yaw_ref_sin, pitch_ref_noisy, 100);
        print_center_output_report("detailed sin/noisy", detailed);
    }

    {
        print_section("Full Horizon Debug Accuracy");

        const auto debug = compare_full_horizon_debug(yaw_ref_sin, pitch_ref_noisy, 100);

        std::cout << "  tiny_ok=" << (debug.tiny_converged ? "Y" : "N")
                  << " dual_ok=" << (debug.dual_converged ? "Y" : "N") << "\n";
        std::cout << "  max_state_err=" << std::scientific << std::setprecision(4)
                  << debug.max_abs_state_err << " rmse_state=" << debug.rmse_state
                  << " max_input_err=" << debug.max_abs_input_err << "\n";

        std::cout << "  note: full horizon errors are debug-only. "
                  << "The pass/fail correctness metric is Center Output Consistency.\n";
    }

    return 0;
}
