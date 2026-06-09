#pragma once

#include "core/trajectory/model/ballistic_model.hpp"
#include "core/trajectory/solver/solver_interfaces.hpp"

#include <Eigen/Core>

#include <memory>
#include <numbers>
#include <utility>
#include <vector>

namespace fcs::core::trajectory::solver {

// ============================================================================
// Direct Solver (Analytical Solution for Ideal/LinearDrag)
// ============================================================================

/// Direct solver using analytical solutions.
///
/// This solver provides analytical (closed-form) solutions for ballistic models
/// that support direct computation. Works with:
/// - IdealModel: Vacuum ballistic trajectory
/// - LinearDragModel: Linear air resistance
///
/// This is the preferred solver for models with analytical solutions as it's
/// faster and more accurate than iterative methods.
class DirectSolver final : public TrajectorySolver {
public:
    /// Construct DirectSolver with any BallisticModel
    template <typename Model>
    requires(std::derived_from<Model, model::BallisticModel>)
    explicit DirectSolver(std::unique_ptr<Model>&& model) noexcept
        : model_(std::move(model)) {}

    /// Factory function for unique_ptr construction
    [[nodiscard]] static std::unique_ptr<DirectSolver>
        create(std::unique_ptr<model::BallisticModel> model) noexcept {
        return std::make_unique<DirectSolver>(std::move(model));
    }

    /// Solve for firing angles to hit target
    [[nodiscard]] std::expected<AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept override;

    /// Generate trajectory points for visualization
    [[nodiscard]] std::vector<std::pair<double, double>>
        generate_trajectory(double pitch, double v0, double max_distance) const noexcept override;

    /// Get solver name for debugging
    [[nodiscard]] std::string_view solver_name() const noexcept override { return "Direct"; }

    /// Access the underlying ballistic model
    [[nodiscard]] const std::unique_ptr<model::BallisticModel>& model() const noexcept {
        return model_;
    }

    /// Get model pointer (virtual override for RTTI-free type-safe dispatch)
    [[nodiscard]] const model::BallisticModel* get_model() const noexcept override {
        return model_.get();
    }

private:
    std::unique_ptr<model::BallisticModel> model_;
};

// ============================================================================
// Iterative Solver (Numerical Solution for QuadraticDrag)
// ============================================================================

/// Iterative solver using successive approximation.
///
/// This solver works with any BallisticModel implementation. It uses
/// a simple iterative method: start with line-of-sight angle, compute
/// impact height error, adjust target height, repeat until convergence.
///
/// This is deliberately stateless - all configuration is passed via
/// the Config struct and stored in the unique_ptr<BallisticModel>.
class IterativeSolver final : public TrajectorySolver {
public:
    struct Config {
        uint32_t max_iterations{20};              ///< Maximum solving iterations
        double height_tolerance{0.01};            ///< Convergence threshold (meters)
        double max_pitch{std::numbers::pi / 2.5}; ///< Maximum valid pitch (radians)
    };

    /// Construct IterativeSolver with any BallisticModel
    template <typename Model>
    requires(std::derived_from<Model, model::BallisticModel>)
    explicit IterativeSolver(std::unique_ptr<Model>&& model, Config config) noexcept
        : model_(std::move(model))
        , config_(config) {}

    /// Factory function for default config
    [[nodiscard]] static std::unique_ptr<IterativeSolver>
        create(std::unique_ptr<model::BallisticModel> model) noexcept {
        return std::make_unique<IterativeSolver>(std::move(model), Config{});
    }

    /// Solve for firing angles to hit target
    [[nodiscard]] std::expected<AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept override;

    /// Generate trajectory points for visualization
    [[nodiscard]] std::vector<std::pair<double, double>>
        generate_trajectory(double pitch, double v0, double max_distance) const noexcept override;

    /// Get solver name for debugging
    [[nodiscard]] std::string_view solver_name() const noexcept override { return "Iterative"; }

    /// Access the underlying ballistic model
    [[nodiscard]] const std::unique_ptr<model::BallisticModel>& model() const noexcept {
        return model_;
    }

    /// Access solver configuration
    [[nodiscard]] const Config& config() const noexcept { return config_; }

    /// Get model pointer (virtual override for RTTI-free type-safe dispatch)
    [[nodiscard]] const model::BallisticModel* get_model() const noexcept override {
        return model_.get();
    }

private:
    std::unique_ptr<model::BallisticModel> model_;
    Config config_;
};

// ============================================================================
// Utility Functions
// ============================================================================

namespace detail {

/// Generic iterative pitch solving algorithm (stateless).
///
/// This function implements the core iteration logic used by IterativeSolver.
/// It's separated to allow reuse in different contexts.
[[nodiscard]] std::expected<AimSolution, std::string> iterative_solve_pitch(
    const model::BallisticModel& model, const Eigen::Vector3d& target_pos, double v0,
    int max_iterations, double height_tolerance, double max_pitch) noexcept;

} // namespace detail

} // namespace fcs::core::trajectory::solver
