#pragma once

#include <Eigen/Core>
#include <expected>
#include <string>

#include <groups/SEn3.hpp>

#include "L3_estimation/energy_meter/types.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/aimer_utils.hpp"
#include "L4_planning/aimer/types.hpp"
#include "L4_planning/config.hpp"
#include "core/trajectory/solver/solver_interfaces.hpp"
#include "frame.hpp"

namespace fcs::L4 {

/// Aimer - predicts aim point for all target types (Robot/Outpost/Rune)
///
/// Single class with overloaded aim() methods for different target types.
/// Uses composition with shared utility functions from aimer_utils.hpp.
class Aimer {
public:
    using GimbalTransform = fast_tf::FrameTransform<fast_tf::odom, fast_tf::gimbal_pitch>;
    using MuzzleTransform = fast_tf::FrameTransform<fast_tf::odom, fast_tf::muzzle>;

    /// Constructor
    explicit Aimer(const AimerConfig& config) noexcept
        : config_(config) {}

    [[nodiscard]] const AimerConfig& config() const noexcept { return config_; }

    /// Compute aim point for robot target (Infantry/Hero/Sentry)
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::RobotTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Compute aim point for robot target with extra delay on top of AimerConfig::delay.
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::RobotTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::RobotTargetState& target, const ArmorAimContext& context,
            const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
            uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Compute aim point for outpost target
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Compute aim point for outpost target with extra delay on top of AimerConfig::delay.
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::OutpostTargetState& target, const ArmorAimContext& context,
            const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
            uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Compute aim point for rune target (EnergyMeterState)
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const energy_meter::EnergyMeterState& state, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t current_ns, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Compute aim point for LDM target
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::ldm::LdmState& state, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t current_ns, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Compute aim point for LDM target with extra prediction delay.
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::ldm::LdmState& state, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t current_ns, double extra_prediction_delay,
            double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

private:
    // ========================================================================
    // Target type utility
    // ========================================================================

    /// Get target type for a given target state type
    template <class TargetType>
    [[nodiscard]] static constexpr AimerTargetType get_target_type() noexcept;

    // ========================================================================
    // Armor selection helper
    // ========================================================================

    struct PredictedAimPoint {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        double delta_angle{0.0};
        int armor_id{0};
    };

    // ========================================================================
    // Common aim angle computation (shared by all target types)
    // ========================================================================

    [[nodiscard]] static std::pair<double, double> compose_gimbal_aim_angles(
        const core::trajectory::solver::AimSolution& solution, const GimbalTransform& gimbal,
        const MuzzleTransform& muzzle) noexcept {
        const auto desired_muzzle = math_fuxk::rpy(0.0, solution.pitch, solution.yaw).so3();
        const auto muzzle_delta   = muzzle.so3().inv() * desired_muzzle;
        const auto desired_gimbal = gimbal.so3() * muzzle_delta;
        const auto [r, p, y]      = math_fuxk::rpy(desired_gimbal).rpy();
        return {y, p};
    }

    /// Common gimbal-based aim angle computation for all target types.
    [[nodiscard]] static std::expected<std::pair<double, double>, std::string>
        compute_gimbal_aim_angles(
            const Eigen::Vector3d& predicted_pos, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, const core::trajectory::solver::TrajectorySolver& solver,
            double bullet_speed) noexcept {
        const auto solution = solver.solve(predicted_pos - muzzle.translation(), bullet_speed);
        if (!solution.has_value()) {
            return std::unexpected(solution.error());
        }
        return compose_gimbal_aim_angles(*solution, gimbal, muzzle);
    }

    // ========================================================================
    // Generic aim() - Template method for common aiming logic
    // ========================================================================

    /// Generic aim implementation for targets with prediction (Robot/Outpost)
    template <class TargetType>
    [[nodiscard]] std::expected<TargetPrediction, std::string> aim_generic(
        const TargetType& target, const ArmorAimContext& context, const GimbalTransform& gimbal,
        const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
        double prediction_delay, double bullet_speed,
        const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Generic aim implementation for targets without prediction (Rune)
    template <class TargetType>
    [[nodiscard]] std::expected<TargetPrediction, std::string> aim_generic_rune(
        const TargetType& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
        uint64_t current_ns, double bullet_speed,
        const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /// Predict robot target position at future time dt
    [[nodiscard]] Eigen::Vector3d
        predict_position(const L3::RobotTargetState& target, double dt) const noexcept;

    /// Predict outpost target position at future time dt
    [[nodiscard]] Eigen::Vector3d
        predict_position(const L3::OutpostTargetState& target, double dt) const noexcept;

    /// Predict outpost target position at future time dt
    [[nodiscard]] Eigen::Vector3d
        predict_position(const energy_meter::EnergyMeterState& target, double dt) const noexcept;

    /// Predict LDM target SE2(3) at future time dt via right-perturbation: X̂₊ = X̂ * exp(xi).
    [[nodiscard]] group::SEn3<double, 2>
        predict_state(const L3::ldm::LdmState& target, double dt) const noexcept;

    [[nodiscard]] PredictedAimPoint predict_aim_point(
        const L3::RobotTargetState& target, double dt, const ArmorAimContext& context,
        const MuzzleTransform& muzzle,
        std::optional<int> forced_armor_id = std::nullopt) const noexcept;

    [[nodiscard]] PredictedAimPoint predict_aim_point(
        const L3::OutpostTargetState& target, double dt, const ArmorAimContext& context,
        const MuzzleTransform& muzzle,
        std::optional<int> forced_armor_id = std::nullopt) const noexcept;

    AimerConfig config_;
};

// ========================================================================
// get_target_type specializations
// ========================================================================

template <>
inline constexpr AimerTargetType Aimer::get_target_type<L3::RobotTargetState>() noexcept {
    return AimerTargetType::Robot;
}

template <>
inline constexpr AimerTargetType Aimer::get_target_type<L3::OutpostTargetState>() noexcept {
    return AimerTargetType::Outpost;
}

template <>
inline constexpr AimerTargetType Aimer::get_target_type<energy_meter::EnergyMeterState>() noexcept {
    return AimerTargetType::Rune;
}
template <>
inline constexpr AimerTargetType Aimer::get_target_type<L3::ldm::LdmState>() noexcept {
    return AimerTargetType::Ldm;
}

} // namespace fcs::L4
