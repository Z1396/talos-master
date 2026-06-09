#pragma once

#include "L4_planning/aimer/armor_target_decider.hpp"
#include "core/armor_types.hpp"
#include "toml/ext/containers.hpp"

namespace fcs::L4 {

struct TargetSelectionWeights {
    double image_center{5.0};
    double track_state{3.0};
    double tof{1.5};
    double gimbal_effort{1.0};
    double armor_name{0.5};
};

struct TargetSelectionRefs {
    double image_center_ref_px{300.0};
    double tof_ref_s{0.35};
    double yaw_effort_ref_deg{20.0};
    double pitch_effort_ref_deg{10.0};
};

struct TargetSelectionArmorNameScore {
    double sentry{0.5};
    double one{0.5};
    double two{0.5};
    double three{0.5};
    double four{0.5};
    double five{0.5};
    double outpost{0.5};
    double base{0.5};
    double base_large{0.5};
    double invalid{0.0};

    [[nodiscard]] double score(ArmorName name) const noexcept {
        switch (name) {
        case ArmorName::Sentry: return sentry;
        case ArmorName::One: return one;
        case ArmorName::Two: return two;
        case ArmorName::Three: return three;
        case ArmorName::Four: return four;
        case ArmorName::Five: return five;
        case ArmorName::Outpost: return outpost;
        case ArmorName::Base: return base;
        case ArmorName::BaseLarge: return base_large;
        case ArmorName::Invalid: return invalid;
        }
        return invalid;
    }
};

struct TargetSelectionConfig {
    ArmorTargetDeciderKind decider{ArmorTargetDeciderKind::Unmanned};
    double temp_lost_state_score{0.05};
    double optical_stale_timeout_s{0.25};
    double switch_margin{0.08};
    TargetSelectionWeights weights{};
    TargetSelectionRefs refs{};
    TargetSelectionArmorNameScore armor_name_score{};
};

/// Aimer configuration (shared by all target types)
struct AimerConfig {
    /// System delay compensation (seconds)
    /// Includes: camera exposure, image processing, communication, gimbal response
    double delay{0.01};

    /// Authoritative four-state FSM thresholds from awakening::auto_aim::AutoAimFsmController.
    double single_whole_up{1.5};
    double single_whole_down{1.0};
    double whole_pair_up{6.5};
    double whole_pair_down{7.5};
    double pair_center_up{16.5};
    double pair_center_down{15.0};
    int transfer_thresh{50};

    /// Front-window limit for single-armor locking (degrees).
    double front_window_deg{60.0};

    double min_coming_vel{1e-2};
    double min_coming_vel_horizon{0.5};

    /// Weighted multi-target selection configuration.
    TargetSelectionConfig target_selection{};
};

/// Reference trajectory generation parameters.
///
/// Owned by L4 (planning), consumed by L5 (weapon) for solver initialization.
/// These define the horizon structure of the reference trajectory, NOT solver
/// constraints — pitch limits belong to L5's WeaponControllerConfig.
struct ReferenceTrajectoryConfig {
    int horizon_ahead{50};
    int horizon_back{50};
    double dt{0.01};
};

/// L4 planning configuration (Aimer + plan adapter architecture)
struct L4Config {
    // Shared aimer configuration (used for Robot/Outpost/Rune)
    AimerConfig aimer{};

    // Reference trajectory generation parameters (shared with L5)
    ReferenceTrajectoryConfig reference_trajectory{};
};

} // namespace fcs::L4
