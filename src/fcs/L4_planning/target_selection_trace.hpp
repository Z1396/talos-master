#pragma once

#include "L3_estimation/tracker/types.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace fcs::L4 {

/// Full diagnostics for a single choose-optimal candidate.
///
/// This lives on a dedicated diagnostic channel instead of inside the control
/// intent so visualization/recording can evolve without changing the execution path.
struct TargetSelectionTraceEntry {
    ArmorName target_name{ArmorName::Invalid};
    ArmorColor target_color{ArmorColor::Neutral};
    L3::TrackerStatus track_status{L3::TrackerStatus::Idle};

    int rank{0};
    bool aim_valid{false};
    bool was_previously_selected{false};
    bool selected{false};
    bool runner_up{false};
    std::string aim_error{};
    std::optional<Eigen::Vector3d> target_center{};

    double image_center_distance_px{std::numeric_limits<double>::infinity()};
    double optical_age_s{0.0};
    double tof_s{std::numeric_limits<double>::infinity()};
    double distance_m{std::numeric_limits<double>::infinity()};
    double yaw_effort_deg{std::numeric_limits<double>::infinity()};
    double pitch_effort_deg{std::numeric_limits<double>::infinity()};

    double image_center_score{0.0};
    double track_state_score{0.0};
    double tof_score{0.0};
    double gimbal_effort_score{0.0};
    double armor_name_score{0.0};

    double image_center_weighted{0.0};
    double track_state_weighted{0.0};
    double tof_weighted{0.0};
    double gimbal_effort_weighted{0.0};
    double armor_name_weighted{0.0};

    double weighted_sum{0.0};
    double total_weight{0.0};
    double total_score{0.0};
};

/// Full choose-optimal trace for one L4 update.
struct TargetSelectionTrace {
    uint64_t timestamp_ns{0};
    bool had_previous_target{false};
    ArmorName previous_target_name{ArmorName::Invalid};
    ArmorColor previous_target_color{ArmorColor::Neutral};
    bool kept_current_target{false};
    double switch_margin{0.0};
    std::vector<TargetSelectionTraceEntry> candidates{};
};

} // namespace fcs::L4
