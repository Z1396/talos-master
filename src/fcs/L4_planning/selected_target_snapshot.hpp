#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/types.hpp"
#include "L4_planning/plan_source.hpp"

namespace fcs::L4 {

/// L4-selected target snapshot for observability consumers.
///
/// This is the decision-layer view of the currently selected armor target.
/// It intentionally excludes execution-only data needed by L5.
struct SelectedTargetSnapshot {
    uint64_t timestamp_ns{0};
    GimbalPlanSource source{GimbalPlanSource::Armor};
    double distance{-1.0};
    uint64_t predicted_future_ns{0};
    ArmorAimPhase aim_phase{ArmorAimPhase::SingleArmor};
    int selected_armor_id{0};
    int rough_selected_armor_id{0};
    L3::TrackerOutput tracker{};

    [[nodiscard]] bool has_target() const noexcept {
        return source == GimbalPlanSource::Armor && distance > 0.0
            && tracker.target_name != ArmorName::Invalid;
    }
};

} // namespace fcs::L4
