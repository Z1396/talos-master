#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/types.hpp"
#include "core/armor_types.hpp"
#include "core/channel_topics.hpp"

#include <cstdint>
#include <optional>
#include <variant>

namespace fcs::L4 {

// ============================================================================
// Planner Seed (L4-internal)
// ============================================================================

/// Contains the raw L3 target state used by the trajectory builder.
struct PlannerSeed {
    using State = std::variant<std::monostate, L3::RobotTargetState, L3::OutpostTargetState>;

    uint64_t state_timestamp_ns{0};
    State state{};
    bool target_jumped{false};
    std::optional<int> tracker_last_armor_id{};
    ArmorAimPhase aim_phase{ArmorAimPhase::SingleArmor};
    int selected_armor_id{0};
    ArmorType armor_type{ArmorType::Invalid};

    [[nodiscard]] bool is_robot() const noexcept {
        return std::holds_alternative<L3::RobotTargetState>(state);
    }

    [[nodiscard]] bool is_outpost() const noexcept {
        return std::holds_alternative<L3::OutpostTargetState>(state);
    }

    [[nodiscard]] const L3::RobotTargetState* robot_state() const noexcept {
        return std::get_if<L3::RobotTargetState>(&state);
    }

    [[nodiscard]] const L3::OutpostTargetState* outpost_state() const noexcept {
        return std::get_if<L3::OutpostTargetState>(&state);
    }
};

// ============================================================================
// L4 Channel Topics
// ============================================================================

using fcs::L3::TrackerOutputChannelTopic;

} // namespace fcs::L4
