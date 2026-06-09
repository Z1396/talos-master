#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "core/target_key.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace fcs::L4 {

enum class ArmorTargetDeciderKind : uint8_t {
    Unmanned = 0,
    Manned   = 1,
};

struct TargetSelectionScores {
    double total{0.0};
    double image_center{0.0};
    double track_state{0.0};
    double tof{0.0};
    double gimbal_effort{0.0};
    double armor_name{0.0};
};

struct ArmorTargetDeciderCandidate {
    core::TargetKey key{};
    L3::TrackerOutput tracker{};
    TargetSelectionScores scores{};
};

struct ArmorTargetDeciderState {
    std::optional<core::TargetKey> selected_key{};
};

struct UnmannedArmorTargetDecider {
    double switch_margin{0.08};
};

struct MannedArmorTargetDecider {};

using ArmorTargetDecider = std::variant<UnmannedArmorTargetDecider, MannedArmorTargetDecider>;

struct ArmorTargetDeciderResult {
    size_t selected_index{0};
    bool kept_current_target{false};
    std::vector<size_t> ranked_indices{};
    std::optional<size_t> runner_up{};
};

[[nodiscard]] auto
    make_armor_target_decider(ArmorTargetDeciderKind kind, double switch_margin) noexcept
    -> ArmorTargetDecider;

[[nodiscard]] auto decide_armor_target(
    const ArmorTargetDecider& decider, ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult>;

} // namespace fcs::L4
