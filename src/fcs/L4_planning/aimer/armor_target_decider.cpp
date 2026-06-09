#include "L4_planning/aimer/armor_target_decider.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace fcs::L4 {

namespace {

[[nodiscard]] auto find_selected_candidate_index(
    std::span<const ArmorTargetDeciderCandidate> candidates,
    const std::optional<core::TargetKey>& selected_key) noexcept -> std::optional<size_t> {
    if (!selected_key.has_value()) {
        return std::nullopt;
    }

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].key == *selected_key) {
            return i;
        }
    }

    return std::nullopt;
}

[[nodiscard]] auto compare_total_score(
    const ArmorTargetDeciderCandidate& lhs, const ArmorTargetDeciderCandidate& rhs) noexcept
    -> bool {
    if (lhs.scores.total != rhs.scores.total) {
        return lhs.scores.total > rhs.scores.total;
    }

    const bool lhs_has_finite = std::isfinite(lhs.tracker.last_image_center_distance_px);
    const bool rhs_has_finite = std::isfinite(rhs.tracker.last_image_center_distance_px);
    if (lhs_has_finite != rhs_has_finite) {
        return lhs_has_finite;
    }

    if (lhs_has_finite
        && lhs.tracker.last_image_center_distance_px != rhs.tracker.last_image_center_distance_px) {
        return lhs.tracker.last_image_center_distance_px
             < rhs.tracker.last_image_center_distance_px;
    }

    return lhs.tracker.last_observation_timestamp_ns > rhs.tracker.last_observation_timestamp_ns;
}

[[nodiscard]] auto compare_image_center_distance(
    const ArmorTargetDeciderCandidate& lhs, const ArmorTargetDeciderCandidate& rhs) noexcept
    -> bool {
    const bool lhs_has_finite = std::isfinite(lhs.tracker.last_image_center_distance_px);
    const bool rhs_has_finite = std::isfinite(rhs.tracker.last_image_center_distance_px);
    if (lhs_has_finite != rhs_has_finite) {
        return lhs_has_finite;
    }

    if (lhs_has_finite
        && lhs.tracker.last_image_center_distance_px != rhs.tracker.last_image_center_distance_px) {
        return lhs.tracker.last_image_center_distance_px
             < rhs.tracker.last_image_center_distance_px;
    }

    if (lhs.scores.total != rhs.scores.total) {
        return lhs.scores.total > rhs.scores.total;
    }

    return lhs.tracker.last_observation_timestamp_ns > rhs.tracker.last_observation_timestamp_ns;
}

template <typename Compare>
[[nodiscard]] auto
    ranked_indices_by(std::span<const ArmorTargetDeciderCandidate> candidates, Compare compare)
        -> std::vector<size_t> {
    std::vector<size_t> indices;
    indices.reserve(candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        indices.push_back(i);
    }

    std::ranges::sort(
        indices, [&](size_t lhs, size_t rhs) { return compare(candidates[lhs], candidates[rhs]); });
    return indices;
}

[[nodiscard]] auto
    first_non_selected_index(std::span<const size_t> ranked_indices, size_t selected_index) noexcept
    -> std::optional<size_t> {
    for (const size_t index : ranked_indices) {
        if (index != selected_index) {
            return index;
        }
    }

    return std::nullopt;
}

[[nodiscard]] auto decide_unmanned_target(
    const UnmannedArmorTargetDecider& decider, ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult> {
    if (candidates.empty()) {
        state.selected_key.reset();
        return std::nullopt;
    }

    auto ranked = ranked_indices_by(candidates, compare_total_score);

    size_t selected_index    = ranked.front();
    bool kept_current_target = false;
    const auto current_index = find_selected_candidate_index(candidates, state.selected_key);
    if (current_index.has_value() && *current_index != ranked.front()) {
        const double best_score    = candidates[ranked.front()].scores.total;
        const double current_score = candidates[*current_index].scores.total;
        if (best_score <= current_score + decider.switch_margin) {
            selected_index      = *current_index;
            kept_current_target = true;
        }
    }

    state.selected_key   = candidates[selected_index].key;
    const auto runner_up = kept_current_target ? std::optional<size_t>{ranked.front()}
                                               : first_non_selected_index(ranked, selected_index);

    return ArmorTargetDeciderResult{
        .selected_index      = selected_index,
        .kept_current_target = kept_current_target,
        .ranked_indices      = std::move(ranked),
        .runner_up           = runner_up,
    };
}

[[nodiscard]] auto decide_manned_target(
    ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult> {
    if (candidates.empty()) {
        state.selected_key.reset();
        return std::nullopt;
    }

    auto ranked = ranked_indices_by(candidates, compare_image_center_distance);

    size_t selected_index    = ranked.front();
    bool kept_current_target = false;

    if (const auto current_index = find_selected_candidate_index(candidates, state.selected_key);
        current_index.has_value()) {
        selected_index      = *current_index;
        kept_current_target = true;

        ranked.erase(std::remove(ranked.begin(), ranked.end(), selected_index), ranked.end());
        ranked.insert(ranked.begin(), selected_index);
    }

    state.selected_key   = candidates[selected_index].key;
    const auto runner_up = first_non_selected_index(ranked, selected_index);

    return ArmorTargetDeciderResult{
        .selected_index      = selected_index,
        .kept_current_target = kept_current_target,
        .ranked_indices      = std::move(ranked),
        .runner_up           = runner_up,
    };
}

} // namespace

auto make_armor_target_decider(ArmorTargetDeciderKind kind, double switch_margin) noexcept
    -> ArmorTargetDecider {
    switch (kind) {
    case ArmorTargetDeciderKind::Unmanned:
        return UnmannedArmorTargetDecider{.switch_margin = switch_margin};
    case ArmorTargetDeciderKind::Manned: return MannedArmorTargetDecider{};
    }

    return UnmannedArmorTargetDecider{.switch_margin = switch_margin};
}

auto decide_armor_target(
    const ArmorTargetDecider& decider, ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult> {
    return std::visit(
        [&](const auto& concrete_decider) -> std::optional<ArmorTargetDeciderResult> {
            using T = std::decay_t<decltype(concrete_decider)>;
            if constexpr (std::is_same_v<T, UnmannedArmorTargetDecider>) {
                return decide_unmanned_target(concrete_decider, state, candidates);
            } else {
                return decide_manned_target(state, candidates);
            }
        },
        decider);
}

} // namespace fcs::L4
