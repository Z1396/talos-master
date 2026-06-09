#pragma once

#include "L3_estimation/tracker/new_tracker.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "core/target_key.hpp"
#include "core/types.hpp"
#include <functional>
#include <memory>
#include <ranges>
#include <unordered_map>
#include <utility>
#include <vector>

#include <magic_enum.hpp>

namespace fcs::L3 {

// ============================================================================
// TrackerManager - Multi-target tracking manager
// ============================================================================

class TrackerManager {
public:
    // Constructor: pre-allocate all (name, color) combinations
    explicit TrackerManager(const TrackerConfig& config) {
        // Iterate over all valid ArmorName values (except Invalid)
        const auto all_names  = magic_enum::enum_values<ArmorName>();
        const auto all_colors = magic_enum::enum_values<ArmorColor>();

        for (const auto name : all_names) {
            if (name == ArmorName::Invalid) {
                continue; // Skip Invalid
            }

            for (const auto color : all_colors) {
                // Skip Neutral color (not a valid enemy color)
                if (color == ArmorColor::Neutral) {
                    continue;
                }

                const core::TargetKey key{name, color};

                // Create tracker with shared config
                auto tracker            = std::make_unique<TrackerNew>(config);
                trackers_[key]          = std::move(tracker);
                last_timestamp_ns_[key] = 0;
            }
        }
    }

    // Non-copyable, non-movable (owns unique_ptrs)
    TrackerManager(const TrackerManager&)            = delete;
    TrackerManager& operator=(const TrackerManager&) = delete;
    TrackerManager(TrackerManager&&)                 = delete;
    TrackerManager& operator=(TrackerManager&&)      = delete;

    // ========================================================================
    // Update API - Batch update with measurement routing
    // ========================================================================

    /// Batch update: process all trackers (matching original singleton semantics)
    /// - All non-Idle trackers: predict(dt)
    /// - All trackers: update() (handles both found/not found via state_machine)
    [[nodiscard]] TrackerOutputs update_all(const ArmorMeasurementBatch& measurements) {
        TrackerOutputs updated_outputs;

        // Group measurements by (name, color) for routing
        std::unordered_map<core::TargetKey, ArmorMeasurementBatch, core::TargetKeyHash> grouped;
        for (const auto& meas : measurements.measurements) {
            const core::TargetKey key{meas.name, meas.color};
            grouped[key].measurements.push_back(meas);
        }

        // Initialize timestamps for groups that have measurements
        for (auto& [key, batch] : grouped) {
            batch.timestamp_ns = measurements.timestamp_ns;
            batch.frame_id     = measurements.frame_id;
        }

        // Process ALL trackers (same logic as original singleton)
        for (auto& [key, tracker] : trackers_) {
            uint64_t& last_ts = last_timestamp_ns_[key];

            // Calculate dt
            double dt = 0.0;
            if (measurements.timestamp_ns > last_ts && last_ts != 0) {
                dt = static_cast<double>(measurements.timestamp_ns - last_ts) / 1e9;
            }
            last_ts = measurements.timestamp_ns;

            // Predict for non-Idle trackers (time update)
            if (tracker->status() != TrackerStatus::Idle) {
                tracker->predict(dt);
            }

            // Update (measurement update or lost handling)
            auto it = grouped.find(key);
            if (it != grouped.end()) {
                // Has measurements for this tracker
                if (tracker->status() == TrackerStatus::Idle) {
                    (void)tracker->first_meet(it->second);
                } else {
                    (void)tracker->update(it->second);
                }
            } else {
                // No measurements: still call update to handle lost tracking
                // Build empty batch for state_machine to process "not found"
                ArmorMeasurementBatch empty_batch;
                empty_batch.timestamp_ns = measurements.timestamp_ns;
                empty_batch.frame_id     = measurements.frame_id;
                (void)tracker->update(empty_batch);
            }

            // Collect output: exclude Idle trackers (semantic: all non-Idle)
            if (tracker->status() != TrackerStatus::Idle) {
                auto output         = tracker->get_output();
                output.timestamp_ns = measurements.timestamp_ns;
                updated_outputs.push_back(std::move(output));
            }
        }

        return updated_outputs;
    }

    // ========================================================================
    // Query APIs - Return materialized vectors for scheduler integration
    // ========================================================================

    /// Only trackers where is_tracking() == true (Tracking or TempLost)
    [[nodiscard]] TrackerOutputs active_outputs() const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            const auto output = tracker->get_output();
            if (output.is_tracking()) {
                outputs.push_back(output);
            }
        }

        return outputs;
    }

    /// All trackers regardless of status
    [[nodiscard]] TrackerOutputs all_outputs() const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            outputs.push_back(tracker->get_output());
        }

        return outputs;
    }

    /// Filter by specific status
    [[nodiscard]] TrackerOutputs outputs_with_status(TrackerStatus status) const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            const auto output = tracker->get_output();
            if (output.status == status) {
                outputs.push_back(output);
            }
        }

        return outputs;
    }

    /// Filter by name only (ignore color)
    /// Returns all trackers with the given (name, color) key, regardless of current state
    [[nodiscard]] TrackerOutputs outputs_with_name(ArmorName name) const {
        TrackerOutputs outputs;
        outputs.reserve(trackers_.size());

        for (const auto& [key, tracker] : trackers_) {
            if (key.name == name) {
                outputs.push_back(tracker->get_output());
            }
        }

        return outputs;
    }

    // ========================================================================
    // Direct Access API
    // ========================================================================

    /// Direct access to individual tracker (e.g., for debug/visualization)
    /// Returns: std::optional<std::reference_wrapper<TrackerNew>>
    [[nodiscard]] std::optional<std::reference_wrapper<TrackerNew>>
        get_tracker(ArmorName name, ArmorColor color) noexcept {
        const core::TargetKey key{name, color};
        auto it = trackers_.find(key);
        if (it == trackers_.end()) {
            return std::nullopt;
        }
        return std::ref(*it->second);
    }

    /// Const overload
    [[nodiscard]] std::optional<std::reference_wrapper<const TrackerNew>>
        get_tracker(ArmorName name, ArmorColor color) const noexcept {
        const core::TargetKey key{name, color};
        auto it = trackers_.find(key);
        if (it == trackers_.end()) {
            return std::nullopt;
        }
        return std::ref(*it->second);
    }

private:
    std::unordered_map<core::TargetKey, std::unique_ptr<TrackerNew>, core::TargetKeyHash> trackers_;
    std::unordered_map<core::TargetKey, uint64_t, core::TargetKeyHash> last_timestamp_ns_;
};

// ============================================================================
// Pipeable Filter Utilities for C++20 Ranges
// ============================================================================

namespace filters {

// Filter: tracking status (Tracking or TempLost)
inline constexpr auto is_tracking =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.is_tracking(); });

// Filter: specific status
[[nodiscard]] inline auto with_status(TrackerStatus status) {
    return std::views::filter(
        [status](const TrackerOutput& output) noexcept { return output.status == status; });
}

// Filter: specific name
[[nodiscard]] inline auto with_name(ArmorName name) {
    return std::views::filter(
        [name](const TrackerOutput& output) noexcept { return output.target_name == name; });
}

// Filter: specific color
[[nodiscard]] inline auto with_color(ArmorColor color) {
    return std::views::filter(
        [color](const TrackerOutput& output) noexcept { return output.target_color == color; });
}

// Filter: name AND color combination
[[nodiscard]] inline auto with_target(ArmorName name, ArmorColor color) {
    return std::views::filter([name, color](const TrackerOutput& output) noexcept {
        return output.target_name == name && output.target_color == color;
    });
}

// Filter: robot targets only (not outpost)
inline constexpr auto is_robot =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.is_robot(); });

// Filter: outpost targets only
inline constexpr auto is_outpost =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.is_outpost(); });

// Filter: target jumped (for debugging)
inline constexpr auto has_jumped =
    std::views::filter([](const TrackerOutput& output) noexcept { return output.target_jumped; });

} // namespace filters

} // namespace fcs::L3
