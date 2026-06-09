#pragma once

#include "scene_builder.hpp"
#include "system_helpers.hpp"
#include "tactical_palette.hpp"

#include "scheduler/thin.hpp"

namespace fcs::visualization::foxglove::systems {

/// @brief Shorthand for commonly used builder components
namespace viz = fcs::visualization;
namespace tac = viz::tactical;

// Pull in commonly used helpers to this namespace
using ::fcs::visualization::detail::foxglove_ready;
using ::fcs::visualization::detail::publish_json_message;
using ::fcs::visualization::detail::publish_scene_if_nonempty;

/// @brief Register L1 sensor layer systems (image publishing)
void register_l1_sensor_systems(talos::scheduler::Scheduler& app);

/// @brief Register L2 perception layer systems (detection, measurement)
void register_l2_perception_systems(talos::scheduler::Scheduler& app);

/// @brief Register L3 estimation layer systems (tracking, association)
void register_l3_estimation_systems(talos::scheduler::Scheduler& app);

/// @brief Register L4 planning layer systems (gimbal, MPC)
void register_l4_planning_systems(talos::scheduler::Scheduler& app);

/// @brief Register rune-specific visualization systems
void register_rune_systems(talos::scheduler::Scheduler& app);

/// @brief Register ground truth visualization systems (daedalus mode)
void register_ground_truth_systems(talos::scheduler::Scheduler& app);

/// @brief Register debug visualization systems
void register_debug_systems(talos::scheduler::Scheduler& app, talos::Scheduler* scheduler_ptr);

} // namespace fcs::visualization::foxglove::systems
