#pragma once

#include "config.hpp"
#include "scheduler/scheduler.hpp"
#include "solver/config.hpp"
#include "solver/solver_interfaces.hpp"

namespace fcs::core::trajectory {

struct bullet_speed_data {
    double bullet_speed;
};
using trajectory_solver = talos::res<std::unique_ptr<solver::TrajectorySolver>>;
using trajectory_config = talos::res<model::ModelConfig>;

using bullet_speed     = talos::res<bullet_speed_data>;
using bullet_speed_mut = talos::res_mut<bullet_speed_data>;

constexpr void register_resource(talos::Scheduler& scheduler, TrajectoryConfig&& config) noexcept {
    scheduler.world().insert_resource(config.model.get());
    auto solver = solver::create_solver(config);
    scheduler.world().insert_resource(std::move(solver));
}

} // namespace fcs::core::trajectory
