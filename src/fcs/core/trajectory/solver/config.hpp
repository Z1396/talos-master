#pragma once

#include "../config.hpp"
#include "../model/config.hpp"
#include "../model/ideal.hpp"
#include "../model/linear_drag.hpp"
#include "solver_interfaces.hpp"
#include "trajectory_solver.hpp"

#include <memory>

namespace fcs::core::trajectory::solver {

[[nodiscard]] inline std::unique_ptr<TrajectorySolver>
    create_solver(const TrajectoryConfig& config) noexcept {
    switch (config.model->type) {
    case model::ModelType::Ideal:
        return std::make_unique<DirectSolver>(
            std::make_unique<model::IdealModel>(config.model->gravity));
    case model::ModelType::LinearDrag:
        return std::make_unique<DirectSolver>(
            std::make_unique<model::LinearDragModel>(model::LinearDragModel::ResistanceParams{
                .gravity = config.model->gravity, .resistance = config.model->resistance}));
    }
}
} // namespace fcs::core::trajectory::solver
