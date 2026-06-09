#pragma once

#include <expected>
#include <string>

#include "core/trajectory/model/config.hpp"
#include "toml/type_wrappers.hpp"

namespace toml::inline v3 {
class table;
}
namespace fcs::core::trajectory {

struct TrajectoryConfig {
    toml_helper::flatten<model::ModelConfig> model{};
};

} // namespace fcs::core::trajectory
