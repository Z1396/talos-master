#include "L4_planning/aimer/aimer_systems.hpp"

namespace fcs::L4 {

void register_l4_planning_systems(talos::Scheduler& scheduler, L4Config&& config) {
    // Register Aimer system (Robot/Outpost/Rune)
    register_aimer_systems(scheduler, std::move(config));
}

} // namespace fcs::L4
