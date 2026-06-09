#include "foxglove_systems.hpp"

#include "foxglove_server.hpp"
#include "systems/base.hpp"

#include "scheduler/scheduler.hpp"

namespace fcs::visualization {

std::shared_ptr<FoxgloveServer> try_create_foxglove_server(FoxgloveConfig config) {
    auto server = create_foxglove_server(std::move(config));
    if (!server.has_value()) {
        SPDLOG_ERROR("foxglove server creation failed: {}. Visualization disabled", server.error());
        return {};
    }
    return std::move(*server);
}

std::shared_ptr<FoxgloveServer> try_create_foxglove_server(uint16_t port, std::string host) {
    return try_create_foxglove_server(
        FoxgloveConfig{
            .transport = FoxgloveTransport::WebSocket,
            .host      = std::move(host),
            .port      = port,
        });
}

void register_foxglove_systems(
    bool daedalus, talos::Scheduler& scheduler, talos::Scheduler* scheduler_ptr) {
    using namespace foxglove::systems;

    // Register all layered systems
    register_l1_sensor_systems(scheduler);
    register_l2_perception_systems(scheduler);
    register_l3_estimation_systems(scheduler);
    register_l4_planning_systems(scheduler);
    register_rune_systems(scheduler);
    if (daedalus) {
        SPDLOG_INFO("register ground truth system");
        register_ground_truth_systems(scheduler);
    }
    register_debug_systems(scheduler, scheduler_ptr);
}

} // namespace fcs::visualization
