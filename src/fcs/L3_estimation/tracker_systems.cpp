#include "L3_estimation/tracker_systems.hpp"

#include "L3_estimation/manager.hpp"
#include "L3_estimation/tracker/config.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include "scheduler/scheduler.hpp"

#include <memory>
#include <vector>

namespace fcs::L3 {
void register_tracker_systems(talos::Scheduler& scheduler, TrackerConfig&& config) {
    scheduler.world().insert_resource(std::move(config));
    scheduler.add_system<talos::fixed_rate<250>>(
        "armor_tracker", [manager = std::unique_ptr<TrackerManager>{}](
                             talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
                             talos::spmc_mut<TrackerOutputs, TrackerOutputChannelTopic> tracker_out,
                             talos::res<TrackerConfig> config) mutable {
            // Initialize manager on first run
            if (!manager) {
                manager = std::make_unique<TrackerManager>(*config);
            }

            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            // Update all trackers with measurements routed by (name, color)
            // Output ALL trackers (including Idle) - L4 selects optimal target
            auto outputs = manager->update_all(*batch);
            tracker_out.write(std::move(outputs));
        });
}

} // namespace fcs::L3
