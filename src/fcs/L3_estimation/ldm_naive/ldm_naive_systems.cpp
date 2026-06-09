#include "L3_estimation/ldm_naive/ldm_naive_systems.hpp"

#include "L2_perception/ldm/types.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "core/channel_topics.hpp"
#include "core/time.hpp"
#include "ldm_tracker.hpp"
#include "scheduler/scheduler.hpp"

#include <Eigen/Core>
#include <optional>
#include <spdlog/spdlog.h>

namespace fcs::L3::ldm {

void register_naive_ldm_systems(talos::Scheduler& scheduler, const NaiveLdmConfig& config) {
    scheduler.world().insert_resource(config);

    scheduler.add_system<talos::fixed_rate<250>>(
        "l3_ldm_tracker",
        [tracker = std::optional<LdmTracker>{}](
            talos::res<NaiveLdmConfig> cfg,
            talos::spmc<fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> meas_in,
            talos::publish<LdmState> state_out) mutable {
            if (!tracker.has_value()) {
                tracker.emplace(*cfg);
            }

            const auto meas       = meas_in.read();
            const uint64_t now_ns = fcs::clock::now_ns();
            uint64_t update_ns    = now_ns;
            std::optional<LdmKinematic::PoseMeasurement> pose_measurement;

            if (meas.has_value()) {
                update_ns = meas->timestamp_ns;
                if (meas->transform_odom.has_value()) {
                    const auto& pose = *meas->transform_odom;
                    pose_measurement = LdmKinematic::PoseMeasurement{
                        .R_world_body = pose.rotation(),
                        .p_world_body = pose.translation(),
                    };
                } else {
                    // SPDLOG_ERROR(
                    //     "LdmMeasurement(t={}, frame={}, depth_quality={}) has no transform_odom:
                    //     " "PnP pose estimation failed or bearing-only mode", meas->timestamp_ns,
                    //     meas->frame_id, meas->depth_quality);
                    // Allow predict-only step when pose is unavailable.
                }
                tracker->update(update_ns, pose_measurement);
            }

            const auto state = tracker->get_output();
            if (state.has_value()) {
                LdmState output = *state;
                if (meas.has_value() && meas->accurate) {
                    output.accurate = true;
                }
                state_out.write(std::move(output));
            } else {
                SPDLOG_TRACE("LdmState unavailable: {}", state.error());
            }
        });
}

} // namespace fcs::L3::ldm
