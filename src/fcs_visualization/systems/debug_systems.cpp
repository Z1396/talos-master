#include "base.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "foxglove_types.hpp"

#include "L3_estimation/energy_meter/types.hpp"
#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include "scheduler/scheduler.hpp"

#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <system_helpers.hpp>
#include <utility.hpp>

namespace fcs::visualization::foxglove::systems {

/// @brief Register debug visualization systems
///
/// This includes all debug publishing systems for L2, L3, and L4 layers:
/// - L2: PnP solver metrics, NN confidence scores
/// - L3: Association phase, permutation, filter innovation/covariance, Viterbi, state machine
/// - L4: MPC horizon, cost, constraints
/// - Misc: Energy meter state, performance statistics
void register_debug_systems(talos::scheduler::Scheduler& app, talos::Scheduler* scheduler_ptr) {

    // =========================================================================
    // L2 DEBUG: PnP Solver Metrics
    // =========================================================================

    app.add_system<talos::fixed_rate_silent<1>>(
        "foxglove_res_pub",
        [](talos::res<std::shared_ptr<FoxgloveServer>> server,
           core::trajectory::bullet_speed bullet_speed, core::detecting_color color,
           core::capabilities cap, core::following following) {
            auto& server_ptr = *server; // 引用绑定
            if (!server_ptr->is_initialized()) {
                return;
            }

            nlohmann::json j;
            auto now               = clock::now_ns();
            j["timestamp"]["sec"]  = now / 1000000000L;
            j["timestamp"]["nsec"] = now % 1000000000L;
            j["bullet_speed"]      = bullet_speed->bullet_speed;
            j["color"]             = magic_enum::enum_name(*color);
            j["following"]         = following->load();
            std::vector<std::string> cap_str;
            const auto active = core::active_capabilities(*cap);
            cap_str.reserve(active.size());
            for (const auto c : active) {
                cap_str.emplace_back(magic_enum::enum_name(c));
            }
            j["capabilities"] = cap_str;

            detail::publish_json_message<ResourceMessage>(*server, j);
        });

    app.add_system<talos::pool_compute>(
        "foxglove_debug_l2_pnp_pub",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, meas_in)) {
                return;
            }

            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json pnp_json;
            pnp_json["timestamp_ns"] = batch->timestamp_ns;
            pnp_json["measurements"] = nlohmann::json::array();

            for (size_t i = 0; i < batch->measurements.size(); ++i) {
                const auto& m = batch->measurements[i];
                nlohmann::json obj;
                obj["index"]      = i;
                obj["name"]       = magic_enum::enum_name(m.name);
                const auto t      = m.transform.translation();
                obj["position"]   = {t.x(), t.y(), t.z()};
                const auto rpy    = m.transform.euler_rot().rpy();
                obj["rpy"]        = {std::get<0>(rpy), std::get<1>(rpy), std::get<2>(rpy)};
                obj["confidence"] = m.confidence;
                pnp_json["measurements"].push_back(obj);
            }

            detail::publish_json_message<PnPSolverMessage>(*server, pnp_json);
        });

    // =========================================================================
    // L2 DEBUG: NN Confidence Scores
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_debug_l2_nn_confidence_pub",
        [](talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, det_in)) {
                return;
            }

            auto batch = det_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json nn_json;
            nn_json["timestamp_ns"] = batch->timestamp_ns;
            nn_json["detections"]   = nlohmann::json::array();

            for (size_t i = 0; i < batch->detections.size(); ++i) {
                const auto& det = batch->detections[i];
                nlohmann::json obj;
                obj["index"]      = i;
                obj["confidence"] = det.confidence;
                obj["color"]      = magic_enum::enum_name(det.color);
                obj["type"]       = magic_enum::enum_name(det.type);
                obj["name"]       = magic_enum::enum_name(det.name);
                nn_json["detections"].push_back(obj);
            }

            detail::publish_json_message<NNConfidenceMessage>(*server, nn_json);
        });

    // =========================================================================
    // ENERGY METER STATE PUBLISHER (L3)
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_debug_energy_meter_pub",
        [](talos::spmc<fcs::energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> state_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, state_in)) {
                return;
            }

            auto state = state_in.read();
            if (!state) {
                return;
            }

            nlohmann::json em_json;
            em_json["timestamp_ns"]   = state->timestamp_ns;
            em_json["tracker_state"]  = magic_enum::enum_name(state->tracker_state);
            em_json["tracking_valid"] = state->tracking_valid;
            em_json["r_center_odom"]  = {
                state->r_center_odom.x(), state->r_center_odom.y(), state->r_center_odom.z()};
            em_json["radius"]   = state->radius;
            em_json["roll"]     = state->roll;
            em_json["t"]        = state->t;
            em_json["blade_id"] = state->blade_id;
            em_json["position"] = {state->position.x(), state->position.y(), state->position.z()};
            em_json["is_big_rune"] = state->is_big_rune;
            em_json["model_valid"] = state->model_valid;
            em_json["direction"]   = state->direction;
            em_json["a"]           = state->a;
            em_json["omega"]       = state->omega;
            em_json["b"]           = state->b;
            em_json["r"]           = state->radius;
            em_json["obs_valid"]   = state->obs_valid;
            em_json["yaw"]         = state->yaw;

            detail::publish_json_message<EnergyMeterMessage>(*server, em_json);
        });

    // =========================================================================
    // PERFORMANCE STATISTICS PUBLISHER (1 Hz, fixed_rate_silent)
    // =========================================================================

    if (scheduler_ptr) {
        app.add_system<talos::fixed_rate_silent<100>>(
            "foxglove_debug_perf_stats_pub",
            [scheduler_ptr](talos::res<std::shared_ptr<FoxgloveServer>> server) {
                auto& server_ptr = *server; // 引用绑定
                if (!server_ptr->is_initialized()) {
                    return;
                }

                // Get scheduler performance statistics JSON
                std::string json_str = scheduler_ptr->get_stats_json();

                // Send to foxglove
                PerfStatsMessage msg;
                msg.payload = json_to_bytes(json_str);
                server_ptr->enqueue_message(std::move(msg));
            });
    }
}

} // namespace fcs::visualization::foxglove::systems
