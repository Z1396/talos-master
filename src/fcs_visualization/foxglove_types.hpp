#pragma once

#include "foxglove_config.hpp"

#include <cstdint>
#include <optional>
#include <string_view>
#include <tuple>
#include <variant>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/schemas.hpp>

namespace fcs::visualization {

// ============================================================================
// Channel Type Aliases
// ============================================================================

using SceneCh = std::optional<::foxglove::schemas::SceneUpdateChannel>;
using ImageCh = std::optional<::foxglove::schemas::CompressedImageChannel>;
using VideoCh = std::optional<::foxglove::schemas::CompressedVideoChannel>;
using JsonCh  = std::optional<::foxglove::RawChannel>;
using TfCh    = std::optional<::foxglove::schemas::FrameTransformsChannel>;
using CalibCh = std::optional<::foxglove::schemas::CameraCalibrationChannel>;
using LogCh   = std::optional<::foxglove::schemas::LogChannel>;

// ============================================================================
// FoxgloveChannels — runtime channel storage
// ============================================================================

struct FoxgloveChannels {
    // Scene
    SceneCh scene_ch;
    SceneCh ldm_track_scene_ch;
    SceneCh track_scene_ch;
    SceneCh gimbal_scene_ch;
    SceneCh association_scene_ch;
    SceneCh mpc_prediction_scene_ch;
    SceneCh rune_scene_ch;
    SceneCh rune_ekf_scene_ch;
    SceneCh ground_truth_scene_ch;

    // Image / Video
    ImageCh img_ch;
    VideoCh video_ch;
    ImageCh calibration_img_ch;
    ImageCh binary_img_ch;
    ImageCh pattern_img_ch;
    ImageCh rune_arrow_img_ch;
    ImageCh rune_target_img_ch;
    ImageCh rune_center_img_ch;
    ImageCh ekf_heatmap_ch;

    // JSON debug
    JsonCh debug_lights_ch;
    JsonCh debug_armors_ch;
    JsonCh measurement_ch;
    JsonCh target_ch;
    JsonCh target_selection_trace_ch;
    JsonCh cmd_gimbal_ch;
    JsonCh resource_ch;
    JsonCh perf_stats_ch;
    JsonCh mpc_traj_ch;
    JsonCh rune_debug_ch;
    JsonCh pnp_solver_ch;
    JsonCh nn_confidence_ch;
    JsonCh energy_meter_ch;
    JsonCh ldm_detection_ch;
    JsonCh ldm_measurement_ch;
    JsonCh ldm_state_ch;
    JsonCh ground_truth_ch;

    // Special
    TfCh tf_ch;
    CalibCh camera_calib_ch;
    LogCh log_ch;
};

// ============================================================================
// Channel Descriptors — single source of truth
// ============================================================================
//
// Each descriptor defines:
//   - topic:         Foxglove channel topic string
//   - channel_type:  SDK channel class (for create/log)
//   - payload_type:  Data type logged to the channel
//   - member:        Pointer-to-FoxgloveChannels-member
//   - is_raw:        true → RawChannel, requires byte-stream log()
//   - encoding:      (raw channels only) message encoding string
//   - transport:     (optional) restrict to a specific transport
//
// Adding a new channel: add one descriptor struct + one using alias.
// That's it — registry, variant, traits, and factory all derive from here.
// ============================================================================

// --- Scene channels ---

struct scene_def {
    static constexpr std::string_view topic = "/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::scene_ch;
    static constexpr bool is_raw            = false;
};

struct track_scene_def {
    static constexpr std::string_view topic = "/track/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::track_scene_ch;
    static constexpr bool is_raw            = false;
};

struct ldm_track_scene_def {
    static constexpr std::string_view topic = "/ldm/track/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::ldm_track_scene_ch;
    static constexpr bool is_raw            = false;
};

struct gimbal_scene_def {
    static constexpr std::string_view topic = "/solver/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::gimbal_scene_ch;
    static constexpr bool is_raw            = false;
};

struct association_scene_def {
    static constexpr std::string_view topic = "/debug/association_scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::association_scene_ch;
    static constexpr bool is_raw            = false;
};

struct mpc_prediction_scene_def {
    static constexpr std::string_view topic = "/debug/mpc_prediction_scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::mpc_prediction_scene_ch;
    static constexpr bool is_raw            = false;
};

struct rune_scene_def {
    static constexpr std::string_view topic = "/rune/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::rune_scene_ch;
    static constexpr bool is_raw            = false;
};

struct rune_ekf_scene_def {
    static constexpr std::string_view topic = "/rune/ekf_scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::rune_ekf_scene_ch;
    static constexpr bool is_raw            = false;
};

struct ground_truth_scene_def {
    static constexpr std::string_view topic = "/ground_truth/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::ground_truth_scene_ch;
    static constexpr bool is_raw            = false;
};

// --- Image channels ---

struct img_def {
    static constexpr std::string_view topic      = "/image";
    using channel_type                           = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                           = ::foxglove::schemas::CompressedImage;
    static constexpr auto member                 = &FoxgloveChannels::img_ch;
    static constexpr bool is_raw                 = false;
    static constexpr FoxgloveTransport transport = FoxgloveTransport::WebSocket;
};

struct calibration_img_def {
    static constexpr std::string_view topic = "/calibration/image";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::calibration_img_ch;
    static constexpr bool is_raw            = false;
};

struct binary_img_def {
    static constexpr std::string_view topic = "/debug/binary_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::binary_img_ch;
    static constexpr bool is_raw            = false;
};

struct pattern_img_def {
    static constexpr std::string_view topic = "/debug/number_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::pattern_img_ch;
    static constexpr bool is_raw            = false;
};

struct rune_arrow_img_def {
    static constexpr std::string_view topic = "/rune/arrow_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::rune_arrow_img_ch;
    static constexpr bool is_raw            = false;
};

struct rune_target_img_def {
    static constexpr std::string_view topic = "/rune/target_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::rune_target_img_ch;
    static constexpr bool is_raw            = false;
};

struct rune_center_img_def {
    static constexpr std::string_view topic = "/rune/center_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::rune_center_img_ch;
    static constexpr bool is_raw            = false;
};

struct ekf_heatmap_def {
    static constexpr std::string_view topic = "/debug/ekf_heatmap";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::ekf_heatmap_ch;
    static constexpr bool is_raw            = false;
};

// --- Video channel (Mcap-only) ---

struct video_def {
    static constexpr std::string_view topic      = "/image";
    using channel_type                           = ::foxglove::schemas::CompressedVideoChannel;
    using payload_type                           = ::foxglove::schemas::CompressedVideo;
    static constexpr auto member                 = &FoxgloveChannels::video_ch;
    static constexpr bool is_raw                 = false;
    static constexpr FoxgloveTransport transport = FoxgloveTransport::Mcap;
};

// --- JSON channels (RawChannel + "json" encoding) ---

struct debug_lights_def {
    static constexpr std::string_view topic    = "/debug/lights";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::debug_lights_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct debug_armors_def {
    static constexpr std::string_view topic    = "/debug/armors";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::debug_armors_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct measurement_def {
    static constexpr std::string_view topic    = "/solver/measurement";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::measurement_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct target_def {
    static constexpr std::string_view topic    = "/solver/target";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::target_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct target_selection_trace_def {
    static constexpr std::string_view topic    = "/solver/target_selection";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::target_selection_trace_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct cmd_gimbal_def {
    static constexpr std::string_view topic    = "/solver/cmd_gimbal";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::cmd_gimbal_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct resources_def {
    static constexpr std::string_view topic    = "/resources";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::resource_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct perf_stats_def {
    static constexpr std::string_view topic    = "/perf_stats";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::perf_stats_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct mpc_traj_def {
    static constexpr std::string_view topic    = "/mpc/trajectory";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::mpc_traj_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct rune_debug_def {
    static constexpr std::string_view topic    = "/rune/debug";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::rune_debug_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct pnp_solver_def {
    static constexpr std::string_view topic    = "/debug/pnp_solver";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::pnp_solver_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct nn_confidence_def {
    static constexpr std::string_view topic    = "/debug/nn_confidence";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::nn_confidence_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct energy_meter_def {
    static constexpr std::string_view topic    = "/energy_meter/state";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::energy_meter_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct ldm_detection_def {
    static constexpr std::string_view topic    = "/ldm/detection";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ldm_detection_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct ldm_measurement_def {
    static constexpr std::string_view topic    = "/ldm/measurement";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ldm_measurement_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct ldm_state_def {
    static constexpr std::string_view topic    = "/ldm/state";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ldm_state_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

struct ground_truth_def {
    static constexpr std::string_view topic    = "/ground_truth";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ground_truth_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// --- Special channels ---

struct tf_def {
    static constexpr std::string_view topic = "/tf";
    using channel_type                      = ::foxglove::schemas::FrameTransformsChannel;
    using payload_type                      = ::foxglove::schemas::FrameTransforms;
    static constexpr auto member            = &FoxgloveChannels::tf_ch;
    static constexpr bool is_raw            = false;
};

struct calib_def {
    static constexpr std::string_view topic = "/camera_info";
    using channel_type                      = ::foxglove::schemas::CameraCalibrationChannel;
    using payload_type                      = ::foxglove::schemas::CameraCalibration;
    static constexpr auto member            = &FoxgloveChannels::camera_calib_ch;
    static constexpr bool is_raw            = false;
};

struct log_def {
    static constexpr std::string_view topic = "/log";
    using channel_type                      = ::foxglove::schemas::LogChannel;
    using payload_type                      = ::foxglove::schemas::Log;
    static constexpr auto member            = &FoxgloveChannels::log_ch;
    static constexpr bool is_raw            = false;
};

// ============================================================================
// Channel Registry — ordered tuple of all channel descriptors
// ============================================================================

using ChannelRegistry = std::tuple<
    scene_def, track_scene_def, ldm_track_scene_def, gimbal_scene_def, association_scene_def,
    mpc_prediction_scene_def, rune_scene_def, rune_ekf_scene_def, ground_truth_scene_def, img_def,
    video_def, calibration_img_def, binary_img_def, pattern_img_def, rune_arrow_img_def,
    rune_target_img_def, rune_center_img_def, ekf_heatmap_def, debug_lights_def, debug_armors_def,
    measurement_def, target_def, target_selection_trace_def, cmd_gimbal_def, perf_stats_def,
    resources_def, mpc_traj_def, rune_debug_def, pnp_solver_def, nn_confidence_def,
    energy_meter_def, ldm_detection_def, ldm_measurement_def, ldm_state_def, ground_truth_def,
    tf_def, calib_def, log_def>;

// ============================================================================
// FoxgloveMsg — unified message type
// ============================================================================

template <typename Descriptor>
struct FoxgloveMsg {
    typename Descriptor::payload_type payload;
};

// ============================================================================
// Named message type aliases
// ============================================================================

using SceneMessage                = FoxgloveMsg<scene_def>;
using TrackSceneMessage           = FoxgloveMsg<track_scene_def>;
using LdmTrackSceneMessage        = FoxgloveMsg<ldm_track_scene_def>;
using GimbalSceneMessage          = FoxgloveMsg<gimbal_scene_def>;
using AssociationSceneMessage     = FoxgloveMsg<association_scene_def>;
using MpcPredictionSceneMessage   = FoxgloveMsg<mpc_prediction_scene_def>;
using RuneSceneMessage            = FoxgloveMsg<rune_scene_def>;
using RuneEkfSceneMessage         = FoxgloveMsg<rune_ekf_scene_def>;
using GroundTruthSceneMessage     = FoxgloveMsg<ground_truth_scene_def>;
using ImageMessage                = FoxgloveMsg<img_def>;
using VideoMessage                = FoxgloveMsg<video_def>;
using CalibrationImageMessage     = FoxgloveMsg<calibration_img_def>;
using BinaryImageMessage          = FoxgloveMsg<binary_img_def>;
using PatternImageMessage         = FoxgloveMsg<pattern_img_def>;
using RuneArrowImageMessage       = FoxgloveMsg<rune_arrow_img_def>;
using RuneTargetImageMessage      = FoxgloveMsg<rune_target_img_def>;
using RuneCenterImageMessage      = FoxgloveMsg<rune_center_img_def>;
using EkfHeatmapMessage           = FoxgloveMsg<ekf_heatmap_def>;
using DebugLightsMessage          = FoxgloveMsg<debug_lights_def>;
using DebugArmorsMessage          = FoxgloveMsg<debug_armors_def>;
using MeasurementMessage          = FoxgloveMsg<measurement_def>;
using TargetMessage               = FoxgloveMsg<target_def>;
using TargetSelectionTraceMessage = FoxgloveMsg<target_selection_trace_def>;
using GimbalCmdMessage            = FoxgloveMsg<cmd_gimbal_def>;
using PerfStatsMessage            = FoxgloveMsg<perf_stats_def>;
using ResourceMessage             = FoxgloveMsg<resources_def>;
using MpcTrajectoryMessage        = FoxgloveMsg<mpc_traj_def>;
using RuneDebugMessage            = FoxgloveMsg<rune_debug_def>;
using PnPSolverMessage            = FoxgloveMsg<pnp_solver_def>;
using NNConfidenceMessage         = FoxgloveMsg<nn_confidence_def>;
using EnergyMeterMessage          = FoxgloveMsg<energy_meter_def>;
using LdmDetectionMessage         = FoxgloveMsg<ldm_detection_def>;
using LdmMeasurementMessage       = FoxgloveMsg<ldm_measurement_def>;
using LdmStateMessage             = FoxgloveMsg<ldm_state_def>;
using GroundTruthMessage          = FoxgloveMsg<ground_truth_def>;
using TfMessage                   = FoxgloveMsg<tf_def>;
using LogMessage                  = FoxgloveMsg<log_def>;

// ============================================================================
// PayloadLogger — per-payload-type dispatch to channel.log()
// ============================================================================

namespace detail {

// Primary template: typed payloads → ch.log(payload)
template <typename Payload>
struct PayloadLogger {
    template <typename Channel>
    static void log(Channel& ch, const Payload& payload) noexcept {
        ch.log(payload);
    }
};

// Specialization: raw byte payloads → ch.log(byte_ptr, size)
template <>
struct PayloadLogger<std::vector<uint8_t>> {
    static void log(::foxglove::RawChannel& ch, const std::vector<uint8_t>& data) noexcept {
        ch.log(reinterpret_cast<const std::byte*>(data.data()), data.size());
    }
};

} // namespace detail

// ============================================================================
// FoxgloveMessage — auto-derived variant from ChannelRegistry
// ============================================================================

namespace detail {
template <typename... Defs>
auto make_message_variant(std::tuple<Defs...>) -> std::variant<FoxgloveMsg<Defs>...>;
} // namespace detail

using FoxgloveMessage = decltype(detail::make_message_variant(ChannelRegistry{}));

// ============================================================================
// Utility Functions
// ============================================================================

} // namespace fcs::visualization
