#include "runtime/capturer.hpp"

#include "L2_perception/rune/types.hpp"
#include "L3_estimation/energy_meter/types.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/control_intent.hpp"
#include "L4_planning/selected_target_snapshot.hpp"
#include "L4_planning/target_selection_trace.hpp"
#include "L5_weapon/fire_control.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "core/types.hpp"
#include "frame.hpp"
#include "scheduler/scheduler.hpp"

#include <fmt/format.h>
#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <bit>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

namespace fcs::runtime {
namespace {

/// std::visit helper for variant exhaustiveness.
template <typename... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

} // namespace
namespace {

namespace fs = std::filesystem;
using json   = nlohmann::json;

constexpr uint64_t kErrorLogThrottleNs   = 10'000'000'000ULL;
constexpr uint32_t kControlChunkMagic    = 0x4C525443U; // CTRL
constexpr uint16_t kControlSchemaVersion = 2;
constexpr uint32_t kControlFrequencyHz   = 250;

constexpr uint32_t kControlTransformWireSize = sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t)
                                             + (3U * sizeof(float)) + (4U * sizeof(float));
constexpr uint32_t kControlChunkHeaderWireSize =
    sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t)
    + sizeof(uint64_t) + sizeof(uint64_t);
constexpr uint32_t kControlSampleWireSize =
    (2U * sizeof(uint64_t)) + (7U * sizeof(float)) + (3U * sizeof(uint8_t)) + sizeof(uint64_t)
    + (7U * sizeof(float)) + (2U * sizeof(uint8_t)) + (2U * sizeof(int32_t))
    + (2U * sizeof(uint64_t)) + (2U * sizeof(float)) + (2U * sizeof(uint8_t)) + sizeof(uint16_t)
    + (8U * sizeof(float)) + (4U * kControlTransformWireSize);
static_assert(kControlTransformWireSize == 32U);
static_assert(kControlChunkHeaderWireSize == 32U);
static_assert(kControlSampleWireSize == 281U);

template <typename T>
struct ChannelSnapshot {
    bool updated_this_tick{false};
    std::optional<T> value{};

    [[nodiscard]] bool present() const noexcept { return value.has_value(); }
    [[nodiscard]] bool stale() const noexcept { return value.has_value() && !updated_this_tick; }
};

template <typename T, typename Topic>
[[nodiscard]] auto sample_channel(talos::spmc<T, Topic>& reader) -> ChannelSnapshot<T> {
    ChannelSnapshot<T> snapshot;
    const auto previous_generation = reader.last_generation();
    snapshot.value                 = reader.read_current();
    snapshot.updated_this_tick =
        snapshot.value.has_value() && reader.last_generation() > previous_generation;
    return snapshot;
}

[[nodiscard]] auto make_channel_meta(bool present, bool updated_this_tick) -> json {
    return json{
        {          "present",                       present},
        {"updated_this_tick",             updated_this_tick},
        {            "stale", present && !updated_this_tick},
    };
}

struct ControlTransformRecord {
    uint8_t present{0};
    uint8_t reserved0{0};
    uint16_t reserved1{0};
    std::array<float, 3> translation{};
    std::array<float, 4> quaternion{};
};

struct ControlSampleRecord {
    uint64_t sample_timestamp_ns{0};
    uint64_t imu_timestamp_ns{0};

    float imu_roll{0.0F};
    float imu_pitch{0.0F};
    float imu_yaw{0.0F};
    float imu_roll_vel{0.0F};
    float imu_pitch_vel{0.0F};
    float imu_yaw_vel{0.0F};

    float bullet_speed_raw{0.0F};
    uint8_t detecting_color{0};
    uint8_t plan_present{0};
    uint8_t weapon_present{0};

    uint64_t plan_timestamp_ns{0};
    float plan_distance{0.0F};
    float plan_yaw{0.0F};
    float plan_pitch{0.0F};
    float plan_yaw_vel{0.0F};
    float plan_pitch_vel{0.0F};
    float plan_yaw_acc{0.0F};
    float plan_pitch_acc{0.0F};
    uint8_t plan_source{0};
    uint8_t plan_aim_phase{0};
    int32_t plan_selected_armor_id{0};
    int32_t plan_rough_selected_armor_id{0};

    uint64_t weapon_timestamp_ns{0};
    uint64_t weapon_plan_timestamp_ns{0};
    float weapon_plan_yaw{0.0F};
    float weapon_plan_pitch{0.0F};
    uint8_t weapon_plan_fire{0};
    uint8_t weapon_fire{0};
    uint16_t reserved2{0};
    float weapon_yaw{0.0F};
    float weapon_pitch{0.0F};
    float weapon_yaw_vel{0.0F};
    float weapon_pitch_vel{0.0F};
    float weapon_yaw_accel{0.0F};
    float weapon_pitch_accel{0.0F};
    float weapon_tof{0.0F};
    float weapon_distance{0.0F};

    ControlTransformRecord odom_to_gimbal_pitch{};
    ControlTransformRecord gimbal_to_camera_link{};
    ControlTransformRecord odom_to_camera_optical{};
    ControlTransformRecord odom_to_muzzle{};
};

struct ControlChunkHeaderRecord {
    uint32_t magic{kControlChunkMagic};
    uint16_t schema_version{kControlSchemaVersion};
    uint16_t header_size{static_cast<uint16_t>(kControlChunkHeaderWireSize)};
    uint32_t sample_size{kControlSampleWireSize};
    uint32_t sample_count{0};
    uint64_t tick_index{0};
    uint64_t tick_timestamp_ns{0};
};

template <typename UInt>
void append_le_uint(std::vector<char>& out, UInt value) {
    static_assert(std::is_unsigned_v<UInt>);
    for (size_t i = 0; i < sizeof(UInt); ++i) {
        out.push_back(static_cast<char>((value >> (i * 8U)) & 0xFFU));
    }
}

void append_le_int32(std::vector<char>& out, int32_t value) {
    append_le_uint<uint32_t>(out, std::bit_cast<uint32_t>(value));
}

void append_le_float32(std::vector<char>& out, float value) {
    append_le_uint<uint32_t>(out, std::bit_cast<uint32_t>(value));
}

void append_control_transform(std::vector<char>& out, const ControlTransformRecord& tf) {
    append_le_uint<uint8_t>(out, tf.present);
    append_le_uint<uint8_t>(out, tf.reserved0);
    append_le_uint<uint16_t>(out, tf.reserved1);
    for (float value : tf.translation) {
        append_le_float32(out, value);
    }
    for (float value : tf.quaternion) {
        append_le_float32(out, value);
    }
}

auto serialize_control_chunk(
    const ControlChunkHeaderRecord& header, const std::vector<ControlSampleRecord>& samples)
    -> std::vector<char> {
    std::vector<char> bytes;
    bytes.reserve(
        static_cast<size_t>(kControlChunkHeaderWireSize)
        + (samples.size() * static_cast<size_t>(kControlSampleWireSize)));

    append_le_uint<uint32_t>(bytes, header.magic);
    append_le_uint<uint16_t>(bytes, header.schema_version);
    append_le_uint<uint16_t>(bytes, header.header_size);
    append_le_uint<uint32_t>(bytes, header.sample_size);
    append_le_uint<uint32_t>(bytes, header.sample_count);
    append_le_uint<uint64_t>(bytes, header.tick_index);
    append_le_uint<uint64_t>(bytes, header.tick_timestamp_ns);

    for (const auto& sample : samples) {
        append_le_uint<uint64_t>(bytes, sample.sample_timestamp_ns);
        append_le_uint<uint64_t>(bytes, sample.imu_timestamp_ns);

        append_le_float32(bytes, sample.imu_roll);
        append_le_float32(bytes, sample.imu_pitch);
        append_le_float32(bytes, sample.imu_yaw);
        append_le_float32(bytes, sample.imu_roll_vel);
        append_le_float32(bytes, sample.imu_pitch_vel);
        append_le_float32(bytes, sample.imu_yaw_vel);
        append_le_float32(bytes, sample.bullet_speed_raw);

        append_le_uint<uint8_t>(bytes, sample.detecting_color);
        append_le_uint<uint8_t>(bytes, sample.plan_present);
        append_le_uint<uint8_t>(bytes, sample.weapon_present);

        append_le_uint<uint64_t>(bytes, sample.plan_timestamp_ns);
        append_le_float32(bytes, sample.plan_distance);
        append_le_float32(bytes, sample.plan_yaw);
        append_le_float32(bytes, sample.plan_pitch);
        append_le_float32(bytes, sample.plan_yaw_vel);
        append_le_float32(bytes, sample.plan_pitch_vel);
        append_le_float32(bytes, sample.plan_yaw_acc);
        append_le_float32(bytes, sample.plan_pitch_acc);
        append_le_uint<uint8_t>(bytes, sample.plan_source);
        append_le_uint<uint8_t>(bytes, sample.plan_aim_phase);
        append_le_int32(bytes, sample.plan_selected_armor_id);
        append_le_int32(bytes, sample.plan_rough_selected_armor_id);

        append_le_uint<uint64_t>(bytes, sample.weapon_timestamp_ns);
        append_le_uint<uint64_t>(bytes, sample.weapon_plan_timestamp_ns);
        append_le_float32(bytes, sample.weapon_plan_yaw);
        append_le_float32(bytes, sample.weapon_plan_pitch);
        append_le_uint<uint8_t>(bytes, sample.weapon_plan_fire);
        append_le_uint<uint8_t>(bytes, sample.weapon_fire);
        append_le_uint<uint16_t>(bytes, sample.reserved2);
        append_le_float32(bytes, sample.weapon_yaw);
        append_le_float32(bytes, sample.weapon_pitch);
        append_le_float32(bytes, sample.weapon_yaw_vel);
        append_le_float32(bytes, sample.weapon_pitch_vel);
        append_le_float32(bytes, sample.weapon_yaw_accel);
        append_le_float32(bytes, sample.weapon_pitch_accel);
        append_le_float32(bytes, sample.weapon_tof);
        append_le_float32(bytes, sample.weapon_distance);

        append_control_transform(bytes, sample.odom_to_gimbal_pitch);
        append_control_transform(bytes, sample.gimbal_to_camera_link);
        append_control_transform(bytes, sample.odom_to_camera_optical);
        append_control_transform(bytes, sample.odom_to_muzzle);
    }

    return bytes;
}

[[nodiscard]] auto to_json_vec3(const Eigen::Vector3d& v) -> json {
    return json::array({v.x(), v.y(), v.z()});
}

[[nodiscard]] auto to_json_vec2(const Eigen::Vector2d& v) -> json {
    return json::array({v.x(), v.y()});
}

[[nodiscard]] auto to_json_filter_convergence(const L3::FilterConvergenceState& state) -> json {
    return json{
        {                       "status",                          magic_enum::enum_name(state.status)},
        {"normalized_innovation_squared", std::isfinite(state.normalized_innovation_squared)
 ? json(state.normalized_innovation_squared)
 : json(nullptr)                                             },
        {          "max_covariance_diag",           std::isfinite(state.max_covariance_diag)
           ? json(state.max_covariance_diag)
           : json(nullptr)                                   },
        {"consecutive_converged_updates",                          state.consecutive_converged_updates},
        { "consecutive_diverged_updates",                           state.consecutive_diverged_updates},
    };
}

[[nodiscard]] auto to_json_quat(const Eigen::Quaterniond& q) -> json {
    return json{
        {"x", q.x()},
        {"y", q.y()},
        {"z", q.z()},
        {"w", q.w()}
    };
}

[[nodiscard]] auto to_json_rpy(const math_fuxk::Ros2EulerRotd& rpy) -> json {
    const auto [roll, pitch, yaw] = rpy.rpy();
    return json::array({roll, pitch, yaw});
}

[[nodiscard]] auto to_json_rect(const cv::Rect2f& rect) -> json {
    return json{
        {"x",      rect.x},
        {"y",      rect.y},
        {"w",  rect.width},
        {"h", rect.height}
    };
}

[[nodiscard]] auto to_json_rect(const rune::RectF& rect) -> json {
    return json{
        {"x", rect.x},
        {"y", rect.y},
        {"w", rect.w},
        {"h", rect.h}
    };
}

[[nodiscard]] auto to_json_point(const cv::Point2f& point) -> json {
    return json::array({point.x, point.y});
}

[[nodiscard]] auto to_json_capturer_config(const CapturerConfig& config) -> json {
    return json{
        {            "enabled",             config.enabled},
        {         "output_dir",          config.output_dir},
        {"reserved_free_bytes", config.reserved_free_bytes},
    };
}

[[nodiscard]] auto resolve_existing_path(const fs::path& path) -> fs::path {
    if (fs::exists(path)) {
        return path;
    }

#ifdef TALOS_SOURCE_DIR
    const fs::path source_root = TALOS_SOURCE_DIR;
    const fs::path rooted_path = source_root / path;
    if (fs::exists(rooted_path)) {
        return rooted_path;
    }
#endif

    return path;
}

[[nodiscard]] auto normalize_path_for_comparison(const fs::path& path, std::string_view context)
    -> std::expected<fs::path, std::string> {
    std::error_code ec;
    const auto absolute_path = fs::absolute(path, ec);
    if (ec) {
        return std::unexpected(
            fmt::format("{} absolute path {}: {}", context, path.string(), ec.message()));
    }

    const auto normalized_path = fs::weakly_canonical(absolute_path, ec);
    if (ec) {
        return std::unexpected(
            fmt::format("{} normalize path {}: {}", context, path.string(), ec.message()));
    }
    return normalized_path;
}

[[nodiscard]] auto is_same_or_descendant(const fs::path& path, const fs::path& ancestor) noexcept
    -> bool {
    auto path_it = path.begin();
    for (const auto& ancestor_component : ancestor) {
        if (path_it == path.end() || *path_it != ancestor_component) {
            return false;
        }
        ++path_it;
    }
    return true;
}

[[nodiscard]] auto to_json_robot_state(const L3::RobotTargetState& state) -> json {
    return json{
        {   "position",                  to_json_vec3(state.position)},
        {   "velocity",                  to_json_vec3(state.velocity)},
        {        "yaw",                                     state.yaw},
        {      "v_yaw",                                   state.v_yaw},
        {    "radius0",                                 state.radius0},
        {    "radius1",                                 state.radius1},
        {         "z1",                                      state.z1},
        { "armors_num",                              state.armors_num},
        {"convergence", to_json_filter_convergence(state.convergence)},
    };
}

[[nodiscard]] auto to_json_outpost_state(const L3::OutpostTargetState& state) -> json {
    json z = json::array();
    for (double value : state.z) {
        z.push_back(value);
    }

    return json{
        {   "position",                  to_json_vec2(state.position)},
        {   "velocity",                  to_json_vec3(state.velocity)},
        {        "yaw",                                     state.yaw},
        {      "v_yaw",                                   state.v_yaw},
        {          "z",                                  std::move(z)},
        {     "radius",                                  state.radius},
        { "armors_num",                              state.armors_num},
        {"convergence", to_json_filter_convergence(state.convergence)},
    };
}

[[nodiscard]] auto to_json_tracker_output(const L3::TrackerOutput& output) -> json {
    json result{
        {                 "timestamp_ns",                                                output.timestamp_ns},
        {                       "status",                               magic_enum::enum_name(output.status)},
        {                  "target_name",                          magic_enum::enum_name(output.target_name)},
        {                 "target_color",                         magic_enum::enum_name(output.target_color)},
        {                "target_jumped",                                               output.target_jumped},
        {                "last_armor_id", output.last_armor_id ? json(*output.last_armor_id) : json(nullptr)},
        {"last_image_center_distance_px",       std::isfinite(output.last_image_center_distance_px)
       ? json(output.last_image_center_distance_px)
       : json(nullptr)                                             },
        {"last_observation_timestamp_ns",                               output.last_observation_timestamp_ns},
        {                   "state_kind",                                                            "empty"},
    };

    if (const auto* robot = output.robot_state()) {
        result["state_kind"] = "robot";
        result["state"]      = to_json_robot_state(*robot);
    } else if (const auto* outpost = output.outpost_state()) {
        result["state_kind"] = "outpost";
        result["state"]      = to_json_outpost_state(*outpost);
    } else {
        result["state"] = json::object();
    }

    return result;
}

[[nodiscard]] auto to_json_target_selection_trace_entry(const L4::TargetSelectionTraceEntry& entry)
    -> json {
    return json{
        {                    "rank",                           entry.rank                                    },
        {             "target_name",                                 magic_enum::enum_name(entry.target_name)},
        {            "target_color",                                magic_enum::enum_name(entry.target_color)},
        {            "track_status",                                magic_enum::enum_name(entry.track_status)},
        {               "aim_valid",                                                          entry.aim_valid},
        { "was_previously_selected",                                            entry.was_previously_selected},
        {                "selected",                                                           entry.selected},
        {               "runner_up",                                                          entry.runner_up},
        {               "aim_error",                                                          entry.aim_error},
        {           "target_center",
         entry.target_center ? json(to_json_vec3(*entry.target_center)) : json(nullptr)                      },
        {"image_center_distance_px",                  std::isfinite(entry.image_center_distance_px)
                  ? json(entry.image_center_distance_px)
                  : json(nullptr)                                        },
        {           "optical_age_s",                                                      entry.optical_age_s},
        {                   "tof_s",           std::isfinite(entry.tof_s) ? json(entry.tof_s) : json(nullptr)},
        {              "distance_m", std::isfinite(entry.distance_m) ? json(entry.distance_m) : json(nullptr)},
        {          "yaw_effort_deg",
         std::isfinite(entry.yaw_effort_deg) ? json(entry.yaw_effort_deg) : json(nullptr)                    },
        {        "pitch_effort_deg",
         std::isfinite(entry.pitch_effort_deg) ? json(entry.pitch_effort_deg) : json(nullptr)                },
        {      "image_center_score",                                                 entry.image_center_score},
        {       "track_state_score",                                                  entry.track_state_score},
        {               "tof_score",                                                          entry.tof_score},
        {     "gimbal_effort_score",                                                entry.gimbal_effort_score},
        {        "armor_name_score",                                                   entry.armor_name_score},
        {   "image_center_weighted",                                              entry.image_center_weighted},
        {    "track_state_weighted",                                               entry.track_state_weighted},
        {            "tof_weighted",                                                       entry.tof_weighted},
        {  "gimbal_effort_weighted",                                             entry.gimbal_effort_weighted},
        {     "armor_name_weighted",                                                entry.armor_name_weighted},
        {            "weighted_sum",                                                       entry.weighted_sum},
        {            "total_weight",                                                       entry.total_weight},
        {             "total_score",                                                        entry.total_score},
    };
}

[[nodiscard]] auto to_json_target_selection_trace(const L4::TargetSelectionTrace& trace) -> json {
    json candidates = json::array();
    for (const auto& candidate : trace.candidates) {
        candidates.push_back(to_json_target_selection_trace_entry(candidate));
    }

    return json{
        {         "timestamp_ns",                                 trace.timestamp_ns},
        {  "had_previous_target",                          trace.had_previous_target},
        { "previous_target_name",  magic_enum::enum_name(trace.previous_target_name)},
        {"previous_target_color", magic_enum::enum_name(trace.previous_target_color)},
        {  "kept_current_target",                          trace.kept_current_target},
        {        "switch_margin",                                trace.switch_margin},
        {           "candidates",                              std::move(candidates)},
    };
}

[[nodiscard]] auto to_json_control_intent(const L4::ControlIntent& intent) -> json {
    json result;
    std::visit(
        [&](const auto& cmd) {
            using T                = std::decay_t<decltype(cmd)>;
            result["timestamp_ns"] = cmd.timestamp_ns;
            if constexpr (std::is_same_v<T, L4::TrackCommand>) {
                result["mode"]            = "Track";
                result["control_horizon"] = cmd.control_trajectory.horizon();
                result["fire_horizon"]    = cmd.fire_trajectory.horizon();
            } else if constexpr (std::is_same_v<T, L4::ShotCommand>) {
                result["mode"]     = "Shot";
                result["yaw"]      = cmd.yaw;
                result["pitch"]    = cmd.pitch;
                result["distance"] = cmd.distance;
            } else if constexpr (std::is_same_v<T, L4::HoldCommand>) {
                result["mode"] = "Hold";
            }
        },
        intent);
    return result;
}

[[nodiscard]] auto to_json_selected_target_snapshot(const L4::SelectedTargetSnapshot& snapshot)
    -> json {
    json result                       = to_json_tracker_output(snapshot.tracker);
    result["timestamp_ns"]            = snapshot.timestamp_ns;
    result["tracking"]                = snapshot.tracker.is_tracking();
    result["valid"]                   = snapshot.has_target();
    result["optimal_target"]          = result["valid"];
    result["source"]                  = magic_enum::enum_name(snapshot.source);
    result["plan_distance"]           = snapshot.distance;
    result["predicted_future_ns"]     = snapshot.predicted_future_ns;
    result["aim_phase"]               = magic_enum::enum_name(snapshot.aim_phase);
    result["selected_armor_id"]       = snapshot.selected_armor_id;
    result["rough_selected_armor_id"] = snapshot.rough_selected_armor_id;
    return result;
}

[[nodiscard]] auto to_json_trajectory_plan_sample(const L5::TrajectoryPlanSample& sample) -> json {
    return json{
        {     "yaw",      sample.yaw},
        {   "pitch",    sample.pitch},
        {"distance", sample.distance},
        {     "tof",      sample.tof},
    };
}

[[nodiscard]] auto to_json_weapon_command(const L5::WeaponCommand& cmd) -> json {
    json result{
        {     "timestamp_ns",      cmd.timestamp_ns},
        {"plan_timestamp_ns", cmd.plan_timestamp_ns},
        {         "plan_yaw",          cmd.plan_yaw},
        {       "plan_pitch",        cmd.plan_pitch},
        {    "plan_distance",     cmd.plan_distance},
        {              "yaw",               cmd.yaw},
        {            "pitch",             cmd.pitch},
        {          "yaw_vel",           cmd.yaw_vel},
        {        "pitch_vel",         cmd.pitch_vel},
        {        "yaw_accel",         cmd.yaw_accel},
        {      "pitch_accel",       cmd.pitch_accel},
        {             "fire",              cmd.fire},
        {              "tof",               cmd.tof},
        {         "distance",          cmd.distance},
    };

    if (!cmd.viz_debug) {
        result["viz_debug"] = nullptr;
        return result;
    }

    json reference_plan = json::array();
    for (const auto& sample : cmd.viz_debug->reference_plan) {
        reference_plan.push_back(to_json_trajectory_plan_sample(sample));
    }

    json optimized_plan = json::array();
    for (const auto& sample : cmd.viz_debug->optimized_plan) {
        optimized_plan.push_back(to_json_trajectory_plan_sample(sample));
    }

    result["viz_debug"] = json{
        { "reference_plan",      std::move(reference_plan)},
        { "optimized_plan",      std::move(optimized_plan)},
        {   "center_index",    cmd.viz_debug->center_index},
        {"lookahead_index", cmd.viz_debug->lookahead_index},
    };
    return result;
}

[[nodiscard]] auto to_json_detection_batch(const ArmorDetectionBatch& batch) -> json {
    json detections = json::array();
    for (const auto& det : batch.detections) {
        json corners = json::array();
        for (const auto& corner : det.corners) {
            corners.push_back(to_json_point(corner));
        }

        detections.push_back(
            json{
                {   "corners",               std::move(corners)},
                {      "rect",           to_json_rect(det.rect)},
                {      "name",  magic_enum::enum_name(det.name)},
                {     "color", magic_enum::enum_name(det.color)},
                {      "type",  magic_enum::enum_name(det.type)},
                {"confidence",                   det.confidence},
                {      "area",                         det.area},
        });
    }

    json result{
        {"timestamp_ns",               batch.timestamp_ns},
        {    "frame_id",                   batch.frame_id},
        {  "detections",            std::move(detections)},
        {"detector_roi", to_json_rect(batch.detector_roi)},
    };

    result["has_detector_roi"] = batch.has_detector_roi;

    return result;
}

template <fast_tf::frame Frame>
[[nodiscard]] auto to_json_measurement_batch(const ArmorMeasurementBatchT<Frame>& batch) -> json {
    json measurements = json::array();
    for (const auto& measurement : batch.measurements) {
        measurements.push_back(
            json{
                {            "timestamp_ns",                          measurement.timestamp_ns},
                {                    "name",           magic_enum::enum_name(measurement.name)},
                {                   "color",          magic_enum::enum_name(measurement.color)},
                {                    "type",           magic_enum::enum_name(measurement.type)},
                {              "confidence",                            measurement.confidence},
                {"distance_to_image_center",              measurement.distance_to_image_center},
                {                "position", to_json_vec3(measurement.transform.translation())},
                {              "quaternion",  to_json_quat(measurement.transform.quaternion())},
                {                     "rpy",    to_json_rpy(measurement.transform.euler_rot())},
        });
    }

    return json{
        {"timestamp_ns",      batch.timestamp_ns},
        {    "frame_id",          batch.frame_id},
        {"measurements", std::move(measurements)},
    };
}

[[nodiscard]] auto to_json_rune_observation(const rune::RuneObservation& observation) -> json {
    json positions = json::array();
    for (const auto& position : observation.target_positions_odom) {
        positions.push_back(to_json_vec3(position));
    }

    json quats = json::array();
    for (const auto& quat : observation.target_quats_odom) {
        quats.push_back(to_json_quat(quat));
    }

    return json{
        {         "timestamp_ns",                              observation.timestamp_ns},
        {             "frame_id",                                  observation.frame_id},
        {                "valid",                                     observation.valid},
        {        "r_center_odom", to_json_vec3(observation.r_center_odom.translation())},
        {"target_positions_odom",                                  std::move(positions)},
        {    "target_quats_odom",                                      std::move(quats)},
    };
}

[[nodiscard]] auto to_json_energy_meter_state(const energy_meter::EnergyMeterState& state) -> json {
    return json{
        {  "timestamp_ns",                state.timestamp_ns},
        {"tracking_valid",              state.tracking_valid},
        { "r_center_odom", to_json_vec3(state.r_center_odom)},
        {        "radius",                      state.radius},
        {          "roll",                        state.roll},
        {             "t",                           state.t},
        {      "blade_id",                    state.blade_id},
        {     "direction",                   state.direction},
        {      "position",      to_json_vec3(state.position)},
        {   "is_big_rune",                 state.is_big_rune},
        {   "model_valid",                 state.model_valid},
        {             "a",                           state.a},
        {         "omega",                       state.omega},
        {             "b",                           state.b},
        {     "obs_vaild",                   state.obs_valid},
    };
}

[[nodiscard]] auto to_json_control_transform_snapshot(const core::ControlTransformSnapshot& tf)
    -> json {
    if (!tf.present) {
        return json{
            {"present", false}
        };
    }

    const Eigen::Quaterniond quaternion(
        tf.quaternion[3], tf.quaternion[0], tf.quaternion[1], tf.quaternion[2]);
    return json{
        {    "present",                                            true                       },
        {"translation", json::array({tf.translation[0], tf.translation[1], tf.translation[2]})},
        { "quaternion",
         json{
         {"x", tf.quaternion[0]},
         {"y", tf.quaternion[1]},
         {"z", tf.quaternion[2]},
         {"w", tf.quaternion[3]}}                                                             },
        {        "rpy",                                to_json_rpy(math_fuxk::rpy(quaternion))},
    };
}

[[nodiscard]] auto to_control_transform(const core::ControlTransformSnapshot& tf)
    -> ControlTransformRecord {
    ControlTransformRecord result;
    if (!tf.present) {
        return result;
    }

    result.present     = 1;
    result.translation = {
        static_cast<float>(tf.translation[0]), static_cast<float>(tf.translation[1]),
        static_cast<float>(tf.translation[2])};
    result.quaternion = {
        static_cast<float>(tf.quaternion[0]), static_cast<float>(tf.quaternion[1]),
        static_cast<float>(tf.quaternion[2]), static_cast<float>(tf.quaternion[3])};
    return result;
}

class RuntimeCapturer {
public:
    static auto create(const CapturerConfig& config, const CapturerLaunchContext& launch)
        -> std::expected<std::shared_ptr<RuntimeCapturer>, std::string> {
        std::error_code ec;
        fs::path base_dir(config.output_dir);

        const auto config_source = normalize_path_for_comparison(
            resolve_existing_path("config"), "capturer config source");
        if (!config_source) {
            return std::unexpected(config_source.error());
        }
        const auto output_dir =
            normalize_path_for_comparison(base_dir, "capturer output directory");
        if (!output_dir) {
            return std::unexpected(output_dir.error());
        }
        if (is_same_or_descendant(*output_dir, *config_source)) {
            return std::unexpected(
                fmt::format(
                    "capturer output directory {} must not be inside config snapshot source {}",
                    output_dir->string(), config_source->string()));
        }

        fs::create_directories(base_dir, ec);
        if (ec) {
            return std::unexpected(
                fmt::format("create capturer base dir {}: {}", base_dir.string(), ec.message()));
        }

        const fs::path run_dir = base_dir / fmt::format("run_{}", fcs::clock::now_ns());
        fs::create_directories(run_dir / "images", ec);
        if (ec) {
            return std::unexpected(
                fmt::format("create capturer run dir {}: {}", run_dir.string(), ec.message()));
        }
        fs::create_directories(run_dir / "control", ec);
        if (ec) {
            return std::unexpected(
                fmt::format("create capturer control dir {}: {}", run_dir.string(), ec.message()));
        }
        fs::create_directories(run_dir / "config", ec);
        if (ec) {
            return std::unexpected(
                fmt::format("create capturer config dir {}: {}", run_dir.string(), ec.message()));
        }

        // std::make_shared cannot access private constructor; direct allocation is required.
        auto capturer = std::shared_ptr<RuntimeCapturer>(new RuntimeCapturer(config, run_dir));
        if (!capturer->jsonl_.is_open()) {
            return std::unexpected(
                fmt::format("open snapshots.jsonl: {}", capturer->jsonl_path_.string()));
        }

        auto config_snapshot = capturer->copy_config_snapshot(launch);
        if (!config_snapshot) {
            return std::unexpected(config_snapshot.error());
        }

        json run_json{
            {           "created_ns",fcs::clock::now_ns()                                     },
            {              "run_dir",                run_dir.string()},
            {             "capturer", to_json_capturer_config(config)},
            {      "config_snapshot",                *config_snapshot},
            {"camera_capture_policy",
             json{
             {"idle_hz", 4},
             {"active_hz", 20},
             {"active_reasons", json::array({"fire", "follow"})},
             }                                                       },
            {               "launch",
             json{
             {"backend", magic_enum::enum_name(launch.backend)},
             {"robot", launch.robot},
             {"vision", launch.vision},
             }                                                       },
            {              "control",
             json{
             {"dir", capturer->relative_path_or_null(capturer->control_dir_)},
             {"schema_version", kControlSchemaVersion},
             {"frequency_hz", kControlFrequencyHz},
             {"sample_size", kControlSampleWireSize},
             {"chunk_header_size", kControlChunkHeaderWireSize},
             {"encoding", "little_endian_explicit"},
             }                                                       },
        };

        if (!capturer->write_file(run_dir / "run.json", run_json.dump(2))) {
            return std::unexpected(
                fmt::format("write run metadata: {}", (run_dir / "run.json").string()));
        }

        return capturer;
    }

    void capture_tick(
        uint64_t tick_timestamp_ns, const ChannelSnapshot<ImageFrame>& image,
        const ChannelSnapshot<ArmorDetectionBatch>& detection,
        const ChannelSnapshot<ArmorMeasurementBatch>& measurement,
        const ChannelSnapshot<L3::TrackerOutputs>& tracker,
        const ChannelSnapshot<rune::RuneObservation>& rune_observation,
        const ChannelSnapshot<rune::RuneDebugFrame>& rune_debug,
        const ChannelSnapshot<energy_meter::EnergyMeterState>& energy_meter,
        const ChannelSnapshot<L4::SelectedTargetSnapshot>& selected_target,
        const ChannelSnapshot<L4::TargetSelectionTrace>& target_selection_trace,
        const ChannelSnapshot<L4::ControlIntent>& control_intent,
        const ChannelSnapshot<L5::WeaponCommand>& weapon_command,
        const ChannelSnapshot<core::ControlResourceSnapshot>& control_state) noexcept {
        try {
            const uint64_t tick_index = ++tick_index_;
            const auto sidecar_policy = evaluate_sidecar_policy();

            json snapshot{
                {       "tick_index",            tick_index},
                {"tick_timestamp_ns",     tick_timestamp_ns},
                {   "capture_status", sidecar_policy.status},
            };

            json streams = json::object();

            json camera_json = make_channel_meta(image.present(), image.updated_this_tick);
            if (image.value) {
                camera_json["timestamp_ns"] = image.value->timestamp_ns;
                camera_json["frame_id"]     = image.value->frame_id;
                camera_json["width"]        = image.value->image.cols;
                camera_json["height"]       = image.value->image.rows;
                camera_json["channels"]     = image.value->image.channels();
                camera_json["image_path"] =
                    lookup_sidecar_path(image.value->frame_id, image.value->timestamp_ns);
            }
            streams["camera"] = std::move(camera_json);

            json detection_json =
                make_channel_meta(detection.present(), detection.updated_this_tick);
            if (detection.value) {
                detection_json.update(to_json_detection_batch(*detection.value));
                detection_json["image_path"] = resolve_detection_image_path(*detection.value);
            }
            streams["detector"] = std::move(detection_json);

            json measurement_json =
                make_channel_meta(measurement.present(), measurement.updated_this_tick);
            if (measurement.value) {
                measurement_json.update(to_json_measurement_batch(*measurement.value));
            }
            streams["measurement"] = std::move(measurement_json);

            json tracker_json = make_channel_meta(tracker.present(), tracker.updated_this_tick);
            if (tracker.value) {
                json trackers_array = json::array();
                for (const auto& output : *tracker.value) {
                    trackers_array.push_back(to_json_tracker_output(output));
                }
                tracker_json["trackers"] = std::move(trackers_array);
                tracker_json["count"]    = tracker.value->size();
            }
            streams["tracker"] = std::move(tracker_json);

            json rune_obs_json =
                make_channel_meta(rune_observation.present(), rune_observation.updated_this_tick);
            if (rune_observation.value) {
                rune_obs_json.update(to_json_rune_observation(*rune_observation.value));
            }
            streams["rune_observation"] = std::move(rune_obs_json);

            json rune_debug_json =
                make_channel_meta(rune_debug.present(), rune_debug.updated_this_tick);
            if (rune_debug.value) {
                rune_debug_json["timestamp_ns"]      = rune_debug.value->timestamp_ns;
                rune_debug_json["frame_id"]          = rune_debug.value->frame_id;
                rune_debug_json["detect_reversed"]   = rune_debug.value->detect_reversed;
                rune_debug_json["tf_ok"]             = rune_debug.value->tf_ok;
                rune_debug_json["solve_ok"]          = rune_debug.value->solve_ok;
                rune_debug_json["observation_valid"] = rune_debug.value->observation_valid;
                rune_debug_json["status_code"]       = rune_debug.value->status_code;
                rune_debug_json["arrows_count"]      = rune_debug.value->arrows_count;
                rune_debug_json["targets_count"]     = rune_debug.value->targets_count;
                rune_debug_json["global_roi"]        = to_json_rect(rune_debug.value->global_roi);
                rune_debug_json["center_roi"]        = to_json_rect(rune_debug.value->center_roi);

                json target_rois = json::array();
                for (const auto& roi : rune_debug.value->target_rois) {
                    target_rois.push_back(to_json_rect(roi));
                }
                rune_debug_json["target_rois"]       = std::move(target_rois);
                rune_debug_json["arrow_jpeg_path"]   = nullptr;
                rune_debug_json["target_jpeg_path"]  = nullptr;
                rune_debug_json["rcenter_jpeg_path"] = nullptr;
            }
            streams["rune_debug"] = std::move(rune_debug_json);

            json energy_json =
                make_channel_meta(energy_meter.present(), energy_meter.updated_this_tick);
            if (energy_meter.value) {
                energy_json.update(to_json_energy_meter_state(*energy_meter.value));
            }
            streams["energy_meter"] = std::move(energy_json);

            json selected_target_json =
                make_channel_meta(selected_target.present(), selected_target.updated_this_tick);
            if (selected_target.value) {
                selected_target_json.update(
                    to_json_selected_target_snapshot(*selected_target.value));
            }
            streams["selected_target"] = std::move(selected_target_json);

            json target_selection_json = make_channel_meta(
                target_selection_trace.present(), target_selection_trace.updated_this_tick);
            if (target_selection_trace.value) {
                target_selection_json.update(
                    to_json_target_selection_trace(*target_selection_trace.value));
            }
            streams["target_selection"] = std::move(target_selection_json);

            json gimbal_json =
                make_channel_meta(control_intent.present(), control_intent.updated_this_tick);
            if (control_intent.value) {
                gimbal_json.update(to_json_control_intent(*control_intent.value));
            }
            streams["gimbal_plan"] = std::move(gimbal_json);

            json weapon_json =
                make_channel_meta(weapon_command.present(), weapon_command.updated_this_tick);
            if (weapon_command.value) {
                weapon_json.update(to_json_weapon_command(*weapon_command.value));
            }
            streams["weapon_command"] = std::move(weapon_json);

            snapshot["streams"] = std::move(streams);
            json resources      = json{
                     {          "present",         control_state.present()},
                     {"updated_this_tick", control_state.updated_this_tick},
                     {            "stale",           control_state.stale()},
            };
            json tf = json::object();
            if (control_state.value) {
                const auto& imu                  = control_state.value->imu;
                resources["sample_timestamp_ns"] = control_state.value->sample_timestamp_ns;
                resources["imu_state"]           = json{
                              {     "present",             true},
                              {"timestamp_ns", imu.timestamp_ns},
                              {         "yaw",          imu.yaw},
                              {       "pitch",        imu.pitch},
                              {        "roll",         imu.roll},
                              {     "yaw_vel",      imu.yaw_vel},
                              {   "pitch_vel",    imu.pitch_vel},
                              {    "roll_vel",     imu.roll_vel},
                };
                resources["detecting_color"] =
                    magic_enum::enum_name(control_state.value->detecting_color);
                resources["bullet_speed_raw"] = control_state.value->bullet_speed_raw;
                resources["bullet_speed"]     = control_state.value->bullet_speed;
                tf["odom_to_gimbal_pitch"] =
                    to_json_control_transform_snapshot(control_state.value->odom_to_gimbal_pitch);
                tf["gimbal_to_camera_link"] =
                    to_json_control_transform_snapshot(control_state.value->gimbal_to_camera_link);
                tf["odom_to_camera_optical"] =
                    to_json_control_transform_snapshot(control_state.value->odom_to_camera_optical);
                tf["odom_to_muzzle"] =
                    to_json_control_transform_snapshot(control_state.value->odom_to_muzzle);
            } else {
                resources["imu_state"] = json{
                    {"present", false}
                };
                resources["detecting_color"]  = nullptr;
                resources["bullet_speed_raw"] = nullptr;
                resources["bullet_speed"]     = nullptr;
                tf["odom_to_gimbal_pitch"]    = json{
                       {"present", false}
                };
                tf["gimbal_to_camera_link"] = json{
                    {"present", false}
                };
                tf["odom_to_camera_optical"] = json{
                    {"present", false}
                };
                tf["odom_to_muzzle"] = json{
                    {"present", false}
                };
            }
            snapshot["resources"] = std::move(resources);
            snapshot["tf"]        = std::move(tf);
            snapshot["control_window"] =
                flush_control_window(tick_index, tick_timestamp_ns, sidecar_policy.allow_sidecars);

            jsonl_ << snapshot.dump() << '\n';
            jsonl_.flush();
            if (!jsonl_) {
                jsonl_.clear();
                log_error_throttled(
                    last_write_error_log_ns_, "capturer failed to append snapshot to {}",
                    jsonl_path_.string());
            }
        } catch (const std::exception& e) {
            log_error_throttled(last_write_error_log_ns_, "capturer tick failed: {}", e.what());
        }
    }

    void record_control_sample(
        uint64_t sample_timestamp_ns, const ChannelSnapshot<L4::ControlIntent>& control_intent,
        const ChannelSnapshot<L5::WeaponCommand>& weapon_command,
        const ChannelSnapshot<core::ControlResourceSnapshot>& control_state) noexcept {
        ControlSampleRecord sample;
        sample.sample_timestamp_ns = sample_timestamp_ns;
        if (control_state.value) {
            const auto& imu            = control_state.value->imu;
            sample.sample_timestamp_ns = control_state.value->sample_timestamp_ns;
            sample.imu_timestamp_ns    = imu.timestamp_ns;
            sample.imu_roll            = static_cast<float>(imu.roll);
            sample.imu_pitch           = static_cast<float>(imu.pitch);
            sample.imu_yaw             = static_cast<float>(imu.yaw);
            sample.imu_roll_vel        = static_cast<float>(imu.roll_vel);
            sample.imu_pitch_vel       = static_cast<float>(imu.pitch_vel);
            sample.imu_yaw_vel         = static_cast<float>(imu.yaw_vel);
            sample.bullet_speed_raw    = static_cast<float>(control_state.value->bullet_speed_raw);
            sample.detecting_color     = static_cast<uint8_t>(control_state.value->detecting_color);
            sample.odom_to_gimbal_pitch =
                to_control_transform(control_state.value->odom_to_gimbal_pitch);
            sample.gimbal_to_camera_link =
                to_control_transform(control_state.value->gimbal_to_camera_link);
            sample.odom_to_camera_optical =
                to_control_transform(control_state.value->odom_to_camera_optical);
            sample.odom_to_muzzle = to_control_transform(control_state.value->odom_to_muzzle);
        }

        if (control_intent.value) {
            sample.plan_present      = 1;
            sample.plan_timestamp_ns = [&]() -> uint64_t {
                return std::visit(
                    [](const auto& cmd) -> uint64_t { return cmd.timestamp_ns; },
                    *control_intent.value);
            }();
            // Map variant to plan_source for compat recording.
            sample.plan_source = std::visit(
                overloaded{
                    [](const L4::TrackCommand&) -> uint8_t { return 1; },
                    [](const L4::ShotCommand&) -> uint8_t { return 2; },
                    [](const L4::HoldCommand&) -> uint8_t { return 0; },
                },
                *control_intent.value);

            std::visit(
                overloaded{
                    [&](const L4::TrackCommand& cmd) {
                        const auto aim       = cmd.control_trajectory.center_aim_point();
                        sample.plan_yaw      = static_cast<float>(aim.yaw);
                        sample.plan_pitch    = static_cast<float>(aim.pitch);
                        sample.plan_distance = static_cast<float>(aim.distance);
                    },
                    [&](const L4::ShotCommand& cmd) {
                        sample.plan_yaw      = static_cast<float>(cmd.yaw);
                        sample.plan_pitch    = static_cast<float>(cmd.pitch);
                        sample.plan_distance = static_cast<float>(cmd.distance);
                    },
                    [&](const L4::HoldCommand&) {
                        // No plan data.
                    },
                },
                *control_intent.value);
        }

        if (weapon_command.value) {
            sample.weapon_present           = 1;
            sample.weapon_timestamp_ns      = weapon_command.value->timestamp_ns;
            sample.weapon_plan_timestamp_ns = weapon_command.value->plan_timestamp_ns;
            sample.weapon_plan_yaw          = static_cast<float>(weapon_command.value->plan_yaw);
            sample.weapon_plan_pitch        = static_cast<float>(weapon_command.value->plan_pitch);
            sample.weapon_fire              = weapon_command.value->fire ? 1 : 0;
            sample.weapon_yaw               = static_cast<float>(weapon_command.value->yaw);
            sample.weapon_pitch             = static_cast<float>(weapon_command.value->pitch);
            sample.weapon_yaw_vel           = static_cast<float>(weapon_command.value->yaw_vel);
            sample.weapon_pitch_vel         = static_cast<float>(weapon_command.value->pitch_vel);
            sample.weapon_yaw_accel         = static_cast<float>(weapon_command.value->yaw_accel);
            sample.weapon_pitch_accel       = static_cast<float>(weapon_command.value->pitch_accel);
            sample.weapon_tof               = static_cast<float>(weapon_command.value->tof);
            sample.weapon_distance          = static_cast<float>(weapon_command.value->distance);
        }

        std::scoped_lock lock(control_mutex_);
        pending_control_samples_.push_back(sample);
    }

    void capture_camera_tick(
        uint64_t tick_timestamp_ns, const ChannelSnapshot<ImageFrame>& image,
        const ChannelSnapshot<L5::WeaponCommand>& weapon_command, bool following_active) noexcept {
        if (!image.value) {
            return;
        }

        const auto mode           = evaluate_camera_capture_mode(weapon_command, following_active);
        const auto sidecar_policy = evaluate_sidecar_policy();
        static_cast<void>(capture_camera_sidecar_if_due(
            image.value->image, image.value->frame_id, image.value->timestamp_ns, tick_timestamp_ns,
            mode, sidecar_policy.allow_sidecars));
    }

private:
    RuntimeCapturer(const CapturerConfig& config, fs::path run_dir)
        : run_dir_(std::move(run_dir))
        , images_dir_(run_dir_ / "images")
        , control_dir_(run_dir_ / "control")
        , config_dir_(run_dir_ / "config")
        , jsonl_path_(run_dir_ / "snapshots.jsonl")
        , jsonl_(jsonl_path_, std::ios::out | std::ios::app)
        , reserved_free_bytes_(config.reserved_free_bytes) {}

    struct SidecarPolicy {
        bool allow_sidecars{true};
        std::string_view status{"ok"};
    };

    enum class CameraCaptureMode : uint8_t {
        Idle,
        Follow,
        Fire,
    };

    struct CachedSidecar {
        uint64_t frame_id{0};
        uint64_t timestamp_ns{0};
        std::string relative_path{};

        [[nodiscard]] bool
            matches(uint64_t other_frame_id, uint64_t other_timestamp_ns) const noexcept {
            return !relative_path.empty() && frame_id == other_frame_id
                && timestamp_ns == other_timestamp_ns;
        }
    };

    [[nodiscard]] auto copy_config_snapshot(const CapturerLaunchContext& launch)
        -> std::expected<json, std::string> {
        static_cast<void>(launch);

        const fs::path entry_source  = resolve_existing_path("at_vision.toml");
        const fs::path config_source = resolve_existing_path("config");
        if (!fs::exists(entry_source)) {
            return std::unexpected(
                fmt::format(
                    "capturer config snapshot missing source file {}",
                    entry_source.generic_string()));
        }
        if (!fs::exists(config_source) || !fs::is_directory(config_source)) {
            return std::unexpected(
                fmt::format(
                    "capturer config snapshot missing config directory {}",
                    config_source.generic_string()));
        }

        const fs::path entry_destination = run_dir_ / "at_vision.toml";
        std::error_code ec;
        fs::copy_file(entry_source, entry_destination, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            return std::unexpected(
                fmt::format(
                    "capturer copy config snapshot {} -> {}: {}", entry_source.string(),
                    entry_destination.string(), ec.message()));
        }

        const auto config_copy = copy_directory_tree(config_source, config_dir_);
        if (!config_copy) {
            return std::unexpected(config_copy.error());
        }

        return json{
            {  "entry_source_path",                         "at_vision.toml"},
            {"entry_snapshot_path", relative_path_or_null(entry_destination)},
            {  "config_source_dir",                                 "config"},
            {"config_snapshot_dir",       relative_path_or_null(config_dir_)},
            {       "copied_files",                config_copy->copied_files},
            {        "copied_dirs",          config_copy->copied_directories},
        };
    }

    struct DirectoryCopySummary {
        size_t copied_files{0};
        size_t copied_directories{0};
    };

    [[nodiscard]] auto copy_directory_tree(const fs::path& source_root, const fs::path& dest_root)
        -> std::expected<DirectoryCopySummary, std::string> {
        DirectoryCopySummary summary;
        std::error_code ec;
        fs::create_directories(dest_root, ec);
        if (ec) {
            return std::unexpected(
                fmt::format(
                    "capturer create config snapshot dir {}: {}", dest_root.string(),
                    ec.message()));
        }

        for (const auto& entry : fs::recursive_directory_iterator(source_root, ec)) {
            if (ec) {
                return std::unexpected(
                    fmt::format(
                        "capturer walk config snapshot dir {}: {}", source_root.string(),
                        ec.message()));
            }

            const auto relative = entry.path().lexically_relative(source_root);
            const auto dest     = dest_root / relative;
            if (entry.is_directory()) {
                fs::create_directories(dest, ec);
                if (ec) {
                    return std::unexpected(
                        fmt::format(
                            "capturer create config snapshot dir {}: {}", dest.string(),
                            ec.message()));
                }
                ++summary.copied_directories;
                continue;
            }
            if (!entry.is_regular_file()) {
                continue;
            }

            fs::create_directories(dest.parent_path(), ec);
            if (ec) {
                return std::unexpected(
                    fmt::format(
                        "capturer create config snapshot dir {}: {}", dest.parent_path().string(),
                        ec.message()));
            }

            fs::copy_file(entry.path(), dest, fs::copy_options::overwrite_existing, ec);
            if (ec) {
                return std::unexpected(
                    fmt::format(
                        "capturer copy config snapshot {} -> {}: {}", entry.path().string(),
                        dest.string(), ec.message()));
            }
            ++summary.copied_files;
        }

        return summary;
    }

    [[nodiscard]] auto
        flush_control_window(uint64_t tick_index, uint64_t tick_timestamp_ns, bool allow_sidecars)
            -> json {
        std::vector<ControlSampleRecord> samples;
        {
            std::scoped_lock lock(control_mutex_);
            samples.swap(pending_control_samples_);
        }

        json result{
            {             "path",                     nullptr},
            {   "schema_version",       kControlSchemaVersion},
            {     "frequency_hz",         kControlFrequencyHz},
            {      "sample_size",      kControlSampleWireSize},
            {"chunk_header_size", kControlChunkHeaderWireSize},
            {         "encoding",    "little_endian_explicit"},
            {     "sample_count",              samples.size()},
            {          "written",                       false},
            {          "dropped",                       false},
        };

        if (samples.empty()) {
            return result;
        }

        if (!allow_sidecars) {
            result["dropped"] = true;
            return result;
        }

        const fs::path path =
            control_dir_ / fmt::format("control_{}_{}.bin", tick_index, tick_timestamp_ns);
        const ControlChunkHeaderRecord header{
            .sample_count      = static_cast<uint32_t>(samples.size()),
            .tick_index        = tick_index,
            .tick_timestamp_ns = tick_timestamp_ns,
        };
        std::ofstream out(path, std::ios::out | std::ios::binary | std::ios::trunc);
        if (!out.is_open()) {
            log_error_throttled(
                last_write_error_log_ns_, "capturer failed to open control sidecar {}",
                path.string());
            result["dropped"] = true;
            return result;
        }
        const auto bytes = serialize_control_chunk(header, samples);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        out.flush();
        if (!out) {
            log_error_throttled(
                last_write_error_log_ns_, "capturer failed to write control sidecar {}",
                path.string());
            result["dropped"] = true;
            return result;
        }

        result["path"]    = relative_path_or_null(path);
        result["written"] = true;
        return result;
    }

    [[nodiscard]] auto evaluate_sidecar_policy() -> SidecarPolicy {
        std::error_code ec;
        const auto space = fs::space(run_dir_, ec);
        if (ec) {
            log_error_throttled(
                last_write_error_log_ns_, "capturer failed to query disk space for {}: {}",
                run_dir_.string(), ec.message());
            return SidecarPolicy{.allow_sidecars = false, .status = "space_query_failed"};
        }

        if (space.available <= reserved_free_bytes_) {
            log_error_throttled(
                last_low_disk_log_ns_,
                "capturer disabled sidecar writes because available disk space {} <= reserved {} "
                "under {}",
                space.available, reserved_free_bytes_, run_dir_.string());
            return SidecarPolicy{.allow_sidecars = false, .status = "low_disk"};
        }
        return SidecarPolicy{};
    }

    [[nodiscard]] bool write_file(const fs::path& path, std::string_view contents) {
        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            return false;
        }
        out << contents;
        out.flush();
        return static_cast<bool>(out);
    }

    [[nodiscard]] auto relative_path_or_null(const fs::path& path) const -> json {
        return path.lexically_relative(run_dir_).generic_string();
    }

    [[nodiscard]] auto evaluate_camera_capture_mode(
        const ChannelSnapshot<L5::WeaponCommand>& weapon_command, bool following_active) const
        -> CameraCaptureMode {
        if (weapon_command.value && weapon_command.value->fire) {
            return CameraCaptureMode::Fire;
        }
        if (following_active) {
            return CameraCaptureMode::Follow;
        }
        return CameraCaptureMode::Idle;
    }

    [[nodiscard]] auto lookup_sidecar_path(uint64_t frame_id, uint64_t timestamp_ns) const -> json {
        std::scoped_lock lock(image_mutex_);
        for (auto it = image_sidecars_.rbegin(); it != image_sidecars_.rend(); ++it) {
            if (it->matches(frame_id, timestamp_ns)) {
                return it->relative_path;
            }
        }
        return nullptr;
    }

    void
        remember_sidecar_path(uint64_t frame_id, uint64_t timestamp_ns, std::string relative_path) {
        std::scoped_lock lock(image_mutex_);
        for (auto& cached : image_sidecars_) {
            if (cached.matches(frame_id, timestamp_ns)) {
                cached.relative_path = std::move(relative_path);
                return;
            }
        }

        image_sidecars_.push_back(
            CachedSidecar{
                .frame_id      = frame_id,
                .timestamp_ns  = timestamp_ns,
                .relative_path = std::move(relative_path),
            });
        while (image_sidecars_.size() > kMaxCachedImageSidecars) {
            image_sidecars_.pop_front();
        }
    }

    [[nodiscard]] auto ensure_camera_sidecar(
        const cv::Mat& image, uint64_t frame_id, uint64_t source_timestamp_ns, uint64_t tick_index,
        uint64_t tick_ns, bool allow_sidecars) -> json {
        if (const auto cached = lookup_sidecar_path(frame_id, source_timestamp_ns);
            !cached.is_null()) {
            return cached;
        }
        if (!allow_sidecars || image.empty()) {
            return nullptr;
        }

        auto path = save_mat_as_jpeg(image, "camera", tick_index, tick_ns, source_timestamp_ns);
        if (path.is_string()) {
            remember_sidecar_path(frame_id, source_timestamp_ns, path.get<std::string>());
        }
        return path;
    }

    [[nodiscard]] auto camera_capture_interval_ns(const CameraCaptureMode mode) const noexcept
        -> uint64_t {
        switch (mode) {
        case CameraCaptureMode::Idle: return 250'000'000ULL;
        case CameraCaptureMode::Follow:
        case CameraCaptureMode::Fire: return 50'000'000ULL;
        }
        return 250'000'000ULL;
    }

    [[nodiscard]] auto capture_camera_sidecar_if_due(
        const cv::Mat& image, uint64_t frame_id, uint64_t source_timestamp_ns, uint64_t tick_ns,
        CameraCaptureMode mode, bool allow_sidecars) -> json {
        if (const auto cached = lookup_sidecar_path(frame_id, source_timestamp_ns);
            !cached.is_null()) {
            return cached;
        }
        if (!allow_sidecars || image.empty()) {
            return nullptr;
        }

        {
            std::scoped_lock lock(image_mutex_);
            const auto interval_ns = camera_capture_interval_ns(mode);
            if (last_camera_capture_tick_ns_ != 0
                && (tick_ns - last_camera_capture_tick_ns_) < interval_ns) {
                return nullptr;
            }
            last_camera_capture_tick_ns_ = tick_ns;
        }

        return ensure_camera_sidecar(
            image, frame_id, source_timestamp_ns, 0, tick_ns, allow_sidecars);
    }

    [[nodiscard]] auto resolve_detection_image_path(const ArmorDetectionBatch& batch) const
        -> json {
        if (const auto cached = lookup_sidecar_path(batch.frame_id, batch.timestamp_ns);
            !cached.is_null()) {
            return cached;
        }
        return nullptr;
    }

    [[nodiscard]] auto save_mat_as_jpeg(
        const cv::Mat& image, std::string_view prefix, uint64_t tick_index, uint64_t tick_ns,
        uint64_t source_timestamp_ns) -> json {
        if (image.empty()) {
            return nullptr;
        }

        const fs::path path =
            images_dir_
            / fmt::format("{}_{}_{}_{}.jpg", prefix, tick_index, tick_ns, source_timestamp_ns);
        try {
            std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 90};
            if (!cv::imwrite(path.string(), image, params)) {
                log_error_throttled(
                    last_write_error_log_ns_, "capturer failed to write image {}", path.string());
                return nullptr;
            }
            return relative_path_or_null(path);
        } catch (const std::exception& e) {
            log_error_throttled(
                last_write_error_log_ns_, "capturer failed to write image {}: {}", path.string(),
                e.what());
            return nullptr;
        }
    }

    template <typename... Args>
    void log_error_throttled(
        uint64_t& last_log_ns, fmt::format_string<Args...> fmt_str, Args&&... args) const {
        std::scoped_lock lock(log_mutex_);
        const uint64_t now_ns = fcs::clock::now_ns();
        if (now_ns - last_log_ns < kErrorLogThrottleNs) {
            return;
        }
        last_log_ns = now_ns;
        SPDLOG_ERROR(fmt_str, std::forward<Args>(args)...);
    }

    static constexpr size_t kMaxCachedImageSidecars = 256;

    fs::path run_dir_;
    fs::path images_dir_;
    fs::path control_dir_;
    fs::path config_dir_;
    fs::path jsonl_path_;
    std::ofstream jsonl_;
    uint64_t reserved_free_bytes_{0};
    uint64_t tick_index_{0};
    std::mutex control_mutex_;
    mutable std::mutex image_mutex_;
    mutable std::mutex log_mutex_;
    std::vector<ControlSampleRecord> pending_control_samples_{};
    std::deque<CachedSidecar> image_sidecars_{};
    uint64_t last_camera_capture_tick_ns_{0};
    mutable uint64_t last_low_disk_log_ns_{0};
    mutable uint64_t last_write_error_log_ns_{0};
};

} // namespace

/**
 * @brief 注册运行时数据录制系统 RuntimeCapturer
 * @param scheduler talos调度器
 * @param config 录制器配置：开关、输出目录、录制参数
 * @param launch 启动上下文，存放进程、日志相关上下文信息
 *
 * 功能：整套机器人全链路数据录制，把图像、检测、跟踪、L4瞄准、L5武器指令、控制器状态全部保存落盘。
 * 用于：离线复现bug、离线仿真回放、数据集采集、问题定位。
 *
 * 三个系统分工：
 * 1. runtime_control_recorder @250Hz：高频控制链路采样（控制意图、武器指令、控制器状态）
 * 2. runtime_camera_recorder  @20Hz：图像帧采样，附带武器开火状态、手控跟随标记
 * 3. runtime_capturer         @4Hz：低频全量大包录制，保存感知、跟踪、瞄准全套调试数据
 *
 * 注意标签 talos::fixed_rate_silent：静默定频系统，**超时阻塞不会触发调度器告警**；
 * 录制写磁盘IO容易阻塞，不能用普通fixed_rate，否则调度器报系统超时警告。
 */
void register_runtime_capturer_system(
    talos::Scheduler& scheduler, const CapturerConfig& config,
    const CapturerLaunchContext& launch)
{
    // 如果配置关闭录制，直接返回，不注册任何录制系统
    if (!config.enabled) {
        return;
    }

    // 创建录制器实例，返回std::expected，成功得到RuntimeCapturer对象；失败携带错误字符串
    auto capturer_result = RuntimeCapturer::create(config, launch);
    if (!capturer_result) {
        SPDLOG_ERROR("runtime capturer disabled: {}", capturer_result.error());
        return;
    }

    SPDLOG_INFO("runtime capturer enabled: {}", config.output_dir);

    //================================================================================
    // 系统1：runtime_control_recorder 250Hz 高频控制链路采样
    // talos::fixed_rate_silent<250>：静默定频，写磁盘IO阻塞不会打印调度超时告警
    // 录制对象：L4控制意图、L5武器指令、底层控制器状态；控制环是250Hz，必须高频采样
    //================================================================================
    scheduler.add_system<talos::fixed_rate_silent<250>>(
        "runtime_control_recorder",
        // 捕获capturer实例，lambda内部持有对象拷贝
        [capturer = *capturer_result](
            // 输入SPMC通道：L4输出瞄准意图
            talos::spmc<L4::ControlIntent, ControlIntentChannelTopic> control_intent_in,
            // 输入SPMC通道：L5输出武器指令
            talos::spmc<L5::WeaponCommand, WeaponCommandChannelTopic> weapon_command_in,
            // 输入SPMC通道：底层控制器硬件快照（云台角度、电流、遥控器状态等）
            talos::spmc<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic>
                control_state_in) mutable
        {
            /**
             * sample_channel()：工具函数，从spmc通道采样最新一帧数据
             * 不是阻塞读；没有新消息就返回std::nullopt，录制器内部处理空样本
             */
            capturer->record_control_sample(
                fcs::clock::now_ns(),
                sample_channel(control_intent_in),
                sample_channel(weapon_command_in),
                sample_channel(control_state_in));
        });

    //================================================================================
    // 系统2：runtime_camera_recorder 20Hz 图像帧录制
    // 图像一般20fps，不需要跑250Hz；同时附带武器指令、手控跟随状态标记
    //================================================================================
    scheduler.add_system<talos::fixed_rate_silent<20>>(
        "runtime_camera_recorder",
        [capturer = *capturer_result](
            // 图像输入通道
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            // 武器指令，标记这一帧图像时刻是否允许开火
            talos::spmc<L5::WeaponCommand, WeaponCommandChannelTopic> weapon_command_in,
            // 全局原子标记：是否手控跟随模式
            core::following following) mutable
        {
            capturer->capture_camera_tick(
                fcs::clock::now_ns(),
                sample_channel(image_in),
                sample_channel(weapon_command_in),
                following->load()); // 读取原子布尔，当前是否手控模式
        });

    //================================================================================
    // 系统3：runtime_capturer 4Hz 低频全量数据录制
    // 4Hz，不需要每一帧都保存全套感知数据，降低磁盘IO压力
    // 一次性采样整条链路所有模块输出：图像、检测、测量、跟踪、能量机关、L4/L5输出
    // 完整复现一整帧业务链路所有中间结果，用于离线调试、复现bug
    //================================================================================
    scheduler.add_system<talos::fixed_rate_silent<4>>(
        "runtime_capturer",
        // std::move：把capturer实例所有权移动到这个lambda；前两个系统是拷贝共享，本系统接管原始实例
        [capturer = std::move(*capturer_result)](
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> detection_in,          // L2装甲检测输出
            talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> measurement_in,    // L2 PnP测量结果
            talos::spmc<L3::TrackerOutputs, TrackerOutputChannelTopic> tracker_in,         // L3跟踪器输出
            talos::spmc<rune::RuneObservation, RuneObservationChannelTopic> rune_observation_in, // 能量机关观测
            // 能量机关调试帧：绘制点、线、包围盒，Foxglove可视化用的调试图元数据
            talos::spmc<rune::RuneDebugFrame, RuneDebugFrameChannelTopic> rune_debug_in,

            // 能量机关模块状态：已打多少块、剩余时间、激活状态等业务状态
            talos::spmc<energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> energy_meter_in,

            // L4输出：选中目标快照，保存当前锁定的目标ID、装甲ID、目标位置信息，给ROI、可视化、录制使用
            talos::spmc<L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> selected_target_in,

            // L4输出：目标选择完整Trace追踪日志
            // 记录为什么选这个目标、为什么切换目标、打分、切换裕量；用于调试目标乱跳问题，Foxglove回放看决策过程
            talos::spmc<L4::TargetSelectionTrace, TargetSelectionTraceChannelTopic> target_selection_trace_in,

            // L4瞄准输出：控制意图variant(Track/Shot/Hold)，传给L5武器系统
            talos::spmc<L4::ControlIntent, ControlIntentChannelTopic> control_intent_in,

            // L5武器系统输出：最终武器指令，包含yaw/pitch、开火门、降级原因
            talos::spmc<L5::WeaponCommand, WeaponCommandChannelTopic> weapon_command_in,

            // 底层硬件控制器快照：云台实际角度、遥控器按键、电流、底盘状态等硬件反馈
            talos::spmc<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_state_in) mutable
            {
                // 获取当前系统时间戳（纳秒），这一整个tick的统一时间戳
                const auto tick_timestamp_ns = fcs::clock::now_ns();

                /*
                capturer->capture_tick()：录制器的主录入接口
                sample_channel(xxx_in)：工具函数，读取spmc通道的最新样本
                    ✅通道有新数据：返回std::optional<T>包含最新消息
                    ❌通道没有新消息：返回std::nullopt
                注意：不是阻塞等待，只是“采样此刻最新的一份”，不会卡住系统。

                虽然各个通道产生时间戳各不相同，但录制tick使用调用时刻 tick_timestamp_ns 作为这一批样本的捆绑时间；
                回放的时候，每条样本内部自带原生timestamp_ns，**真正对齐依靠每条消息自己内部的时间戳，不是这个tick时间**。
                */
                // 调用录制器tick接口，一次性采样全部通道，打包写入磁盘
                capturer->capture_tick(
                    tick_timestamp_ns,
                    sample_channel(image_in),                 // 图像帧
                    sample_channel(detection_in),             // L2装甲检测结果
                    sample_channel(measurement_in),           // L2 PnP测量结果
                    sample_channel(tracker_in),               // L3跟踪器输出
                    sample_channel(rune_observation_in),      // 能量机关观测结果
                    sample_channel(rune_debug_in),            // 能量机关调试可视化帧
                    sample_channel(energy_meter_in),          // 能量机关业务状态
                    sample_channel(selected_target_in),       // L4选中目标快照
                    sample_channel(target_selection_trace_in), // L4目标选择决策trace日志
                    sample_channel(control_intent_in),        // L4控制意图
                    sample_channel(weapon_command_in),        // L5武器输出指令
                    sample_channel(control_state_in));        // 底层硬件控制器快照
            });
}

} // namespace fcs::runtime
