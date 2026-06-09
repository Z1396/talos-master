#include "runtime/replay.hpp"

#include "euler.hpp"
#include "shm_client.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace fcs::runtime {
namespace {

namespace fs = std::filesystem;
using json   = nlohmann::json;

constexpr uint32_t kControlChunkMagic    = 0x4C525443U;
constexpr uint16_t kControlSchemaVersion = 2;
constexpr uint32_t kControlHeaderSize    = 32U;
constexpr uint32_t kControlSampleSize    = 281U;
constexpr std::chrono::milliseconds kHeartbeatTick{100};

struct ReplayPaths {
    fs::path run_dir;
    fs::path run_json_path;
    fs::path images_dir;
    fs::path control_dir;
};

struct ControlEvent {
    uint64_t timestamp_ns{0};
    uint64_t imu_timestamp_ns{0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double roll_vel{0.0};
    double pitch_vel{0.0};
    double yaw_vel{0.0};
};

struct ImageEvent {
    fs::path path;
    uint64_t timestamp_ns{0};
};

struct StaticPoseBundle {
    ipc::ShmClient::Pose odom{};
    ipc::ShmClient::Pose camera{};
    ipc::ShmClient::Pose muzzle{};
};

struct ReplayCameraSnapshot {
    std::array<double, 9> camera_matrix{};
    std::array<double, 5> distortion{};
    uint32_t width{0};
    uint32_t height{0};
};

struct ReplayExtrinsicSnapshot {
    std::array<double, 3> camera_translation{};
    std::array<double, 3> camera_rpy_deg{};
    std::array<double, 3> muzzle_translation{};
    std::array<double, 3> muzzle_rpy_deg{};
};

struct ReplayHardwareSnapshot {
    ReplayCameraSnapshot camera{};
    ReplayExtrinsicSnapshot extrinsic{};
};

struct ReplayDataset {
    ReplayPaths paths;
    std::string robot;
    ipc::CameraInfo camera_info{};
    StaticPoseBundle static_poses{};
    std::vector<ControlEvent> control_events;
    std::vector<ImageEvent> image_events;
};

[[nodiscard]] auto format_parse_error(const toml::parse_error& e) -> std::string {
    const auto& src = e.source();
    return fmt::format(
        "{} at {}:{}:{}", e.description(), src.path->data(), src.begin.line, src.begin.column);
}

[[nodiscard]] auto to_rad(const double deg) noexcept -> double {
    return deg * std::numbers::pi_v<double> / 180.0;
}

[[nodiscard]] auto parse_uint64(std::string_view text, std::string_view what) noexcept
    -> std::expected<uint64_t, std::string> {
    uint64_t value       = 0;
    const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (ec != std::errc{} || ptr != text.data() + text.size()) {
        return std::unexpected(fmt::format("invalid {} '{}'", what, text));
    }
    return value;
}

[[nodiscard]] auto parse_size_t(std::string_view text, std::string_view what) noexcept
    -> std::expected<size_t, std::string> {
    auto value = parse_uint64(text, what);
    if (!value) {
        return std::unexpected(value.error());
    }
    return static_cast<size_t>(*value);
}

[[nodiscard]] auto parse_double(std::string_view text, std::string_view what) noexcept
    -> std::expected<double, std::string> {
    std::string owned(text);
    char* end          = nullptr;
    const double value = std::strtod(owned.c_str(), &end);
    if (end == owned.c_str() || end != owned.c_str() + owned.size() || !std::isfinite(value)) {
        return std::unexpected(fmt::format("invalid {} '{}'", what, text));
    }
    return value;
}

[[nodiscard]] auto replay_start_timestamp(const ReplayDataset& dataset)
    -> std::expected<uint64_t, std::string> {
    std::optional<uint64_t> min_timestamp;
    if (!dataset.control_events.empty()) {
        min_timestamp = dataset.control_events.front().timestamp_ns;
    }
    if (!dataset.image_events.empty()) {
        min_timestamp = min_timestamp
                          ? std::min(*min_timestamp, dataset.image_events.front().timestamp_ns)
                          : std::optional<uint64_t>{dataset.image_events.front().timestamp_ns};
    }
    if (!min_timestamp || *min_timestamp == 0) {
        return std::unexpected("replay dataset contains no valid timestamps");
    }
    return *min_timestamp;
}

[[nodiscard]] auto replay_end_timestamp(const ReplayDataset& dataset)
    -> std::expected<uint64_t, std::string> {
    std::optional<uint64_t> max_timestamp;
    if (!dataset.control_events.empty()) {
        max_timestamp = dataset.control_events.back().timestamp_ns;
    }
    if (!dataset.image_events.empty()) {
        max_timestamp = max_timestamp
                          ? std::max(*max_timestamp, dataset.image_events.back().timestamp_ns)
                          : std::optional<uint64_t>{dataset.image_events.back().timestamp_ns};
    }
    if (!max_timestamp || *max_timestamp == 0) {
        return std::unexpected("replay dataset contains no valid end timestamp");
    }
    return *max_timestamp;
}

[[nodiscard]] auto latest_run_dir(const fs::path& base_dir)
    -> std::expected<fs::path, std::string> {
    if (!fs::exists(base_dir) || !fs::is_directory(base_dir)) {
        return std::unexpected(
            fmt::format("replay base directory not found: {}", base_dir.string()));
    }

    std::vector<fs::path> runs;
    for (const auto& entry : fs::directory_iterator(base_dir)) {
        if (!entry.is_directory()) {
            continue;
        }
        const auto name = entry.path().filename().string();
        if (name.rfind("run_", 0) != 0) {
            continue;
        }
        if (fs::exists(entry.path() / "run.json")) {
            runs.push_back(entry.path());
        }
    }

    if (runs.empty()) {
        return std::unexpected(fmt::format("no replay runs found under {}", base_dir.string()));
    }

    std::sort(runs.begin(), runs.end());
    return runs.back();
}

[[nodiscard]] auto resolve_replay_paths(const std::optional<fs::path>& input_path)
    -> std::expected<ReplayPaths, std::string> {
    fs::path run_dir;
    if (!input_path) {
        auto latest = latest_run_dir("record");
        if (!latest) {
            return std::unexpected(latest.error());
        }
        run_dir = *latest;
    } else if (!fs::exists(*input_path)) {
        return std::unexpected(fmt::format("replay path not found: {}", input_path->string()));
    } else if (fs::is_regular_file(*input_path) && input_path->filename() == "run.json") {
        run_dir = input_path->parent_path();
    } else if (fs::is_directory(*input_path) && fs::exists(*input_path / "run.json")) {
        run_dir = *input_path;
    } else if (
        fs::is_directory(*input_path)
        && (input_path->filename() == "images" || input_path->filename() == "control")
        && fs::exists(input_path->parent_path() / "run.json")) {
        run_dir = input_path->parent_path();
    } else {
        return std::unexpected(
            fmt::format(
                "replay path must be a run directory, run.json, images/, or control/ path: {}",
                input_path->string()));
    }

    ReplayPaths paths{
        .run_dir       = run_dir,
        .run_json_path = run_dir / "run.json",
        .images_dir    = run_dir / "images",
        .control_dir   = run_dir / "control",
    };

    if (!fs::exists(paths.run_json_path)) {
        return std::unexpected(fmt::format("missing run.json: {}", paths.run_json_path.string()));
    }
    if (!fs::exists(paths.images_dir) || !fs::is_directory(paths.images_dir)) {
        return std::unexpected(
            fmt::format("missing images directory: {}", paths.images_dir.string()));
    }
    if (!fs::exists(paths.control_dir) || !fs::is_directory(paths.control_dir)) {
        return std::unexpected(
            fmt::format("missing control directory: {}", paths.control_dir.string()));
    }
    return paths;
}

[[nodiscard]] auto read_json_file(const fs::path& path) -> std::expected<json, std::string> {
    try {
        std::ifstream in(path);
        if (!in.is_open()) {
            return std::unexpected(fmt::format("open json file {} failed", path.string()));
        }
        return json::parse(in);
    } catch (const std::exception& e) {
        return std::unexpected(fmt::format("parse json file {}: {}", path.string(), e.what()));
    }
}

[[nodiscard]] auto load_run_robot(const fs::path& run_json_path)
    -> std::expected<std::string, std::string> {
    auto run_json = read_json_file(run_json_path);
    if (!run_json) {
        return std::unexpected(run_json.error());
    }

    try {
        const auto& robot = run_json->at("launch").at("robot");
        if (!robot.is_string()) {
            return std::unexpected(
                fmt::format("run.json {} launch.robot must be a string", run_json_path.string()));
        }
        return robot.get<std::string>();
    } catch (const std::exception& e) {
        return std::unexpected(
            fmt::format("parse run.json {}: {}", run_json_path.string(), e.what()));
    }
}

[[nodiscard]] auto read_required_double_array(
    const toml::table& table, const std::string_view key, const size_t expected_size,
    const fs::path& path) -> std::expected<std::vector<double>, std::string> {
    const auto* node = table.at_path(key).node();
    if (node == nullptr || !node->is_array()) {
        return std::unexpected(
            fmt::format("robot snapshot {} missing array key '{}'", path.string(), key));
    }

    const auto& array = *node->as_array();
    if (array.size() != expected_size) {
        return std::unexpected(
            fmt::format(
                "robot snapshot {} key '{}' expected {} values, got {}", path.string(), key,
                expected_size, array.size()));
    }

    std::vector<double> values;
    values.reserve(expected_size);
    for (size_t i = 0; i < expected_size; ++i) {
        if (const auto value = array[i].value<double>()) {
            values.push_back(*value);
            continue;
        }
        if (const auto value = array[i].value<int64_t>()) {
            values.push_back(static_cast<double>(*value));
            continue;
        }
        return std::unexpected(
            fmt::format(
                "robot snapshot {} key '{}' element {} must be numeric", path.string(), key, i));
    }
    return values;
}

[[nodiscard]] auto
    read_required_u32(const toml::table& table, const std::string_view key, const fs::path& path)
        -> std::expected<uint32_t, std::string> {
    if (const auto value = table.at_path(key).value<int64_t>()) {
        if (*value < 0) {
            return std::unexpected(
                fmt::format("robot snapshot {} key '{}' must be non-negative", path.string(), key));
        }
        return static_cast<uint32_t>(*value);
    }
    return std::unexpected(
        fmt::format("robot snapshot {} missing integer key '{}'", path.string(), key));
}

[[nodiscard]] auto read_optional_double(
    const toml::table& table, const std::string_view key, const double fallback) noexcept
    -> double {
    if (const auto value = table.at_path(key).value<double>()) {
        return *value;
    }
    if (const auto value = table.at_path(key).value<int64_t>()) {
        return static_cast<double>(*value);
    }
    return fallback;
}

[[nodiscard]] auto load_hardware_snapshot(const fs::path& robot_config_path)
    -> std::expected<ReplayHardwareSnapshot, std::string> {
    try {
        auto parsed = toml::parse_file(robot_config_path.string());
        if (parsed.failed()) {
            return std::unexpected(
                fmt::format(
                    "{}: {}", robot_config_path.string(), format_parse_error(parsed.error())));
        }

        const auto& root         = parsed.table();
        const auto* camera_table = root.get_as<toml::table>("camera");
        if (camera_table == nullptr) {
            return std::unexpected(
                fmt::format(
                    "robot snapshot {} missing [camera] table", robot_config_path.string()));
        }

        const auto* pitch_table = root.at_path("extrinsic.gimbal_yaw.gimbal_pitch").as_table();
        if (pitch_table == nullptr) {
            return std::unexpected(
                fmt::format(
                    "robot snapshot {} missing [extrinsic.gimbal_yaw.gimbal_pitch] table",
                    robot_config_path.string()));
        }

        ReplayHardwareSnapshot snapshot;

        const auto camera_matrix =
            read_required_double_array(*camera_table, "camera_matrix", 9, robot_config_path);
        if (!camera_matrix) {
            return std::unexpected(camera_matrix.error());
        }
        std::copy(
            camera_matrix->begin(), camera_matrix->end(), snapshot.camera.camera_matrix.begin());

        const auto distortion =
            read_required_double_array(*camera_table, "distort_coefficient", 5, robot_config_path);
        if (!distortion) {
            return std::unexpected(distortion.error());
        }
        std::copy(distortion->begin(), distortion->end(), snapshot.camera.distortion.begin());

        const auto width = read_required_u32(*camera_table, "width", robot_config_path);
        if (!width) {
            return std::unexpected(width.error());
        }
        snapshot.camera.width = *width;

        const auto height = read_required_u32(*camera_table, "height", robot_config_path);
        if (!height) {
            return std::unexpected(height.error());
        }
        snapshot.camera.height = *height;

        const auto camera_translation = read_required_double_array(
            *pitch_table, "camera_link.translation", 3, robot_config_path);
        if (!camera_translation) {
            return std::unexpected(camera_translation.error());
        }
        std::copy(
            camera_translation->begin(), camera_translation->end(),
            snapshot.extrinsic.camera_translation.begin());
        snapshot.extrinsic.camera_rpy_deg = {
            read_optional_double(*pitch_table, "camera_link.roll", 0.0),
            read_optional_double(*pitch_table, "camera_link.pitch", 0.0),
            read_optional_double(*pitch_table, "camera_link.yaw", 0.0),
        };

        const auto muzzle_translation = read_required_double_array(
            *pitch_table, "muzzle_link.translation", 3, robot_config_path);
        if (!muzzle_translation) {
            return std::unexpected(muzzle_translation.error());
        }
        std::copy(
            muzzle_translation->begin(), muzzle_translation->end(),
            snapshot.extrinsic.muzzle_translation.begin());
        snapshot.extrinsic.muzzle_rpy_deg = {
            read_optional_double(*pitch_table, "muzzle_link.roll", 0.0),
            read_optional_double(*pitch_table, "muzzle_link.pitch", 0.0),
            read_optional_double(*pitch_table, "muzzle_link.yaw", 0.0),
        };

        return snapshot;
    } catch (const std::exception& e) {
        return std::unexpected(
            fmt::format("load hardware snapshot {}: {}", robot_config_path.string(), e.what()));
    }
}

[[nodiscard]] auto make_camera_info(const ReplayHardwareSnapshot& hardware) noexcept
    -> ipc::CameraInfo {
    ipc::CameraInfo info{};
    info.timestamp_ns  = 1;
    info.fx            = hardware.camera.camera_matrix[0];
    info.fy            = hardware.camera.camera_matrix[4];
    info.cx            = hardware.camera.camera_matrix[2];
    info.cy            = hardware.camera.camera_matrix[5];
    info.distortion[0] = hardware.camera.distortion[0];
    info.distortion[1] = hardware.camera.distortion[1];
    info.distortion[2] = hardware.camera.distortion[2];
    info.distortion[3] = hardware.camera.distortion[3];
    info.distortion[4] = hardware.camera.distortion[4];
    info.width         = hardware.camera.width;
    info.height        = hardware.camera.height;
    return info;
}

[[nodiscard]] auto make_static_pose_bundle(const ReplayHardwareSnapshot& hardware) noexcept
    -> StaticPoseBundle {
    StaticPoseBundle bundle;

    bundle.odom = ipc::ShmClient::Pose{
        .x            = 0.0,
        .y            = 0.0,
        .z            = 0.0,
        .qw           = 1.0,
        .qx           = 0.0,
        .qy           = 0.0,
        .qz           = 0.0,
        .frame_seq    = 0,
        .timestamp_ns = 1,
    };

    const auto camera_q = math_fuxk::rpy(
                              to_rad(hardware.extrinsic.camera_rpy_deg[0]),
                              to_rad(hardware.extrinsic.camera_rpy_deg[1]),
                              to_rad(hardware.extrinsic.camera_rpy_deg[2]))
                              .quat();
    bundle.camera = ipc::ShmClient::Pose{
        .x            = hardware.extrinsic.camera_translation[0],
        .y            = hardware.extrinsic.camera_translation[1],
        .z            = hardware.extrinsic.camera_translation[2],
        .qw           = camera_q.w(),
        .qx           = camera_q.x(),
        .qy           = camera_q.y(),
        .qz           = camera_q.z(),
        .frame_seq    = 0,
        .timestamp_ns = 1,
    };

    const auto muzzle_q = math_fuxk::rpy(
                              to_rad(hardware.extrinsic.muzzle_rpy_deg[0]),
                              to_rad(hardware.extrinsic.muzzle_rpy_deg[1]),
                              to_rad(hardware.extrinsic.muzzle_rpy_deg[2]))
                              .quat();
    bundle.muzzle = ipc::ShmClient::Pose{
        .x            = hardware.extrinsic.muzzle_translation[0],
        .y            = hardware.extrinsic.muzzle_translation[1],
        .z            = hardware.extrinsic.muzzle_translation[2],
        .qw           = muzzle_q.w(),
        .qx           = muzzle_q.x(),
        .qy           = muzzle_q.y(),
        .qz           = muzzle_q.z(),
        .frame_seq    = 0,
        .timestamp_ns = 1,
    };

    return bundle;
}

template <typename UInt>
[[nodiscard]] auto read_le_uint(std::span<const char> bytes, const size_t offset) -> UInt {
    static_assert(std::is_unsigned_v<UInt>);
    UInt value = 0;
    for (size_t i = 0; i < sizeof(UInt); ++i) {
        value |=
            (static_cast<UInt>(static_cast<unsigned char>(bytes[offset + i]))
             << static_cast<unsigned>(i * 8U));
    }
    return value;
}

[[nodiscard]] auto read_le_float32(std::span<const char> bytes, const size_t offset) -> float {
    const auto raw = read_le_uint<uint32_t>(bytes, offset);
    return std::bit_cast<float>(raw);
}

[[nodiscard]] auto read_binary_file(const fs::path& path)
    -> std::expected<std::vector<char>, std::string> {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        return std::unexpected(fmt::format("open binary file {} failed", path.string()));
    }

    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    if (size < 0) {
        return std::unexpected(fmt::format("read binary file size {} failed", path.string()));
    }
    in.seekg(0, std::ios::beg);

    std::vector<char> bytes(static_cast<size_t>(size));
    if (!bytes.empty()) {
        in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    if (!in) {
        return std::unexpected(fmt::format("read binary file {} failed", path.string()));
    }
    return bytes;
}

[[nodiscard]] auto load_control_events(const fs::path& control_dir)
    -> std::expected<std::vector<ControlEvent>, std::string> {
    constexpr size_t kSkippedControlLogLimit = 8;

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(control_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".bin") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());

    std::vector<ControlEvent> events;
    size_t empty_file_count         = 0;
    size_t truncated_file_count     = 0;
    size_t size_mismatch_file_count = 0;
    size_t skipped_log_count        = 0;

    const auto log_skipped_control = [&](std::string message) {
        if (skipped_log_count < kSkippedControlLogLimit) {
            SPDLOG_WARN("{}", message);
        }
        ++skipped_log_count;
    };

    for (const auto& path : files) {
        auto bytes_result = read_binary_file(path);
        if (!bytes_result) {
            return std::unexpected(bytes_result.error());
        }

        const auto& bytes = *bytes_result;
        if (bytes.empty()) {
            ++empty_file_count;
            log_skipped_control(
                fmt::format("talos-replay ignoring empty control file {}", path.string()));
            continue;
        }
        if (bytes.size() < kControlHeaderSize) {
            ++truncated_file_count;
            log_skipped_control(
                fmt::format(
                    "talos-replay ignoring truncated control file {}: {} bytes", path.string(),
                    bytes.size()));
            continue;
        }

        const std::span<const char> span(bytes.data(), bytes.size());
        const auto magic          = read_le_uint<uint32_t>(span, 0);
        const auto schema_version = read_le_uint<uint16_t>(span, 4);
        const auto header_size    = read_le_uint<uint16_t>(span, 6);
        const auto sample_size    = read_le_uint<uint32_t>(span, 8);
        const auto sample_count   = read_le_uint<uint32_t>(span, 12);

        if (magic != kControlChunkMagic) {
            return std::unexpected(
                fmt::format("control file {} has invalid magic {:#x}", path.string(), magic));
        }
        if (schema_version != kControlSchemaVersion) {
            return std::unexpected(
                fmt::format(
                    "control file {} has unsupported schema_version {}", path.string(),
                    schema_version));
        }
        if (header_size != kControlHeaderSize || sample_size != kControlSampleSize) {
            return std::unexpected(
                fmt::format(
                    "control file {} has unsupported layout header_size={} sample_size={}",
                    path.string(), header_size, sample_size));
        }
        const auto expected_size =
            static_cast<size_t>(header_size)
            + (static_cast<size_t>(sample_count) * static_cast<size_t>(sample_size));
        if (bytes.size() != expected_size) {
            ++size_mismatch_file_count;
            log_skipped_control(
                fmt::format(
                    "talos-replay ignoring size-mismatched control file {}: expected {} bytes, got "
                    "{}",
                    path.string(), expected_size, bytes.size()));
            continue;
        }

        for (size_t i = 0; i < sample_count; ++i) {
            const auto sample_offset = static_cast<size_t>(header_size) + (i * sample_size);
            const auto sample_ts     = read_le_uint<uint64_t>(span, sample_offset);
            const auto imu_ts        = read_le_uint<uint64_t>(span, sample_offset + 8);
            if (imu_ts == 0) {
                continue;
            }

            ControlEvent event;
            event.timestamp_ns     = sample_ts != 0 ? sample_ts : imu_ts;
            event.imu_timestamp_ns = imu_ts;
            event.roll             = static_cast<double>(read_le_float32(span, sample_offset + 16));
            event.pitch            = static_cast<double>(read_le_float32(span, sample_offset + 20));
            event.yaw              = static_cast<double>(read_le_float32(span, sample_offset + 24));
            event.roll_vel         = static_cast<double>(read_le_float32(span, sample_offset + 28));
            event.pitch_vel        = static_cast<double>(read_le_float32(span, sample_offset + 32));
            event.yaw_vel          = static_cast<double>(read_le_float32(span, sample_offset + 36));
            events.push_back(event);
        }
    }

    if (skipped_log_count > kSkippedControlLogLimit) {
        SPDLOG_WARN(
            "talos-replay skipped {} additional malformed control files "
            "(empty={}, truncated={}, size_mismatch={})",
            skipped_log_count - kSkippedControlLogLimit, empty_file_count, truncated_file_count,
            size_mismatch_file_count);
    } else if (skipped_log_count > 0) {
        SPDLOG_WARN(
            "talos-replay skipped malformed control files "
            "(empty={}, truncated={}, size_mismatch={})",
            empty_file_count, truncated_file_count, size_mismatch_file_count);
    }

    std::sort(events.begin(), events.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp_ns < rhs.timestamp_ns;
    });

    if (events.empty()) {
        return std::unexpected(
            fmt::format("no valid imu samples found under {}", control_dir.string()));
    }
    return events;
}

[[nodiscard]] auto parse_image_timestamp(const fs::path& image_path)
    -> std::expected<uint64_t, std::string> {
    const auto stem            = image_path.stem().string();
    const auto last_underscore = stem.rfind('_');
    if (last_underscore == std::string::npos || last_underscore + 1 >= stem.size()) {
        return std::unexpected(
            fmt::format(
                "image filename does not contain source timestamp: {}", image_path.string()));
    }
    return parse_uint64(std::string_view(stem).substr(last_underscore + 1), "image timestamp");
}

[[nodiscard]] auto load_image_events(const fs::path& images_dir)
    -> std::expected<std::vector<ImageEvent>, std::string> {
    std::vector<ImageEvent> events;
    for (const auto& entry : fs::directory_iterator(images_dir)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".jpg") {
            continue;
        }
        const auto name = entry.path().filename().string();
        if (name.rfind("camera_", 0) != 0) {
            continue;
        }
        auto timestamp = parse_image_timestamp(entry.path());
        if (!timestamp) {
            return std::unexpected(timestamp.error());
        }
        events.push_back(ImageEvent{.path = entry.path(), .timestamp_ns = *timestamp});
    }

    std::sort(events.begin(), events.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp_ns < rhs.timestamp_ns;
    });

    if (events.empty()) {
        return std::unexpected(
            fmt::format("no camera replay images found under {}", images_dir.string()));
    }
    return events;
}

[[nodiscard]] auto load_replay_dataset(const ReplayPaths& paths)
    -> std::expected<ReplayDataset, std::string> {
    auto robot_result = load_run_robot(paths.run_json_path);
    if (!robot_result) {
        return std::unexpected(robot_result.error());
    }

    const fs::path robot_config_path =
        paths.run_dir / "config" / "robot" / fmt::format("{}.toml", *robot_result);
    if (!fs::exists(robot_config_path)) {
        return std::unexpected(
            fmt::format("replay robot snapshot not found: {}", robot_config_path.string()));
    }

    auto hardware_result = load_hardware_snapshot(robot_config_path);
    if (!hardware_result) {
        return std::unexpected(hardware_result.error());
    }

    auto control_result = load_control_events(paths.control_dir);
    if (!control_result) {
        return std::unexpected(control_result.error());
    }

    auto image_result = load_image_events(paths.images_dir);
    if (!image_result) {
        return std::unexpected(image_result.error());
    }

    return ReplayDataset{
        .paths          = paths,
        .robot          = *robot_result,
        .camera_info    = make_camera_info(*hardware_result),
        .static_poses   = make_static_pose_bundle(*hardware_result),
        .control_events = std::move(*control_result),
        .image_events   = std::move(*image_result),
    };
}

void sleep_with_heartbeat(
    const ipc::ShmClient& client, const std::chrono::steady_clock::time_point deadline) noexcept {
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return;
        }

        client.update_heartbeat();
        const auto remaining = deadline - now;
        std::this_thread::sleep_for(
            std::min(
                remaining,
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(kHeartbeatTick)));
    }
}

void publish_static_poses(
    ipc::ShmClient& client, const ReplayDataset& dataset, const uint64_t timestamp_ns) {
    auto odom   = dataset.static_poses.odom;
    auto camera = dataset.static_poses.camera;
    auto muzzle = dataset.static_poses.muzzle;

    odom.timestamp_ns   = timestamp_ns;
    camera.timestamp_ns = timestamp_ns;
    muzzle.timestamp_ns = timestamp_ns;

    client.publish_pose(ipc::POSE_ODOM, odom);
    client.publish_pose(ipc::POSE_CAMERA, camera);
    client.publish_pose(ipc::POSE_MUZZLE, muzzle);
}

void publish_control_pose(
    ipc::ShmClient& client, const ControlEvent& event, const uint64_t frame_seq,
    const uint64_t timestamp_ns) {
    const auto quaternion = math_fuxk::rpy(event.roll, event.pitch, event.yaw).quat();
    client.publish_pose(
        ipc::POSE_GIMBAL, ipc::ShmClient::Pose{
                              .x            = 0.0,
                              .y            = 0.0,
                              .z            = 0.0,
                              .qw           = quaternion.w(),
                              .qx           = quaternion.x(),
                              .qy           = quaternion.y(),
                              .qz           = quaternion.z(),
                              .frame_seq    = frame_seq,
                              .timestamp_ns = timestamp_ns,
                          });
}

void publish_control_event(
    ipc::ShmClient& client, const ControlEvent& event, const uint64_t frame_seq) {
    publish_control_pose(
        client, event, frame_seq,
        event.imu_timestamp_ns != 0 ? event.imu_timestamp_ns : event.timestamp_ns);
}

[[nodiscard]] auto load_replay_image_rgb(const fs::path& path)
    -> std::expected<cv::Mat, std::string> {
    const cv::Mat bgr = cv::imread(path.string(), cv::IMREAD_COLOR);
    if (bgr.empty()) {
        return std::unexpected(fmt::format("load replay image {} failed", path.string()));
    }

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

[[nodiscard]] auto scaled_offset(
    const uint64_t base_timestamp_ns, const uint64_t event_timestamp_ns, const double speed)
    -> std::chrono::steady_clock::duration {
    const long double delta_ns = static_cast<long double>(event_timestamp_ns - base_timestamp_ns)
                               / static_cast<long double>(speed);
    const auto duration = std::chrono::duration<long double, std::nano>(delta_ns);
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(duration);
}

[[nodiscard]] auto run_replay_cycle(
    ipc::ShmClient& producer, const ReplayDataset& dataset, const ReplayOptions& options,
    uint64_t& next_frame_seq) -> std::expected<size_t, std::string> {
    constexpr auto kProgressInterval = std::chrono::seconds(1);

    auto start_timestamp = replay_start_timestamp(dataset);
    if (!start_timestamp) {
        return std::unexpected(start_timestamp.error());
    }
    auto end_timestamp = replay_end_timestamp(dataset);
    if (!end_timestamp) {
        return std::unexpected(end_timestamp.error());
    }

    const auto startup_offset_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(options.startup_offset).count());
    const uint64_t cycle_start_timestamp = *start_timestamp + startup_offset_ns;
    if (cycle_start_timestamp > *end_timestamp) {
        return std::unexpected(
            fmt::format(
                "startup offset {}ms is past replay end ts={}", options.startup_offset.count(),
                *end_timestamp));
    }

    const auto control_begin = std::lower_bound(
        dataset.control_events.begin(), dataset.control_events.end(), cycle_start_timestamp,
        [](const ControlEvent& event, const uint64_t timestamp_ns) {
            return event.timestamp_ns < timestamp_ns;
        });
    const auto image_begin = std::lower_bound(
        dataset.image_events.begin(), dataset.image_events.end(), cycle_start_timestamp,
        [](const ImageEvent& event, const uint64_t timestamp_ns) {
            return event.timestamp_ns < timestamp_ns;
        });

    const size_t start_control_index =
        static_cast<size_t>(std::distance(dataset.control_events.begin(), control_begin));
    const size_t start_image_index =
        static_cast<size_t>(std::distance(dataset.image_events.begin(), image_begin));

    if (start_image_index >= dataset.image_events.size()) {
        return std::unexpected(
            fmt::format(
                "startup offset {}ms leaves no images to replay under {}",
                options.startup_offset.count(), dataset.paths.images_dir.string()));
    }

    const size_t remaining_controls = dataset.control_events.size() - start_control_index;
    const size_t remaining_images   = dataset.image_events.size() - start_image_index;
    const size_t target_image_count =
        options.max_images ? std::min(*options.max_images, remaining_images) : remaining_images;
    const uint64_t progress_end_timestamp =
        options.max_images
            ? dataset.image_events[start_image_index + target_image_count - 1].timestamp_ns
            : *end_timestamp;

    producer.publish_camera_info(dataset.camera_info);
    publish_static_poses(producer, dataset, cycle_start_timestamp);
    producer.publish_runtime_state(false, cycle_start_timestamp);
    if (start_control_index > 0) {
        publish_control_pose(
            producer, dataset.control_events[start_control_index - 1], 0, cycle_start_timestamp);
    }
    producer.update_heartbeat();

    SPDLOG_INFO(
        "talos-replay cycle start: startup_offset={}ms start_control={} start_image={} "
        "remaining_controls={} target_images={} start_ts={} end_ts={}",
        options.startup_offset.count(), start_control_index, start_image_index, remaining_controls,
        target_image_count, cycle_start_timestamp, progress_end_timestamp);

    const auto steady_start = std::chrono::steady_clock::now();
    auto next_progress_log  = steady_start + kProgressInterval;
    size_t control_index    = start_control_index;
    size_t image_index      = start_image_index;
    size_t published_images = 0;
    uint64_t last_frame_seq = 0;
    uint64_t last_event_ts  = cycle_start_timestamp;

    auto maybe_log_progress = [&](const uint64_t current_ts, const bool force) {
        const auto now = std::chrono::steady_clock::now();
        if (!force && now < next_progress_log) {
            return;
        }
        next_progress_log = now + kProgressInterval;

        const size_t processed_controls = control_index - start_control_index;
        const double image_progress_pct = target_image_count > 0
                                            ? (100.0 * static_cast<double>(published_images))
                                                  / static_cast<double>(target_image_count)
                                            : 100.0;
        const double timeline_progress_pct =
            progress_end_timestamp > cycle_start_timestamp
                ? (100.0 * static_cast<double>(current_ts - cycle_start_timestamp))
                      / static_cast<double>(progress_end_timestamp - cycle_start_timestamp)
                : 100.0;

        SPDLOG_INFO(
            "talos-replay progress images={}/{} ({:.1f}%) controls={}/{} timeline={:.1f}% ts={}",
            published_images, target_image_count, image_progress_pct, processed_controls,
            remaining_controls, std::clamp(timeline_progress_pct, 0.0, 100.0), current_ts);
    };

    while (control_index < dataset.control_events.size()
           || image_index < dataset.image_events.size()) {
        const auto next_control_ts =
            control_index < dataset.control_events.size()
                ? std::optional<uint64_t>{dataset.control_events[control_index].timestamp_ns}
                : std::nullopt;
        const auto next_image_ts =
            image_index < dataset.image_events.size()
                ? std::optional<uint64_t>{dataset.image_events[image_index].timestamp_ns}
                : std::nullopt;
        const auto next_ts = [&]() -> uint64_t {
            if (next_control_ts && next_image_ts) {
                return std::min(*next_control_ts, *next_image_ts);
            }
            if (next_control_ts) {
                return *next_control_ts;
            }
            return *next_image_ts;
        }();

        sleep_with_heartbeat(
            producer, steady_start + scaled_offset(cycle_start_timestamp, next_ts, options.speed));

        while (control_index < dataset.control_events.size()
               && dataset.control_events[control_index].timestamp_ns == next_ts) {
            publish_control_event(producer, dataset.control_events[control_index], last_frame_seq);
            ++control_index;
        }

        while (image_index < dataset.image_events.size()
               && dataset.image_events[image_index].timestamp_ns == next_ts) {
            auto image_result = load_replay_image_rgb(dataset.image_events[image_index].path);
            if (!image_result) {
                return std::unexpected(image_result.error());
            }

            producer.publish_image(
                *image_result, next_frame_seq, dataset.image_events[image_index].timestamp_ns);
            last_frame_seq = next_frame_seq;
            ++next_frame_seq;
            ++image_index;
            ++published_images;
            last_event_ts = dataset.image_events[image_index - 1].timestamp_ns;

            if (options.max_images && published_images >= *options.max_images) {
                maybe_log_progress(last_event_ts, true);
                producer.update_heartbeat();
                return published_images;
            }
        }

        last_event_ts = next_ts;
        maybe_log_progress(last_event_ts, false);
        producer.update_heartbeat();
    }

    maybe_log_progress(last_event_ts, true);
    return published_images;
}

} // namespace

auto replay_usage(const std::string_view program) -> std::string {
    return fmt::format(
        "usage: {} [run_dir|run.json|images_dir] [--loop] [--speed <factor>] "
        "[--startup-offset-ms <ms>] [--max-images <count>]",
        program);
}

auto parse_replay_options(const int argc, char** argv) noexcept
    -> std::expected<ReplayOptions, std::string> {
    try {
        ReplayOptions options;
        bool positional_consumed = false;

        for (int i = 1; i < argc; ++i) {
            const std::string_view arg(argv[i]);
            if (arg == "--loop") {
                options.loop = true;
                continue;
            }
            if (arg == "--help" || arg == "-h") {
                return std::unexpected(replay_usage(argc > 0 ? argv[0] : "talos-replay"));
            }

            auto read_value = [&](std::string_view flag)
                -> std::optional<std::expected<std::string_view, std::string>> {
                if (arg.starts_with(flag) && arg.size() > flag.size() && arg[flag.size()] == '=') {
                    return std::expected<std::string_view, std::string>(
                        arg.substr(flag.size() + 1));
                }
                if (arg == flag) {
                    if (i + 1 >= argc) {
                        return std::expected<std::string_view, std::string>(
                            std::unexpected(fmt::format("missing value for {}", flag)));
                    }
                    ++i;
                    return std::expected<std::string_view, std::string>(std::string_view(argv[i]));
                }
                return std::nullopt;
            };

            if (auto speed_value = read_value("--speed")) {
                if (!speed_value->has_value()) {
                    return std::unexpected(speed_value->error());
                }
                auto speed = parse_double(**speed_value, "--speed");
                if (!speed) {
                    return std::unexpected(speed.error());
                }
                options.speed = *speed;
                continue;
            }
            if (auto offset_value = read_value("--startup-offset-ms")) {
                if (!offset_value->has_value()) {
                    return std::unexpected(offset_value->error());
                }
                auto offset = parse_uint64(**offset_value, "--startup-offset-ms");
                if (!offset) {
                    return std::unexpected(offset.error());
                }
                options.startup_offset = std::chrono::milliseconds(*offset);
                continue;
            }
            if (auto max_images_value = read_value("--max-images")) {
                if (!max_images_value->has_value()) {
                    return std::unexpected(max_images_value->error());
                }
                auto max_images = parse_size_t(**max_images_value, "--max-images");
                if (!max_images) {
                    return std::unexpected(max_images.error());
                }
                options.max_images = *max_images;
                continue;
            }

            if (arg.starts_with("--")) {
                return std::unexpected(fmt::format("unknown option '{}'", arg));
            }
            if (positional_consumed) {
                return std::unexpected("talos-replay accepts at most one input path");
            }

            positional_consumed = true;
            options.input_path  = fs::path(arg);
        }

        if (!(options.speed > 0.0) || !std::isfinite(options.speed)) {
            return std::unexpected("--speed must be a positive finite number");
        }

        return options;
    } catch (const std::exception& e) {
        return std::unexpected(fmt::format("parse replay options: {}", e.what()));
    }
}

auto run_replay(const ReplayOptions& options) noexcept -> std::expected<void, std::string> {
    try {
        auto paths = resolve_replay_paths(options.input_path);
        if (!paths) {
            return std::unexpected(paths.error());
        }

        auto dataset = load_replay_dataset(*paths);
        if (!dataset) {
            return std::unexpected(dataset.error());
        }

        SPDLOG_INFO(
            "talos-replay loaded run={} robot={} images={} imu_samples={} speed={}x loop={} "
            "startup_offset={}ms",
            dataset->paths.run_dir.string(), dataset->robot, dataset->image_events.size(),
            dataset->control_events.size(), options.speed, options.loop,
            options.startup_offset.count());
        SPDLOG_INFO("talos-replay publishes camera + imu via daedalus shm and ignores commands");

        auto producer_result = ipc::ShmClient::create();
        if (!producer_result) {
            return std::unexpected(
                fmt::format("create daedalus shm producer: {}", producer_result.error()));
        }
        auto producer           = std::move(*producer_result);
        size_t total_images     = 0;
        uint64_t next_frame_seq = 1;
        do {
            ReplayOptions cycle_options = options;
            if (options.max_images) {
                if (total_images >= *options.max_images) {
                    break;
                }
                cycle_options.max_images = *options.max_images - total_images;
            }

            auto cycle = run_replay_cycle(producer, *dataset, cycle_options, next_frame_seq);
            if (!cycle) {
                return std::unexpected(cycle.error());
            }
            total_images += *cycle;
            SPDLOG_INFO("talos-replay cycle finished: published {} images", *cycle);
        } while (options.loop && (!options.max_images || total_images < *options.max_images));

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(fmt::format("run replay: {}", e.what()));
    }
}

} // namespace fcs::runtime
