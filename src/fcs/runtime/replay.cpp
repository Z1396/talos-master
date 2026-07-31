/**
 * @file replay.cpp
 * @brief 回放系统实现 - 从录制数据中重放图像与IMU控制事件
 *
 * 本文件实现了Talos火控系统的数据回放功能，支持：
 * - 从record目录加载录制数据（图像、IMU数据、相机标定信息）
 * - 按时间戳同步重放图像和控制事件
 * - 通过共享内存（Daedalus SHM）向下游系统发布数据
 * - 支持回放速度调节、启动偏移、循环播放等功能
 *
 * 核心数据流：
 * 录制目录 → 解析元数据 → 加载硬件快照 → 加载控制事件 → 加载图像事件
 * → 按时间戳同步发布到共享内存 → 下游系统消费
 *
 * @author Talos Team
 * @date 2024
 */

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

/// 控制数据块的魔数，用于识别二进制文件格式 (ASCII "CRTC")
constexpr uint32_t kControlChunkMagic = 0x4C525443U;
/// 控制数据格式版本，用于兼容性检查
constexpr uint16_t kControlSchemaVersion = 2;
/// 控制数据头部大小（字节）
constexpr uint32_t kControlHeaderSize = 32U;
/// 单个控制样本大小（字节）
constexpr uint32_t kControlSampleSize = 281U;
/// 心跳包发送间隔，用于保持共享内存连接活跃
constexpr std::chrono::milliseconds kHeartbeatTick{100};

/**
 * @brief 回放路径集合
 *
 * 存储回放会话所需的所有文件系统路径
 */
struct ReplayPaths {
    fs::path run_dir;       ///< 录制根目录（包含run.json）
    fs::path run_json_path; ///< run.json元数据文件路径
    fs::path images_dir;    ///< 图像文件目录
    fs::path control_dir;   ///< 控制数据（IMU）目录
};

/**
 * @brief 控制事件（IMU数据）
 *
 * 存储单个时刻的云台姿态和角速度信息
 */
struct ControlEvent {
    uint64_t timestamp_ns{0};     ///< 数据采集时间戳（纳秒）
    uint64_t imu_timestamp_ns{0}; ///< IMU传感器时间戳（纳秒）
    double roll{0.0};             ///< 横滚角（弧度）
    double pitch{0.0};            ///< 俯仰角（弧度）
    double yaw{0.0};              ///< 偏航角（弧度）
    double roll_vel{0.0};         ///< 横滚角速度（弧度/秒）
    double pitch_vel{0.0};        ///< 俯仰角速度（弧度/秒）
    double yaw_vel{0.0};          ///< 偏航角速度（弧度/秒）
};

/**
 * @brief 图像事件
 *
 * 存储单个图像文件的路径和时间戳
 */
struct ImageEvent {
    fs::path path;            ///< 图像文件路径
    uint64_t timestamp_ns{0}; ///< 图像采集时间戳（纳秒）
};

/**
 * @brief 静态位姿集合
 *
 * 存储相机、枪口相对于里程计的静态变换关系
 */
struct StaticPoseBundle {
    ipc::ShmClient::Pose odom{};   ///< 里程计坐标系原点（世界坐标系）
    ipc::ShmClient::Pose camera{}; ///< 相机坐标系相对于里程计的位姿
    ipc::ShmClient::Pose muzzle{}; ///< 枪口坐标系相对于里程计的位姿
};

/**
 * @brief 回放相机快照
 *
 * 存储相机内参和分辨率信息
 */
struct ReplayCameraSnapshot {
    std::array<double, 9> camera_matrix{}; ///< 相机内参矩阵（3x3，行优先）
    std::array<double, 5> distortion{};    ///< 畸变系数（k1, k2, p1, p2, k3）
    uint32_t width{0};                     ///< 图像宽度（像素）
    uint32_t height{0};                    ///< 图像高度（像素）
};

/**
 * @brief 回放外参快照
 *
 * 存储相机和枪口的外参信息
 */
struct ReplayExtrinsicSnapshot {
    std::array<double, 3> camera_translation{}; ///< 相机平移向量（米）
    std::array<double, 3> camera_rpy_deg{};     ///< 相机姿态RPY角（度）
    std::array<double, 3> muzzle_translation{}; ///< 枪口平移向量（米）
    std::array<double, 3> muzzle_rpy_deg{};     ///< 枪口姿态RPY角（度）
};

/**
 * @brief 回放硬件快照
 *
 * 组合相机内参和外参信息
 */
struct ReplayHardwareSnapshot {
    ReplayCameraSnapshot camera{};       ///< 相机内参
    ReplayExtrinsicSnapshot extrinsic{}; ///< 外参
};

/**
 * @brief 回放数据集
 *
 * 完整的回放会话数据，包含所有路径、配置、事件流
 */
struct ReplayDataset {
    ReplayPaths paths;                        ///< 文件路径
    std::string robot;                        ///< 机器人名称（用于查找配置）
    ipc::CameraInfo camera_info{};            ///< 相机信息（用于发布到共享内存）
    StaticPoseBundle static_poses{};          ///< 静态位姿
    std::vector<ControlEvent> control_events; ///< 控制事件流（按时间戳排序）
    std::vector<ImageEvent> image_events;     ///< 图像事件流（按时间戳排序）
};

/**
 * @brief 格式化TOML解析错误信息
 * @param e TOML解析错误对象
 * @return 人类可读的错误信息字符串
 *
 * 将TOML解析错误转换为包含文件路径、行号、列号的详细错误信息
 */
[[nodiscard]] auto format_parse_error(const toml::parse_error& e) -> std::string {
    const auto& src = e.source();
    return fmt::format(
        "{} at {}:{}:{}", e.description(), src.path->data(), src.begin.line, src.begin.column);
}

/**
 * @brief 角度制转弧度制
 * @param deg 角度值
 * @return 弧度值
 *
 * 使用数学常量π进行高精度转换
 */
[[nodiscard]] auto to_rad(const double deg) noexcept -> double {
    return deg * std::numbers::pi_v<double> / 180.0;
}

/**
 * @brief 解析无符号64位整数
 * @param text 输入文本
 * @param what 字段名称（用于错误信息）
 * @return 解析结果或错误信息
 *
 * 使用std::from_chars进行高性能解析，避免内存分配
 */
[[nodiscard]] auto parse_uint64(std::string_view text, std::string_view what) noexcept
    -> std::expected<uint64_t, std::string> {
    uint64_t value       = 0;
    const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (ec != std::errc{} || ptr != text.data() + text.size()) {
        return std::unexpected(fmt::format("invalid {} '{}'", what, text));
    }
    return value;
}

/**
 * @brief 解析size_t类型整数
 * @param text 输入文本
 * @param what 字段名称（用于错误信息）
 * @return 解析结果或错误信息
 *
 * 封装parse_uint64，适配size_t类型
 */
[[nodiscard]] auto parse_size_t(std::string_view text, std::string_view what) noexcept
    -> std::expected<size_t, std::string> {
    auto value = parse_uint64(text, what);
    if (!value) {
        return std::unexpected(value.error());
    }
    return static_cast<size_t>(*value);
}

/**
 * @brief 解析双精度浮点数
 * @param text 输入文本
 * @param what 字段名称（用于错误信息）
 * @return 解析结果或错误信息
 *
 * 注意：使用std::strtod，需要创建临时string对象，性能略低于from_chars
 * 但C++标准库尚未提供浮点数的from_chars实现（部分编译器支持）
 */
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

/**
 * @brief 获取回放数据集的起始时间戳
 * @param dataset 回放数据集
 * @return 最早的有效时间戳或错误信息
 *
 * 遍历控制事件和图像事件，返回最小的非零时间戳
 * 若所有时间戳均为0或数据集为空，则返回错误
 */
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

/**
 * @brief 获取回放数据集的结束时间戳
 * @param dataset 回放数据集
 * @return 最晚的有效时间戳或错误信息
 *
 * 遍历控制事件和图像事件，返回最大的非零时间戳
 * 若所有时间戳均为0或数据集为空，则返回错误
 */
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

/**
 * @brief 查找最新的回放目录
 * @param base_dir 基础目录（默认为"record"）
 * @return 最新的run目录路径或错误信息
 *
 * 搜索基目录下所有以"run_"开头且包含run.json的子目录，
 * 返回字典序最大的目录名（即最新的录制）
 */
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

/**
 * @brief 解析回放路径
 * @param input_path 用户输入的路径（可为空）
 * @return 标准化的回放路径结构或错误信息
 *
 * 智能解析多种输入格式：
 * - 空路径：自动查找最新的录制目录
 * - run.json文件：提取其父目录作为运行目录
 * - 运行目录：直接使用
 * - images/control子目录：提取其父目录
 *
 * 验证必需的文件和目录是否存在
 */
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

/**
 * @brief 读取JSON文件
 * @param path JSON文件路径
 * @return 解析后的JSON对象或错误信息
 *
 * 使用nlohmann/json库进行解析，异常转为expected错误
 */
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

/**
 * @brief 从run.json加载机器人名称
 * @param run_json_path run.json文件路径
 * @return 机器人名称或错误信息
 *
 * 解析run.json中的launch.robot字段，用于定位机器人配置文件
 */
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

/**
 * @brief 从TOML表读取必需的双精度数组
 * @param table TOML表对象
 * @param key 键路径（支持点分隔的嵌套路径）
 * @param expected_size 期望的数组长度
 * @param path 配置文件路径（用于错误信息）
 * @return 双精度数组或错误信息
 *
 * 支持double和int64_t类型的自动转换，严格验证数组长度
 */
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

/**
 * @brief 从TOML表读取必需的uint32_t值
 * @param table TOML表对象
 * @param key 键路径
 * @param path 配置文件路径（用于错误信息）
 * @return uint32_t值或错误信息
 *
 * 读取int64_t类型并验证非负，然后转换为uint32_t
 */
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

/**
 * @brief 从TOML表读取可选的双精度值
 * @param table TOML表对象
 * @param key 键路径
 * @param fallback 默认值
 * @return 读取到的值或默认值
 *
 * 若键不存在或类型不匹配，返回提供的默认值
 * 支持double和int64_t类型的自动转换
 */
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

/**
 * @brief 加载硬件快照（相机内参和外参）
 * @param robot_config_path 机器人配置文件路径
 * @return 硬件快照或错误信息
 *
 * 从TOML配置文件中解析相机内参、分辨率、相机和枪口的外参
 * 配置文件格式：
 *   [camera]
 *   camera_matrix = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
 *   distort_coefficient = [k1, k2, p1, p2, k3]
 *   width = 1280
 *   height = 1024
 *
 *   [extrinsic.gimbal_yaw.gimbal_pitch]
 *   [extrinsic.gimbal_yaw.gimbal_pitch.camera_link]
 *   translation = [x, y, z]
 *   roll = 0.0
 *   pitch = 0.0
 *   yaw = 0.0
 */
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

/**
 * @brief 构造相机信息结构
 * @param hardware 硬件快照
 * @return 用于共享内存发布的相机信息
 *
 * 从硬件快照提取相机内参矩阵和畸变系数，
 * 转换为共享内存协议所需的格式
 */
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

/**
 * @brief 构造静态位姿集合
 * @param hardware 硬件快照
 * @return 包含里程计、相机、枪口位姿的集合
 *
 * 将相机和枪口的RPY角度转换为四元数，
 * 构造完整的位姿结构用于坐标变换
 */
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

/**
 * @brief 从字节数组读取小端序无符号整数
 * @tparam UInt 目标无符号整数类型
 * @param bytes 字节数组
 * @param offset 起始偏移量
 * @return 解析后的整数值
 *
 * 手动实现小端序解析，避免平台依赖性问题
 */
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

/**
 * @brief 从字节数组读取小端序单精度浮点数
 * @param bytes 字节数组
 * @param offset 起始偏移量
 * @return 解析后的浮点值
 *
 * 使用std::bit_cast进行类型安全转换
 */
[[nodiscard]] auto read_le_float32(std::span<const char> bytes, const size_t offset) -> float {
    const auto raw = read_le_uint<uint32_t>(bytes, offset);
    return std::bit_cast<float>(raw);
}

/**
 * @brief 读取二进制文件到内存
 * @param path 文件路径
 * @return 文件内容字节数组或错误信息
 *
 * 一次性读取整个文件到内存，适用于小文件
 * 注意：大文件应使用流式读取或内存映射
 */
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

/**
 * @brief 加载控制事件（IMU数据）
 * @param control_dir 控制数据目录路径
 * @return 控制事件列表或错误信息
 *
 * 从二进制文件中加载IMU数据，文件格式：
 * - 文件头（32字节）：
 *   - 魔数（4字节）：0x4C525443 ("CRTC")
 *   - 格式版本（2字节）：当前为2
 *   - 头部大小（2字节）：32
 *   - 样本大小（4字节）：281
 *   - 样本数量（4字节）
 * - 样本数据（每个281字节）：
 *   - 时间戳（8字节）
 *   - IMU时间戳（8字节）
 *   - 姿态RPY（各4字节，float）
 *   - 角速度RPY（各4字节，float）
 *
 * 容错处理：
 * - 跳过空文件、截断文件、大小不匹配文件
 * - 跳过IMU时间戳为0的样本
 * - 限制日志输出数量避免刷屏
 */
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

/**
 * @brief 解析图像文件名中的时间戳
 * @param image_path 图像文件路径
 * @return 时间戳或错误信息
 *
 * 图像文件名格式：camera_<源>_<时间戳>.jpg
 * 例如：camera_0_1234567890123456.jpg
 * 提取最后一个下划线后的数字作为时间戳
 */
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

/**
 * @brief 加载图像事件列表
 * @param images_dir 图像文件目录
 * @return 图像事件列表或错误信息
 *
 * 扫描目录下所有camera_*.jpg文件，解析时间戳并按时间戳排序
 * 过滤非相机图像文件（如thumbnail、other格式）
 */
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

/**
 * @brief 加载完整的回放数据集
 * @param paths 回放路径结构
 * @return 回放数据集或错误信息
 *
 * 组合加载所有回放数据：
 * 1. 从run.json加载机器人名称
 * 2. 从机器人配置加载硬件快照（相机内参、外参）
 * 3. 加载控制事件流（IMU数据）
 * 4. 加载图像事件流
 *
 * 所有数据加载成功后才返回完整数据集
 */
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

/**
 * @brief 带心跳的睡眠函数
 * @param client 共享内存客户端
 * @param deadline 睡眠截止时间
 *
 * 在等待期间定期发送心跳包，防止共享内存连接超时
 * 使用分段睡眠，每100ms检查一次是否到达截止时间
 */
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

/**
 * @brief 发布静态位姿到共享内存
 * @param client 共享内存客户端
 * @param dataset 回放数据集
 * @param timestamp_ns 当前时间戳
 *
 * 发布里程计、相机、枪口的静态位姿
 * 这些位姿在整个回放过程中保持不变（录制时刻的静态标定结果）
 */
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

/**
 * @brief 发布控制位姿（云台姿态）
 * @param client 共享内存客户端
 * @param event 控制事件
 * @param frame_seq 帧序列号
 * @param timestamp_ns 时间戳
 *
 * 将RPY角度转换为四元数，发布云台姿态
 */
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

/**
 * @brief 发布控制事件
 * @param client 共享内存客户端
 * @param event 控制事件
 * @param frame_seq 帧序列号
 *
 * 优先使用IMU时间戳，若为0则使用事件时间戳
 */
void publish_control_event(
    ipc::ShmClient& client, const ControlEvent& event, const uint64_t frame_seq) {
    publish_control_pose(
        client, event, frame_seq,
        event.imu_timestamp_ns != 0 ? event.imu_timestamp_ns : event.timestamp_ns);
}

/**
 * @brief 加载回放图像并转换为RGB格式
 * @param path 图像文件路径
 * @return RGB格式图像或错误信息
 *
 * OpenCV默认读取为BGR格式，转换为RGB以匹配共享内存协议
 */
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

/**
 * @brief 计算按速度缩放的时间偏移
 * @param base_timestamp_ns 基准时间戳
 * @param event_timestamp_ns 事件时间戳
 * @param speed 回放速度倍率
 * @return 缩放后的时长
 *
 * 支持加速（speed > 1）和减速（speed < 1）回放
 * 使用高精度long double计算，避免时间戳差值溢出
 */
[[nodiscard]] auto scaled_offset(
    const uint64_t base_timestamp_ns, const uint64_t event_timestamp_ns, const double speed)
    -> std::chrono::steady_clock::duration {
    const long double delta_ns = static_cast<long double>(event_timestamp_ns - base_timestamp_ns)
                               / static_cast<long double>(speed);
    const auto duration = std::chrono::duration<long double, std::nano>(delta_ns);
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(duration);
}

/**
 * @brief 运行单次回放循环
 * @param producer 共享内存生产者客户端
 * @param dataset 回放数据集
 * @param options 回放选项
 * @param next_frame_seq 帧序列号计数器（用于跨循环保持连续）
 * @return 已发布的图像数量或错误信息
 *
 * 核心回放算法：
 * 1. 计算启动偏移，跳过前面的数据
 * 2. 发布静态位姿和相机信息（初始状态）
 * 3. 按时间戳同步遍历控制事件和图像事件：
 *    - 找到下一个最近的时间戳
 *    - 等待到对应时刻（考虑速度缩放）
 *    - 发布该时刻的所有控制事件
 *    - 发布该时刻的所有图像事件
 *    - 更新心跳和进度日志
 * 4. 达到图像数量限制或数据结束则返回
 *
 * 时间同步策略：
 * - 使用std::lower_bound快速定位起始位置
 * - 双指针法遍历两个有序事件流
 * - 精确等待到下一个事件时刻
 */
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

    // 使用二分查找定位启动偏移后的起始位置
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

    // 发布初始状态：相机信息、静态位姿、运行时状态
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

    // 进度日志辅助函数
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

    // 主事件循环：同步遍历控制事件和图像事件
    while (control_index < dataset.control_events.size()
           || image_index < dataset.image_events.size()) {
        // 找到下一个最近的时间戳
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

        // 等待到事件时刻（考虑速度缩放）
        sleep_with_heartbeat(
            producer, steady_start + scaled_offset(cycle_start_timestamp, next_ts, options.speed));

        // 发布该时刻的所有控制事件
        while (control_index < dataset.control_events.size()
               && dataset.control_events[control_index].timestamp_ns == next_ts) {
            publish_control_event(producer, dataset.control_events[control_index], last_frame_seq);
            ++control_index;
        }

        // 发布该时刻的所有图像事件
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

            // 达到图像数量限制则提前退出
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

/**
 * @brief 生成回放程序使用说明
 * @param program 程序名称
 * @return 使用说明字符串
 */
auto replay_usage(const std::string_view program) -> std::string {
    return fmt::format(
        "usage: {} [run_dir|run.json|images_dir] [--loop] [--speed <factor>] "
        "[--startup-offset-ms <ms>] [--max-images <count>]",
        program);
}

/**
 * @brief 解析命令行参数为回放选项
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 回放选项或错误信息
 *
 * 支持的参数：
 * - 位置参数：run_dir | run.json | images_dir（可选）
 * - --loop：循环播放
 * - --speed <factor>：播放速度倍率（默认1.0）
 * - --startup-offset-ms <ms>：启动偏移（跳过前N毫秒）
 * - --max-images <count>：最大图像数量限制
 * - --help / -h：显示使用说明
 */
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

            // 通用参数值读取辅助函数，支持 --flag=value 和 --flag value 两种格式
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

/**
 * @brief 运行回放主流程
 * @param options 回放选项
 * @return 成功或错误信息
 *
 * 完整的回放流程：
 * 1. 解析回放路径（自动查找最新或使用指定路径）
 * 2. 加载回放数据集（机器人配置、控制事件、图像事件）
 * 3. 创建共享内存生产者客户端
 * 4. 循环执行回放循环（若启用--loop）
 * 5. 每个循环结束后更新剩余图像计数
 *
 * 注意：回放过程中忽略来自下游的命令，仅单向发布数据
 */
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
