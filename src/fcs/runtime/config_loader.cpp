// 引入本模块头文件：配置加载对外声明
#include "runtime/config_loader.hpp"

// 项目内部配置结构体定义
#include "config.hpp"
// TOML 解析工具封装
#include "toml_helper.hpp"
// 针对 Eigen 矩阵/向量的 TOML 适配（视觉/外参常用）
#include "toml_helper_eigen.hpp"

// 字符串格式化库 fmt
#include <fmt/core.h>
// 系统信息工具类（获取主机名）
#include <primitive/system_info.hpp>
// 第三方 toml++ 解析库主头文件
#include <toml++/toml.hpp>

// 项目顶层命名空间 fcs
namespace fcs {

// 匿名命名空间：内部私有函数/变量，仅当前 .cpp 文件可见，对外不可访问
namespace {

/**
 * @brief 格式化 TOML 解析错误信息
 * @param e toml++ 抛出的解析异常对象
 * @return 拼接好的完整错误字符串：错误描述 + 文件路径 + 行号 + 列号
 */
auto format_parse_error(const toml::parse_error& e) -> std::string {
    // 获取错误来源信息（文件、行列）
    // 注意：e.source() 返回的是一个智能指针，需要解引用获取实际的 source 对象
    const auto& src = e.source();
    // fmt 格式化：拼接错误描述、文件、行、列
    return fmt::format(
        "{} at {}:{}:{}", e.description(), src.path->data(), src.begin.line, src.begin.column);
        // 注意：src.path->data() 返回的是一个智能指针，需要解引用获取实际的字符串数据  
}

/**
 * @brief 解析[preset]主机名预设配置
 * @param launch 主入口配置 LaunchConfig
 * @return 成功返回 PresetEntry 预设项；失败返回错误字符串
 * 
 * 逻辑说明：
 * 1. 获取当前机器主机名
 * 2. 在配置 [preset] 表中查找对应主机名
 * 3. 找到：使用该主机名对应的 robot/vision/backend 配置
 * 4. 未找到：使用默认 fallback_preset 兜底配置
 */
auto parse_preset(LaunchConfig& launch) -> std::expected<PresetEntry, std::string> {
    // 调用系统工具类获取本机主机名，返回 std::expected
    const auto hostname_result = talos::primitive::SystemInfo::get_hostname();
    // 判断：获取主机名失败（比如系统接口异常）
    if (!hostname_result) {
        return std::unexpected(
            fmt::format(
                "entry config: [preset] table present but hostname lookup failed: {}",
                hostname_result.error()));
    }
    // 取出正常的主机名字符串
    const std::string_view hostname = *hostname_result;

    // 在 preset 字典中查找当前主机名对应的配置项
    const auto it = launch.preset.find(std::string{hostname});
    // 主机名不在预设列表中，使用兜底默认配置
    if (it == launch.preset.end()) {
        SPDLOG_INFO("hostname '{}' not in [preset], using explicit robot/vision fields", hostname);
        return launch.fallback_preset;
    }

    // 找到对应主机名的预设配置
    const auto& entry = it->second;

    // 日志打印当前生效的预设：机器人名称、视觉配置、后端类型
    SPDLOG_INFO(
        "hostname: '{}' -> robot='{}', vision='{}', backend={}", hostname, entry.robot.get(),
        entry.vision.get(), entry.backend);

    // 返回匹配到的预设项
    return entry;
}

/**
 * @brief 加载机器人硬件配置
 * @param robot 机器人名称（字符串）
 * @return 成功返回 HardwareConfig 硬件配置；失败返回错误信息
 * 配置路径固定：config/robot/{机器人名}.toml
 */
auto load_hardware_config(std::string_view robot) -> std::expected<HardwareConfig, std::string> {
    // 拼接 TOML 配置文件路径
    auto hardware_path = fmt::format("config/robot/{}.toml", robot);
    // 解析 TOML 文件
    auto hardware_tbl  = toml::parse_file(hardware_path);
    // 文件解析失败（不存在/格式错误）
    if (hardware_tbl.failed()) {
        return std::unexpected(
            fmt::format("{}: {}", hardware_path, format_parse_error(hardware_tbl.error())));
    }

    // 工具函数：将 TOML 表 反序列化为 C++ 结构体 HardwareConfig
    auto hardware_config = toml_helper::from_table<HardwareConfig>(hardware_tbl.table());
    // 结构体映射失败（字段不匹配、类型错误）
    if (!hardware_config) {
        return std::unexpected(fmt::format("hardware config: {}", hardware_config.error()));
    }
    // 移动语义返回，避免拷贝
    return std::move(*hardware_config);
}

/**
 * @brief 加载机器人视觉外参配置（相机外参、标定参数等）
 * @param robot 机器人名称
 * @return 成功返回 RobotExtrinsicConfig 外参配置；失败返回错误
 * 外参存放位置：同机器人硬件配置文件的 [extrinsic] 子表
 */
auto load_robot_extrinsic_config(std::string_view robot)
    -> std::expected<RobotExtrinsicConfig, std::string> {
    // 拼接机器人配置文件路径
    auto hardware_path = fmt::format("config/robot/{}.toml", robot);
    auto hardware_tbl  = toml::parse_file(hardware_path);
    // 文件解析失败
    if (hardware_tbl.failed()) {
        return std::unexpected(
            fmt::format("{}: {}", hardware_path, format_parse_error(hardware_tbl.error())));
    }

    // 从 TOML 表中读取 [extrinsic] 子表，并映射为外参结构体
    auto extrinsic = toml_helper::read<RobotExtrinsicConfig>(hardware_tbl.table(), "extrinsic");
    if (!extrinsic) {
        return std::unexpected(fmt::format("hardware extrinsic: {}", extrinsic.error()));
    }
    return *extrinsic;
}

} // 匿名命名空间 结束

/**
 * @brief 对外主接口：加载整套程序运行时配置
 * @param path 入口主配置 TOML 文件路径
 * @return 成功返回完整 RuntimeConfig 运行时配置；失败返回错误字符串
 * 
 * 整体执行流程：
 * 1. 解析主入口 TOML 配置
 * 2. 根据本机主机名匹配预设 preset，确定 robot / vision / backend
 * 3. 校验录制器 capturer 配置合法性
 * 4. 加载视觉基础配置 + 视觉差异化配置，合并两份 TOML
 * 5. 根据 backend 后端类型，分支加载硬件/外参配置
 * 6. 统一拼装成最终 RuntimeConfig 并返回
 */
 /* template<class T, class E>
    class expected;
    T：成功时承载的正常业务数据类型（toml::table、自定义句柄、PnPSolver 智能指针等）
    E：失败时承载的错误类型（字符串、自定义错误结构体、parse_error）
    两种特殊变体
    std::expected<void, E>：函数成功无返回值，仅区分成功 / 失败（如纯初始化函数）
    std::unexpected<E>：用于构造失败分支，专门包装错误对象*/
[[nodiscard]] std::expected<RuntimeConfig, std::string> load_config(std::string_view path) {
    // ========== 1. 解析主入口配置文件 ==========
    auto entry_tbl = toml::parse_file(path);
    // 主配置文件解析失败
    if (entry_tbl.failed()) {
        return std::unexpected(format_parse_error(entry_tbl.error()));
    }

    // 将主 TOML 表映射为 LaunchConfig 启动配置结构体
    auto launch_config = toml_helper::from_table<LaunchConfig>(entry_tbl.table());
    if (!launch_config) {
        return std::unexpected(fmt::format("entry config: {}", launch_config.error()));
    }
    // 移动构造，转移所有权
    auto launch = std::move(*launch_config);

    // ========== 2. 根据主机名匹配预设配置 ==========
    auto preset_result = parse_preset(launch);
    // 预设解析失败（如获取主机名失败）
    if (!preset_result.has_value()) {
        return std::unexpected(preset_result.error());
    }
    auto preset = preset_result.value();

    // ========== 3. 合法性校验：录制器开启时必须指定输出目录 ==========
    if (launch.capturer.enabled && launch.capturer.output_dir.empty()) {
        return std::unexpected(
            "entry config: capturer.output_dir is required when capturer.enabled=true");
    }

    // ========== 4. 加载 & 合并 视觉配置 ==========
    // 拼接视觉差异化配置路径：config/vision/{视觉名}.toml
    auto vision_override_path = fmt::format("config/vision/{}.toml", preset.vision.get());
    // 加载视觉基础公共配置
    auto base_tbl = toml::parse_file("config/vision_base.toml");
    if (base_tbl.failed()) {
        return std::unexpected(
            fmt::format("config/vision_base.toml: {}", format_parse_error(base_tbl.error())));
    }
    // 加载当前视觉方案的差异化配置
    auto override_tbl = toml::parse_file(vision_override_path);
    if (override_tbl.failed()) {
        return std::unexpected(
            fmt::format("{}: {}", vision_override_path, format_parse_error(override_tbl.error())));
    }

    // 合并两份 TOML 配置：差异化配置覆盖基础配置
    // transform_error：统一改写错误描述文案
    auto merged_vision = toml_helper::merge_configs(base_tbl.table(), override_tbl.table())
                             .transform_error([](const auto& err) {
                                 return fmt::format("merge vision config: {}", err);
                             });
    if (!merged_vision) {
        return std::unexpected(merged_vision.error());
    }

    // 将合并后的视觉 TOML 转为 VisionConfig 结构体
    auto vision_config = toml_helper::from_table<VisionConfig>(*merged_vision);
    if (!vision_config) {
        return std::unexpected(fmt::format("vision config: {}", vision_config.error()));
    }

    // ========== 5. 根据后端类型 backend 分支加载硬件/外参 ==========
    // 硬件后端总配置（多态容器，统一包装不同后端）
    hardware::HardwareBackendConfig cfg;

    // 按预设的后端类型分支处理
    switch (preset.backend) {
    // 直连/相机专用/Chiral 链路：加载机器人硬件配置
    case Chiral:
    case CameraOnly:
    case Direct: {
        // 是否仅使用相机模式
        bool camera_only     = preset.backend == CameraOnly;
        // 加载对应机器人硬件配置
        auto hardware_config = load_hardware_config(preset.robot.get());
        if (!hardware_config) {
            return std::unexpected(fmt::format("hardware config: {}", hardware_config.error()));
        }
        // 构造直连类硬件配置
        cfg = hardware::DirectConfig{
            .camera_only = camera_only,
            // 区分传输链路类型
            .transport   = preset.backend != Chiral ? hardware::Transport::Direct
                                                    : hardware::Transport::Chiral,
            .hardware    = std::move(*hardware_config)};
        break;
    };
    // Daedalus 后端：加载机器人视觉外参（标定、坐标转换等）
    case Daedalus: {
        auto extrinsic = load_robot_extrinsic_config(preset.robot.get());
        if (!extrinsic) {
            return std::unexpected(extrinsic.error());
        }
        // 构造 Daedalus 后端配置
        cfg = hardware::DaedalusConfig{
            .bullet_speed = launch.daedalus.bullet_speed,
            .extrinsic    = std::move(*extrinsic),
        };
        break;
    }
    }

    // ========== 6. 拼装最终 RuntimeConfig 并返回 ==========
    // C++17 指定成员初始化，组装全量运行时配置
    return RuntimeConfig{
        .backend  = std::move(cfg),                  // 硬件后端配置
        .foxglove = std::move(launch.foxglove),     // Foxglove 可视化工具配置
        .capturer = std::move(launch.capturer),     // 数据录制器配置
        .vision   = std::move(*vision_config),     // 视觉整套配置
        // 运行时上下文：记录当前使用的后端、机器人、视觉方案
        .launch = runtime::CapturerLaunchContext{
                                           .backend = preset.backend,
                                           .robot   = preset.robot.get(),
                                           .vision  = preset.vision.get(),
        },
        .scheduler = launch.scheduler               // 调度器配置
    };
}

} // namespace fcs