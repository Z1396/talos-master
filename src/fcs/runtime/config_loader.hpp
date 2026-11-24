// 头文件保护，防止重复引入引发编译冲突
#pragma once

// 引入各类配置结构体依赖头文件
#include "config.hpp"               // 基础通用配置、枚举、类型定义
#include "foxglove_config.hpp"      // Foxglove 可视化服务配置
#include "runtime/capturer.hpp"     // 图像/数据采集器配置与上下文
#include "scheduler/thin.hpp"       // Talos 调度器基础配置

// C++ 标准库
#include <expected>    // C++23 结果+错误返回类型
#include <map>          // 有序映射，存放主机名与预设配置
#include <optional>     // 可选值类型（本文件未直接使用，为依赖预留）
#include <string>       // 标准字符串
#include <string_view>  // 轻量字符串视图，函数入参优先使用，无拷贝开销

// 项目顶层命名空间 fcs，整套机器人视觉/控制框架根命名空间
namespace fcs {

// 硬件层子命名空间：所有硬件驱动、通信、设备相关配置
namespace hardware {

/**
 * @brief 硬件数据传输模式枚举
 * Direct：直连模式，本地硬件直接通信
 * Chiral：分流/转发模式，经过中间链路转发数据
 */
enum Transport { Direct, Chiral };

/**
 * @brief Daedalus 设备专属配置结构体
 * Daedalus 为项目内一类机器人执行机构（如发射机构、运动机构）
 */
struct DaedalusConfig {
    double bullet_speed{25.0};       // 弹丸发射速度，默认值 25.0
    RobotExtrinsicConfig extrinsic{};// 机器人外参配置（相机/机构位姿、标定参数）
};

/**
 * @brief 通用直连硬件配置
 */
struct DirectConfig {
    bool camera_only;                // true：仅启用相机，关闭其余硬件
    bool imu_only{false};            // true：仅启用MCU(IMU+云台输出)，不打开相机
    Transport transport;             // 数据传输模式：直连 / 转发
    HardwareConfig hardware;         // 通用硬件底层配置
};

/// 单一波形的幅度/周期
struct ImuOnlyWaveConfig {
    double amplitude_deg{1.25}; // 摆动幅度（度）
    double period_s{0.7};       // 完整周期（秒）
};

/// 边走边转：平移分量 + 陀螺分量各自独立配置
struct ImuOnlyLateralSpinConfig {
    double lateral_amplitude_deg{17.0};
    double lateral_period_s{8.0};
    double spin_amplitude_deg{1.25};
    double spin_period_s{0.7};
};

/**
 * @brief ImuOnly 调试模式配置
 * 不用相机/视觉，直接按波形规律生成云台指令下发给MCU，用于调试伺服响应
 * 三种模式各有一套独立参数，切换 mode 不需要改数值
 */
struct ImuOnlyDemoConfig {
    // 波形模式：sine=yaw/pitch 正弦；spin=原地小陀螺；lateral=左右平移；lateral_spin=边走边转
    std::string mode{"sine"};

    // 模式1：原地小陀螺（锯齿波）
    ImuOnlyWaveConfig spin{1.25, 0.7};
    // 模式2：左右平移（三角波）
    ImuOnlyWaveConfig lateral{17.0, 8.0};
    // 模式3：边走边转（三角波 + 锯齿波叠加）
    ImuOnlyLateralSpinConfig lateral_spin{};

    // sine 模式：yaw/pitch 各自独立正弦
    double yaw_amplitude_deg{20.0};   // yaw 摆动幅度（度）
    double yaw_frequency_hz{0.5};     // yaw 摆动频率（Hz）
    double pitch_amplitude_deg{8.0};  // pitch 摆动幅度（度），0=不动
    double pitch_frequency_hz{0.25};  // pitch 摆动频率（Hz）
    double phase_deg{90.0};           // pitch 相对 yaw 的相位差（度）

    // 安全边界：输出指令会被钳制在 ±limit 内
    double yaw_limit_deg{30.0};   // yaw 允许范围（度）
    double pitch_limit_deg{10.0}; // pitch 允许范围（度）

    // true：等待下位机能力帧 following=1 才启动；false：启动后立即摆动
    bool arm_by_mcu{true};

    double distance{0.0}; // 下发给MCU的距离字段（米），仅调试显示用
};

/**
 * @brief 硬件后端配置变体类型
 * std::variant：C++17 类型联合体，二选一存储
 * 运行时根据业务选择使用 Daedalus 设备配置 或 通用直连硬件配置
 */
using HardwareBackendConfig = std::variant<DaedalusConfig, DirectConfig>;

} // namespace hardware

/**
 * @brief 预设配置条目
 * 对应配置文件 [preset] 分组下，单条主机预设规则
 * 按主机名区分不同机器人/视觉方案
 */
struct PresetEntry {
    HardwareBackend backend{HardwareBackend::Direct};  // 硬件后端类型，默认直连模式
    toml_helper::required<std::string> robot;         // 【必填项】机器人方案名称
    toml_helper::required<std::string> vision;        // 【必填项】视觉方案名称
};

/**
 * @brief 启动阶段原始配置结构体
 * 对应 at_vision.toml 配置文件完整解析结果
 * 加载后**未做合并、未做解析匹配**，保留文件原始结构
 */
struct LaunchConfig {
    // 兜底预设：无匹配主机名时使用的默认配置
    toml_helper::flatten<PresetEntry> fallback_preset;
    // 预设映射表：key = 主机名，value = 对应预设配置
    std::map<std::string, PresetEntry> preset;
    // Daedalus 设备全局配置
    hardware::DaedalusConfig daedalus;
    // Foxglove 可视化服务配置
    FoxgloveConfig foxglove;
    // 数据采集器配置
    CapturerConfig capturer;
    // Talos 任务调度器配置
    talos::scheduler::SchedulerConfig scheduler;
    // ImuOnly 调试模式的正弦指令参数
    hardware::ImuOnlyDemoConfig imu_only_demo;
};

/**
 * @brief 运行时最终生效配置
 * 由 LaunchConfig 合并、匹配、解析后生成，**程序运行全程使用此结构体**
 * 已完成主机名预设匹配、配置合并、字段补全，结构扁平化，无多余分层
 */
struct RuntimeConfig {
    hardware::HardwareBackendConfig backend;  // 最终生效的硬件后端配置（二选一）
    FoxgloveConfig foxglove;                  // 可视化服务配置
    CapturerConfig capturer;                  // 采集器配置
    VisionConfig vision;                      // 解析后最终视觉方案配置
    runtime::CapturerLaunchContext launch;    // 采集器运行上下文（运行时状态、句柄等）
    talos::scheduler::SchedulerConfig scheduler; // 调度器最终配置
    hardware::ImuOnlyDemoConfig imu_only_demo; // ImuOnly 模式正弦指令参数
};

/**
 * @brief 全局配置加载入口函数
 * 从指定路径加载并合并所有 TOML 配置文件
 *
 * 逻辑规则：
 * 1. 解析主配置 at_vision.toml
 * 2. 若配置存在 [preset] 预设表，自动读取**本机主机名**匹配对应预设
 * 3. 主机名无匹配时，使用 fallback_preset 兜底配置
 * 4. 合并基础配置 + 预设配置，生成完整 RuntimeConfig
 *
 * @param path 配置文件根路径/主配置文件路径
 * @return std::expected<RuntimeConfig, std::string>
 *         - 成功：返回解析合并完成的运行时配置
 *         - 失败：返回格式化后的错误描述字符串（文件不存在、字段缺失、解析错误等）
 * @note [[nodiscard]] 强制要求调用方处理返回结果，禁止忽略加载失败
 */
[[nodiscard]] std::expected<RuntimeConfig, std::string> load_config(std::string_view path);

} // namespace fcs