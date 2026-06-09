// 框架核心启动引导头文件，本文件实现 fcs::boot 主初始化逻辑
#include "runtime/boot.hpp"

// ===================== 分层业务模块头文件（按机器人软件分层架构组织）=====================
// L1 硬件传感器层：传感器输出接口
#include "L1_sensor/output_interface.hpp"
// L2 感知层 - 装甲板：ROI 区域读取、装甲检测相关系统
#include "L2_perception/armor/readback_roi.hpp"
#include "L2_perception/armor/systems.hpp"
// L2 感知层 - 大符（LDM）：大符检测系统
#include "L2_perception/ldm/ldm_systems.hpp"
// L2 感知层 - 能量机关（Rune）：能量机关检测系统
#include "L2_perception/rune/rune_systems.hpp"
// L3 估计层 - 能量计量、大符简易估计、目标跟踪器
#include "L3_estimation/energy_meter/energy_meter_systems.hpp"
#include "L3_estimation/ldm_naive/ldm_naive_systems.hpp"
#include "L3_estimation/tracker_systems.hpp"
// L4 规划层：运动规划、瞄准策略系统
#include "L4_planning/planning_systems.hpp"
// L5 武器控制层：武器发射、火控系统
#include "L5_weapon/enhanced/weapon_systems.hpp"
// 手性数据采集系统（共享内存数据上报）
#include "chiral/chiral_collector_system.hpp"

// ===================== 框架核心基础头文件 =====================
// 装甲板相关数据类型定义
#include "core/armor_types.hpp"
// 全局通信通道、话题定义
#include "core/channel_topics.hpp"
// 运行时核心上下文
#include "core/runtime.hpp"
// 弹道解算资源、参数管理
#include "core/trajectory/resource.hpp"
// 全局基础类型别名
#include "core/types.hpp"
// 坐标系帧定义（fast_tf 静态坐标系）
#include "frame.hpp"
// 高速数据流传输组件
#include "quanta/stream_transport.hpp"
// 数据录制/抓包组件
#include "runtime/capturer.hpp"
// 配置文件加载解析
#include "runtime/config_loader.hpp"
// L1/L2 感知层统一初始化工具
#include "runtime/l1_l2_setup.hpp"
// 视频/数据流编码组件
#include "runtime/stream_encode.hpp"
// 调度器错误信息格式化工具
#include "scheduler/error_formatter.hpp"

// ===================== 标准库 & 第三方工具库 =====================
// fmt 格式化库（字符串拼接、容器打印）
#include <fmt/core.h>
#include <fmt/ranges.h>
// 数学常量（圆周率）
#include <numbers>
// 多态重载工具（std::visit 多分支匹配）
#include <primitive/overloaded.hpp>
// spdlog 日志库
#include <spdlog/spdlog.h>

/**
 * @namespace fcs
 * @brief 整个机器人视觉火控框架顶层命名空间
 * 本文件实现框架启动引导函数 boot，负责**全链路模块初始化、资源注册、系统注册、调度器构建**
 * 软件分层架构：L1传感器 → L2感知 → L3估计 → L4规划 → L5武器控制，自上而下依次初始化
 */
namespace fcs {

/**
 * @namespace 匿名命名空间
 * 存放仅当前文件内部使用的工具函数、常量，对外不可见
 */
namespace {

/**
 * @brief 角度转弧度工具函数
 * @param x 角度值(°)
 * @return 弧度值(rad)
 * @note noexcept 无异常，实时系统友好
 */
inline constexpr auto to_rad(double x) noexcept {
    return (x / 180.0) * std::numbers::pi_v<double>;
}

/**
 * @brief 初始化全局静态坐标系树 TF
 * @param system fast_tf 坐标系系统实例
 * @param config 机器人外参配置（机械安装偏移、旋转角）
 * 功能：根据机械外参，初始化所有相邻坐标系之间的固定变换矩阵
 * 坐标系链路：world → odom → gimbal_yaw → gimbal_pitch → camera_link / muzzle_link
 *                                 → camera_optical
 */
void init_coordinate_system(fast_tf::CoordinateSystem& system, const RobotExtrinsicConfig& config) {
    // 圆周率常量
    constexpr fp_t pi = std::numbers::pi;

    // 1. 世界坐标系、里程计坐标系、云台偏航轴：默认单位矩阵（无偏移），时间戳0
    fast_tf::update<fast_tf::odom>(system, {}, 0);
    fast_tf::update<fast_tf::gimbal_yaw_fuxk_frame>(system, {}, 0);

    // 2. 云台俯仰轴 gimbal_pitch：仅平移偏移，无旋转
    auto a = config.gimbal_yaw.gimbal_pitch.translation;
    fast_tf::update<fast_tf::gimbal_pitch_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_translation(a.x(), a.y(), a.z()),
        0);

    // 3. 相机机械安装帧 camera_link：平移 + 三轴旋转（外参标定值）
    auto b = config.gimbal_yaw.gimbal_pitch.camera_link.translation;
    auto c = config.gimbal_yaw.gimbal_pitch.camera_link.rotation();
    fast_tf::update<fast_tf::camera_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_rpy(
            to_rad(c.roll), to_rad(c.pitch), to_rad(c.yaw),
            b.x(), b.y(), b.z()),
        0);

    // 4. 相机光学帧 camera_optical：相机内固有旋转（机械帧 → 光学帧固定偏转）
    // 相机标准坐标系转换：绕X-90°、绕Z-90°，无平移
    fast_tf::update<fast_tf::camera_optical_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_optical_fuxk_frame>::from_rpy(
            -pi / 2.0, 0.0, -pi / 2.0,
            0.0, 0.0, 0.0),
        0);

    // 5. 枪口坐标系 muzzle_link：云台俯仰轴到枪口的平移+旋转外参
    auto d = config.gimbal_yaw.gimbal_pitch.muzzle_link.translation;
    auto e = config.gimbal_yaw.gimbal_pitch.muzzle_link.rotation();
    fast_tf::update<fast_tf::muzzle_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_rpy(
            to_rad(e.roll), to_rad(e.pitch), to_rad(e.yaw),
            d.x(), d.y(), d.z()),
        0);
}

} // 匿名命名空间结束

/**
 * @brief 框架核心启动引导函数
 * @param scheduler 全局任务调度器实例
 * @param config 全局运行时配置（TOML 解析后的完整配置），移动语义转移所有权
 * @return std::expected<void, std::string> 成功返回空，失败返回错误描述字符串
 *
 * 整体执行流程（严格按分层顺序初始化）：
 * 1. 注入全局通用资源（可视化配置、坐标系系统、硬件参数）
 * 2. 区分不同硬件后端，初始化 TF 坐标系、弹速、手性标记
 * 3. 注入视觉通用配置（识别颜色、功能开关、ROI、缓存等）
 * 4. 初始化 L1 传感器层（相机、硬件接口）
 * 5. 创建检测后端、PnP 解算器等算法实例
 * 6. 注册弹道解算资源
 * 7. 逐层注册业务系统：L2感知 → L3估计 → L4规划 → L5武器
 * 8. 按需注册手性数据采集系统
 * 9. 注册高速数据流、数据录制系统
 * 10. 构建调度器（冻结拓扑结构，正式就绪）
 */
[[nodiscard]] std::expected<void, std::string>
boot(talos::Scheduler& scheduler, RuntimeConfig&& config) {
    // 获取调度器内部 ECS 全局世界容器（存放所有资源、组件）
    auto& world = scheduler.world();

    // 1. 注入 Foxglove 可视化配置为全局资源，供可视化系统使用
    world.insert_resource(config.foxglove);

    // 手性标记：是否启用左右向数据采集（不同机器人机械结构适配）
    bool chiral = false;

    // 2. 在 World 中就地创建 TF 坐标系系统（避免大元组拷贝，优化启动性能）
    auto& coordinate_system = world.emplace_resource<fast_tf::CoordinateSystem>();

    // 3. 多分支处理：区分两种硬件后端配置（变体类型 std::visit + 重载匹配）
    std::visit(
        overloaded{
            // 分支1：直连硬件后端 DirectConfig
            [&coordinate_system, &world, &chiral](hardware::DirectConfig cfg) {
                // 初始化机械外参对应的坐标系变换
                init_coordinate_system(coordinate_system, cfg.hardware.extrinsic);
                // 注入弹丸初速配置（弹道解算使用）
                world.insert_resource(
                    core::trajectory::bullet_speed_data{
                        .bullet_speed = cfg.hardware.mcu->bullet_speed_default});
                // 读取手性开关
                chiral = cfg.hardware.chiral;
            },
            // 分支2：Daedalus 专用硬件后端
            [&coordinate_system, &world](hardware::DaedalusConfig cfg) {
                init_coordinate_system(coordinate_system, cfg.extrinsic);
                // 注入弹丸初速
                world.insert_resource(
                    core::trajectory::bullet_speed_data{.bullet_speed = cfg.bullet_speed});
            }},
        config.backend); // 传入变体类型，自动匹配对应分支

    // 4. 注入视觉基础配置资源
    // 目标识别颜色（红/蓝方）
    world.insert_resource(config.vision.detect_color);
    // 功能能力掩码：启用/关闭各项视觉功能
    static_cast<void>(world.emplace_resource<core::CapabilityState>(
        std::span<const core::Capability>(config.vision.capabilities.get())));
    // 目标跟随状态机资源
    static_cast<void>(world.emplace_resource<core::FollowingState>());
    // 装甲检测 ROI 感兴趣区域配置
    world.insert_resource(config.vision.armor->readback_roi);
    // 跟踪器数据缓存
    world.insert_resource(L2::TrackerReadbackCache{});

    // 打印基础视觉配置，便于启动核查
    SPDLOG_INFO("detect color: {}", config.vision.detect_color);
    SPDLOG_INFO("enabled capabilities: [{}]", fmt::join(*config.vision.capabilities, ", "));

    // ===================== 5. 初始化 L1 传感器层（最底层硬件） =====================
    auto setup_result = runtime::setup_l1(world, scheduler, config.backend);
    if (!setup_result) {
        return std::unexpected(fmt::format("L1 传感器初始化失败: {}", setup_result.error()));
    }

    // ===================== 6. 初始化 L2 感知层算法后端 =====================
    // 创建装甲检测后端（不同推理引擎/模型）
    auto backend_handle_result = L2::create_detector_backend_handle(config.vision.armor.get());
    if (!backend_handle_result) {
        return std::unexpected(
            fmt::format("创建检测后端失败: {}", backend_handle_result.error()));
    }
    SPDLOG_INFO("当前装甲检测后端: {}", backend_handle_result->name);

    // 将检测后端、PnP 位姿解算器注册为全局资源
    world.insert_resource(std::move(backend_handle_result->backend));
    world.insert_resource(L2::create_pnp_solver(setup_result.value().camera_config));

    // ===================== 7. 注册弹道解算资源（L4/L5 依赖） =====================
    core::trajectory::register_resource(scheduler, std::move(config.vision.trajectory));
    // 打印弹丸速度参数
    SPDLOG_INFO(
        "bullet initial speed = {:.3f} m/s",
        world.get_res<core::trajectory::bullet_speed_data>()->bullet_speed);

    // ===================== 8. 逐层注册业务系统（按数据流顺序） =====================
    // L2 感知系统：装甲检测、大符检测、能量机关检测
    L2::register_detection_systems(scheduler);
    L2::ldm::register_ldm_systems(
        scheduler, std::move(config.vision.ldm), setup_result.value().camera_config);
    rune::register_rune_detection_systems(scheduler, std::move(config.vision.rune_detector));

    // L3 估计系统：目标跟踪、大符简易估计、能量计量
    L3::register_tracker_systems(scheduler, std::move(config.vision.l3->tracker));
    L3::ldm::register_naive_ldm_systems(scheduler, config.vision.naive_ldm);
    energy_meter::register_energy_meter_systems(scheduler, std::move(config.vision.energy_meter));

    // L4 规划系统：目标运动预测、瞄准规划
    L4::register_l4_planning_systems(scheduler, std::move(config.vision.l4.get()));

    // L5 武器控制系统：最终火控、发射控制（依赖L4规划输出）
    L5::register_enhanced_weapon_system(scheduler, std::move(config.vision.l5.get()));

    // ===================== 9. 按需注册手性数据采集系统 =====================
    // 启用手性时，注册共享内存数据上报系统
    if (chiral) {
        chiral::register_chiral_collector_system(scheduler);
        SPDLOG_INFO("手性数据采集系统已注册");
    }

    // ===================== 10. 注册高速数据流 & 数据录制系统 =====================
    // 数据流传输配置
    auto quanta_cfg                  = config.vision.quanta;
    quanta_cfg.filter                = config.vision.quanta_filter;
    const auto& stream_camera_config = setup_result.value().camera_config;
    // 注册视频流/数据流传输系统
    auto quanta_stream_result        = runtime::register_quanta_stream_systems(
        world, scheduler, quanta_cfg,
        static_cast<int>(stream_camera_config.width),
        static_cast<int>(stream_camera_config.height));
    if (!quanta_stream_result) {
        return std::unexpected(
            fmt::format("注册高速数据流系统失败: {}", quanta_stream_result.error()));
    }

    // 注册离线数据录制、抓包系统
    runtime::register_runtime_capturer_system(scheduler, config.capturer, config.launch);

    // ===================== 11. 构建调度器（收尾步骤） =====================
    // build()：冻结所有系统拓扑、依赖关系、资源布局，调度器进入就绪状态
    if (auto build_result = scheduler.build(); !build_result) {
        return std::unexpected(fmt::format("构建调度器失败: {}", build_result.error()));
    }

    // 全部初始化完成
    return {};
}

} // namespace fcs