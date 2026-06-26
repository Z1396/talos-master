// 框架核心启动引导头文件，本文件实现 fcs::boot 顶层主初始化逻辑
// boot 函数是整个机器人火控软件唯一入口，完成资源、坐标系、分层系统全量注册
#include "runtime/boot.hpp"

// ===================== 分层业务模块头文件（机器人标准五层软件架构：L1硬件→L2感知→L3估计→L4规划→L5武器）=====================
// L1 硬件传感器层：底层相机、IMU、MCU硬件数据输出接口
#include "L1_sensor/output_interface.hpp"

// L2 感知层 - 装甲板识别：ROI裁剪读取、装甲检测全套ECS系统注册函数
#include "L2_perception/armor/readback_roi.hpp"
#include "L2_perception/armor/systems.hpp"
// L2 感知层 - 大符LDM：能量机关光斑检测系统注册
#include "L2_perception/ldm/ldm_systems.hpp"
// L2 感知层 - 旋转能量机关Rune：大符图像检测、光斑提取系统
#include "L2_perception/rune/rune_systems.hpp"

// L3 状态估计层：滤波跟踪、能量机关角速度解算、简易LDM跟踪
#include "L3_estimation/energy_meter/energy_meter_systems.hpp"
#include "L3_estimation/ldm_naive/ldm_naive_systems.hpp"
#include "L3_estimation/tracker_systems.hpp"

// L4 运动规划层：目标运动预测、MPC瞄准轨迹规划、补偿策略系统
#include "L4_planning/planning_systems.hpp"

// L5 武器控制层：最终火控解算、发射逻辑、摩擦轮速度控制增强系统
#include "L5_weapon/enhanced/weapon_systems.hpp"

// 手性采集系统：左右手机构共享内存数据采集、上报调试数据流
#include "chiral/chiral_collector_system.hpp"

// ===================== 框架核心基础头文件（ECS调度、通信、坐标系、资源、工具） =====================
// 装甲板数据结构：单装甲、批量检测结果、角点、PnP输出定义
#include "core/armor_types.hpp"
// 全局SPMC无锁通信通道Topic常量：所有图像/跟踪/控制数据流唯一标识
#include "core/channel_topics.hpp"
// 运行时全局上下文：启动配置、硬件后端区分、资源生命周期
#include "core/runtime.hpp"
// 弹道解算全局资源：弹速、重力补偿、弹道查表参数容器
#include "core/trajectory/resource.hpp"
// 全局通用基础类型：Eigen向量、位姿、枚举、状态机、能力掩码别名
#include "core/types.hpp"
// fast_tf静态坐标系枚举定义：world/odom/gimbal/camera/muzzle等帧标识
#include "frame.hpp"
// Quanta高速视频流编码传输组件：H.265硬件编码、WebSocket/Mcap推流
#include "quanta/stream_transport.hpp"
// 数据录制抓包组件：离线保存图像/跟踪/JSON调试消息
#include "runtime/capturer.hpp"
// TOML配置文件加载、解析、校验工具
#include "runtime/config_loader.hpp"
// L1传感器+L2感知层统一初始化工具：相机打开、推理后端创建
#include "runtime/l1_l2_setup.hpp"
// 图像编码通用工具：JPEG/PNG/视频流封装
#include "runtime/stream_encode.hpp"
// 调度器错误格式化：将std::expected错误转为可读日志字符串
#include "scheduler/error_formatter.hpp"

// ===================== 标准库 & 第三方开源工具库 =====================
// fmt格式化库：高性能字符串拼接、容器打印、数值格式化，替代std::stringstream
#include <fmt/core.h>
#include <fmt/ranges.h>
// 标准数学常量：圆周率pi
#include <numbers>
// primitive多态工具：std::visit 多路分支重载匹配，简化std::variant硬件分支
#include <primitive/overloaded.hpp>
// spdlog日志库：全框架统一日志输出、分级打印
#include <spdlog/spdlog.h>

/**
 * @namespace fcs
 * @brief 整个机器人视觉火控框架顶层根命名空间
 * 分层架构自上而下：
 * L1 硬件传感器输入 → L2 图像感知检测 → L3 目标状态滤波估计 → L4 运动瞄准规划 → L5 武器发射控制
 * 本文件核心函数 boot()：软件唯一初始化入口，按分层顺序完成：
 * 1 全局资源注入 2 TF坐标系初始化 3 硬件/视觉后端创建 4 全分层ECS系统注册 5 调度器冻结拓扑就绪
 */
namespace fcs {

/**
 * @namespace 匿名命名空间（文件内私有作用域）
 * 仅当前boot.cc内部可访问的工具函数、常量，对外不可见，避免全局命名污染
 */
namespace {

/**
 * @brief 角度转弧度 constexpr编译期计算工具，实时系统无运行时开销
 * @param x 输入角度（单位 ° 度）
 * @return 输出弧度（rad）
 * @note noexcept 无异常抛出，硬实时系统安全
 */
inline constexpr auto to_rad(double x) noexcept {
    // 角度转弧度公式：deg / 180 * π
    return (x / 180.0) * std::numbers::pi_v<double>;
}

/**
 * @brief 初始化全局fast_tf静态坐标变换树
 * @param system fast_tf坐标系管理实例，存放在ECS全局资源中
 * @param config 机器人机械外参标定配置（云台、相机、枪口安装偏移/旋转）
 * 坐标系完整链路（父→子静态固定变换）：
 * world(世界原点) → odom(里程计) → gimbal_yaw(云台偏航) → gimbal_pitch(云台俯仰)
 *  gimbal_pitch 分出两条子分支：
 *  1. camera_link(相机机械安装架) → camera_optical(相机光学成像坐标系，标准OpenCV相机系)
 *  2. muzzle_link(枪口机械坐标系，用于弹道解算发射位姿)
 * 所有变换为静态固定外参，程序启动一次性写入，运行时不再修改
 */
void init_coordinate_system(fast_tf::CoordinateSystem& system, const RobotExtrinsicConfig& config) {
    // 圆周率浮点常量
    constexpr fp_t pi = std::numbers::pi;

    // 1. world、odom、云台偏航轴：无平移无旋转，单位矩阵恒等变换
    fast_tf::update<fast_tf::odom>(system, {}, 0);
    fast_tf::update<fast_tf::gimbal_yaw_fuxk_frame>(system, {}, 0);

    // 2. 云台俯仰 gimbal_pitch：仅平移偏移，无旋转
    auto pitch_trans = config.gimbal_yaw.gimbal_pitch.translation;
    fast_tf::update<fast_tf::gimbal_pitch_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_translation(
            pitch_trans.x(), pitch_trans.y(), pitch_trans.z()),
        0);

    // 3. camera_link 相机机械安装帧：平移 + 三轴RPY旋转（标定外参）
    auto cam_trans = config.gimbal_yaw.gimbal_pitch.camera_link.translation;
    auto cam_rot   = config.gimbal_yaw.gimbal_pitch.camera_link.rotation();
    fast_tf::update<fast_tf::camera_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_rpy(
            to_rad(cam_rot.roll), to_rad(cam_rot.pitch), to_rad(cam_rot.yaw),
            cam_trans.x(), cam_trans.y(), cam_trans.z()),
        0);

    // 4. camera_optical 相机光学帧固定转换规则（行业标准OpenCV相机坐标系）
    // 机械安装坐标系 → 成像光学坐标系：绕X轴-90°，再绕Z轴-90°，无平移
    // 目的：统一相机Z轴指向镜头前方（光轴），Y轴向下匹配图像像素行
    fast_tf::update<fast_tf::camera_optical_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_optical_fuxk_frame>::from_rpy(
            -pi / 2.0, 0.0, -pi / 2.0,
            0.0, 0.0, 0.0),
        0);

    // 5. muzzle_link 枪口坐标系：云台俯仰到枪口的平移+旋转外参，弹道解算依赖
    auto muzzle_trans = config.gimbal_yaw.gimbal_pitch.muzzle_link.translation;
    auto muzzle_rot   = config.gimbal_yaw.gimbal_pitch.muzzle_link.rotation();
    fast_tf::update<fast_tf::muzzle_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_rpy(
            to_rad(muzzle_rot.roll), to_rad(muzzle_rot.pitch), to_rad(muzzle_rot.yaw),
            muzzle_trans.x(), muzzle_trans.y(), muzzle_trans.z()),
        0);
}

} // 匿名命名空间结束

/**
 * @brief 框架顶层启动引导主函数，整个机器人软件唯一初始化入口
 * @param scheduler ECS全局任务调度器实例（引用传递，操作真实调度世界）
 * @param config RuntimeConfig&& 运行时全局配置，使用右值引用移动语义，转移配置所有权，避免大结构体拷贝
 * @return std::expected<void, std::string> C++23预期类型
 *        成功：返回空std::expected<void>；失败：返回std::unexpected，携带错误字符串描述
 * @ [[nodiscard]] 强制调用方接收返回值，禁止忽略启动失败，防止调度器异常运行
 *
 * 严格固定初始化执行流程（分层顺序不可颠倒，存在强数据依赖）：
 * 1. 注入Foxglove可视化全局资源，供所有调试可视化System读取配置
 * 2. 在ECS世界就地创建fast_tf坐标系系统资源，区分两种硬件后端初始化静态TF变换
 * 3. 注入视觉全局配置资源：识别颜色、功能开关掩码、ROI、跟踪缓存等
 * 4. 初始化L1底层传感器（相机、MCU硬件通信），失败直接终止启动
 * 5. 创建L2视觉推理后端、PnP位姿解算器，注册为全局资源
 * 6. 注册弹道解算参数资源，供L4规划/L5火控读取弹速补偿
 * 7. 自上而下逐层注册ECS业务System：L2感知 → L3估计 → L4规划 → L5武器
 * 8. 根据硬件手性标记，条件注册左右手共享内存采集系统
 * 9. 注册Quanta高速视频流、离线数据录制抓包系统
 * 10. 调用scheduler.build()冻结调度拓扑、资源布局，调度器进入就绪可运行状态
 */
[[nodiscard]] std::expected<void, std::string>
boot(talos::Scheduler& scheduler, RuntimeConfig&& config) {
    // 获取调度器内置ECS世界容器：所有全局资源、组件统一存储容器
    auto& world = scheduler.world();

    // 步骤1：注入Foxglove可视化全局配置资源，所有foxglove可视化系统依赖此资源
    world.insert_resource(config.foxglove);

    // 手性开关标记：区分左右向机械结构，控制共享内存采集系统是否注册
    bool chiral = false;

    // 步骤2：就地创建fast_tf坐标系系统资源（emplace_resource原地构造，无拷贝，启动性能优化）
    auto& coordinate_system = world.emplace_resource<fast_tf::CoordinateSystem>();

    // 步骤3：std::visit 多路分支匹配std::variant硬件后端配置
    // overloaded工具生成多分支lambda，自动匹配Direct直连硬件 / Daedalus专用硬件两种变体
    std::visit(
        overloaded{
            // 分支A：通用直连MCU硬件后端 DirectConfig
            [&coordinate_system, &world, &chiral](hardware::DirectConfig cfg) {
                // 加载机械外参初始化全部静态TF坐标变换
                init_coordinate_system(coordinate_system, cfg.hardware.extrinsic);
                // 注入弹道弹速全局资源，取自MCU硬件默认弹速
                world.insert_resource(
                    core::trajectory::bullet_speed_data{
                        .bullet_speed = cfg.hardware.mcu->bullet_speed_default});
                // 读取配置手性开关
                chiral = cfg.hardware.chiral;
            },
            // 分支B：Daedalus专用一体化硬件后端
            [&coordinate_system, &world](hardware::DaedalusConfig cfg) {
                init_coordinate_system(coordinate_system, cfg.extrinsic);
                // 注入Daedalus配置内的弹丸初速
                world.insert_resource(
                    core::trajectory::bullet_speed_data{.bullet_speed = cfg.bullet_speed});
            }},
        config.backend); // 传入variant变体，visit自动分发对应分支执行

    // 步骤4：注入L2视觉基础全局资源
    // 敌方识别颜色（红/蓝）
    world.insert_resource(config.vision.detect_color);
    // 机器人功能能力掩码：控制各类视觉/跟踪功能启用/关闭
    static_cast<void>(world.emplace_resource<core::CapabilityState>(
        std::span<const core::Capability>(config.vision.capabilities.get())));
    // 目标跟随模式状态机（自瞄/能量机关优先）
    static_cast<void>(world.emplace_resource<core::FollowingState>());
    // 装甲检测感兴趣区域ROI配置
    world.insert_resource(config.vision.armor->readback_roi);
    // L2视觉跟踪帧缓存资源
    world.insert_resource(L2::TrackerReadbackCache{});

    // 打印视觉基础配置日志，启动阶段快速核查参数
    SPDLOG_INFO("detect color: {}", config.vision.detect_color);
    SPDLOG_INFO("enabled capabilities: [{}]", fmt::join(*config.vision.capabilities, ", "));

    // ===================== 步骤5：初始化L1底层传感器硬件 =====================
    // runtime::setup_l1 打开相机、初始化MCU通信、创建硬件数据流SPMC通道
    auto setup_result = runtime::setup_l1(world, scheduler, config.backend);
    // 初始化失败：返回错误字符串，终止整个启动流程
    if (!setup_result) {
        return std::unexpected(fmt::format("L1 传感器初始化失败: {}", setup_result.error()));
    }

    // ===================== 步骤6：初始化L2感知层推理与几何解算后端 =====================
    // 创建装甲目标检测推理后端（YOLO/ONNX/TensorRT等推理引擎封装）
    auto backend_handle_result = L2::create_detector_backend_handle(config.vision.armor.get());
    if (!backend_handle_result) {
        return std::unexpected(
            fmt::format("创建装甲检测推理后端失败: {}", backend_handle_result.error()));
    }
    SPDLOG_INFO("当前装甲检测推理后端: {}", backend_handle_result->name);

    // 将推理后端实例、PnP位姿解算器注册为全局共享资源，所有L2系统共用
    world.insert_resource(std::move(backend_handle_result->backend));
    world.insert_resource(L2::create_pnp_solver(setup_result.value().camera_config));

    // ===================== 步骤7：注册弹道解算全局资源 =====================
    core::trajectory::register_resource(scheduler, std::move(config.vision.trajectory));
    // 打印弹丸初速用于启动校验
    SPDLOG_INFO(
        "bullet initial speed = {:.3f} m/s",
        world.get_res<core::trajectory::bullet_speed_data>()->bullet_speed);

    // ===================== 步骤8：自上而下分层注册所有ECS业务系统 =====================
    // L2 感知层系统：装甲检测、LDM大符光斑、Rune能量机关图像检测
    L2::register_detection_systems(scheduler);
    L2::ldm::register_ldm_systems(
        scheduler, std::move(config.vision.ldm), setup_result.value().camera_config);
    rune::register_rune_detection_systems(scheduler, std::move(config.vision.rune_detector));

    // L3 状态估计层：多目标卡尔曼跟踪、简易LDM滤波、能量机关角速度估计算法系统
    L3::register_tracker_systems(scheduler, std::move(config.vision.l3->tracker));
    L3::ldm::register_naive_ldm_systems(scheduler, config.vision.naive_ldm);
    energy_meter::register_energy_meter_systems(scheduler, std::move(config.vision.energy_meter));

    // L4 运动规划层：目标轨迹预测、MPC瞄准补偿、最优解算系统
    L4::register_l4_planning_systems(scheduler, std::move(config.vision.l4.get()));

    // L5 武器控制层：最终火控解算、摩擦轮发射速度控制、发射判定系统（依赖L4规划输出）
    L5::register_enhanced_weapon_system(scheduler, std::move(config.vision.l5.get()));

    // ===================== 步骤9：条件注册手性数据采集系统 =====================
    // 手性开关开启时，注册共享内存采集系统，用于左右手机构数据调试
    if (chiral) {
        chiral::register_chiral_collector_system(scheduler);
        SPDLOG_INFO("手性数据采集系统已注册完成");
    }

    // ===================== 步骤10：注册高速视频流、离线录制抓包系统 =====================
    // 提取Quanta视频编码全局配置，填充图像滤波参数
    auto quanta_cfg                  = config.vision.quanta;
    quanta_cfg.filter                = config.vision.quanta_filter;
    const auto& stream_camera_config = setup_result.value().camera_config;
    // 注册H.265视频流编码、Foxglove推流ECS系统
    auto quanta_stream_result        = runtime::register_quanta_stream_systems(
        world, scheduler, quanta_cfg,
        static_cast<int>(stream_camera_config.width),
        static_cast<int>(stream_camera_config.height));
    if (!quanta_stream_result) {
        return std::unexpected(
            fmt::format("注册Quanta高速数据流系统失败: {}", quanta_stream_result.error()));
    }

    // 注册离线数据录制系统：抓取图像、跟踪、调试JSON消息保存至Mcap文件
    runtime::register_runtime_capturer_system(scheduler, config.capturer, config.launch);

    // ===================== 步骤11：调度器收尾构建，冻结拓扑进入就绪状态 =====================
    /**
     * scheduler.build() 核心行为：
     * 1. 解析全部注册System的资源/SPMC通道读写依赖关系
     * 2. 构建并行拓扑图：无依赖系统并行调度，存在读写依赖串行调度
     * 3. 锁定lifecycle_原子状态为Configuring→Running，冻结资源布局、系统列表
     * 4. 分配线程池、固定周期定时器，完成全部运行时初始化
     * build后不可再注册任何系统、插入资源，否则触发运行时断言崩溃
     */
    if (auto build_result = scheduler.build(); !build_result) {
        return std::unexpected(fmt::format("调度器构建拓扑失败: {}", build_result.error()));
    }

    // 全流程初始化无错误，返回空预期，表示启动成功
    return {};
}

} // namespace fcs