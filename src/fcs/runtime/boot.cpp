// ==============================================
// 文件用途：fcs框架全局唯一启动入口实现文件
// 头文件声明boot顶层初始化函数，本.cc实现完整启动逻辑
// 整个机器人火控程序从main最终调用fcs::boot完成全部初始化
// ==============================================
#include "runtime/boot.hpp"

// ===================== L1-L5五层业务模块头文件 =====================
// L1：硬件原始数据输出层，只负责相机、MCU、IMU硬件读写，无算法
#include "L1_sensor/output_interface.hpp"

// L2感知-装甲识别：ROI图像裁剪逻辑、装甲检测ECS系统注册函数
#include "L2_perception/armor/readback_roi.hpp"
#include "L2_perception/armor/systems.hpp"
// L2感知-固定能量机关LDM：光斑检测任务系统注册
#include "L2_perception/ldm/ldm_systems.hpp"
// L2感知-旋转大符Rune：动态能量机关图像检测、光斑提取系统
#include "L2_perception/rune/rune_systems.hpp"

// L3状态估计层：滤波、角速度解算，消除图像噪声，输出三维目标
#include "L3_estimation/energy_meter/energy_meter_systems.hpp"
#include "L3_estimation/ldm_naive/ldm_naive_systems.hpp"
#include "L3_estimation/tracker_systems.hpp"

// L4运动规划层：目标轨迹预测、MPC云台角度补偿、弹道预计算
#include "L4_planning/planning_systems.hpp"

// L5武器顶层：最终火控解算、摩擦轮转速控制、发射判定逻辑
#include "L5_weapon/enhanced/weapon_systems.hpp"

// 左右手分体机器人专用共享内存调试采集系统
#include "chiral/chiral_collector_system.hpp"

// ===================== 框架底层核心通用组件头文件 =====================
// 装甲全套数据结构：单装甲、批量检测、角点、PnP三维结果
#include "core/armor_types.hpp"
// 全局SPMC无锁通道Topic枚举，定义所有线程间数据流标识
#include "core/channel_topics.hpp"
// Runtime全局上下文结构体，存储启动、硬件、视觉全部配置
#include "core/runtime.hpp"
// 弹道计算资源容器：弹速、空气阻力、重力补偿、分段查表
#include "core/trajectory/resource.hpp"
// 全局基础别名：Eigen向量/位姿、枚举、状态机、功能掩码
#include "core/types.hpp"
// fast_tf静态坐标系枚举：world/odom/gimbal/camera/muzzle所有坐标系标识
#include "frame.hpp"
// Quanta视频编码组件：H.265硬编码、WebSocket Foxglove实时推流
#include "quanta/stream_transport.hpp"
// 离线录制组件：图像/跟踪/调试消息存入Mcap文件离线回放
#include "runtime/capturer.hpp"
// TOML配置文件加载、语法校验、类型转换工具
#include "runtime/config_loader.hpp"
// L1相机+MCU统一初始化工具，创建图像SPMC输入通道
#include "runtime/l1_l2_setup.hpp"
// JPEG/PNG/视频流通用图像编码工具封装
#include "runtime/stream_encode.hpp"
// 将stdexpected错误码转为人类可读日志字符串工具
#include "scheduler/error_formatter.hpp"

// ===================== 第三方开源库 & C++标准库 =====================
// fmt高性能格式化库，替代老旧std::stringstream，无内存碎片
#include <fmt/core.h>
// fmt打印容器/数组专用头
#include <fmt/ranges.h>
// C++20标准数学常量，包含高精度圆周率pi
#include <numbers>
// std::visit多路匹配辅助工具，简化variant多硬件分支代码
#include <primitive/overloaded.hpp>
// 分级日志库：INFO/WARN/ERROR全局统一日志输出
#include <spdlog/spdlog.h>

/**
 * @namespace fcs
 * 框架顶层根命名空间，隔离所有框架代码，避免第三方库命名冲突
 * 软件分层顺序（数据流从上到下不可逆）：
 * L1硬件原始数据 → L2图像目标检测 → L3三维滤波跟踪 → L4云台运动规划 → L5武器发射控制
 * 核心函数boot：程序唯一初始化入口，所有资源/任务/通道在此一次性注册
 * boot执行固定顺序：全局资源注入 → TF坐标系初始化 → 硬件初始化 → 五层ECS系统注册 → 调度拓扑冻结
 */
namespace fcs {

/**
 * @namespace 匿名命名空间
 * 仅本boot.cc文件内部可见工具函数，不会导出全局符号
 * 作用：避免污染全局命名空间，外部其他文件无法调用这里的函数
 */
namespace {

/**
 * @brief 角度转弧度工具函数
 * @param x 输入角度，单位：度(°)
 * @return 转换后弧度值(rad)
 * inline：函数逻辑直接嵌入调用处，无函数调用开销
 * constexpr：编译期即可完成计算，硬实时无运行计算耗时
 * noexcept：函数绝对不会抛出异常，实时调度无异常分支开销
 */
inline constexpr auto to_rad(double x) noexcept {
    // 角度转弧度数学公式：弧度 = 角度 / 180 * π
    return (x / 180.0) * std::numbers::pi_v<double>;
}

/**
 * @brief 初始化全局fast_tf静态坐标变换树
 * @param system ECS全局存储的TF坐标系实例（引用直接修改资源）
 * @param config 机械标定外参：云台、相机、枪口安装平移旋转
 * 静态坐标系父子层级（程序启动一次性写入，运行全程只读不修改）：
 * world世界原点 → odom里程计 → gimbal_yaw云台偏航轴 → gimbal_pitch云台俯仰轴
 * gimbal_pitch分出两条独立子坐标系：
 * 1.camera_link相机机械安装架 → camera_optical标准OpenCV光学坐标系
 * 2.muzzle_link枪口发射坐标系（弹道解算专用）
 */
void init_coordinate_system(fast_tf::CoordinateSystem& system, const RobotExtrinsicConfig& config) {
    // 定义全局圆周率常量，编译期求值
    constexpr fp_t pi = std::numbers::pi;

    // ========== 1.world、odom、云台偏航轴：无平移无旋转，单位矩阵 ==========
    // fast_tf::update<Frame> 模板萃取：编译期传入坐标系枚举，写入静态变换
    fast_tf::update<fast_tf::odom>(system, {}, 0);
    fast_tf::update<fast_tf::gimbal_yaw_fuxk_frame>(system, {}, 0);

    // ========== 2.云台俯仰轴：仅存在平移偏移，无旋转角度 ==========
    // 从标定配置读取云台俯仰机械平移向量
    auto pitch_trans = config.gimbal_yaw.gimbal_pitch.translation;
    // 构造纯平移变换，写入TF系统
    fast_tf::update<fast_tf::gimbal_pitch_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_translation(
            pitch_trans.x(), pitch_trans.y(), pitch_trans.z()),
        0);

    // ========== 3.camera_link相机机械安装坐标系：平移+三轴RPY旋转 ==========
    // 读取相机安装平移向量
    auto cam_trans = config.gimbal_yaw.gimbal_pitch.camera_link.translation;
    // 读取相机安装滚转/俯仰/偏航角度（单位度）
    auto cam_rot   = config.gimbal_yaw.gimbal_pitch.camera_link.rotation();
    // from_rpy：输入三轴角度(自动转弧度)+三维平移，生成完整坐标变换
    fast_tf::update<fast_tf::camera_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_rpy(
            to_rad(cam_rot.roll), to_rad(cam_rot.pitch), to_rad(cam_rot.yaw),
            cam_trans.x(), cam_trans.y(), cam_trans.z()),
        0);

    // ========== 4.camera_optical相机光学标准坐标系固定转换 ==========
    // 行业统一标准转换规则：机械相机系 → 成像光学系
    // 变换：先绕X轴旋转-90°，再绕Z轴旋转-90°，无平移
    // 作用：让相机Z轴指向镜头前方光轴，Y轴匹配图像向下像素行
    fast_tf::update<fast_tf::camera_optical_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_optical_fuxk_frame>::from_rpy(
            -pi / 2.0, 0.0, -pi / 2.0,
            0.0, 0.0, 0.0),
        0);

    // ========== 5.muzzle_link枪口发射坐标系：弹道解算核心外参 ==========
    // 读取云台到枪口机械平移
    auto muzzle_trans = config.gimbal_yaw.gimbal_pitch.muzzle_link.translation;
    // 读取枪口三轴安装角度
    auto muzzle_rot   = config.gimbal_yaw.gimbal_pitch.muzzle_link.rotation();
    // 构造枪口完整坐标变换存入TF系统
    fast_tf::update<fast_tf::muzzle_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_rpy(
            to_rad(muzzle_rot.roll), to_rad(muzzle_rot.pitch), to_rad(muzzle_rot.yaw),
            muzzle_trans.x(), muzzle_trans.y(), muzzle_trans.z()),
        0);
}

} // 匿名命名空间结束，内部工具仅本文件可用

/**
 * @brief 框架顶层唯一启动函数，整个火控程序初始化总入口
 * @param scheduler talos::Scheduler& ECS全局调度器引用
 *        引用传递，直接操作调度器内部世界、资源、任务列表
 * @param config RuntimeConfig&& 运行配置右值引用
 *        使用移动语义转移配置所有权，超大TOML结构体无需深拷贝
 * @return std::expected<void, std::string> C++23无异常错误处理类型
 *        初始化成功：返回空expected<void>
 *        初始化失败：std::unexpected携带可读错误字符串，无抛异常
 * @ [[nodiscard]] 强制调用代码接收返回值，禁止忽略启动失败
 *        防止硬件初始化失败后调度器异常运行
 *
 * 初始化顺序严格固定，存在强数据依赖，不可调换顺序：
 * 1. 注入Foxglove可视化全局资源
 * 2. 创建TF坐标系全局资源，根据硬件variant分支加载机械外参
 * 3. 注入视觉基础全局配置（识别颜色、功能开关、ROI、状态机）
 * 4. L1底层相机/MCU硬件初始化，失败直接终止启动
 * 5. 创建装甲检测推理引擎、PnP三维解算器，注册全局共享资源
 * 6. 注入弹道弹速、重力补偿全局资源
 * 7. 自上而下注册L2→L3→L4→L5全部ECS业务任务系统
 * 8. 手性标记开启时，注册左右手共享内存采集系统
 * 9. 注册Quanta视频推流、离线Mcap录制系统
 * 10. scheduler.build()解析任务依赖，构建并行拓扑、冻结调度环境
 *        build执行后禁止新增任何资源/系统，否则运行断言崩溃
 */
[[nodiscard]] std::expected<void, std::string>
boot(talos::Scheduler& scheduler, RuntimeConfig&& config) {
    // 从调度器获取ECS全局World容器
    // World是全局资源仓库，所有单例资源（TF、推理、弹道）统一存放
    auto& world = scheduler.world();

    // ========== 步骤1：注入Foxglove可视化全局配置资源 ==========
    // insert_resource：将foxglove配置存入全局资源，所有可视化任务可读取
    world.insert_resource(config.foxglove);

    // 手性开关标记：bool变量记录硬件是否为左右手分体结构
    // 用于后续判断是否注册共享内存采集系统
    bool chiral = false;

    // ========== 步骤2：原地创建fast_tf坐标系全局资源 ==========
    // emplace_resource：直接在ECS堆内存构造对象，无临时拷贝，性能优于insert
    auto& coordinate_system = world.emplace_resource<fast_tf::CoordinateSystem>();

    // ========== 步骤3：std::visit处理两种硬件variant分支 ==========
    // config.backend是std::variant<DirectConfig, DaedalusConfig>变体类型
    // std::visit：根据当前硬件类型自动匹配对应lambda分支
    // overloaded{}：简化多分支lambda写法，无需手写模板重载
    std::visit(
        overloaded{
            // 分支A：通用直连MCU硬件配置 DirectConfig
            // &捕获外部所有需要使用的变量：坐标系统、world容器、手性标记
            [&coordinate_system, &world, &chiral](hardware::DirectConfig cfg) {
                // 调用本文件私有函数，加载整套机械外参写入TF树
                init_coordinate_system(coordinate_system, cfg.hardware.extrinsic);
                // 创建弹道初速全局资源，取值来自MCU硬件默认弹速
                world.insert_resource(
                    core::trajectory::bullet_speed_data{
                        .bullet_speed = cfg.hardware.mcu->bullet_speed_default});
                // 读取配置内手性开关，赋值给外层chiral变量
                chiral = cfg.hardware.chiral;
            },
            // 分支B：Daedalus一体化集成硬件配置
            // 无需chiral，仅捕获坐标系统与world容器
            [&coordinate_system, &world](hardware::DaedalusConfig cfg) {
                // 同样初始化TF静态坐标树，使用Daedalus专属外参
                init_coordinate_system(coordinate_system, cfg.extrinsic);
                // 注入一体化硬件自带弹速参数
                world.insert_resource(
                    core::trajectory::bullet_speed_data{.bullet_speed = cfg.bullet_speed});
            }},
        // 传入variant变体变量，visit自动分发匹配分支
        config.backend);

    // ========== 步骤4：注入视觉层全局静态配置资源 ==========
    // 1. 敌方识别颜色（红/蓝队伍）存入全局资源
    world.insert_resource(config.vision.detect_color);
    // 2. 功能能力掩码：控制各类视觉/跟踪算法启用关闭
    // emplace_resource原地构造CapabilityState全局状态
    static_cast<void>(world.emplace_resource<core::CapabilityState>(
        std::span<const core::Capability>(config.vision.capabilities.get())));
    // 3. 目标跟踪模式状态机：自瞄优先 / 能量机关优先
    static_cast<void>(world.emplace_resource<core::FollowingState>());
    // 4. 装甲检测图像裁剪ROI区域配置存入全局
    world.insert_resource(config.vision.armor->readback_roi);
    // 5. L2层检测结果缓存资源，存放单帧装甲检测数据
    world.insert_resource(L2::TrackerReadbackCache{});

    // 打印启动日志，快速核对视觉基础配置是否加载正确
    // fmt格式化打印识别颜色
    SPDLOG_INFO("detect color: {}", config.vision.detect_color);
    // fmt::join遍历容器，打印所有启用的功能掩码
    SPDLOG_INFO("enabled capabilities: [{}]", fmt::join(*config.vision.capabilities, ", "));

    // ===================== 步骤5：初始化L1底层相机、MCU硬件 =====================
    // runtime::setup_l1：底层硬件统一初始化函数
    // 内部逻辑：打开相机设备、初始化MCU串口、创建图像SPMC输入无锁通道
    // 返回stdexpected：成功携带相机配置；失败携带错误字符串
    auto setup_result = runtime::setup_l1(world, scheduler, config.backend);
    // 判断硬件初始化失败分支
    if (!setup_result) {
        // fmt拼接错误信息，返回unexpected终止整个启动流程
        return std::unexpected(fmt::format("L1 传感器初始化失败: {}", setup_result.error()));
    }

    // ===================== 步骤6：创建L2装甲推理后端、PnP解算器 =====================
    // 调用工厂函数创建YOLO/ONNX/TensorRT推理引擎句柄
    auto backend_handle_result = L2::create_detector_backend_handle(config.vision.armor.get());
    // 推理引擎创建失败，返回错误终止启动
    if (!backend_handle_result) {
        return std::unexpected(
            fmt::format("创建装甲检测推理后端失败: {}", backend_handle_result.error()));
    }
    // 打印当前使用的推理后端名称（TensorRT/CPU ONNX）
    SPDLOG_INFO("当前装甲检测推理后端: {}", backend_handle_result->name);

    // 将推理引擎所有权移动到全局资源，所有L2检测任务共享同一个推理实例
    world.insert_resource(std::move(backend_handle_result->backend));
    // 根据相机内参创建PnP三维位姿解算器，存入全局资源   //1. 调用 .value() 拿到整个结构体（这里会发生一次拷贝）
    world.insert_resource(L2::create_pnp_solver(setup_result.value().camera_config));

    // ===================== 步骤7：注册弹道解算全局资源 =====================
    // 弹道配置移动传入注册函数，存入全局资源容器
    core::trajectory::register_resource(scheduler, std::move(config.vision.trajectory));
    // 读取全局弹速资源并打印，启动校验弹速参数
    SPDLOG_INFO(
        "bullet initial speed = {:.3f} m/s",
        world.get_res<core::trajectory::bullet_speed_data>()->bullet_speed);

    // ===================== 步骤8：自上而下分层注册全部ECS业务系统 =====================
    // L2感知层：装甲检测、LDM大符、Rune旋转能量机关图像任务
    L2::register_detection_systems(scheduler);
    L2::ldm::register_ldm_systems(
        scheduler, std::move(config.vision.ldm), setup_result.value().camera_config);
    rune::register_rune_detection_systems(scheduler, std::move(config.vision.rune_detector));

    // L3估计层：卡尔曼多目标跟踪、简易LDM滤波、能量机关角速度估计
    L3::register_tracker_systems(scheduler, std::move(config.vision.l3->tracker));
    L3::ldm::register_naive_ldm_systems(scheduler, config.vision.naive_ldm);
    energy_meter::register_energy_meter_systems(scheduler, std::move(config.vision.energy_meter));

    // L4规划层：目标运动预测、MPC云台角度补偿、最优瞄准轨迹计算
    L4::register_l4_planning_systems(scheduler, std::move(config.vision.l4.get()));

    // L5武器顶层：弹道最终解算、摩擦轮调速、发射判定逻辑任务
    L5::register_enhanced_weapon_system(scheduler, std::move(config.vision.l5.get()));

    // ===================== 步骤9：手性功能条件注册 =====================
    // chiral标记为true代表左右手分体机器人，注册共享内存采集任务
    if (chiral) {
        chiral::register_chiral_collector_system(scheduler);
        SPDLOG_INFO("手性数据采集系统已注册完成");
    }

    // ===================== 步骤10：Quanta视频流、离线录制系统注册 =====================

    auto quanta_cfg      = config.vision.quanta;         // 复制视频基础配置
    quanta_cfg.filter                          = config.vision.quanta_filter;  //补充图像滤波参数
    // 从硬件初始化结果读取相机分辨率
    const auto& stream_camera_config = setup_result.value().camera_config;
    // 注册H.265编码、Foxglove WebSocket推流ECS任务
    auto quanta_stream_result        = runtime::register_quanta_stream_systems(
        world, scheduler, quanta_cfg,
        static_cast<int>(stream_camera_config.width),
        static_cast<int>(stream_camera_config.height));
    // 视频流注册失败，返回错误终止启动
    if (!quanta_stream_result) {
        return std::unexpected(
            fmt::format("注册Quanta高速数据流系统失败: {}", quanta_stream_result.error()));
    }

    // 注册离线Mcap录制任务：抓取图像/跟踪/调试消息离线保存
    runtime::register_runtime_capturer_system(scheduler, config.capturer, config.launch);

    // ===================== 步骤11：调度器构建拓扑，冻结运行环境 =====================
    // scheduler.build()核心行为逐句注释：
    // 1. 遍历所有注册System，分析通道读写、全局资源依赖关系
    // 2. 自动生成并行调度拓扑：无依赖任务多线程并行，存在读写依赖串行执行
    // 3. 标记调度器状态从配置阶段转为运行阶段，锁定资源与任务列表
    // 4. 初始化线程池、周期定时器、无锁通道缓冲区
    // build执行后禁止新增任何资源/任务，否则运行断言崩溃
    if (auto build_result = scheduler.build(); !build_result) {
        return std::unexpected(fmt::format("调度器构建拓扑失败: {}", build_result.error()));
    }

    // 所有初始化步骤无任何错误，返回空expected代表启动成功
    return {};
}

} // fcs顶层命名空间结束