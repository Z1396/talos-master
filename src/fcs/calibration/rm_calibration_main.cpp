/**
 * @file rm_calibration_main.cpp
 * @brief RM 机器人标定工具主程序 - 相机内参标定 + 手眼外参标定一体化工具
 *
 * 支持三种工作模式：
 * 1. Intrinsic：仅相机内参标定（棋盘格/ChArUco板）
 * 2. Handeye：手眼标定，必须传入已标定好的相机内参文件
 * 3. Full：完整流程，先采图标定内参，再采集云台位姿做手眼标定
 *
 * 运行命令示例：
 *   ./rm_calibration --config calibration.toml
 *   ./rm_calibration --mode Intrinsic
 *   ./rm_calibration --mode Handeye --intrinsic camera_intrinsic.toml
 *   ./rm_calibration --daedalus  # 使用Daedalus仿真器，不连接真实硬件相机/MCU
 *
 * 交互按键说明：
 * 空格：采集当前帧标定样本
 * c：手动触发计算标定结果
 * q：退出程序
 */

// ===================== 标定业务模块头文件 =====================
#include "calibration/calibration_board.hpp"        // 标定板基类、棋盘格/ChArUco板抽象
#include "calibration/calibration_config.hpp"         // 标定工具总配置结构体 CalibrationConfig
#include "calibration/calibration_systems.hpp"        // 注册标定系统任务（调度器System注册函数）
#include "calibration/chessboard_detector.hpp"        // 棋盘格角点/ChArUco标记检测算法封装
#include "calibration/handeye_calibrator.hpp"         // 手眼标定求解器（AX=XB多种求解方法）
#include "calibration/intrinsic_calibrator.hpp"        // 相机内参标定求解器（OpenCV标定封装）

// ===================== 机器人通用业务头文件 =====================
#include "camera_config.hpp"                          // 相机参数结构体 CameraConfig
#include "core/armor_types.hpp"                       // 装甲板相关基础类型（颜色、装甲结构）
#include "core/trajectory/resource.hpp"               // 弹道、弹速全局资源定义

// ===================== 日志、调试可视化 =====================
#include "spdlog_hook.hpp"                            // spdlog日志全局初始化、格式化钩子

// ===================== Talos框架底层公共头文件 =====================
#include "config.hpp"                                 // CMake编译宏、全局编译配置
#include "core/channel_topics.hpp"                    // 消息通道话题定义（相机图像、云台数据）
#include "core/time.hpp"                              // 高精度时间戳、时钟工具
#include "foxglove_server.hpp"                        // Foxglove可视化服务（Websocket/MCAP录包）
#include "foxglove_sink.hpp"                          // spdlog日志输出到Foxglove可视化端
#include "runtime/l1_l2_setup.hpp"                    // L1硬件层+L2感知层运行时初始化（相机/MCU/IMU）
#include "scheduler/scheduler.hpp"                    // 全局任务调度器（ECS架构，System资源驱动）
#include "toml_helper.hpp"                             // TOML通用解析工具，toml表转C++结构体
#include "toml_helper_eigen.hpp"                      // Eigen矩阵/向量TOML序列化/反序列化扩展

// ===================== 调度器错误格式化打印 =====================
#include "scheduler/error_formatter.hpp"

// ===================== C++标准库、系统API、第三方库 =====================
#include <atomic>                                     // 原子布尔，跨线程安全的运行标志
#include <chrono>                                     // 高精度时间休眠、时间戳
#include <csignal>                                    // Linux信号捕获（Ctrl+C退出）
#include <iostream>                                   // 控制台标准输出（帮助文档打印）
#include <spdlog/spdlog.h>                            // 高性能日志库（INFO/WARN/ERROR/CRITICAL）
#include <thread>                                     // C++标准线程库（调度器后台运行）

// 简化命名空间：所有标定相关类型直接使用，无需重复 fcs::calibration::
using namespace fcs::calibration;

// ===================== 全局信号处理变量 =====================
/**
 * @brief 全局原子运行标志
 * atomic保证主线程、调度器子线程、信号处理函数之间无数据竞争
 * Ctrl+C信号触发后置为false，主循环退出，程序安全释放资源
 */
std::atomic<bool> g_running{true};

/**
 * @brief 系统信号回调函数
 * @param sig 捕获到的信号值（SIGINT Ctrl+C / SIGTERM 程序终止）
 * 作用：捕获退出信号，修改全局运行标志，优雅退出而非直接强制kill
 */
void signal_handler(int /*sig*/) {
    g_running = false;
}

/**
 * @brief 打印程序帮助文档、命令行参数说明、运行示例、交互按键
 */
void print_usage() {
    std::cout << R"(
RM Calibration Tool - Camera Intrinsic and Hand-Eye Calibration

Usage:
  rm_calibration [options]

Options:
  --config <path>       标定配置文件路径 (默认: calibration.toml)
  --mode <mode>         强制覆盖标定模式: Intrinsic(内参), Handeye(手眼), Full(全流程)
  --intrinsic <path>    加载已存在的相机内参文件（Handeye模式必须传入）
  --daedalus            使用Daedalus仿真器，不启动真实硬件相机/MCU
  --help / -h           打印当前帮助信息并退出

Examples:
  # 使用真实硬件，仅执行相机内参标定
  rm_calibration --mode Intrinsic

  # Daedalus仿真环境下执行内参标定
  rm_calibration --mode Intrinsic --daedalus

  # 加载已有内参，执行云台手眼标定
  rm_calibration --mode Handeye --intrinsic camera_intrinsic.toml

  # 完整流程：先采图标内参，再采云台位姿做手眼标定
  rm_calibration --mode Full

交互按键：
空格Space：抓拍当前帧作为标定样本
c：手动触发执行标定求解
q：退出标定程序
)";
}

/**
 * @brief 程序入口主函数
 * @param argc 命令行参数总个数
 * @param argv 命令行参数字符串数组
 * @return int 程序退出码：0正常退出，非0为异常错误
 */
int main(int argc, char* argv[]) {
    // 1. 全局日志系统初始化，加载自定义日志格式化钩子
    init_logger();

    // ========== 命令行参数默认初始化 ==========
    std::string config_path = "calibration.toml";          // 默认TOML配置文件
    std::string intrinsic_path;                            // 外部加载的相机内参文件路径
    std::optional<CalibrationMode> mode_override;          // 命令行覆盖标定模式（可选）
    std::optional<bool> daedalus_override;                  // 命令行强制开启仿真器（可选）

    // ========== 循环解析所有命令行参数 ==========
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            // 打印帮助并直接退出程序
            print_usage();
            return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            // 读取--config后的文件路径
            config_path = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            // 解析标定模式枚举字符串（magic_enum实现字符串<->枚举转换）
            std::string mode_str = argv[++i];
            auto mode = magic_enum::enum_cast<CalibrationMode>(mode_str);
            if (mode) {
                mode_override = *mode;
            } else {
                SPDLOG_ERROR("无效标定模式: {}", mode_str);
                return 1;
            }
        } else if (arg == "--intrinsic" && i + 1 < argc) {
            // 读取外部内参TOML路径，手眼标定依赖该文件
            intrinsic_path = argv[++i];
        } else if (arg == "--daedalus") {
            // 标记使用仿真器，跳过真实硬件初始化
            daedalus_override = true;
        }
    }

    // ========== 注册系统退出信号捕获 ==========
    std::signal(SIGINT, signal_handler);   // 捕获 Ctrl+C
    std::signal(SIGTERM, signal_handler);   // 捕获系统终止信号 kill
    SPDLOG_INFO("RM标定工具启动中...");
    SPDLOG_INFO("加载配置文件: {}", config_path);

    // ========== 加载标定总配置TOML ==========
    auto config_result = CalibrationConfig::load_from_file(config_path);
    if (!config_result) {
        SPDLOG_ERROR("加载标定配置失败: {}", config_result.error());
        return 1;
    }
    // 配置使用shared_ptr全局共享，供所有标定System读取
    auto config = std::make_shared<CalibrationConfig>(std::move(*config_result));

    // 命令行覆盖标定模式（优先级高于TOML文件内mode配置）
    if (mode_override) {
        config->mode = *mode_override;
    }
    // 命令行强制启用仿真器
    if (daedalus_override) {
        config->input.daedalus = *daedalus_override;
    }

    // 标记当前使用仿真/真实硬件
    const bool use_daedalus = config->input.daedalus;

    // 打印运行时核心配置日志，方便调试确认参数
    SPDLOG_INFO("标定模式: {}", magic_enum::enum_name(config->mode));
    SPDLOG_INFO("图像输入源: {}", use_daedalus ? "Daedalus仿真器" : "真实硬件相机");
    SPDLOG_INFO("标定板类型: {}", magic_enum::enum_name(config->board.type));
    SPDLOG_INFO("标定板尺寸: {}x{}格, 方格边长 {} mm",
                config->board.width, config->board.height,
                config->board.square_size * 1000);

    // ========== 创建标定板检测器（棋盘格/ChArUco） ==========
    auto board_result = create_board(config->board, config->charuco);
    if (!board_result) {
        SPDLOG_ERROR("创建标定板检测器失败: {}", board_result.error());
        return 1;
    }
    // 标定板检测器全局共享
    auto board = std::shared_ptr<CalibrationBoard>(std::move(*board_result));

    // ========== 实例化内参、手眼标定求解器 ==========
    // 内参标定器：绑定采集参数、内参输出配置
    auto intrinsic_calibrator = std::make_shared<IntrinsicCalibrator>(config->capture, config->intrinsic);
    // 手眼标定器：绑定采集参数、手眼求解配置
    auto handeye_calibrator = std::make_shared<HandEyeCalibrator>(config->capture, config->handeye);

    // ========== 加载外部传入的相机内参文件 ==========
    auto intrinsic_result = std::make_shared<IntrinsicResult>();
    if (!intrinsic_path.empty()) {
        // 解析外部相机内参TOML
        auto loaded = toml::parse_file(intrinsic_path);
        if (!loaded) {
            SPDLOG_ERROR("加载外部内参文件失败: {}", intrinsic_path);
            return 1;
        }
        // 读取[相机]表，反序列化为CameraConfig
        if (auto cam = loaded.table()["camera"].as_table()) {
            auto cam_cfg = toml_helper::from_table<fcs::CameraConfig>(*cam);
            if (cam_cfg) {
                // 将TOML内参复制到全局标定内参结果容器
                intrinsic_result->camera_matrix       = cam_cfg->camera_matrix;
                intrinsic_result->distort_coefficient = cam_cfg->distort_coefficient;
                intrinsic_result->width               = cam_cfg->width;
                intrinsic_result->height              = cam_cfg->height;
                SPDLOG_INFO("成功从 {} 加载相机内参", intrinsic_path);
            }
        }
    }

    // ========== 模式合法性校验 ==========
    // 手眼标定必须提前拥有有效相机内参，宽高为0代表未加载
    if (config->mode == CalibrationMode::Handeye && intrinsic_result->width == 0) {
        SPDLOG_ERROR("手眼标定模式必须传入相机内参文件，请使用 --intrinsic <路径>");
        return 1;
    }

    // ========== 全局标定状态管理 ==========
    auto status = std::make_shared<CalibrationStatus>();
    status->state = CalibrationState::Idle;                // 初始状态：空闲
    status->target_samples = config->capture.min_samples;  // 自动标定所需最少样本数

    // ========== 初始化Talos ECS调度器与世界资源容器 ==========
    talos::World world;                                    // ECS全局资源仓库（所有单例资源存在此处）
    talos::Scheduler scheduler(world);                     // 任务调度器，驱动所有感知/标定System运行

    // 创建坐标变换TF树资源（云台、相机、枪口坐标转换依赖）
    [[maybe_unused]] auto& tf_buffer = world.emplace_resource<fast_tf::CoordinateSystem>();

    // ========== 构造硬件配置结构体（真实硬件初始化使用） ==========
    fcs::HardwareConfig hardware_config;
    hardware_config.camera          = fcs::CameraConfig{};
    hardware_config.camera->width   = config->width;
    hardware_config.camera->height  = config->height;
    hardware_config.camera->profile = config->profile;
    hardware_config.mcu             = fcs::McuConfig{};
    hardware_config.extrinsic       = fcs::RobotExtrinsicConfig{};
    // 插入弹道弹速默认资源、机器人颜色资源（底层L1初始化依赖）
    world.insert_resource(fcs::core::trajectory::bullet_speed_data{.bullet_speed = 25.0});
    world.insert_resource(fcs::ArmorColor::Blue);

    // ========== L1底层硬件+L2感知层初始化 ==========
    // 参数说明：
    // mock_mcu=false 不模拟电控板；use_daedalus仿真时硬件配置传nullptr跳过相机/MCU
    auto setup_result = fcs::runtime::setup_l1(
        world, scheduler,
        /*mock_mcu=*/false,
        use_daedalus,
        use_daedalus ? nullptr : &hardware_config);
    if (!setup_result) {
        SPDLOG_ERROR("L1/L2运行时环境初始化失败: {}", setup_result.error());
        return 1;
    }

    // ========== Foxglove可视化服务初始化（Web可视化/录包MCAP） ==========
    std::shared_ptr<fcs::visualization::FoxgloveServer> foxglove_server;
    if (config->foxglove.enabled) {
        auto server_result = fcs::visualization::create_foxglove_server(config->foxglove);
        if (!server_result) {
            SPDLOG_WARN("Foxglove可视化服务启动失败: {}", server_result.error());
        } else {
            foxglove_server = std::move(*server_result);
            // 将全局spdlog日志输出同步到Foxglove网页端
            fcs::visualization::attach_foxglove_sink(*foxglove_server);
            world.insert_resource(foxglove_server);
            // 区分Websocket实时可视化 / MCAP离线录包两种模式打印日志
            if (config->foxglove.transport == fcs::FoxgloveTransport::WebSocket) {
                SPDLOG_INFO("Foxglove WebSocket可视化服务启动 {}:{}", config->foxglove.host, config->foxglove.port);
            } else {
                SPDLOG_INFO("Foxglove MCAP录包文件输出路径: {}", config->foxglove.mcap_path);
            }
        }
    }

    // ========== 将核心标定资源插入ECS世界容器，供System访问 ==========
    world.insert_resource(config);
    world.insert_resource(status);
    world.insert_resource(intrinsic_result);

    // ========== 注册标定专用System到调度器 ==========
    // 图像采集、角点检测、样本存储、可视化、自动标定等逻辑全部封装为System
    register_calibration_systems(
        scheduler, config, board, intrinsic_calibrator, handeye_calibrator, intrinsic_result);

    // ========== 构建调度器执行链路，校验System依赖关系 ==========
    if (auto build_result = scheduler.build(); !build_result) {
        SPDLOG_CRITICAL("调度器构建失败，System依赖冲突: {}", build_result.error());
        return 1;
    }
    SPDLOG_INFO("标定系统调度器构建完成");
    SPDLOG_INFO("调度器后台线程启动，Ctrl+C安全退出");

    // 初始状态切换为采集模式，等待用户抓拍样本
    status->state = CalibrationState::Capturing;

    // ========== 启动调度器后台子线程 ==========
    std::thread scheduler_thread([&scheduler]() {
        // 子线程持续运行调度器循环，直到stop触发
        if (const auto result = scheduler.run(); !result) {
            SPDLOG_ERROR("调度器运行过程发生异常");
        }
    });

    // ========== 主线程主循环：监控样本数量、自动标定、监听退出信号 ==========
    while (g_running) {
        // 100ms轮询一次，降低CPU占用
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 根据标定模式读取当前已采集有效样本数量
        uint32_t current_samples = 0;
        if (config->mode == CalibrationMode::Intrinsic) {
            current_samples = intrinsic_calibrator->sample_count();
        } else if (config->mode == CalibrationMode::Handeye) {
            current_samples = handeye_calibrator->sample_count();
        }

        // ========== 自动标定逻辑：开启auto_capture且样本数达标则自动求解 ==========
        if (config->capture.auto_capture
            && current_samples >= config->capture.min_samples
            && status->state == CalibrationState::Capturing)
        {
            SPDLOG_INFO("已采集足够标定样本，自动执行标定计算...");
            status->state = CalibrationState::Calibrating;

            // -------- 分支1：相机内参自动标定 --------
            if (config->mode == CalibrationMode::Intrinsic) {
                // 传入图像分辨率执行OpenCV内参标定
                auto result = intrinsic_calibrator->calibrate(cv::Size(config->width, config->height));
                if (result) {
                    // 保存标定结果
                    *intrinsic_result = *result;
                    // 写入TOML内参文件
                    if (auto save_result = save_intrinsic_result(*result, config->output.intrinsic_path); !save_result) {
                        SPDLOG_ERROR("内参结果保存失败: {}", save_result.error());
                    } else {
                        SPDLOG_INFO("相机内参已保存至: {}", config->output.intrinsic_path);
                        SPDLOG_INFO("内参标定重投影RMS误差: {:.4f} 像素", result->rms_error);
                        status->state = CalibrationState::Completed;
                    }
                } else {
                    SPDLOG_ERROR("内参标定求解失败: {}", result.error());
                    status->state = CalibrationState::Failed;
                }
            }
            // -------- 分支2：手眼自动标定 --------
            else if (config->mode == CalibrationMode::Handeye) {
                auto result = handeye_calibrator->calibrate();
                if (result) {
                    // 保存手眼外参TOML
                    if (auto save_result = save_handeye_result(*result, config->output.extrinsic_path, config->handeye.method); !save_result) {
                        SPDLOG_ERROR("手眼外参保存失败: {}", save_result.error());
                    } else {
                        // 打印手眼标定旋转平移结果
                        print_handeye_result(*result);
                        SPDLOG_INFO("手眼外参矩阵已保存至: {}", config->output.extrinsic_path);
                        status->state = CalibrationState::Completed;
                    }
                } else {
                    SPDLOG_ERROR("手眼标定求解失败: {}", result.error());
                    status->state = CalibrationState::Failed;
                }
            }

            // 标定完成后自动设置全局退出标志，程序收尾释放资源
            if (status->state == CalibrationState::Completed) {
                g_running = false;
            }
        }
    }

    // ========== 程序安全收尾、资源释放 ==========
    SPDLOG_INFO("收到退出信号，开始清理资源...");
    // 停止调度器循环
    scheduler.stop();
    // 等待后台调度线程正常退出，避免资源泄漏
    if (scheduler_thread.joinable()) {
        scheduler_thread.join();
    }

    return 0;
}