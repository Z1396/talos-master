// 框架启动核心头文件
#include "fcs/runtime/boot.hpp"                   // 系统核心启动引导函数 fcs::boot
#include "fcs_visualization/foxglove_sink.hpp"    // Foxglove 可视化数据接收器
#include "fcs_visualization/foxglove_systems.hpp" // Foxglove 对应调度系统注册
#include "runtime/build_info.hpp"                 // 编译构建信息（Git分支、提交ID、编译时间等）
#include "runtime/config_loader.hpp"              // 配置文件加载器（解析 TOML 配置）
#include "scheduler/error_formatter.hpp"          // 调度器错误信息格式化工具
#include "scheduler/scheduler.hpp"                // 核心任务调度器 talos::Scheduler
#include "spdlog_hook.hpp"                        // 日志库 spdlog 初始化、标准流重定向钩子

// 标准库依赖
#include <atomic>  // 原子变量：线程安全标记，无锁跨线程通信
#include <chrono>  // 时间戳、时长计算，用于计时、休眠
#include <csignal> // 操作系统信号捕获（Ctrl+C、进程终止信号）
#include <thread>  // 标准线程库，创建独立监听线程
#include <variant> // 变体类型，用于区分不同后端硬件配置

/**
 * @namespace 匿名命名空间
 * 作用：全局变量、工具函数仅当前文件可见，避免命名污染
 */
namespace {

/**
 * @brief 全局原子标记：请求程序退出
 * atomic 保证多线程读写安全，无需额外互斥锁
 */
std::atomic<bool> g_shutdown_requested{false};

/**
 * @brief 系统信号回调函数
 * 捕获操作系统下发的终止信号，置位退出标记
 * @param sig 信号值（未使用）
 */
void signal_handler(int /*sig*/) {
    // 内存序 release：保证本次写操作对后续读线程可见
    g_shutdown_requested.store(true, std::memory_order_release);
}

} // namespace

/**
 * @brief 条件编译：若未定义 TALOS_DISABLE_PROGRAM_MAIN，则编译主函数
 * 作用：支持单元测试/动态库模式下禁用默认 main 函数，避免冲突
 */
#ifndef TALOS_DISABLE_PROGRAM_MAIN

/**
 * @brief 程序入口函数
 * 整体启动流程：
 * 1. 注册系统退出信号
 * 2. 初始化日志系统
 * 3. 加载构建信息、打印版本
 * 4. 解析 TOML 配置文件
 * 5. 初始化 Foxglove 可视化（ROS 生态可视化工具，用于机器人调试、数据回看）
 * 6. 执行框架引导初始化 fcs::boot
 * 7. 启动独立线程监听退出信号
 * 8. 运行核心调度器
 * 9. 等待监听线程退出、收尾、打印运行结果
 */
int main() {
    // ===================== 1. 注册系统信号处理器 =====================
    // 捕获 Ctrl+C 信号
    std::signal(SIGINT, signal_handler);
    // 捕获进程终止信号（kill 命令、系统关闭等）
    std::signal(SIGTERM, signal_handler);

    // ===================== 2. 初始化日志与标准流重定向 =====================
    init_logger();  // 初始化 spdlog 日志器，配置日志级别、输出路径、格式
    hook_cstream(); // 劫持标准输出/标准错误流，统一重定向到日志系统

    // 记录程序启动时间点，用于统计整体初始化耗时
    const auto start = std::chrono::system_clock::now();

    // ===================== 3. 打印项目编译构建信息 =====================
    const auto build = fcs::build_info();
    // 输出：Git分支、编译日期、Git提交哈希、编译主机
    SPDLOG_INFO(
        "build version={}@{} git={} host={}", build.git_branch, build.build_date, build.git_commit,
        build.build_host);

    // ===================== 4. 加载配置文件 at_vision.toml =====================
    // 加载视觉业务对应的 TOML 配置文件
    auto config = fcs::load_config("at_vision.toml");
    // 配置加载失败：打印致命日志，直接退出程序
    if (!config) {
        SPDLOG_CRITICAL("{}", config.error());
        return 1;
    }

    // ===================== 5. 提前提取可视化配置 =====================
    // 注释说明：fcs::boot 会消费主配置，而 Foxglove 属于独立可视化库
    // 必须在 boot 之前单独取出可视化配置，防止配置被释放/覆盖
    auto foxglove_cfg = config->foxglove;

    // ===================== 6. 创建核心调度器实例 =====================
    // 使用配置文件中的调度器参数（周期、线程数、优先级、任务拓扑等）初始化调度器
    talos::Scheduler scheduler(config->scheduler);

    // ===================== 7. 初始化 Foxglove 可视化模块 =====================
    // 判断配置是否启用可视化功能
    if (foxglove_cfg.enabled) {
        // 创建 Foxglove 服务实例（返回 expected，承载结果或错误信息）
        auto server_result = fcs::visualization::create_foxglove_server(foxglove_cfg);
        if (server_result) {
            // 服务创建成功：转移所有权到智能指针
            std::shared_ptr<fcs::visualization::FoxgloveServer> server = std::move(*server_result);

            // 挂载数据接收器：业务数据转发至 Foxglove 服务
            fcs::visualization::attach_foxglove_sink(*server);

            // 将 Foxglove 服务作为全局资源插入 ECS World，供所有系统访问
            scheduler.world().insert_resource(server);

            // 注册 Foxglove 对应的调度系统（数据发布、话题转换、帧转换等任务）
            // 第二个参数：判断后端硬件类型，区分不同机器人设备的适配逻辑
            fcs::visualization::register_foxglove_systems(
                std::holds_alternative<fcs::hardware::DaedalusConfig>(config->backend), scheduler,
                &scheduler);

            // 根据传输类型打印启动信息
            if (foxglove_cfg.transport == fcs::FoxgloveTransport::WebSocket) {
                // WebSocket 模式：实时在线可视化（Foxglove Studio 网页/客户端连接）
                SPDLOG_INFO(
                    "Foxglove WebSocket enabled on ws://{}:{}", foxglove_cfg.host,
                    foxglove_cfg.port);
            } else {
                // MCAP 文件模式：离线录制数据，事后回放分析
                SPDLOG_INFO("Foxglove MCAP enabled at {}", foxglove_cfg.mcap_path);
            }
        } else {
            // 服务创建失败：打印警告，可视化功能降级关闭
            SPDLOG_WARN(
                "Foxglove transport failed to initialize: {}. Visualization disabled",
                server_result.error());
        }
    } else {
        // 配置显式关闭可视化
        SPDLOG_INFO("Foxglove visualization disabled by config");
    }

    // ===================== 8. 执行框架全局引导初始化 =====================
    // 加载所有业务系统、组件、通道、时序逻辑，完成整个机器人框架初始化
    // 传入调度器 + 转移配置对象所有权
    if (auto r = fcs::boot(scheduler, std::move(config.value())); !r) {
        // 引导初始化失败，致命错误，退出程序
        SPDLOG_CRITICAL("{}", r.error());
        return 1;
    }

    // 统计并打印整体初始化耗时
    SPDLOG_INFO(
        "init done. ({}ms)", std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now() - start)
                                 .count());

    // ===================== 9. 启动独立线程：监听退出信号 =====================
    // 原子标记：标记调度器是否已运行结束
    std::atomic<bool> scheduler_finished{false};

    // 新建后台监听线程
    std::thread shutdown_watcher([&scheduler, &scheduler_finished]() {
        bool shutdown_logged = false; // 保证“收到退出信号”日志只打印一次

        // 循环监听：调度器未结束则持续轮询
        while (!scheduler_finished.load(std::memory_order_acquire)) {
            // 读取全局退出标记
            if (g_shutdown_requested.load(std::memory_order_acquire)) {
                // 仅首次触发时打印日志
                if (!shutdown_logged) {
                    SPDLOG_INFO("shutdown signal received, stopping scheduler...");
                    shutdown_logged = true;
                }
                // 调用调度器停止接口：优雅停止所有周期任务、线程
                scheduler.stop();
            }
            // 休眠 50ms，降低 CPU 占用，避免死循环空转
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // ===================== 10. 启动核心调度器（主业务循环） =====================
    // 阻塞执行：调度器开始周期运行所有系统、任务
    const auto result = scheduler.run();

    // 标记调度器已退出，唤醒监听线程退出循环
    scheduler_finished.store(true, std::memory_order_release);

    // 等待监听线程正常退出，防止线程残留
    if (shutdown_watcher.joinable()) {
        shutdown_watcher.join();
    }

    // ===================== 11. 收尾检查 =====================
    // 调度器异常退出：打印致命日志
    if (!result) {
        SPDLOG_CRITICAL("run scheduler: {}", result.error());
    }

    return 0;
}

#endif // TALOS_DISABLE_PROGRAM_MAIN