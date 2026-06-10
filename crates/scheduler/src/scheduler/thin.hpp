// 头文件保护，防止重复包含
#pragma once

// 引入标准类型定义，std::size_t 用于表示线程数、数量等无符号整型
#include <cstddef>

// Talos 框架顶层命名空间
namespace talos {
// 调度器子命名空间
namespace scheduler {

// 前置声明调度器核心类，仅声明不定义，减少头文件依赖、缩短编译时间
class Scheduler;
class World;

/**
 * @brief 调度器运行配置结构体
 * 用于配置 Talos 调度器的线程池、统计打印等全局参数
 */
struct SchedulerConfig {
    /**
     * @brief TBB 线程池并发数配置
     *
     * 含义：计算任务池的工作线程数量
     * 取值规则：
     *   - 0：自动使用当前硬件CPU核心数（默认行为）
     *   - 大于0：手动指定固定线程数
     *
     * 默认值：0
     */
    std::size_t compute_concurrency = 0;

    /**
     * @brief 是否在程序运行结束后打印性能统计信息
     * true：输出延迟、运行次数、吞吐等统计数据
     * false：关闭统计打印
     * 默认开启
     */
    bool print_stats{true};
};

} // namespace scheduler

// 全局别名，外部使用时可直接写 SchedulerConfig / Scheduler / World
// 无需逐层嵌套命名空间，简化代码书写
using namespace talos::scheduler;

} // namespace talos