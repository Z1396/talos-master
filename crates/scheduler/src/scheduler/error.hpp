#pragma once

// 标准库头文件引入
#include <cstddef>       // 基础类型定义，如 std::size_t
#include <expected>      // C++23 标准库，用于带错误码的返回值（成功/失败分支）
#include <fmt/format.h>  // 第三方格式化库，用于字符串格式化输出
#include <string>        // 标准字符串类型 std::string
#include <typeindex>     // 运行时类型索引，用于类型唯一标识
#include <variant>       // C++17 变体类型，可存储多种不同类型（多错误类型聚合）
#include <vector>        // 动态数组容器，存储字符串、对象列表等

// 命名空间：talos 框架 -> 调度器模块
namespace talos::scheduler {

/**
 * @brief 调度器生命周期相关错误枚举
 * 描述调度器在启动、运行阶段出现的状态类错误
 */
enum class SchedulerError {
    AlreadyRunning, ///< 调度器当前已处于运行状态，重复启动触发该错误
    NotBuilt,       ///< 调度器未执行构建流程，直接运行触发该错误
};

/**
 * @brief 通道唯一标识结构体，用于错误信息上报
 * 组合「数据类型 + 标签类型」+「通道类型」，全局唯一标识一条通信通道
 */
struct ChannelKeyInfo {
    std::type_index type;    // 通道传输数据的运行时类型索引
    std::type_index tag;    // 通道附加标签的运行时类型索引（用于区分同数据类型的不同通道）
    bool is_spsc;           // 标记通道类型：true = SPSC(单生产者单消费者)，false = SPMC(单生产者多消费者)

    /**
     * @brief 构造函数：初始化通道标识信息
     * @param t 数据类型索引
     * @param tg 标签类型索引
     * @param spsc 是否为 SPSC 通道
     */
    ChannelKeyInfo(const std::type_index t, const std::type_index tg, const bool spsc)
        : type(t)
        , tag(tg)
        , is_spsc(spsc) {}
};

/**
 * @brief 通道类型冲突错误
 * 同一个逻辑通道，同时被注册为 SPSC 和 SPMC 两种类型，引发冲突
 */
struct ChannelKindConflict {
    ChannelKeyInfo key;        // 发生冲突的通道唯一标识
    std::string first_system;  // 第一个注册该通道的计算系统名称
    std::string second_system; // 第二个注册该通道、类型不一致的计算系统名称
};

/**
 * @brief 多生产者错误
 * 单生产者通道（SPSC/SPMC）被绑定了多个写入端（生产者），违反通道设计规则
 */
struct MultipleWritersError {
    ChannelKeyInfo key;                // 异常通道标识
    std::vector<std::string> writers;  // 所有占用该通道的生产者（系统）名称列表
};

/**
 * @brief SPSC 通道多消费者错误
 * SPSC（单生产者单消费者）通道被绑定了多个读取端（消费者），违反通道规则
 */
struct MultipleReadersError {
    ChannelKeyInfo key;               // 异常通道标识
    std::vector<std::string> readers; // 所有占用该通道的消费者（系统）名称列表
};

/**
 * @brief 孤立消费者错误
 * 存在通道读取端（消费者），但没有对应的写入端（生产者），数据来源缺失
 */
struct OrphanedReaderError {
    ChannelKeyInfo key;               // 异常通道标识
    std::vector<std::string> readers; // 无数据来源的消费者（系统）名称列表
};

/**
 * @brief 依赖环错误
 * 计算系统之间形成循环依赖，调度图出现闭环，无法正常调度执行
 */
struct DependencyCycleError {
    std::vector<std::string> cycle; // 构成循环依赖的系统名称链路
};

/**
 * @brief 计算系统数量超限错误
 * 调度器基于位掩码做高效调度，限制最大计算系统数量为 64 个
 */
struct TooManyComputeSystemsError {
    std::size_t count;                // 当前已注册的计算系统总数
    static constexpr std::size_t max_count = 64; // 调度器支持的计算系统上限
};

/**
 * @brief 不可达计算系统错误
 * 部分计算系统没有任何外部触发源、前置依赖，永远无法被调度执行
 */
struct UnreachableComputeSystemsError {
    std::vector<std::string> systems; // 所有不可达、无法触发的系统名称列表
};

/**
 * @brief 调度器构建阶段所有错误的聚合变体类型
 * 使用 std::variant 统一承载各类构建异常，一个变量可表示任意一种构建错误
 */
using BuildError = std::variant<
    SchedulerError,               // 调度器生命周期/运行态错误
    ChannelKindConflict,          // 通道类型冲突错误
    MultipleWritersError,         // 通道多生产者错误
    MultipleReadersError,         // SPSC 通道多消费者错误
    OrphanedReaderError,          // 孤立消费者错误
    DependencyCycleError,         // 系统依赖环错误
    TooManyComputeSystemsError,   // 计算系统数量超限错误
    UnreachableComputeSystemsError>; // 不可达系统错误

/**
 * @brief 调度器构建结果类型
 * 基于 C++23 std::expected：
 *  - 成功：返回 void（无数据）
 *  - 失败：返回 BuildError 类型的具体错误信息
 */
using BuildResult = std::expected<void, BuildError>;

// ============================================================================
// 致命错误（panic）工具函数模块
// 用于全局统一处理程序不可恢复的严重错误，日志输出并终止进程
// ============================================================================

namespace detail {
/**
 * @brief 底层 panic 实现函数
 * @param message 拼接完成的错误描述字符串
 * @note [[noreturn]] 表示函数执行后永不返回；noexcept 保证无异常抛出
 */
[[noreturn]] void panic_message(std::string message) noexcept;
} // namespace detail

/**
 * @brief 格式化输出致命错误日志并终止程序（panic 接口）
 * 框架统一的致命错误入口，所有不可恢复错误统一使用该接口上报
 *
 * @tparam Args 可变参数包，自动推导格式化参数类型
 * @param fmt 格式化字符串
 * @param args 填充格式化字符串的可变参数
 *
 * 功能说明：
 *  1. 通过 fmt 库拼接完整错误信息
 *  2. 调用底层接口输出 CRITICAL 级别日志
 *  3. 直接终止程序运行，函数永不返回
 *  4. 标记 noexcept 保证不会抛出异常
 */
template <typename... Args>
/*标准属性，告诉编译器：这个函数永远不会正常返回。
函数内部一定会终止程序、抛出致命异常、死循环，不会执行完回到调用处。
作用：
编译器优化：不需要生成返回栈代码；
消除告警：if 分支调用 panic 后，编译器知道后续代码不可达，不会报 “缺少 return”。*/
[[noreturn]] inline void panic(const char* fmt, const Args&... args) noexcept {
    talos::scheduler::detail::panic_message(fmt::format(fmt::runtime(fmt), args...));
}

} // namespace talos::scheduler