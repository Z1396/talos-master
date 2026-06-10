// 头文件保护：防止该头文件被重复包含（C++ 标准写法，替代传统 #ifndef/#define）
#pragma once

// 引入 Foxglove 服务端核心类、消息类型定义
#include "foxglove_server.hpp"
#include "foxglove_types.hpp"

// C++ 标准库：原子变量、时间、智能指针
#include <atomic>
#include <chrono>
#include <memory>

// spdlog 日志库基础定义、日志消息结构体、自定义日志输出槽(sink)基类
#include <spdlog/common.h>
#include <spdlog/details/log_msg.h>
#include <spdlog/sinks/base_sink.h>

// 项目内部通用工具函数/工具类
#include <utility.hpp>

/**
 * 命名空间：fcs::visualization
 * 语义：fcs 为项目/框架名（Talos 相关机器人框架），visualization 可视化模块
 * 该模块负责对接 Foxglove Studio 可视化工具，实现日志、数据实时上发
 */
namespace fcs::visualization {

// ============================================================================
// 类说明：FoxgloveSink
// 作用：自定义 spdlog 日志输出槽(Sink)
//       将本地 spdlog 日志，转发为 Foxglove 协议日志消息，推送到 FoxgloveServer
//       最终在 Foxglove Studio 的 Log 日志面板中展示程序日志
//
// 线程安全说明：
// 1. 继承 spdlog::sinks::base_sink<Mutex>，基类内置互斥锁，天然保证多线程写日志安全
// 2. 使用 std::shared_ptr<std::atomic<bool>> 做生命周期标记，解决异步线程析构野指针
//
// 生命周期设计：
// - alive_ 原子标记由 FoxgloveServer 持有并共享
// - FoxgloveServer 析构时先将 alive_ 置 false，再等待发送线程退出
// - 本Sink检测到 alive_=false 直接丢弃日志，杜绝访问已销毁的 Server
//
// 使用方式（外部调用）：
//   1. 创建 FoxgloveServer 实例
//   2. 调用 attach_foxglove_sink(server) 绑定日志槽
//   3. 正常使用 spdlog 打印日志，自动转发到 Foxglove
// ============================================================================

/**
 * 模板类 FoxgloveSink
 * @tparam Mutex 互斥锁类型，由基类 base_sink 控制多线程同步
 * final 关键字：禁止该类被继承
 * 继承 spdlog 自定义 Sink 基类：所有自定义日志输出都必须继承此类
 */
template <typename Mutex>
class FoxgloveSink final : public spdlog::sinks::base_sink<Mutex> {
public:
    /**
     * 构造函数
     * @param server 引用外部创建的 FoxgloveServer 服务实例（消息发送入口）
     * @param alive 原子布尔标记，用于标记 Server 是否存活，管控生命周期
     * noexcept：保证构造不会抛出异常
     */
    FoxgloveSink(FoxgloveServer& server, std::shared_ptr<std::atomic<bool>> alive) noexcept
        : server_(server)       // 初始化服务端引用
        , alive_(std::move(alive))  // 移动语义接管原子标记智能指针，减少拷贝
    {}

protected:
    /**
     * 重写基类纯虚函数：sink_it_
     * spdlog 每产生一条日志，都会调用该函数，是日志处理核心入口
     * @param msg spdlog 原始日志消息结构体，包含日志级别、内容、源码行号、日志器名等
     */
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // 安全校验：如果 FoxgloveServer 已销毁(alive=false)，直接丢弃本条日志，避免野指针访问
        if (!alive_->load(std::memory_order_acquire))
            return;

        // 1. 获取当前系统高精度时间
        const auto now = std::chrono::system_clock::now();
        // 2. 将时间转为「自纪元起的纳秒数」，Foxglove 协议统一使用纳秒时间戳
        const auto ns  = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());

        // 定义 Foxglove 标准日志协议结构体（对应 Foxglove Log 消息 schema）
        ::foxglove::schemas::Log log;
        // 填充时间戳：通过工具函数把纳秒转为 Foxglove 标准时间格式
        log.timestamp = timestamp_from_ns(ns);
        // 将 spdlog 日志级别 映射为 Foxglove 协议定义的日志级别
        log.level     = to_foxglove_level(msg.level);
        // 日志正文：从 spdlog 消息载荷中拷贝字符串
        log.message   = std::string(msg.payload.data(), msg.payload.size());
        // 日志器名称（区分不同模块日志）
        log.name      = std::string(msg.logger_name.data(), msg.logger_name.size());

        // 可选字段：源码文件名（仅日志开启源码位置时才有值）
        if (msg.source.filename != nullptr)
            log.file = msg.source.filename;
        // 可选字段：源码行号
        if (msg.source.line >= 0)
            log.line = static_cast<uint32_t>(msg.source.line);

        // 外层包装结构体：项目自定义消息封装层
        LogMessage wrapper;
        // 移动语义：把构造好的 Foxglove 日志对象转移到包装体，避免拷贝开销
        wrapper.payload = std::move(log);
        // 将完整消息入队到 FoxgloveServer 的消息队列，由后台线程异步发送
        server_.enqueue_message(FoxgloveMessage{std::move(wrapper)});
    }

    /**
     * 重写基类纯虚函数：flush_
     * 作用：刷新日志缓冲区（同步落地/发送）
     * 本场景：日志是异步入队发送，无缓冲区，所以空实现
     */
    void flush_() override {}

private:
    /**
     * 私有静态工具函数：日志级别映射
     * 将 spdlog 枚举级别 → Foxglove 协议定义的日志级别
     * @param lvl spdlog 原始日志级别
     * @return Foxglove 协议日志级别
     * noexcept：无异常
     * [[nodiscard]]：编译器警告忽略返回值的调用
     */
    [[nodiscard]] static ::foxglove::schemas::Log::LogLevel
        to_foxglove_level(spdlog::level::level_enum lvl) noexcept {
        // 别名简化代码书写
        using Spd = spdlog::level::level_enum;
        using Fg  = ::foxglove::schemas::Log::LogLevel;

        // 分支匹配日志级别
        switch (lvl) {
        // trace/debug 统一映射为 Foxglove DEBUG 级别
        case Spd::trace: [[fallthrough]]; // 穿透分支，继续执行下一个case
        case Spd::debug: return Fg::DEBUG;

        // 普通信息日志
        case Spd::info: return Fg::INFO;
        // 警告日志
        case Spd::warn: return Fg::WARNING;
        // 错误日志
        case Spd::err: return Fg::ERROR;
        // 致命错误
        case Spd::critical: return Fg::FATAL;
        // 未知级别兜底
        default: return Fg::UNKNOWN;
        }
    }

    // 成员变量
    FoxgloveServer& server_;                // 引用：Foxglove 服务端实例（消息发送主体）
    std::shared_ptr<std::atomic<bool>> alive_;  // 共享原子标记：标记服务端是否存活，用于生命周期安全
};

/**
 * 全局内联函数：attach_foxglove_sink
 * 功能：把上面自定义的 FoxgloveSink 挂载到 spdlog 默认全局日志器
 * 调用时机：
 *   1. 必须先完成 spdlog 全局日志初始化 init_logger()
 *   2. 必须先创建好 FoxgloveServer 实例
 * @param server 已创建的 FoxgloveServer 实例
 * noexcept：函数不会抛出异常
 * inline：内联函数，头文件中定义避免多定义问题
 */
inline void attach_foxglove_sink(FoxgloveServer& server) noexcept {
    // 1. 创建自定义日志槽实例，传入服务端、存活标记（从 Server 获取原子标记）
    auto sink   = std::make_shared<FoxgloveSink<std::mutex>>(server, server.sink_alive());
    // 2. 获取 spdlog 的默认全局日志器
    auto logger = spdlog::default_logger();
    // 3. 如果日志器有效，将自定义 Sink 添加到日志器的输出槽列表
    if (logger)
        logger->sinks().push_back(std::move(sink));
}

} // namespace fcs::visualization