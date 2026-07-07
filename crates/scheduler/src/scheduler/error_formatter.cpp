#pragma once
// 调度器错误格式化器头文件
#include "scheduler/error_formatter.hpp"

// magic_enum 枚举转字符串工具
#include <magic_enum.hpp>

// 连续只读视图span
#include <span>
// 字符串存储
#include <string>
// std::variant 多类型错误变体
#include <variant>

namespace talos::scheduler::detail {

/**
 * @brief 字符串数组拼接工具函数
 * @param strs 字符串只读span数组
 * @param sep 分隔符字符串视图
 * @return 拼接完成的长字符串
 * 逻辑：空数组直接返回空串；先取第一个元素，循环追加分隔符+后续字符串
 */
static std::string join_strings(std::span<const std::string> strs, std::string_view sep) {
    if (strs.empty()) {
        return "";
    }
    // 初始化结果为第一个字符串
    std::string result = strs[0];
    // 从第二个元素开始循环拼接
    for (std::size_t i = 1; i < strs.size(); ++i) {
        result += sep;
        result += strs[i];
    }
    return result;
}

} // namespace talos::scheduler::detail

// fmt 格式化库自定义类型格式化器命名空间
namespace fmt {

/**
 * @brief SchedulerError 枚举格式化器
 * 将调度器错误枚举自动转为可读字符串输出
 * @param e 调度器错误枚举值
 * @param ctx fmt输出上下文
 * @return 格式化迭代器，写入输出缓冲区
 * 实现：magic_enum::enum_name获取枚举字面量，复用string_view格式化器
 */
auto formatter<talos::scheduler::SchedulerError>::format(
    const talos::scheduler::SchedulerError e, format_context& ctx) const
    -> format_context::iterator {
    return formatter<std::string_view>::format(magic_enum::enum_name(e), ctx);
}

/**
 * @brief 通道类型冲突错误格式化器
 * 场景：同一个频道同时被标记为SPSC、SPMC通道，拓扑冲突
 * 打印冲突双方系统名称
 */
auto formatter<talos::scheduler::ChannelKindConflict>::format(
    const talos::scheduler::ChannelKindConflict& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "ChannelKindConflict: channel used as both SPSC and SPMC by '{}' and '{}'",
        e.first_system, e.second_system);
}

/**
 * @brief 多写入器错误格式化器
 * 场景：SPSC单生产者通道存在多个写入系统，违反单写约束
 * 拼接所有写入系统名称逗号分隔展示
 */
auto formatter<talos::scheduler::MultipleWritersError>::format(
    const talos::scheduler::MultipleWritersError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "MultipleWriters: channel has multiple writers: {}",
        talos::scheduler::detail::join_strings(e.writers, ", "));
}

/**
 * @brief 多读取器错误格式化器
 * 场景：SPSC单消费者通道存在多个订阅读取系统，违反单读约束
 */
auto formatter<talos::scheduler::MultipleReadersError>::format(
    const talos::scheduler::MultipleReadersError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "MultipleReaders: SPSC channel has multiple readers: {}",
        talos::scheduler::detail::join_strings(e.readers, ", "));
}

/**
 * @brief 无生产者孤儿读取器错误格式化器
 * 场景：存在订阅读取系统，但整个调度图无任何对应发布写入器
 */
auto formatter<talos::scheduler::OrphanedReaderError>::format(
    const talos::scheduler::OrphanedReaderError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "OrphanedReader: reader(s) without writer: {}",
        talos::scheduler::detail::join_strings(e.readers, ", "));
}

/**
 * @brief 依赖循环环路错误格式化器
 * 场景：系统读写资源/通道形成环形依赖，调度拓扑无法排序
 * 使用 -> 拼接依赖链路，直观展示循环链
 */
auto formatter<talos::scheduler::DependencyCycleError>::format(
    const talos::scheduler::DependencyCycleError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "DependencyCycle: {}", talos::scheduler::detail::join_strings(e.cycle, " -> "));
}

/**
 * @brief 计算系统数量超限错误格式化器
 * 场景：调度池计算任务数量超过配置最大上限
 * 打印当前数量与最大限制值
 */
auto formatter<talos::scheduler::TooManyComputeSystemsError>::format(
    const talos::scheduler::TooManyComputeSystemsError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(ctx.out(), "TooManyComputeSystems: {} (max {})", e.count, e.max_count);
}

/**
 * @brief 不可达系统错误格式化器
 * 场景：存在无任何外部触发、无前置依赖的孤立计算系统，永远不会执行
 * 打印所有不可达系统名称列表
 */
auto formatter<talos::scheduler::UnreachableComputeSystemsError>::format(
    const talos::scheduler::UnreachableComputeSystemsError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "UnreachableComputeSystems: {}",
        talos::scheduler::detail::join_strings(e.systems, ", "));
}

/**
 * @brief 统一构建错误BuildError变体格式化器
 * BuildError是std::variant包装所有上述具体错误类型，使用std::visit多态分发
 * lambda捕获输出上下文，自动匹配变体内部存储的错误结构体，调用对应formatter输出
 */
auto formatter<talos::scheduler::BuildError>::format(
    const talos::scheduler::BuildError& err, format_context& ctx) const
    -> format_context::iterator {
    return std::visit(
        [&ctx](const auto& e) -> format_context::iterator {
            return fmt::format_to(ctx.out(), "{}", e);
        },
        err);
}

} // namespace fmt