// 头文件保护，防止重复包含
#pragma once

/**
 * @file
 * @brief 为调度器错误类型提供 fmt 库格式化特化实现
 *
 * 作用：给 Talos 调度器内所有错误枚举/错误结构体实现 fmt::formatter 特化，
 * 让这些错误类型可以**直接使用 fmt::format、fmt::print、spdlog 日志**打印输出，
 * 无需手动转为字符串，统一日志与错误输出风格。
 *
 * 使用示例：
 *   fmt::print("Error: {}\n", error);          // 直接打印错误对象
 *   logger.error("Build failed: {}", error);  // 结合日志库输出错误信息
 */

// 引入调度器所有错误类型定义
#include "error.hpp"

// 高性能字符串格式化库 fmt 核心头文件
#include <fmt/format.h>
// 轻量只读字符串视图，无拷贝、性能更高
#include <string_view>

// ============================================================================
// fmt::formatter 模板特化说明
// C++ 规范要求：对 fmt::formatter 进行特化，必须放在 fmt 命名空间内
// ============================================================================
namespace fmt {

/**
 * @brief 基础枚举错误类型格式化器
 * 针对简单枚举：talos::scheduler::SchedulerError
 * 继承自 std::string_view 的格式化器，复用基础解析逻辑
 */
template <>
struct formatter<talos::scheduler::SchedulerError> : formatter<std::string_view> {
    /**
     * @brief 格式化入口函数
     * @param e 调度器基础错误枚举值
     * @param ctx fmt 格式化上下文（输出缓冲区、迭代器）
     * @return 格式化完成后的迭代器位置
     */
    auto format(talos::scheduler::SchedulerError e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 错误格式化器基类（通用模板）
 * 作用：抽取公共 parse 逻辑，避免每个错误类型重复编写相同代码
 * @tparam T 具体错误结构体类型
 */
template <typename T>
struct error_formatter_base {
    /**
     * @brief 格式解析函数
     * constexpr 编译期常量，无运行时开销
     * 本项目不使用自定义格式符，直接返回上下文起始迭代器即可
     */
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
};

// ----------------------------------------------------------------------------
// 各类具体业务错误结构体的格式化器
// 全部继承 error_formatter_base，复用 parse 方法，仅需实现 format 输出逻辑
// ----------------------------------------------------------------------------

/**
 * @brief 通道类型冲突错误格式化器
 * 该错误结构体内部包含两个冲突的系统名称
 */
template <>
struct formatter<talos::scheduler::ChannelKindConflict>
    : error_formatter_base<talos::scheduler::ChannelKindConflict> {
    auto format(const talos::scheduler::ChannelKindConflict& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 单通道多写入者错误格式化器
 * 结构体内部保存多个违规写入系统名称列表
 */
template <>
struct formatter<talos::scheduler::MultipleWritersError>
    : error_formatter_base<talos::scheduler::MultipleWritersError> {
    auto format(const talos::scheduler::MultipleWritersError& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 单通道多读取者错误格式化器
 * 对应通道不允许多读，但检测到多个读取系统
 */
template <>
struct formatter<talos::scheduler::MultipleReadersError>
    : error_formatter_base<talos::scheduler::MultipleReadersError> {
    auto format(const talos::scheduler::MultipleReadersError& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 孤立读取者错误格式化器
 * 存在读取系统，但对应通道没有任何写入源（数据来源缺失）
 */
template <>
struct formatter<talos::scheduler::OrphanedReaderError>
    : error_formatter_base<talos::scheduler::OrphanedReaderError> {
    auto format(const talos::scheduler::OrphanedReaderError& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 依赖环路错误格式化器
 * 数据流依赖图出现循环依赖，会导致调度死锁
 */
template <>
struct formatter<talos::scheduler::DependencyCycleError>
    : error_formatter_base<talos::scheduler::DependencyCycleError> {
    auto format(const talos::scheduler::DependencyCycleError& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 计算系统数量超限错误格式化器
 * 调度器基于 64 位位掩码调度，计算系统最大数量为 64
 */
template <>
struct formatter<talos::scheduler::TooManyComputeSystemsError>
    : error_formatter_base<talos::scheduler::TooManyComputeSystemsError> {
    auto format(const talos::scheduler::TooManyComputeSystemsError& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 不可达计算系统错误格式化器
 * 计算系统无任何上游触发源，永远不会被执行
 */
template <>
struct formatter<talos::scheduler::UnreachableComputeSystemsError>
    : error_formatter_base<talos::scheduler::UnreachableComputeSystemsError> {
    auto
        format(const talos::scheduler::UnreachableComputeSystemsError& e, format_context& ctx) const
        -> format_context::iterator;
};

/**
 * @brief 统一构建错误总类格式化器
 * BuildError 是上层包装类型，可包含上述任意一种具体错误
 */
template <>
struct formatter<talos::scheduler::BuildError>
    : error_formatter_base<talos::scheduler::BuildError> {
    auto format(const talos::scheduler::BuildError& err, format_context& ctx) const
        -> format_context::iterator;
};

} // namespace fmt