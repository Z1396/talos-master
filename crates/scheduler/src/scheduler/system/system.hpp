#pragma once
// 头文件保护，防止重复包含

// 框架依赖头文件
#include "../world.hpp"          // ECS 核心 World 容器：管理所有实体、组件、通信通道
#include "components.hpp"        // 组件类型定义、读写标记、local 本地类型包装
#include "execution_policy.hpp"  // 执行策略（串行/并行、优先级、调度规则）
#include "system_meta.hpp"       // 系统元数据：名称、依赖、读写权限、调度属性

// 标准库依赖
#include <atomic>        // 原子变量，跨线程就绪标记
#include <cstddef>       // 标准尺寸类型 size_t
#include <cstdint>       // 固定宽度整型
#include <memory>        // 智能指针 unique_ptr
#include <optional>      // 可选容器，延迟初始化参数组
#include <string>        // 系统名称字符串
#include <tuple>         // 元组，批量管理系统运行参数
#include <type_traits>   // 模板元编程、类型萃取
#include <utility>       // 移动语义、std::forward

/**
 * @namespace talos::scheduler::system
 * @brief Talos 调度器 - 系统模块命名空间
 * 职责：
 * 1. 定义系统统一抽象接口 SystemBase
 * 2. 实现可被外部线程唤醒的系统扩展接口 ExternalComputeSource
 * 3. 模板工具：编译期解析函数参数、区分组件/本地变量
 * 4. 通用 System 模板：把函数/lambda 包装为调度器可运行的系统
 * 5. 工厂函数：快速创建系统实例
 */
namespace talos::scheduler::system {

// ============================================================================
// 一、系统基础抽象接口 & 外部唤醒扩展接口
// ============================================================================

// 前置声明：供 SystemBase::as_external_compute() 使用
class ExternalComputeSource;

/**
 * @class SystemBase
 * @brief 所有调度系统的**统一抽象基类**
 * 调度器仅依赖该基类，实现多态统一管理，隔离具体系统实现
 * 生命周期三阶段：bind 绑定资源 → run 执行逻辑 → 析构释放
 */
class SystemBase {
public:
    /**
     * @brief 虚析构函数
     * 基类虚析构，保证派生类资源正常释放，noexcept 无异常
     */
    virtual ~SystemBase() noexcept = default;

    /**
     * @brief 资源绑定阶段（生命周期第一阶段）
     * 仅在此阶段允许从 World 获取组件/通道句柄，**全局仅执行一次、单线程执行**
     * @param world ECS 全局世界容器
     */
    virtual void bind(World& world) noexcept = 0;

    /**
     * @brief 系统运行逻辑（生命周期第二阶段）
     * 调度器触发时执行系统核心业务
     * @param world ECS 全局世界容器
     * @return bool true = 本系统写入了输出通道/组件；false = 无输出
     * @note 禁止动态绑定资源、禁止修改调度拓扑
     */
    virtual bool run(World& world) noexcept         = 0;

    /**
     * @brief 获取系统元数据
     * @return 系统名称、依赖、读写策略等描述信息
     */
    virtual const SystemMeta& meta() const noexcept = 0;

    /**
     * @brief 无RTTI类型转换：转为外部可唤醒源接口
     * 替代 dynamic_cast，零运行时开销、不依赖C++ RTTI
     * @return 实现了 ExternalComputeSource 则返回自身指针，否则返回 nullptr
     */
    virtual ExternalComputeSource* as_external_compute() noexcept { return nullptr; }
};

/**
 * @class ExternalComputeSource
 * @brief 扩展接口：支持**外部线程唤醒**的计算系统标记接口
 * 适用场景：传感器回调、网络接收、硬件中断等异步事件驱动系统
 *
 * 契约约束（必须遵守）：
 * 1. 仅计算类系统可实现该接口
 * 2. 外部唤醒只能设置当前系统自身的就绪标记位
 * 3. 组件/通道句柄必须在 bind() 阶段提前获取，禁止 run 时懒加载
 */
class ExternalComputeSource {
public:
    virtual ~ExternalComputeSource() noexcept = default;

    /**
     * @brief 绑定系统就绪位槽位
     * 调度器构建阶段调用，把当前系统关联到全局原子就绪掩码
     * @param ready_systems 全局原子就绪位掩码数组（多个系统共用）
     * @param system_index 当前系统在掩码中的位下标
     */
    virtual void bind_external_ready_slot(
        std::atomic<std::uint64_t>* ready_systems, std::size_t system_index) noexcept = 0;
};

// ============================================================================
// 二、内部模板工具集 detail 命名空间
// 全部为**编译期模板元编程**，解析函数参数、区分「组件参数」和「本地局部参数」
// ============================================================================
namespace detail {

/**
 * @struct count_locals_before
 * @brief 编译期计数器：统计「目标下标之前」有多少个 local<T> 本地类型参数
 * @tparam ArgsTuple 函数参数整体元组
 * @tparam TargetIdx 目标参数下标
 * @tparam CurrentIdx 当前递归遍历下标
 * 作用：local 类型会单独存放在本地存储元组，需要用偏移量定位
 */
template <typename ArgsTuple, std::size_t TargetIdx, std::size_t CurrentIdx = 0>
struct count_locals_before;

/**
 * @brief 递归终止条件：遍历到目标下标，计数归0
 */
template <typename... Args, std::size_t TargetIdx>
struct count_locals_before<std::tuple<Args...>, TargetIdx, TargetIdx> {
    static constexpr std::size_t value = 0;
};

/**
 * @brief 递归遍历：逐个判断当前参数是否为 local 类型，累加计数
 */
template <typename... Args, std::size_t TargetIdx, std::size_t CurrentIdx>
requires(CurrentIdx < TargetIdx)
struct count_locals_before<std::tuple<Args...>, TargetIdx, CurrentIdx> {
    // 当前下标的参数类型
    using CurrentArg                       = std::tuple_element_t<CurrentIdx, std::tuple<Args...>>;
    // 判断是否为本地类型 local<T>
    static constexpr bool is_current_local = is_local_type<CurrentArg>::value;
    // 递归累加：当前是local则+1，继续向后遍历
    static constexpr std::size_t value =
        (is_current_local ? 1 : 0)
        + count_locals_before<std::tuple<Args...>, TargetIdx, CurrentIdx + 1>::value;
};

/**
 * @struct extract_local_types
 * @brief 从参数元组中**提取所有 local<T> 类型**，生成纯本地类型元组
 * 非 local 类型直接丢弃，仅保留需要本地存储的变量
 */
template <typename Tuple>
struct extract_local_types;

/**
 * @struct local_to_tuple_impl
 * @brief 辅助模板：单个类型转元组
 * 是 local<T> → 取出内部真实类型，包装为单元素元组
 * 非 local  → 空元组
 */
template <typename T, bool is_local>
struct local_to_tuple_impl;

// 分支1：本地类型 local<T> → 提取内层类型，生成单元素元组
template <typename T>
struct local_to_tuple_impl<T, true> {
    using type = std::tuple<inner_type_t<T>>;
};

// 分支2：非本地类型 → 空元组
template <typename T>
struct local_to_tuple_impl<T, false> {
    using type = std::tuple<>;
};

// 别名：简化调用
template <typename T>
using local_to_tuple_t = local_to_tuple_impl<T, is_local_type<T>::value>::type;

/**
 * @brief 遍历整个参数元组，拼接所有 local 类型为一个总元组
 */
template <typename... Ts>
struct extract_local_types<std::tuple<Ts...>> {
    // 折叠表达式拼接所有子元组
    using type = decltype(std::tuple_cat(std::declval<local_to_tuple_t<Ts>>()...));
};

// 对外别名：提取所有本地类型
template <typename Tuple>
using extract_local_types_t = extract_local_types<Tuple>::type;

/**
 * @brief 构造单个函数参数
 * @tparam T 参数原始类型
 * @tparam LocalStorage 本地存储元组
 * @tparam LocalIdx 当前local参数在本地存储中的下标
 * @param world ECS世界，用于获取组件/通道
 * @param local_storage 系统内部本地变量存储
 * @return 构造完成的参数（local 取本地存储，其余从World获取）
 */
template <typename T, typename LocalStorage, std::size_t LocalIdx>
auto make_arg(
    World& world, LocalStorage& local_storage,
    std::integral_constant<std::size_t, LocalIdx>) noexcept {
    // 编译期分支：区分本地参数 / 组件参数
    if constexpr (is_local_type<T>::value) {
        // local<T>：从系统内部本地存储取值
        using Inner   = inner_type_t<T>;
        auto& storage = std::get<LocalIdx>(local_storage);
        return local<Inner>{&storage};
    } else {
        // 普通组件/通道参数：从全局 World 中获取
        return world.get<T>();
    }
}

/**
 * @brief 按从左到右顺序批量构造所有参数（保证求值顺序）
 * C++17 花括号初始化列表保证顺序执行，规避模板参数乱序问题
 * @tparam ArgsTuple 目标参数元组类型
 * @tparam LocalStorage 本地存储
 * @tparam Is 下标索引序列
 * @return 组装完成的函数参数元组
 */
template <typename ArgsTuple, typename LocalStorage, std::size_t... Is>
auto make_args_sequenced(
    World& world, LocalStorage& local_storage, std::index_sequence<Is...>) noexcept {
    return ArgsTuple{make_arg<std::tuple_element_t<Is, ArgsTuple>>(
        world, local_storage,
        // 传入当前下标之前的local数量，定位本地存储偏移
        std::integral_constant<std::size_t, count_locals_before<ArgsTuple, Is>::value>{})...};
}

} // namespace detail

// ============================================================================
// 三、具体系统实现模板 System<F, Policy>
// 将任意可调用对象 F（函数、lambda、仿函数）包装为 SystemBase 派生类
// ============================================================================

/**
 * @class System
 * @brief 通用系统模板实现
 * @tparam F 被包装的可调用对象（函数/lambda）
 * @tparam Policy 执行策略（默认 default_policy）
 * 核心能力：
 * 1. 编译期解析函数参数列表
 * 2. bind 阶段一次性预取所有组件/本地变量，缓存参数
 * 3. run 阶段直接调用函数，零重复查询开销
 * 4. 自动标记「是否写入输出组件」
 */
template <typename F, typename Policy = default_policy>
class System : public SystemBase {
    // 函数特征萃取：解析 F 的参数列表、参数个数
    using traits     = detail::function_traits<F>;
    // 函数所有参数组成的元组类型
    using args_tuple = traits::args_tuple;

    F func_;                                // 原始可调用对象（业务逻辑本体）
    SystemMeta meta_;                       // 系统元数据（名称、依赖、策略）
    std::optional<args_tuple> cached_args_; // 缓存：bind 阶段构造好的参数组（延迟初始化）
    bool written_ = false;                  // 标记：当前运行帧是否写入了输出组件
    // 系统内部本地变量存储：仅存放 local<T> 类型参数
    detail::extract_local_types_t<args_tuple> local_storage_{};

    /**
     * @brief 递归设置「写标记指针」
     * 遍历参数元组，给所有写类型(writer)组件绑定 written_ 标记
     * 函数写入组件时会自动置位 written_，调度器据此判断下游是否需要触发
     * @tparam I 当前遍历下标
     */
    template <std::size_t I = 0>
    void bind_written_flags() noexcept {
        // 递归终止：遍历完所有参数
        if constexpr (I < std::tuple_size_v<args_tuple>) {
            using ArgType = std::tuple_element_t<I, args_tuple>;
            // 如果当前参数是「写组件」，绑定标记位
            if constexpr (detail::is_writer<ArgType>::value) {
                std::get<I>(*cached_args_).written_flag_ = &written_;
            }
            // 递归处理下一个参数
            bind_written_flags<I + 1>();
        }
    }

public:
    /**
     * @brief 构造函数
     * @param name 系统名称
     * @param func 业务可调用对象
     * 提取函数特征 + 执行策略，生成系统元数据
     */
    System(std::string name, F func) noexcept
        : func_(std::move(func))
        , meta_(extract_system_meta<F, Policy>(std::move(name))) {}

    /**
     * @brief 重写基类 bind 接口：资源绑定
     * 1. 仅首次执行，重复调用直接返回
     * 2. 批量构造并缓存所有函数参数（组件 + 本地变量）
     * 3. 为写组件绑定输出标记
     */
    void bind(World& world) noexcept override {
        if (cached_args_) {
            return; // 已绑定，直接跳过
        }
        // 按顺序构造所有参数，存入可选容器缓存
        cached_args_ = detail::make_args_sequenced<args_tuple>(
            world, local_storage_, std::make_index_sequence<traits::arity>{});
        // 绑定写标记指针
        bind_written_flags();
    }

    /**
     * @brief 重写基类 run 接口：执行系统逻辑
     * 1. 每帧清零输出标记
     * 2. 用缓存参数调用原始业务函数
     * 3. 返回本帧是否产生输出
     */
    bool run([[maybe_unused]] World& world) noexcept override {
        written_ = false;
        // 解包参数元组，调用函数
        std::apply(func_, *cached_args_);
        return written_;
    }

    /**
     * @brief 获取系统元数据
     */
    const SystemMeta& meta() const noexcept override { return meta_; }
};

// ============================================================================
// 四、系统工厂函数 make_system
// 对外统一创建入口，隐藏模板细节，返回基类智能指针
// ============================================================================

/**
 * @brief 系统工厂函数
 * @tparam F 可调用对象类型
 * @tparam Policy 执行策略
 * @param name 系统名称
 * @param func 业务函数/lambda
 * @return std::unique_ptr<SystemBase> 基类智能指针，调度器统一持有
 * @note [[nodiscard]] 必须接收返回值，防止创建后丢弃
 */
template <typename F, typename Policy = default_policy>
[[nodiscard]] std::unique_ptr<SystemBase> make_system(std::string name, F&& func) noexcept {
    // 退化类型、转发可调用对象，创建具体 System 实例
    return std::make_unique<System<std::decay_t<F>, Policy>>(
        std::move(name), std::forward<F>(func));
}

} // namespace talos::scheduler::system