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
 * @brief 构造单个函数参数的辅助函数（编译期多态分派）
 *
 * 根据参数类型 T 的编译期属性，自动选择「从本地存储取值」或「从全局 World 获取」，
 * 统一生成可用于调用业务函数的参数实例。
 *
 * 工作原理：
 * 通过 if constexpr 在编译期分支——
 *   - 若 T 是 local<Inner> 本地类型：从 local_storage 元组中按偏移量取出预先分配的存储槽，
 *     构造 local<Inner> 包装对象（仅保存指针，不拷贝数据）
 *   - 若 T 是普通组件/通道类型：调用 World::get<T>() 从 ECS 全局容器查询并返回句柄
 *
 * @tparam T             目标参数的原始类型（如 local<MyLocal> 或 res<ComponentA>）
 * @tparam LocalStorage  本地存储元组类型（仅存放 local<T> 参数的底层存储）
 * @tparam LocalIdx      当前参数在本地存储元组中的偏移下标（由 count_locals_before 计算得到）
 *
 * @param world          ECS 全局世界容器，用于查询组件/通道句柄
 * @param local_storage  系统内部本地变量存储元组（local<T> 参数的实际存放位置）
 * @param LocalIdxTag    编译期下标标签（integral_constant 包装值，驱动模板参数推导）
 *
 * @return 构造完成的参数实例：
 *         - local<Inner> 类型：包装了本地存储槽的指针，支持引用语义
 *         - 组件/通道类型：从 World 获取的句柄对象
 *
 * @note noexcept：所有操作均为编译期类型推导 + 运行期非异常查询，保证无异常抛出
 */
template <typename T, typename LocalStorage, std::size_t LocalIdx>
auto make_arg(
    World& world, LocalStorage& local_storage,
    std::integral_constant<std::size_t, LocalIdx>) noexcept {
    // 编译期分支：根据 T 是否为 local<T> 类型，选择不同的构造路径
    // if constexpr 保证未选中的分支在编译期直接丢弃，不会产生运行期开销
    if constexpr (is_local_type<T>::value) {
        // ======== 分支 A：local<T> 本地类型参数 ========
        // 场景：系统函数参数中声明了 local<T>，需要从系统内部存储中取值而非查询 World

        // 提取 local<T> 内部包装的真实类型（如 local<MyLocal> → MyLocal）
        using Inner = inner_type_t<T>;

        // 从本地存储元组中取出第 LocalIdx 个存储槽的引用
        // LocalIdx 由 count_locals_before 计算，跳过当前参数之前的所有 local 类型参数
        auto& storage = std::get<LocalIdx>(local_storage);

        // 构造 local<Inner> 包装对象，仅传入存储槽的指针（零拷贝）
        // 业务函数通过 local<Inner> 可以读写该本地变量，生命周期由 System 管理
        return local<Inner>{&storage};

    } else {
        // ======== 分支 B：普通组件/通道类型参数 ========
        // 场景：参数是组件（如 res<T>）、通道（如 spmc<Topic>）或其他 World 注册的类型

        // 从全局 World 容器中查询并获取类型 T 的句柄
        // World::get<T>() 内部会根据 T 的类型元信息定位对应的组件存储或通道实例
        return world.get<T>();
    }
}

/**
 * @brief 按从左到右顺序批量构造所有函数参数（严格保证求值顺序）
 *
 * 核心原理：
 * 使用 C++17 花括号初始化列表（{ }）的特性——编译器会严格按照模板参数包展开的
 * 从左到右顺序执行各元素的构造，从而规避了函数参数包展开时可能存在的求值顺序不确定问题。
 *
 * 工作流程：
 * 1. 利用 std::index_sequence<Is...> 将参数元组的每个下标编译期展开
 * 2. 对每个下标 Is，通过 std::tuple_element_t 获取该位置的参数原始类型
 * 3. 调用 make_arg 为每个参数构造实际值：
 *    - 组件/通道类型：从 World 中查询并获取
 *    - local<T> 本地类型：从 local_storage 元组中按偏移量取出
 * 4. 将所有构造好的参数打包为 ArgsTuple（即 std::tuple<Args...>）返回
 *
 * @tparam ArgsTuple      目标参数元组类型（通常为 std::tuple<组件类型...>）
 * @tparam LocalStorage   本地存储元组类型（仅包含 local<T> 类型的存储）
 * @tparam Is             编译期下标索引序列（由 std::make_index_sequence 生成）
 *
 * @param world          ECS 全局世界容器，用于查询组件/通道句柄
 * @param local_storage  系统内部本地变量存储元组
 * @param index_seq      编译期索引序列（函数参数包展开驱动，无运行时开销）
 *
 * @return ArgsTuple     构造完成的函数参数元组，可直接用于 std::apply 调用
 *
 * @note 此函数 noexcept，因为所有组件查询和本地存储访问均为 noexcept 操作
 *
 * 性能分析：
 * - 所有类型推导、下标计算（count_locals_before）均在编译期完成
 * - 运行期仅做对象构造，无额外分支判断
 */
template <typename ArgsTuple, typename LocalStorage, std::size_t... Is>
auto make_args_sequenced(
    World& world, LocalStorage& local_storage, std::index_sequence<Is...>) noexcept {
    // 通过花括号初始化列表，按 Is 从小到大的顺序依次构造每个参数
    //
    // 展开示意（假设 Is = [0, 1, 2]）：
    // ArgsTuple{
    //     make_arg<tuple_element_t<0, ArgsTuple>>(world, local_storage, integral_constant<size_t, count_locals_before<ArgsTuple, 0>::value>{}),
    //     make_arg<tuple_element_t<1, ArgsTuple>>(world, local_storage, integral_constant<size_t, count_locals_before<ArgsTuple, 1>::value>{}),
    //     make_arg<tuple_element_t<2, ArgsTuple>>(world, local_storage, integral_constant<size_t, count_locals_before<ArgsTuple, 2>::value>{})
    // }
    //
    // 其中 count_locals_before<ArgsTuple, Is>::value 的作用：
    //   计算「在第 Is 个参数之前」有多少个 local<T> 类型的参数，
    //   从而确定当前 local<T> 参数在 local_storage 元组中的实际存储偏移位置。
    //   例如：参数列表 [local<A>, ComponentB, local<C>]，
    //   对 Is=0 (local<A>)，偏移=0（之前0个local）
    //   对 Is=2 (local<C>)，偏移=1（之前有1个local<A>）
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
 /*function_traits<F>                                              [system_meta.hpp:134]
   └─ decltype(&F::operator()) → R(C::*)(Args...) const  (lambda 是 mutable，去掉 const)
       └─ 继承 function_traits<R(*)(Args...)>                    [system_meta.hpp:97]
           └─ args_tuple = std::tuple<
                  spmc<ArmorDetectionBatch, DetectionChannelTopic>,    // [0]
                  spmc<LdmDetection, LdmDetectionChannelTopic>,        // [1]
                  spmc<LdmMeasurement, LdmMeasurementChannelTopic>,    // [2]
                  res<std::shared_ptr<FoxgloveServer>>,                // [3]
                  res<CameraConfig>,                                    // [4]
                  res<FoxgloveConfig>,                                  // [5]
                  detecting_color,                                      // [6] 非组件，被丢弃
                  res<fast_tf::CoordinateSystem>,                       // [7]
                  res<LdmDetectorConfig>                                // [8]
              >

extract_system_meta 展开 (折叠表达式)                              [system_meta.hpp:296]
   for I in 0..8:
       extract_one_param<tuple_element_t<I, args>>(meta)          [system_meta.hpp:244]
           │
           ├─ [0,1,2] is_spmc_reader → meta.spmc_channels.emplace_back({typeid(T), typeid(Topic), spmc_reader})
           ├─ [3,4,5,7,8] is_res_type → meta.reads.emplace_back(typeid(T))
           └─ [6] 非 component_kind → 静默跳过（裸 enum）*/
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
        // 编译期分支：如果下标I还没超出tuple长度，才走逻辑
        if constexpr (I < std::tuple_size_v<args_tuple>) 
        {
            // 拿到tuple第I个位置的类型（编译期）
            using ArgType = std::tuple_element_t<I, args_tuple>;

            // 编译期判断：这个类型是不是Writer写组件
            if constexpr (detail::is_writer<ArgType>::value) 
            {
                // 取出cached_args_这个tuple里第I个writer实例
                // 把writer内部的written_flag_指针，指向外面的written_标记
                std::get<I>(*cached_args_).written_flag_ = &written_;
            }

            // 递归下一个下标
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
    /*和 std::remove_reference_t 的区别
    remove_reference_t：仅仅删掉 & / &&，不处理 const、数组；
    decay_t：全套退化，行为完全等同于值传递传参时的类型转换。
    示例对比：
    // remove_reference 只去引用
    std::remove_reference_t<const int&> → const int
    // decay 去引用 + 剥const
    std::decay_t<const int&> → int*/
    return std::make_unique<System<std::decay_t<F>, Policy>>(
        /*std::forward<F>(func) 配合 decay 完整流程
        入参 F&& func：万能引用，既能接收左值 lambda，也能接收右值临时 lambda；
        std::decay_t<F>：拿到 lambda 原始类型，作为 System 的模板参数；
        std::forward<F>(func)：完美转发，把 lambda 原样移入 System 内部，避免拷贝；*/
        std::move(name), std::forward<F>(func));
}
        /*1. 先拆分两个核心部分
        std::decay_t<F>：类型退化工具元函数
        System<T>：模板包装类，用来包裹任意可调用对象（lambda / 函数指针 / 仿函数）
        一、std::decay_t<F> 作用
        std::decay 是类型转换工具，等价于：
        先去掉 const / volatile 修饰
        去掉引用 & / &&
        数组转指针、函数转函数指针
        模板别名 decay_t<T> = typename std::decay<T>::type
        为什么这里必须用 decay？
        传入的 F 往往是：
        右值 lambda（auto&& 万能引用捕获）
        带 &/&& 的可调用对象
        如果直接写 System<F>：
        若 F 是引用类型 Func&，模板实例化会带引用，无法存到 unique_ptr/ 容器；
        lambda 万能引用 auto&& f 推导出 Func&&，带右值引用，不能作为模板参数存到结构体；
        decay_t<F> 统一剥离引用、cv 限定，得到纯粹的值类型，保证 System<T> 存的是实体类型，而非引用。*/

} // namespace talos::scheduler::system