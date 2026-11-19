#pragma once

/**
 * @file
 * @brief 系统元数据、类型萃取工具库
 *
 * 本模块提供三类核心能力：
 * 1. 类型特征萃取：从系统回调参数提取通道、资源类型信息
 * 2. 运行时系统元数据结构体：描述任务调度策略、读写通道、资源依赖
 * 3. 频道唯一键 ChannelKey：用于全局依赖追踪、冲突检测、所有权管控
 */

// 组件基础定义（spmc/spsc/res 等通道包装类型）
#include "components.hpp"
// 调度执行策略、PolicyInfo 定义
#include "execution_policy.hpp"

// 标准库
#include <cstddef>
// 可调用对象包装
#include <functional>
// 系统名称字符串
#include <string>
// 元组，存储函数参数包
#include <tuple>
// 类型特征工具
#include <type_traits>
// 运行时类型索引 typeid
#include <typeindex>
// 移动语义
#include <utility>
// 动态数组存储通道元数据
#include <vector>

namespace talos::scheduler::system {

// ============================================================================
// ChannelKey：频道唯一标识键（消息类型 + 话题标签二元组）
// ============================================================================
/**
 * @brief 唯一标识一个通信频道：(数据类型type, 话题标签topic)
 *
 * 用于全局追踪系统间频道占用、重复发布冲突检测、所有权注册表匹配。
 *设计妙处 ：同一数据类型可以用 Topic 区分不同业务流，
 比如 spmc<ImageFrame, ImageChannelTopic> 和 spmc<ImageFrame, DebugImageTopic> 互不干扰。
 */
struct ChannelKey {
    // 消息载荷类型 type_index
    std::type_index type;
    // 话题标签类型 type_index
    std::type_index topic;

    // C++20 默认相等比较运算符，逐成员相等判断
    bool operator==(const ChannelKey&) const noexcept = default;

    /**
     * @brief 字典序小于比较，用于 std::map 有序映射排序
     * 先比较消息类型，类型不同直接返回大小；类型相等再比较话题标签
     */
    constexpr bool operator<(const ChannelKey& other) const noexcept {
        if (type != other.type) {
            return type < other.type;
        }
        return topic < other.topic;
    }
};

/**
 * @brief ChannelKey 哈希函数，用于 std::unordered_map / std::unordered_set
 *
 * 采用Boost经典哈希组合算法，相比简单异或大幅降低哈希碰撞概率
 * 魔数 0x9e3779b9 = 2^32 / φ（黄金分割倒数），打散高低位，减少哈希聚集
 */
struct ChannelKeyHash {
    constexpr std::size_t operator()(const ChannelKey& k) const noexcept {
        // 第一步：哈希消息类型
        std::size_t seed = std::hash<std::type_index>{}(k.type);
        // Boost 哈希合并公式：seed ^= 新哈希 + 魔数 + 左移6 + 右移2
        seed ^= std::hash<std::type_index>{}(k.topic) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

// ============================================================================
// 函数特征萃取工具 function_traits
// ============================================================================
namespace detail {

// 前置声明：函数特征萃取模板
template <typename F>
struct function_traits;

/**
 * @brief 普通函数指针 特征特化
 * 提取返回值类型、参数元组、参数个数
 */
template <typename R, typename... Args>
/*R      (*)        (Args...)
返回值 指针标记   参数列表*/
struct function_traits<R (*)(Args...)> {
    using return_type                  = R;
    using args_tuple                   = std::tuple<Args...>;
    // 参数数量 arity
    static constexpr std::size_t arity = sizeof...(Args);
};

/**
 * @brief 普通成员函数 特征特化
 * 继承普通函数指针萃取逻辑，复用参数/返回值解析
 */
template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...)> : function_traits<R (*)(Args...)> {};

/**
 * @brief const 常成员函数 特征特化
 */
template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...) const> : function_traits<R (*)(Args...)> {};

/**
 * @brief Lambda/自定义仿函数 通用萃取
 * 取仿函数的 operator() 重载，转发到上面成员函数特化分支
 */
 /*这段代码分为三个部分：
1. template <typename F>：接收一个类型参数。
2. &F::operator()：获取类型 F 的函数调用运算符的成员函数指针。
3. decltype(...)：获取这个成员函数指针的类型，并将其传递给另一个 function_traits 模板进行进一步解析。
其中最重要的是：
decltype(&F::operator())

2. 为什么 Lambda 有 operator()？
我们先写一个 Lambda：
auto func = [](int a, double b) -> float {
    return a + b;
};

编译器会生成一个类似这样的匿名类：
class Lambda {
public:
    float operator()(int a, double b) const {
        return a + b;
    }
};

所以：F实际上就是这个 Lambda 闭包类型。

而：&F::operator()就是获取这个类的成员函数 operator() 的地址。

注意，这里取的不是某个 Lambda 对象的成员函数地址，而是类的成员函数地址。
3. decltype 到底获取了什么？
看这个表达式：
decltype(&F::operator())

对于上面的 Lambda：
auto func = [](int a, double b) -> float {
    return a + b;
};

它得到的类型是：
float (Lambda::*)(int, double) const

我们拆开看：
类型部分	含义
float	返回类型
Lambda::*	指向 Lambda 类成员的成员函数指针
(int, double)	函数参数类型
const	operator() 是 const 成员函数

为什么有 const？

因为普通 Lambda 默认生成的 operator() 是 const 的。

因此，整个类型不是普通函数指针：
float (*)(int, double)

而是成员函数指针：
float (Lambda::*)(int, double) const

这两者完全不同。
4. 为什么要继承自己？
重点来了：
struct function_traits
    : function_traits<decltype(&F::operator())> {};

这里叫作模板递归继承，但它不是无限递归。

假设：
auto func = [](int a, double b) -> float {
    return a + b;
};

编译器首先实例化：
function_traits<Lambda>

继承列表会进一步变成：
function_traits<float (Lambda::*)(int, double) const>

这时，编译器会尝试寻找一个能够匹配成员函数指针类型的 function_traits 特化版本。

例如，项目中可能还存在这样的代码：
template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...) const> {
    using return_type = R;
    using args_tuple = std::tuple<Args...>;

    static constexpr std::size_t arity = sizeof...(Args);
};

注意：这是为了说明机制而写的示例，不代表你项目中一定就是这个定义。

此时：
function_traits<float (Lambda::*)(int, double) const>

就能匹配上面的偏特化。

最终得到：
return_type = float;
args_tuple = std::tuple<int, double>;
arity = 2;

所以最开始的模板：
template <typename F>
struct function_traits
    : function_traits<decltype(&F::operator())> {};
实际上只是一个入口：它负责识别 Lambda，然后把真正的类型解析工作交给成员函数指针版本。*/
template <typename F>
struct function_traits : function_traits<decltype(&F::operator())> {};

/**
 * @brief 提取通道包装类型内部存储的消息类型（T）
 * T = spmc<T,Tag> / spsc_mut<T,Tag> ...
 */
template <component_kind T>
// ::value_type：提取通道包装类型内部存储的消息类型（T）
using inner_type_t = T::value_type;

/**
 * @brief 提取通道包装类型内部话题标签类型（Tag）
 */
template <component_kind T>
// ::topic_type：提取通道包装类型内部话题标签类型（Tag）
using inner_topic_t = T::topic_type;

// ============================================================================
// 类型判断谓词模板：判断通道/资源类型
// ============================================================================
/**
 * @brief 统一通道类型判断模板基类
 * 消除7套重复手写判断结构体，模板参数Kind为通道枚举
 */
template <channel_kind Kind>
struct is_channel_kind {
    // 内层谓词：编译期判断T::kind是否等于指定Kind
    template <typename T>
    // std::bool_constant<T::kind == Kind>：根据T::kind是否等于指定Kind，生成一个bool_constant
    // 该bool_constant的value为T::kind等于指定Kind的布尔值
    // 例如：is_spsc_reader<int>::value == true
    // is_spsc_reader<float>::value == false
    struct trait : std::bool_constant<T::kind == Kind> {};
};

// ========== 各类通道快捷别名谓词 ==========
template <typename T>
struct is_spsc_reader : is_channel_kind<channel_kind::spsc_reader>::trait<T> {};

template <typename T>
struct is_spsc_writer : is_channel_kind<channel_kind::spsc_writer>::trait<T> {};

template <typename T>
struct is_spmc_reader : is_channel_kind<channel_kind::spmc_reader>::trait<T> {};

template <typename T>
struct is_spmc_writer : is_channel_kind<channel_kind::spmc_writer>::trait<T> {};

// 全局只读资源 res<T>
template <typename T>
struct is_res_type : is_channel_kind<channel_kind::res>::trait<T> {};

// 全局读写资源 res_mut<T>
template <typename T>
struct is_res_mut_type : is_channel_kind<channel_kind::res_mut>::trait<T> {};

// 线程局部私有变量 local<T>
template <typename T>
struct is_local_type : is_channel_kind<channel_kind::local>::trait<T> {};

/**
 * @brief 统一写通道判断：spsc_writer || spmc_writer 任意一种返回true
 * 简化业务代码判断“是否为写入通道”
 */
template <typename T>
struct is_writer : std::bool_constant<is_spsc_writer<T>::value || is_spmc_writer<T>::value> {};

} // namespace detail

// ============================================================================
// ChannelMeta：单通道元数据
// ============================================================================
/**
 * @brief 单个通道的元数据描述
 * 存储消息类型、话题标签、通道种类（SPSC/SPMC 读/写）
 */
struct ChannelMeta {
    std::type_index type;
    std::type_index topic;
    channel_kind kind;
};

/**
 * @brief 系统完整运行时元数据 SystemMeta
 *
 * 描述一个调度任务系统全部依赖：调度策略、所有读写通道、全局资源、原子变量
 * 调度器构建拓扑、依赖分析、冲突检测的核心数据源
 */
struct SystemMeta {
    // 系统名称
    std::string name;
    // 执行策略：线程池/固定频率、亲和、优先级
    PolicyInfo policy;
    // SPSC 单生产者单消费者通道列表
    std::vector<ChannelMeta> spsc_channels;
    // SPMC 单生产者多消费者通道列表
    std::vector<ChannelMeta> spmc_channels;
    // 内部原子变量类型（预留扩展）
    std::vector<std::type_index> atomics;
    // 读取的全局资源类型集合 res<T>
    std::vector<std::type_index> reads;
    // 修改的全局资源类型集合 res_mut<T>
    std::vector<std::type_index> writes;
};

namespace detail {

/**
 * @brief 单个参数元数据提取辅助函数
 * 编译期分支匹配参数类型，写入对应SystemMeta容器
 * @tparam T 回调参数包装类型 spmc/spsc/res/local
 * @param meta 待填充系统元数据引用
 */
template <typename T>
constexpr void extract_one_param(SystemMeta& meta) noexcept {
    // 提取内层消息类型、话题标签类型、通道种类类型（SPSC/SPMC 读/写）
    // 例如：Inner = 该通道传输的消息类型（value_type）, Topic = 该通道的话题标签类型（Tag）, kind = spsc_reader
    using Inner = inner_type_t<T>;
    using Topic = inner_topic_t<T>;

    if constexpr (is_spsc_reader<T>::value || is_spsc_writer<T>::value) {
        // SPSC 通道存入spsc_channels
        // emplace_back：在容器末尾构造新元素，避免复制构造和移动构造
        // typeid(Inner)：获取消息类型的类型索引，用于存储在ChannelMeta::type
        // typeid(Topic)：获取话题标签类型的类型索引，用于存储在ChannelMeta::topic
        // T::kind：获取通道种类类型，用于存储在ChannelMeta::kind
        meta.spsc_channels.emplace_back(ChannelMeta{typeid(Inner), typeid(Topic), T::kind});
    } else if constexpr (is_spmc_reader<T>::value || is_spmc_writer<T>::value) {
        // SPMC 通道存入spmc_channels
        meta.spmc_channels.emplace_back(ChannelMeta{typeid(Inner), typeid(Topic), T::kind});
    } else if constexpr (is_res_type<T>::value) {
        // 只读全局资源，加入reads列表
        meta.reads.emplace_back(typeid(Inner));
    } else if constexpr (is_res_mut_type<T>::value) {
        // 可修改全局资源，加入writes列表
        meta.writes.emplace_back(typeid(Inner));
    } else if constexpr (is_local_type<T>::value) {
        // 线程局部变量，不参与依赖图分析，忽略
    }
}

} // namespace detail

/**
 * @brief 从可调用对象（函数/回调/lambda）提取完整系统元数据
 *
 * 编译期遍历函数所有参数，自动解析：
 * 1. 所有读写通道 SPMC/SPSC
 * 2. 全局读写资源 res / res_mut
 * 3. 绑定执行调度策略
 *
 * @tparam F 回调函数/仿函数/lambda 类型
 * @tparam Policy 调度策略，默认default_policy
 * @param name 系统名称，移动语义
 * @return 填充完成的SystemMeta常量对象（编译期可求值constexpr）
 */
template <typename F, typename Policy = default_policy>
constexpr SystemMeta extract_system_meta(std::string name) noexcept {
    SystemMeta meta;
    meta.name   = std::move(name);
    // 填充调度策略信息
    meta.policy = make_policy_info<Policy>();

    // 萃取函数参数元组、参数数量
    using traits = detail::function_traits<F>;
    using args   = traits::args_tuple;

    // 编译期索引序列展开，遍历所有参数，逐个提取元数据
    /*std::index_sequence作用：编译期萃取元组第 I 个元素的类型，纯编译期计算，无运行开销。
    I：编译期下标，从 0 开始
    Tuple：任意 std::tuple / std::pair / std::array 等类元组类型*/
    // [&]：表示lambda内部可以通过引用使用外部变量。
    [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        /*std::tuple_element_t作用：编译期从类元组类型中萃取第 I 个元素的类型，无运行开销。
        下标 I 从 0 开始，且必须是 constexpr 编译常量。*/

        /*...：展开所有参数，逐个提取元数据
        展开后相当于：
        detail::extract_one_param<int>(meta);

        detail::extract_one_param<double>(meta);

        detail::extract_one_param<float>(meta);*/
        // extract_one_param<std::tuple_element_t<Is, args>>(meta)：根据第 Is 个参数类型，提取元数据
        (detail::extract_one_param<std::tuple_element_t<Is, args>>(meta), ...);
    }(std::make_index_sequence<traits::arity>{});
    /*}(std::make_index_sequence<traits::arity>{});
    这个写法容易让人困惑。实际上它就是：
    定义一个 Lambda，然后立即调用它。
    例如普通 Lambda：
    auto func = []() {
        std::cout << "Hello\n";
    };

    func();

    也可以直接写成：
    []() {
        std::cout << "Hello\n";
    }();

    Talos 这里也是一样：
    [&]<std::size_t... Is>(
        std::index_sequence<Is...>
    ) {
        // 提取参数元数据
    }(std::make_index_sequence<traits::arity>{});
    只不过它不是普通 Lambda，而是带模板参数的泛型 Lambda。
    传入的：std::make_index_sequence<traits::arity>{}
    负责告诉它需要处理哪些索引。*/
    return meta;
}

} // namespace talos::scheduler::system