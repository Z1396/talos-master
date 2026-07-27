#pragma once
// 头文件保护宏，防止当前头文件被重复包含，替代老式 #ifndef 头文件守卫

// field_reflection compatibility shim — backed by Boost.PFR
//
// Drop-in replacement for the original yosh-matsuda/field_reflection library.
// Same namespace-level API, drastically reduced compile times by delegating to
// Boost.PFR's heavily optimized reflection engine.
/*
文档注释翻译：
本文件是 field_reflection 兼容适配层，底层基于 Boost.PFR 实现
用途：无缝替换原版第三方反射库 yosh-matsuda/field_reflection
特性：对外命名空间、API 调用逻辑完全对齐旧库，上层业务代码无需修改
优化点：将反射核心逻辑转交高度优化的 Boost.PFR，大幅缩短编译耗时
*/

// Boost.PFR 编译期结构体反射核心头文件，提供字段遍历、字段名提取能力
#include <boost/pfr.hpp>
// std::size_t 无符号整型，用于存储结构体字段数量、数组下标
#include <cstddef>
// std::string_view 只读字符串视图，无内存拷贝，存储编译期字段名称
#include <string_view>
// 类型萃取工具：std::is_aggregate_v、std::remove_cvref_t 等类型判断/修饰剥离工具
#include <type_traits>
// 万能转发 std::forward、编译期索引序列 std::index_sequence 工具
#include <utility>

// 对外顶层命名空间，和原版反射库保持一致，业务层不用修改导入代码
namespace field_reflection {

/// @brief Concept：判断类型T是否为「可遍历成员字段」的聚合类型
/// 前置防护说明：
/// Boost.PFR 内部大量使用 static_assert，不兼容 SFINAE 容错机制
/// 如果直接对非聚合类型调用PFR接口，会直接抛出致命编译错误
/// 因此先用 std::is_aggregate_v 前置过滤，保证concept判断不会崩溃
template <typename T>
/*把 T 身上的 const、volatile、左值引用、右值引用全部剥离，
判断【裸类型】是不是聚合类型（朴素聚合结构体）。
返回 true / false（编译期常量）。*/
// 没有手写构造函数 → 聚合类型
/*std::remove_cvref_t<T>
作用：剥掉类型所有修饰符
去掉引用 & / &&
去掉 const
去掉 volatile*/
concept field_referenceable = std::is_aggregate_v<std::remove_cvref_t<T>>;
/*
拆解逻辑：
std::remove_cvref_t<T>：剥离T身上所有 const/volatile/左值引用&/右值引用&& 修饰，得到原始裸类型
std::is_aggregate_v<X>：编译期常量判断，X是否为C++聚合体（纯数据struct，无自定义构造函数）
只有是聚合体，才满足「可遍历字段」约束
*/

/// @brief Concept：判断类型T是否支持「编译期提取成员字段名」
/// 需要同时满足两个条件：1.是聚合体 2.Boost.PFR能取出它的字段名数组
template <typename T>
/*auto arr = boost::pfr::names_as_array<LaunchConfig>();
返回一个编译期字符串数组，里面存放结构体所有成员的名字（字符串字面量）。
举实例
struct LaunchConfig {
    std::string name;
    int port;
    bool enable_log;
};
调用
auto names = boost::pfr::names_as_array<LaunchConfig>();
得到数组等价：
std::array<std::string_view, 3>{"name", "port", "enable_log"};*/
concept field_namable = std::is_aggregate_v<std::remove_cvref_t<T>>
                     && requires { boost::pfr::names_as_array<std::remove_cvref_t<T>>(); };
/*
两段校验逻辑：
1. 基础门槛：必须是聚合体
2. requires表达式校验：编译期可调用 boost::pfr::names_as_array<裸类型>()
   该接口返回 constexpr std::string_view 数组，存储结构体全部成员的变量名
两个条件同时成立，才代表该类型可以反射拿到字段名字符串
*/

// detail 私有实现命名空间，存放底层工具函数，外部业务禁止直接使用
namespace detail {

/// @brief for_each_field 的底层实现，编译期索引序列展开
/// @tparam T 传入的结构体对象类型（可能带const/&/&&修饰）
/// @tparam F 用户传入的回调函数类型
/// @tparam Is 编译期下标参数包：0,1,2,...,字段总数-1
/// @param obj 待反射遍历的聚合结构体实例，万能转发保留值类别
/// @param func 用户回调，格式 void(std::string_view 字段名, auto&& 字段引用)
/// @param 编译期索引序列，用于包展开遍历所有字段
template <typename T, typename F, std::size_t... Is>
constexpr void for_each_field_impl(T&& obj, F&& func, std::index_sequence<Is...>) {
    // Clean：剥离对象所有修饰符，拿到纯粹的结构体原始类型
    using Clean          = std::remove_cvref_t<T>;
    // 编译期一次性取出当前结构体全部字段名，存为 constexpr 字符串数组
    /*boost::pfr::names_as_array<Clean>()
    Boost.PFR 提供的编译期反射函数。
    作用：提取聚合结构体所有成员变量的名字，返回一个 std::array<std::string_view, N>
    N = 结构体成员数量。*/
    constexpr auto names = boost::pfr::names_as_array<Clean>();
    // C++17 折叠表达式，循环展开所有下标Is，依次执行回调
    // 对每个下标：传入对应字段名 + 通过PFR获取对应字段的引用，调用用户func
    /*### 第 5 部分：, ...
    作用 ：逗号表达式（忽略结果，只执行副作用）
    // 折叠表达式：
    (expr0, expr1, expr2, ...);
    // 等价于：
    (expr0, (expr1, (expr2, ...)));
    - 从左到右执行
    - 返回最后一个表达式的值
    - 但这里我们不关心返回值，只关心执行过程
    ### 第 6 部分：...（折叠展开）
    作用 ：C++17 折叠表达式，展开参数包
    // 假设 Is... = 0, 1, 2（3个字段）
    // 原始代码：
    (func(names[Is], boost::pfr::get<Is>(obj)), ...);
    // 编译器展开为：
    (func(names[0], boost::pfr::get<0>(obj)),
     func(names[1], boost::pfr::get<1>(obj)),
     func(names[2], boost::pfr::get<2>(obj))

    ```*/
    (func(names[Is], boost::pfr::get<Is>(std::forward<T>(obj))), ...);
}

} // namespace detail

/// @brief 对外暴露的反射遍历接口：遍历聚合体所有成员字段
/// @tparam T 待反射的聚合结构体类型
/// @tparam F 回调函数，接收 (std::string_view name, auto&& field_val)
/// @param obj 待遍历的结构体实例，支持左值/右值/const对象万能转发
/*关键知识点 ：

- T&& 在模板中是 转发引用 （万能引用）
- T&& 在非模板中是 右值引用 （只接受右值）
- std::forward<T>() 实现完美转发，保留原始值类别*/
template <typename T, typename F>
constexpr void for_each_field(T&& obj, F&& func) {
    // 剥离对象修饰符，得到纯净裸类型，用于获取字段数量
    using Clean = std::remove_cvref_t<T>;
    // 调用底层实现，传入万能转发对象、回调、以及0~字段总数的索引序列
    detail::for_each_field_impl(
        std::forward<T>(obj), std::forward<F>(func),
        // boost::pfr::tuple_size_v<Clean>：编译期获取结构体成员字段总数
        // std::make_index_sequence<N>：生成编译期索引序列 0,1,2,...,N-1
        // 组合效果：创建与结构体字段数量匹配的索引序列，用于后续编译期字段遍历展开
        std::make_index_sequence<boost::pfr::tuple_size_v<Clean>>{});
}

} // namespace field_reflection