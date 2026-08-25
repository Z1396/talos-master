// ===========================================================================
// 练习3：编译期元编程 - 计算 tuple 大小
// 目标：手写 tuple_size，理解模板递归与特化
// 学习要点：模板特化、编译期递归、if constexpr
// ===========================================================================

#include <array>       // std::array（项目真实萃取：array_value）
#include <iostream>    // std::cout, std::endl
#include <map>         // std::map（项目真实萃取：map_key/map_value）
#include <optional>    // std::optional（项目真实萃取：optional_value）
#include <tuple>       // std::tuple, std::tuple_size_v
#include <type_traits> // std::is_same_v, std::false_type, std::bool_constant, std::remove_cvref_t
#include <utility>     // std::move（项目真实包装器 flatten/required 依赖）
#include <vector>      // std::vector（项目真实萃取：vector_value）

// ---------------------------------------------------------------------------
// 1. 自定义编译期类型列表：TypeList<T...>
// 用于学习可变参数模板展开
// ---------------------------------------------------------------------------

/**
 * TypeList - 编译期类型容器
 *
 * 作用：在编译期持有任意数量的类型，类似于运行期的 std::tuple
 *
 * 示例：
 *   TypeList<int, double, char>  // 持有 int, double, char 三个类型
 *   TypeList<>                   // 空列表
 *
 * 注意：这个结构体本身是空的，它只用来"包装"类型信息
 */
template <typename... Ts>
struct TypeList {};

// ---------------------------------------------------------------------------
// 2. 编译期计算 TypeList 长度（核心：递归模板特化）
// ---------------------------------------------------------------------------

/**
 * TypeListSize - 计算 TypeList 中类型的数量
 *
 * 实现原理：模板递归 + 特化
 * 1. 主模板：只声明，不实现（防止误用）
 * 2. 特化1：空列表 -> 大小 0（递归终止条件）
 * 3. 特化2：非空列表 -> 1 + 剩余部分大小（递归步骤）
 *
 * 编译期递归过程（以 TypeList<int, double, char> 为例）：
 *   TypeListSize<TypeList<int, double, char>>::value
 *   = 1 + TypeListSize<TypeList<double, char>>::value
 *   = 1 + (1 + TypeListSize<TypeList<char>>::value)
 *   = 1 + (1 + (1 + TypeListSize<TypeList<>>::value))
 *   = 1 + (1 + (1 + 0))
 *   = 3
 */
template <typename List>
struct TypeListSize; // 主模板：只声明，不实现

// 特化1：空列表 -> 大小 0（递归终止）
template <>
struct TypeListSize<TypeList<>> {
    static constexpr std::size_t value = 0;
    // static constexpr：编译期常量，不占用运行时内存
    // std::size_t：无符号整数类型，通常为 64 位
};

// 特化2：非空列表 -> 1 + 剩余部分大小（递归步骤）
template <typename Head, typename... Tail>
struct TypeListSize<TypeList<Head, Tail...>> {
    static constexpr std::size_t value = 1 + TypeListSize<TypeList<Tail...>>::value;
    // 关键点：
    // 1. TypeList<Head, Tail...> 匹配非空列表
    // 2. Head 是第一个类型（当前计数 1）
    // 3. Tail... 是剩余类型（递归计算）
    // 4. 递归实例化 TypeListSize<TypeList<Tail...>>
    // 5. 直到 Tail... 为空，匹配到空列表特化（终止）
};

// 便捷别名：简化使用语法（C++14 风格）
// 使用方式：tuple_list_size_v<MyList> 替代 TypeListSize<MyList>::value
template <typename List>
inline constexpr std::size_t tuple_list_size_v = TypeListSize<List>::value;
// inline：避免多重定义（C++17 起）
// constexpr：编译期可求值

// ---------------------------------------------------------------------------
// 3. 编译期取 TypeList 第 N 个类型（索引从 0 开始）
// ---------------------------------------------------------------------------

/**
 * TypeAt - 获取 TypeList 中第 N 个类型
 *
 * 实现原理：递归 + 特化
 * 1. 主模板：只声明
 * 2. 特化1：N == 0 -> 返回 Head（第一个类型）
 * 3. 特化2：N > 0 -> 去掉 Head，在 Tail 中查找 N-1
 *
 * 编译期递归过程（以 TypeAt<1, TypeList<int, double, char>> 为例）：
 *   TypeAt<1, TypeList<int, double, char>>::type
 *   = TypeAt<0, TypeList<double, char>>::type  // 去掉 int，索引减为 0
 *   = double  // 索引 0，返回 Head
 */
template <std::size_t N, typename List>
struct TypeAt; // 主模板：只声明

// 特化1：N == 0，返回第一个类型 Head
template <typename Head, typename... Tail>
struct TypeAt<0, TypeList<Head, Tail...>> {
    using type = Head;
    // using 别名：将 Head 暴露为 type
    // 使用：typename TypeAt<0, MyList>::type
};

// 特化2：N > 0，递归到 N-1
template <std::size_t N, typename Head, typename... Tail>
struct TypeAt<N, TypeList<Head, Tail...>> {
    using type = typename TypeAt<N - 1, TypeList<Tail...>>::type;
    // 关键点：
    // 1. N > 0，不是要找的索引
    // 2. 忽略 Head，在 Tail... 中继续找
    // 3. 索引减 1（N - 1）
    // 4. typename 关键字：告诉编译器 TypeAt<...>::type 是一个类型
};

// ---------------------------------------------------------------------------
// 4. 编译期判断类型是否在 TypeList 中
// ---------------------------------------------------------------------------

/**
 * Contains - 判断类型 T 是否在 TypeList 中
 *
 * 实现原理：递归 + 继承 std::bool_constant
 * 1. 空列表：继承 std::false_type（value = false）
 * 2. Head == T：继承 std::true_type（value = true）
 * 3. Head != T：递归检查 Tail...
 *
 * 编译期递归过程（以 Contains<int, TypeList<int, double>> 为例）：
 *   Contains<int, TypeList<int, double>>
 *   = std::true_type  // 匹配到 Head == int
 *   -> value = true
 *
 * 以 Contains<bool, TypeList<int, double>> 为例：
 *   Contains<bool, TypeList<int, double>>
 *   = Contains<bool, TypeList<double>>  // int != bool，继续递归
 *   = Contains<bool, TypeList<>>        // double != bool，继续递归
 *   = std::false_type                   // 空列表，未找到
 *   -> value = false
 */
template <typename T, typename List>
struct Contains; // 主模板：只声明

// 特化1：空列表 -> 未找到（false）
template <typename T>
struct Contains<T, TypeList<>> : std::false_type {};
// std::false_type：标准库类型，包含静态成员 value = false
// 继承后，Contains 自动获得 value = false

// 特化2：非空列表，且第一个类型就是 T -> 找到了（true）
// 注意：这个特化必须在特化3之前，因为更匹配（T 和 Head 精确匹配）
template <typename T, typename Head, typename... Tail>
struct Contains<T, TypeList<Head, Tail...>>
    : std::bool_constant<std::is_same_v<T, Head> || Contains<T, TypeList<Tail...>>::value> {};
// std::bool_constant<bool>：编译期布尔常量包装器
// 等价于：std::integral_constant<bool, ...>
//
// 逻辑：
//   1. 检查 T 是否与 Head 相同（std::is_same_v）
//   2. 如果不相同，递归检查 Tail...
//   3. OR 短路：如果相同，直接 true，不递归
//   4. 最终结果是一个编译期布尔常量

// 便捷别名：简化使用语法
template <typename T, typename List>
inline constexpr bool contains_v = Contains<T, List>::value;
// 使用方式：contains_v<int, MyList> 替代 Contains<int, MyList>::value

// ---------------------------------------------------------------------------
// 5. 运行期函数：打印 std::tuple 信息
// ---------------------------------------------------------------------------

/**
 * print_tuple_info - 打印 std::tuple 的大小（编译期计算）
 *
 * 关键点：
 * 1. std::tuple_size_v：标准库提供的编译期 tuple 大小
 * 2. std::remove_cvref_t：去除 const、volatile、引用修饰符
 *    - 如果传入 const std::tuple<int>&，会剥离得到 std::tuple<int>
 *    - 这样才能正确使用 std::tuple_size_v
 * 3. constexpr：size 在编译期就能确定
 */
template <typename Tuple>
void print_tuple_info() {
    // remove_cvref_t：剥离 const、volatile、引用，拿到原始 tuple 类型
    // 为什么需要？因为 tuple_size_v 要求传入的是"完整类型"，不能带修饰符
    constexpr std::size_t size = std::tuple_size_v<std::remove_cvref_t<Tuple>>;
    // std::tuple_size_v：标准库的 TypeListSize 版本
    // 实现原理完全相同：模板递归 + 特化

    std::cout << "tuple size = " << size << "\n";
}

// ---------------------------------------------------------------------------
// 6. 折叠表达式：编译期参数包求和
// ---------------------------------------------------------------------------

/**
 * sum_all - 对任意数量的参数求和
 *
 * 实现原理：C++17 折叠表达式（fold expression）
 * (... + args) 展开为：arg1 + arg2 + arg3 + ...
 *
 * constexpr：如果所有参数都是编译期常量，结果也在编译期计算
 *
 * 示例：
 *   sum_all(1, 2, 3, 4, 5)
 *   = (((1 + 2) + 3) + 4) + 5
 *   = 15
 *
 * 注意：C++17 支持四种折叠表达式：
 *   - 左折叠：(... + args)  -> ((arg1 + arg2) + arg3)
 *   - 右折叠：(args + ...)  -> (arg1 + (arg2 + arg3))
 *   - 带初始值：(... + args + 0)  -> (((arg1 + arg2) + arg3) + 0)
 *   - 带初始值：(0 + ... + args)  -> (0 + (arg1 + (arg2 + arg3)))
 */
template <typename... Ts>
constexpr auto sum_all(Ts... args) {
    return (... + args); // 一元左折叠，展开为 arg1 + arg2 + arg3 + ...
    // 如果参数包为空，会编译报错（无法折叠空包）
}

// ===========================================================================
// 7. 项目真实代码：模板偏特化 / 全特化实战
// ===========================================================================
// 以下代码直接抽取自 Talos 项目真实源码（仅合并到本文件的 detail 命名空间，
// 未改动实现逻辑与命名），用于展示项目里真正在用的模板元编程写法：
//
//   7.1 function_traits —— crates/scheduler/src/scheduler/system/system_meta.hpp
//        调度器解析 System 回调函数（普通函数/成员函数/lambda）的参数与返回值。
//        同时展示【全特化】（函数指针）与【偏特化】（成员函数）两种写法，
//        并将回调参数包直接变成 std::tuple<Args...>，与本文 tuple 主题呼应。
//
//   7.2 容器类型萃取 —— crates/toml/src/toml/detail/common.hpp
//        TOML 反序列化时在编译期区分 std::optional / std::vector / std::array / std::map，
//        这是「变量模板偏特化置 true + 结构体偏特化萃取内部类型」的标准套路。
//
//   7.3 包装器类型萃取 —— crates/toml/src/toml/type_wrappers.hpp
//        用 flatten<T> / required<T> 标记配置字段语义，偏特化区分「普通T / 包装T」。
// ===========================================================================

// ---------------------------------------------------------------------------
// 7.1 function_traits —— 函数特征萃取（来源：system_meta.hpp）
// 编译期递归实例化：F 为 lambda 时，经通用转发命中成员函数指针偏特化；
// 成员函数偏特化继承函数指针全特化，最终拿到 return_type / args_tuple / arity。
// ---------------------------------------------------------------------------
namespace detail {

// 前置声明：函数特征萃取模板
template <typename F>
struct function_traits;

/**
 * @brief 普通函数指针【全特化】 R (*)(Args...)
 * 提取返回值类型、参数元组、参数个数
 */
template <typename R, typename... Args>
struct function_traits<R (*)(Args...)> {
    using return_type                  = R;                   // 返回值类型
    using args_tuple                   = std::tuple<Args...>; // 参数元组（tuple 主题呼应点）
    static constexpr std::size_t arity = sizeof...(Args);     // 参数个数
};

/**
 * @brief 普通成员函数【偏特化】 R (C::*)(Args...)
 * 继承函数指针全特化，复用参数/返回值解析逻辑
 */
template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...)> : function_traits<R (*)(Args...)> {};

/**
 * @brief const 常成员函数【偏特化】 R (C::*)(Args...) const
 */
template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...) const> : function_traits<R (*)(Args...)> {};

/**
 * @brief Lambda/仿函数【通用转发】
 * 取 decltype(&F::operator()) 得到成员函数指针类型，转发给上面的成员函数偏特化，
 * 通过继承自动获得父特化中的 return_type / args_tuple / arity
 */
template <typename F>
struct function_traits : function_traits<decltype(&F::operator())> {};

} // namespace detail

// ---------------------------------------------------------------------------
// 7.2 容器类型萃取 —— 偏特化识别标准容器（来源：toml/detail/common.hpp）
// 双模板套路：
//   a) 变量模板偏特化 is_std_xxx_v<容器<T>> = true —— 编译期「是不是这种容器」
//   b) 结构体偏特化 xxx_value<容器<T>> 抽出内部 T —— 编译期「容器里装的是什么」
// ---------------------------------------------------------------------------
namespace detail {

// ---- std::optional<T> ----
template <typename T>
inline constexpr bool is_std_optional_v = false;
template <typename T>
inline constexpr bool is_std_optional_v<std::optional<T>> = true;

template <typename T>
struct optional_value;                    // 主模板：只声明
template <typename T>
struct optional_value<std::optional<T>> { // 偏特化：匹配 optional 包装
    using type = T;
};
template <typename T>
using optional_value_t = typename optional_value<T>::type;

// ---- std::vector<T> ----
template <typename T>
inline constexpr bool is_std_vector_v = false;
template <typename T>
inline constexpr bool is_std_vector_v<std::vector<T>> = true;

template <typename T>
struct vector_value;
template <typename T>
struct vector_value<std::vector<T>> {
    using type = T;
};
template <typename T>
using vector_value_t = typename vector_value<T>::type;

// ---- std::array<T, N>（带长度参数的偏特化） ----
template <typename T>
inline constexpr bool is_std_array_v = false;
template <typename T, std::size_t N>
inline constexpr bool is_std_array_v<std::array<T, N>> = true;

template <typename T>
struct array_value;
template <typename T, std::size_t N>
struct array_value<std::array<T, N>> {
    using type                        = T; // 元素类型
    static constexpr std::size_t size = N; // 编译期数组长度
};
template <typename T>
using array_value_t = typename array_value<T>::type;

// ---- std::map<K, V>（双类型参数偏特化） ----
template <typename T>
inline constexpr bool is_std_map_v = false;
template <typename K, typename V>
inline constexpr bool is_std_map_v<std::map<K, V>> = true;

template <typename T>
struct map_key;
template <typename K, typename V>
struct map_key<std::map<K, V>> {
    using type = K;
};
template <typename T>
using map_key_t = typename map_key<T>::type;

template <typename T>
struct map_value;
template <typename K, typename V>
struct map_value<std::map<K, V>> {
    using type = V;
};
template <typename T>
using map_value_t = typename map_value<T>::type;

} // namespace detail

// ---------------------------------------------------------------------------
// 7.3 flatten<T> / required<T> 包装器 + 萃取（来源：toml/type_wrappers.hpp）
// flatten<T>：标记配置字段不单独分子表，直接平铺到父层级
// required<T>：标记配置字段必须存在，缺失即报错
// 偏特化用 is_flatten_v / flatten_value 区分「普通T」与「包装T」
// ---------------------------------------------------------------------------

// ---- flatten<T> 平铺包装器（核心接口，自包含） ----
template <typename T>
struct flatten {
    // 内部包裹原始业务类型T，默认值初始化
    T value{};

    // 取值 get 接口：左值/const左值
    [[nodiscard]] constexpr T& get() noexcept { return value; }
    [[nodiscard]] constexpr const T& get() const noexcept { return value; }

    // 箭头重载：wrapper->member 直接访问内部字段
    [[nodiscard]] constexpr T* operator->() noexcept { return &value; }
    [[nodiscard]] constexpr const T* operator->() const noexcept { return &value; }

    // 解引用重载：*wrapper 直接取出 T
    [[nodiscard]] constexpr T& operator*() noexcept { return value; }
    [[nodiscard]] constexpr const T& operator*() const noexcept { return value; }

    // 隐式转换：左值对象隐式转为 T&，右值对象隐式转为 T&&（自动 move 转移资源）
    constexpr operator T&() & noexcept { return value; }
    constexpr operator const T&() const& noexcept { return value; }
    constexpr operator T&&() && noexcept { return std::move(value); }
    constexpr operator const T&&() const&& noexcept { return std::move(value); }

    // 赋值运算符：接收普通左值T / 右值T
    constexpr flatten& operator=(const T& rhs) {
        value = rhs;
        return *this;
    }
    constexpr flatten& operator=(T&& rhs) noexcept(std::is_nothrow_move_assignable_v<T>) {
        value = std::move(rhs);
        return *this;
    }
};

// ---- required<T> 必填字段包装器（核心接口，自包含） ----
template <typename T>
struct required {
    // optional 包裹业务类型，默认无值（代表配置缺失）
    std::optional<T> value{};

    // 默认构造：value 为空
    constexpr required() = default;
    // 拷贝/移动构造：由 T 直接构造 optional
    constexpr required(const T& rhs)
        : value(rhs) {}
    constexpr required(T&& rhs) noexcept(std::is_nothrow_move_constructible_v<T>)
        : value(std::move(rhs)) {}

    /// 判断当前是否已成功赋值（配置存在）
    [[nodiscard]] constexpr bool has_value() const noexcept { return value.has_value(); }

    // 取值 get 接口：无值时抛出 std::bad_optional_access，提前暴露必填字段缺失
    [[nodiscard]] constexpr T& get() & noexcept(false) { return value.value(); }
    [[nodiscard]] constexpr const T& get() const& noexcept(false) { return value.value(); }
    [[nodiscard]] constexpr T&& get() && noexcept(false) { return std::move(value).value(); }
    [[nodiscard]] constexpr const T&& get() const&& noexcept(false) {
        return std::move(value).value();
    }
};

// ---- 包装器类型萃取（偏特化） ----
namespace detail {

// 编译期常量：判断类型是否为 flatten<T> 包装器（默认 false，偏特化置 true）
template <typename>
inline constexpr bool is_flatten_v = false;
template <typename T>
inline constexpr bool is_flatten_v<flatten<T>> = true;

// 萃取 flatten<T> 内部原始类型T
template <typename T>
struct flatten_value;
template <typename T>
struct flatten_value<flatten<T>> {
    using type = T;
};
template <typename T>
using flatten_value_t = typename flatten_value<T>::type;

// 编译期常量：判断类型是否为 required<T> 包装器
template <typename>
inline constexpr bool is_required_v = false;
template <typename T>
inline constexpr bool is_required_v<required<T>> = true;

// 萃取 required<T> 内部原始类型T
template <typename T>
struct required_value;
template <typename T>
struct required_value<required<T>> {
    using type = T;
};
template <typename T>
using required_value_t = typename required_value<T>::type;

} // namespace detail

// ---------------------------------------------------------------------------
// 7.4 测试用辅助定义（仅用于验证 function_traits）
// 放在 main 外部：普通函数、成员函数必须在命名空间/文件作用域定义，不能嵌在函数体内
// ---------------------------------------------------------------------------

/// 普通自由函数：测试 function_traits 的函数指针【全特化】分支
int free_func(int a, double b) { return a + static_cast<int>(b); }

/// 普通成员函数：测试 function_traits 的成员函数【偏特化】分支
struct Worker {
    void run(int x, char c) {
        (void)x;
        (void)c;
    }
};

/// const 常成员函数：测试 function_traits 的 const 成员函数【偏特化】分支
struct ConstWorker {
    double calc(double x) const { return x * 2.0; }
};

// ---------------------------------------------------------------------------
// 8. main 函数：测试所有功能
// ---------------------------------------------------------------------------

int main() {
    // ===== 测试1：TypeList 大小 =====
    // 定义一个包含 4 个类型的列表
    using MyList = TypeList<int, double, char, float>;

    // 输出大小（编译期计算，运行期只是打印）
    std::cout << "TypeList size = " << tuple_list_size_v<MyList> << "\n";
    // 预期输出：TypeList size = 4

    // ===== 测试2：获取第 N 个类型（编译期断言） =====
    // static_assert：编译期断言，如果条件不满足，编译报错
    // 这里验证索引 1（第二个元素）是 double

    // std::is_same_v<A, B>：编译期判断 A 和 B 是否相同
    static_assert(std::is_same_v<typename TypeAt<1, MyList>::type, double>);
    // 如果 TypeAt<1, MyList>::type 不是 double，编译失败

    // ===== 测试3：判断类型是否存在（编译期断言） =====
    static_assert(contains_v<int, MyList>);   // int 存在 -> 通过
    static_assert(!contains_v<bool, MyList>); // bool 不存在 -> 通过（取反）

    // ===== 测试4：打印 contains 结果（运行期输出） =====
    // std::boolalpha：将 bool 输出为 true/false 而不是 1/0
    std::cout << "contains int: " << std::boolalpha << contains_v<int, MyList> << "\n";
    // 输出：contains int: true

    std::cout << "contains bool: " << contains_v<bool, MyList> << "\n";
    // 输出：contains bool: false

    // ===== 测试5：标准库 std::tuple 对比 =====
    // 创建一个运行期的 tuple（包含值）
    std::tuple<int, double, char, float> t{1, 2.0, 'c', 3.0f};
    // decltype(t)：获取 t 的类型（std::tuple<int, double, char, float>）
    print_tuple_info<decltype(t)>();
    // 输出：tuple size = 4

    // ===== 测试6：折叠表达式求和 =====
    // 编译期计算 1+2+3+4+5 = 15
    std::cout << "sum = " << sum_all(1, 2, 3, 4, 5) << "\n";
    // 输出：sum = 15

    // =========================================================================
    // 以下是【项目真实代码】测试 —— 直接验证 7.1/7.2/7.3 抽取的模板特化
    // =========================================================================

    // ===== 测试7：function_traits（来源 system_meta.hpp） =====
    // 普通自由函数指针：全特化 R(*)(Args...) 命中（free_func 定义见 7.4）
    using FreeTraits = detail::function_traits<decltype(&free_func)>;
    // 返回值类型、参数个数、参数元组全部编译期可查
    static_assert(std::is_same_v<FreeTraits::return_type, int>);
    static_assert(FreeTraits::arity == 2);
    static_assert(std::is_same_v<FreeTraits::args_tuple, std::tuple<int, double>>);
    // 用 std::tuple_size_v 验证参数元组大小 == arity（与本文主题呼应）
    static_assert(std::tuple_size_v<FreeTraits::args_tuple> == FreeTraits::arity);

    // 普通成员函数：偏特化 R(C::*)(Args...) 命中（Worker 定义见 7.4）
    using MemberTraits = detail::function_traits<decltype(&Worker::run)>;
    static_assert(MemberTraits::arity == 2);
    static_assert(std::is_same_v<MemberTraits::args_tuple, std::tuple<int, char>>);

    // const 常成员函数：偏特化 R(C::*)(Args...) const 命中（ConstWorker 定义见 7.4）
    using ConstMemberTraits = detail::function_traits<decltype(&ConstWorker::calc)>;
    static_assert(ConstMemberTraits::arity == 1);
    static_assert(std::is_same_v<ConstMemberTraits::return_type, double>);

    // Lambda：通用转发 decltype(&F::operator()) 命中成员函数偏特化
    // 这是调度器 add_system 最常遇到的情况（回调都是 lambda）
    using LambdaTraits = detail::function_traits<decltype([](int a, double b, char c) {
        (void)b;
        (void)c;
        return a;
    })>;
    static_assert(LambdaTraits::arity == 3);
    static_assert(std::is_same_v<LambdaTraits::return_type, int>);
    static_assert(std::is_same_v<LambdaTraits::args_tuple, std::tuple<int, double, char>>);

    std::cout << "function_traits: lambda arity = " << LambdaTraits::arity
              << ", args_tuple size = " << std::tuple_size_v<LambdaTraits::args_tuple> << "\n";
    // 输出：function_traits: lambda arity = 3, args_tuple size = 3

    // ===== 测试8：容器类型萃取（来源 toml/detail/common.hpp） =====
    // 变量模板偏特化：判断「是不是这种容器」
    static_assert(detail::is_std_optional_v<std::optional<int>>);    // optional<int> -> true
    static_assert(!detail::is_std_optional_v<int>);                  // 普通 int  -> false
    static_assert(detail::is_std_vector_v<std::vector<float>>);      // vector<float> -> true
    static_assert(detail::is_std_array_v<std::array<char, 4>>);      // array<char,4> -> true
    static_assert(!detail::is_std_array_v<std::vector<char>>);       // 不是 array -> false
    static_assert(detail::is_std_map_v<std::map<std::string, int>>); // map -> true

    // 结构体偏特化：萃取容器内部类型（含编译期数组长度）
    static_assert(std::is_same_v<detail::optional_value_t<std::optional<double>>, double>);
    static_assert(std::is_same_v<detail::vector_value_t<std::vector<long>>, long>);
    static_assert(std::is_same_v<detail::array_value_t<std::array<int, 6>>, int>);
    static_assert(detail::array_value<std::array<int, 6>>::size == 6); // 编译期长度
    static_assert(std::is_same_v<detail::map_key_t<std::map<std::string, int>>, std::string>);
    static_assert(std::is_same_v<detail::map_value_t<std::map<std::string, int>>, int>);

    std::cout << "容器萃取: optional<int>=" << detail::is_std_optional_v<std::optional<int>>
              << " array<char,4>=" << detail::is_std_array_v<std::array<char, 4>>
              << " array<int,6> 长度=" << detail::array_value<std::array<int, 6>>::size << "\n";
    // 输出：容器萃取: optional<int>=true array<char,4>=true array<int,6> 长度=6

    // ===== 测试9：flatten/required 包装器萃取（来源 toml/type_wrappers.hpp） =====
    // 偏特化区分「普通T」与「包装T」
    static_assert(!detail::is_flatten_v<int>);            // 普通 int -> false
    static_assert(detail::is_flatten_v<flatten<double>>); // flatten 包 -> true
    static_assert(std::is_same_v<detail::flatten_value_t<flatten<float>>, float>);
    static_assert(!detail::is_required_v<int>);
    static_assert(detail::is_required_v<required<long>>); // required 包 -> true
    static_assert(std::is_same_v<detail::required_value_t<required<short>>, short>);

    // 包装器本身的核心接口也可用（配置反序列化场景）
    flatten<int> fld;     // 默认值 0
    fld = 42;             // 走 operator=(T&&) 移动赋值
    required<int> req{7}; // 拷贝构造：由 7 直接构造 optional
    std::cout << "flatten/required: flatten=" << fld.get() << " required=" << req.get()
              << " has_value=" << std::boolalpha << req.has_value() << "\n";
    // 输出：flatten/required: flatten=42 required=7 has_value=true

    std::cout << "模板元编程演示完成\n";
    return 0;
}