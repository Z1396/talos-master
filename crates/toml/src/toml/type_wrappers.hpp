#pragma once
// 头文件保护，防止重复包含，保证整个编译单元仅加载一次本文件

// 标准库头文件依赖
#include <optional>      // std::optional，required<T> 内部存储依赖
#include <type_traits>   // 类型萃取工具：is_nothrow_move_assignable_v 等类型判断
#include <utility>       // std::move，右值转移工具

// 项目自研TOML反序列化工具命名空间
namespace toml_helper {

// ============================================================================
// 轻量类型包装器说明
// 设计目的：将类型包装器 flatten<T> / required<T> 与繁重的TOML完整反序列化逻辑解耦
// 业务场景：配置结构体头文件只需引入本轻量头文件，不需要引入重量级 <toml++/toml.hpp>
// 大幅减少编译依赖、缩短编译耗时，实现头文件轻量化设计
// ============================================================================

/// flatten<T> 平铺包装器
/// 作用：标记该结构体字段在TOML中**不单独分子表**，直接平铺到父层级
/// 例：struct A { flatten<B> b; }; 不会去 [b] 子表，直接读取顶层b.*键
template <typename T>
struct flatten {
    // 内部包裹原始业务类型T，默认值初始化
    T value{};

    // -------------------------- 取值get接口 --------------------------
    // 左值对象获取内部T引用
    [[nodiscard]] constexpr T& get() noexcept { return value; }
    // const左值对象获取const T引用
    [[nodiscard]] constexpr const T& get() const noexcept { return value; }

    // -------------------------- 箭头重载 -> 便捷访问内部成员 --------------------------
    // 非const左值，支持 wrapper->member 直接访问T内部字段
    [[nodiscard]] constexpr T* operator->() noexcept { return &value; }
    [[nodiscard]] constexpr const T* operator->() const noexcept { return &value; }

    // -------------------------- 解引用重载 * 直接取出T --------------------------
    [[nodiscard]] constexpr T& operator*() noexcept { return value; }
    [[nodiscard]] constexpr const T& operator*() const noexcept { return value; }

    // -------------------------- 隐式类型转换：自动转T引用/右值引用 --------------------------
    // 左值对象隐式转为 T&
    constexpr operator T&() & noexcept { return value; }
    // const左值对象隐式转为 const T&
    constexpr operator const T&() const& noexcept { return value; }
    // 右值对象隐式转为 T&&，自动std::move转移资源，减少拷贝
    constexpr operator T&&() && noexcept { return std::move(value); }
    // const右值对象隐式转为 const T&&
    constexpr operator const T&&() const&& noexcept { return std::move(value); }

    // -------------------------- 赋值运算符重载 --------------------------
    // 拷贝赋值：接收普通左值T
    constexpr flatten& operator=(const T& rhs) {
        value = rhs;
        return *this;
    }
    // 移动赋值：接收T右值，noexcept由T的移动赋值是否无抛决定
    constexpr flatten& operator=(T&& rhs) noexcept(std::is_nothrow_move_assignable_v<T>) {
        value = std::move(rhs);
        return *this;
    }
};

/// required<T> 必填字段包装器
/// 作用：标记TOML配置中**必须存在该字段**，反序列化时缺失直接返回错误
/// 底层依赖 std::optional<T> 存储，区分「未赋值」和「正常赋值」两种状态
template <typename T>
struct required {
    // optional包裹业务类型，默认无值（代表配置缺失）
    std::optional<T> value{};

    // 默认构造：value为空
    constexpr required() = default;
    // 拷贝构造：传入左值T，存入optional
    constexpr required(const T& rhs)
        : value(rhs) {}
    // 移动构造：传入T右值，std::move转移资源，无抛异常由T自身决定
    constexpr required(T&& rhs) noexcept(std::is_nothrow_move_constructible_v<T>)
        : value(std::move(rhs)) {}

    /// 判断当前是否已成功赋值（配置存在）
    [[nodiscard]] constexpr bool has_value() const noexcept { return value.has_value(); }

    // -------------------------- get() 取值接口（全部noexcept(false)，无值时抛异常） --------------------------
    /*写法	调用对象要求	返回值
    get() &	仅左值对象	T&
    get() const &	const 左值	const T&
    get() &&	临时右值	T&&*/
    //noexcept(false)相当于不写noexcept，会主动抛出异常，提前暴露必填字段缺失问题、而且也为了接口统一
    // 左值对象取出T引用
    [[nodiscard]] constexpr T& get() & noexcept(false) { return value.value(); }
    // const左值取出const T引用
    [[nodiscard]] constexpr const T& get() const& noexcept(false) { return value.value(); }
    // 右值对象，移动取出T&&，避免拷贝
    [[nodiscard]] constexpr T&& get() && noexcept(false) { return std::move(value).value(); }
    // const右值取出const T&&
    [[nodiscard]] constexpr const T&& get() const&& noexcept(false) {
        return std::move(value).value();
    }

    // -------------------------- -> 箭头重载，直接访问T内部成员 --------------------------
    [[nodiscard]] constexpr T* operator->() noexcept(false) {
        // 无值时主动抛出 std::bad_optional_access，提前暴露必填字段缺失问题
        if (!value.has_value()) {
            throw std::bad_optional_access{};
        }
        return value.operator->();
    }
    [[nodiscard]] constexpr const T* operator->() const noexcept(false) {
        if (!value.has_value()) {
            throw std::bad_optional_access{};
        }
        return value.operator->();
    }

    // -------------------------- * 解引用重载，等价调用get() --------------------------
    [[nodiscard]] constexpr T& operator*() & { return get(); }
    [[nodiscard]] constexpr const T& operator*() const& { return get(); }
    [[nodiscard]] constexpr T&& operator*() && { return std::move(*this).get(); }
    [[nodiscard]] constexpr const T&& operator*() const&& { return std::move(*this).get(); }

    // -------------------------- 隐式类型转换，自动转T引用/右值 --------------------------
    constexpr operator T&() & { return get(); }
    constexpr operator const T&() const& { return get(); }
    constexpr operator T&&() && { return std::move(get()); }
    constexpr operator const T&&() const&& { return std::move(get()); }

    // -------------------------- 赋值运算符重载 --------------------------
    // 拷贝赋值左值T
    constexpr required& operator=(const T& rhs) {
        value = rhs;
        return *this;
    }
    // 移动赋值右值T，std::move减少内存拷贝
    constexpr required& operator=(T&& rhs) noexcept(std::is_nothrow_move_assignable_v<T>) {
        value = std::move(rhs);
        return *this;
    }
};

// ============================================================================
// detail 内部类型萃取工具集（仅库内部使用，外部业务代码不可调用）
// 作用：给反序列化模板提供编译期类型判断，区分普通T / flatten<T> / required<T>
// ============================================================================
namespace detail {

/// 编译期常量：判断类型是否为 flatten<T> 包装器
// 默认所有类型is_flatten_v = false
template <typename>
inline constexpr bool is_flatten_v = false;
// 模板特化：flatten<T> 匹配时置为true
template <typename T>
inline constexpr bool is_flatten_v<flatten<T>> = true;

/// 萃取 flatten<T> 内部原始类型T
template <typename T>
struct flatten_value;
// 特化匹配 flatten<T>，取出内部T作为type
template <typename T>
struct flatten_value<flatten<T>> {
    using type = T;
};
// 别名简化写法：flatten_value_t<包装类型> 直接拿到内部原始类型
template <typename T>
using flatten_value_t = typename flatten_value<T>::type;

/// 编译期常量：判断类型是否为 required<T> 包装器
template <typename>
inline constexpr bool is_required_v = false;
template <typename T>
inline constexpr bool is_required_v<required<T>> = true;

/// 萃取 required<T> 内部原始类型T
template <typename T>
struct required_value;
template <typename T>
struct required_value<required<T>> {
    using type = T;
};
template <typename T>
using required_value_t = typename required_value<T>::type;

} // namespace detail

} // namespace toml_helper