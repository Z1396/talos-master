#pragma once
// 头文件保护，防止重复包含

// 引入之前定义的 flatten<T> / required<T> 类型包装器
#include "toml/type_wrappers.hpp"

// 标准库基础依赖
#include <array>
#include <concepts>        // C++20 约束concept，用于类型校验
#include <cstdint>
#include <expected>        // C++23 std::expected 错误处理核心
#include <fmt/format.h>    // 高性能格式化日志
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>     // 编译期类型判断萃取
#include <utility>         // std::move / std::forward

// 容器支持：vector动态数组
#include <vector>

// magic_enum 枚举反射配置，限定枚举取值范围0~16，避免过大编译期开销
#ifndef MAGIC_ENUM_RANGE_MIN
# define MAGIC_ENUM_RANGE_MIN 0
#endif
#ifndef MAGIC_ENUM_RANGE_MAX
# define MAGIC_ENUM_RANGE_MAX 16
#endif
// 结构体字段反射库，自动遍历结构体成员名与成员引用
#include <field_reflection.hpp>
// 枚举反射库，枚举字符串/值互相转换
#include <magic_enum.hpp>

// toml++ 全局配置：关闭头文件拆分、关闭异常（适配实时无抛代码）
#define TOML_HEADER_ONLY 0
#define TOML_EXCEPTIONS  0
#include <toml++/toml.hpp>

// TOML 工具核心命名空间
namespace toml_helper {

// ============================================================================
// C++20 Concept 类型约束（编译期校验类型能力）
// ============================================================================
/// 约束：类型T具备 read_from 方法，可从toml::table反序列化自身
template <typename T>
concept ReadFrom = requires(T& t, const toml::table& table) {
    // 方法签名必须是：std::expected<void, std::string> read_from(const toml::table&)
    { t.read_from(table) } -> std::same_as<std::expected<void, std::string>>;
};

/// 约束：TOML原生标量基础类型（可直接从节点读取，无需嵌套子表）
template <typename T>
concept TomlScalarValue = std::is_arithmetic_v<T>          // int/float/bool
                       || std::is_same_v<T, std::string>
                       || std::is_same_v<T, std::string_view>
                       || std::is_same_v<T, toml::date>
                       || std::is_same_v<T, toml::time>
                       || std::is_same_v<T, toml::date_time>;

/// 约束：支持字段反射的结构体，可自动遍历字段名+成员引用
template <typename T>
concept Reflectable = field_reflection::field_referenceable<std::remove_cvref_t<T>>
                   && field_reflection::field_namable<std::remove_cvref_t<T>>;

// ============================================================================
// 枚举工具函数：获取全部合法枚举字符串列表，用于报错提示
// ============================================================================
/// 生成枚举所有合法选项字符串，如 "'A', 'B', 'C'"
template <typename T>
requires(std::is_enum_v<T>) [[nodiscard]] inline std::string enum_options() {
    // magic_enum 获取所有枚举名称
    auto all = magic_enum::enum_names<T>();
    std::string options;
    for (const auto& name : all) {
        if (!options.empty()) {
            options += ", ";
        }
        options += fmt::format("'{}'", name);
    }
    return options;
}

// ============================================================================
// detail 内部底层工具集，业务代码禁止直接调用
// ============================================================================
namespace detail {

// 永远为false的编译期常量，用于static_assert报错提示
template <typename>
inline constexpr bool always_false_v = false;

// ------------------------------
// std::optional 类型萃取
// ------------------------------
template <typename T>
inline constexpr bool is_std_optional_v = false;
template <typename T>
inline constexpr bool is_std_optional_v<std::optional<T>> = true;

// 提取 std::optional<T> 内部原始类型T
template <typename T>
struct optional_value;
template <typename T>
struct optional_value<std::optional<T>> {
    using type = T;
};
template <typename T>
using optional_value_t = typename optional_value<T>::type;

// ------------------------------
// std::vector 容器萃取
// ------------------------------
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

// ------------------------------
// std::array 固定数组萃取
// ------------------------------
template <typename T>
inline constexpr bool is_std_array_v = false;
template <typename T, std::size_t N>
inline constexpr bool is_std_array_v<std::array<T, N>> = true;

template <typename T>
struct array_value;
template <typename T, std::size_t N>
struct array_value<std::array<T, N>> {
    using type                        = T;
    static constexpr std::size_t size = N; // 静态数组长度
};
template <typename T>
using array_value_t = typename array_value<T>::type;

// ------------------------------
// std::map 字典萃取
// ------------------------------
template <typename T>
inline constexpr bool is_std_map_v = false;
template <typename K, typename V>
inline constexpr bool is_std_map_v<std::map<K, V>> = true;

// 提取map键类型
template <typename T>
struct map_key;
template <typename K, typename V>
struct map_key<std::map<K, V>> { using type = K; };
template <typename T>
using map_key_t = typename map_key<T>::type;

// 提取map值类型
template <typename T>
struct map_value;
template <typename K, typename V>
struct map_value<std::map<K, V>> { using type = V; };
template <typename T>
using map_value_t = typename map_value<T>::type;

// ============================================================================
// 错误文本辅助工具
// ============================================================================
/// 将toml节点枚举转为可读字符串，用于日志
[[nodiscard]] inline const char* node_type_name(toml::node_type t) noexcept {
    switch (t) {
    case toml::node_type::none: return "none";
    case toml::node_type::table: return "table";
    case toml::node_type::array: return "array";
    case toml::node_type::string: return "string";
    case toml::node_type::integer: return "integer";
    case toml::node_type::floating_point: return "float";
    case toml::node_type::boolean: return "bool";
    case toml::node_type::date: return "date";
    case toml::node_type::time: return "time";
    case toml::node_type::date_time: return "datetime";
    }
    return "unknown";
}

/// 拼接table内所有未读取key，用于冗余配置报错
[[nodiscard]] inline std::string join_unread_keys(const toml::table& table) {
    std::string out;
    // 预分配内存减少扩容
    out.reserve(table.size() * 25);

    bool first = true;
    for (const auto& [k, v] : table) {
        if (!first) out += ", ";
        first = false;
        out += fmt::format("'{}'({})", std::string_view{k}, node_type_name(v.type()));
    }
    return out;
}

/// 校验表内存在未读取key，返回错误
[[nodiscard]] inline std::expected<void, std::string>
    error_if_unread(const toml::table& table, std::string_view context) noexcept {
    if (table.empty()) return {};
    if (context.empty()) {
        return std::unexpected(fmt::format("Unread keys: {}", join_unread_keys(table)));
    }
    return std::unexpected(fmt::format("{}: unread keys: {}", context, join_unread_keys(table)));
}

// 各类固定错误模板
[[nodiscard]] inline std::string missing_key_error(std::string_view key) {
    return fmt::format("Missing key '{}'", key);
}
[[nodiscard]] inline std::string missing_table_error(std::string_view key) {
    return fmt::format("Missing table '{}'", key);
}
[[nodiscard]] inline std::string invalid_table_error(std::string_view key) {
    return fmt::format("Invalid value for key '{}': expected table", key);
}
[[nodiscard]] inline std::string
    invalid_value_error(std::string_view key, std::string_view expected, toml::node_type actual) {
    return fmt::format(
        "Invalid value for key '{}': expected {}, got {}", key, expected, node_type_name(actual));
}
/// 给错误信息增加上下文前缀（如字段名）
[[nodiscard]] inline std::string
    prefixed_error(std::string_view context, std::string_view message) {
    if (context.empty()) return std::string{message};
    return fmt::format("{}: {}", context, message);
}

// 根据C++类型返回对应TOML类型名称
template <typename T>
[[nodiscard]] constexpr const char* expected_type_name() noexcept {
    if constexpr (std::is_same_v<T, bool>) return "bool";
    else if constexpr (std::is_same_v<T, std::string>) return "string";
    else if constexpr (std::is_floating_point_v<T>) return "float";
    else if constexpr (std::is_integral_v<T>) return "integer";
    else return "value";
}

// ============================================================================
// 数值兼容读取：int自动转float，支持数字类型隐式转换
// ============================================================================
/// 读取节点值，支持整数自动强转浮点；失败返回nullopt
template <typename T>
[[nodiscard]] std::optional<T> value_with_numeric_cast(const toml::node& node) noexcept {
    // 原生类型匹配直接返回
    if (auto v = node.value<T>()) {
        return *v;
    }
    // 浮点类型兼容读取整数
    if constexpr (std::is_floating_point_v<T>) {
        if (auto i = node.value<int64_t>()) return static_cast<T>(*i);
        if (auto u = node.value<uint64_t>()) return static_cast<T>(*u);
    }
    return std::nullopt;
}

// ============================================================================
// 标量/枚举解析核心函数
// ============================================================================
/// 解析基础标量（int/float/string/date等）
template <TomlScalarValue T>
[[nodiscard]] std::expected<T, std::string>
    parse_scalar_node(const toml::node& node, std::string_view key) {
    if (auto val = value_with_numeric_cast<T>(node)) {
        return *val;
    }
    // 类型不匹配，返回格式化错误
    return std::unexpected(invalid_value_error(key, expected_type_name<T>(), node.type()));
}

/// 解析枚举：支持字符串/数字两种写法，大小写不敏感
template <typename T>
requires(std::is_enum_v<T>) [[nodiscard]] std::expected<T, std::string>
    parse_enum_node(const toml::node& node, std::string_view key) {
    const std::string options = enum_options<T>();
    // 字符串形式枚举
    if (auto s = node.value<std::string>()) {
        if (auto enum_val = magic_enum::enum_cast<T>(*s, magic_enum::case_insensitive)) {
            return *enum_val;
        }
        return std::unexpected(fmt::format(
            "Invalid enum value for key '{}': '{}', must be one of {}", key, *s, options));
    }
    // 数字底层值枚举
    if (auto i = node.value<int64_t>()) {
        using U = std::underlying_type_t<T>;
        if (auto enum_val = magic_enum::enum_cast<T>(static_cast<U>(*i))) {
            return *enum_val;
        }
        return std::unexpected(fmt::format(
            "Invalid enum value for key '{}': {}, must be one of {}", key, *i, options));
    }
    // 既不是字符串也不是数字
    return std::unexpected(fmt::format(
        "Invalid value for key '{}': expected enum as string/integer, got {}; options: {}", key,
        node_type_name(node.type()), options));
}

// ============================================================================
// 结果包装转换工具
// ============================================================================
/// 将 T 类型成功结果包装为 std::optional<T> 用于可选字段
template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    as_optional(std::expected<T, std::string>&& value) {
    if (!value) return std::unexpected(value.error());
    return std::optional<T>{std::move(*value)};
}

/// take 专用：读取成功后从table中删除该key，标记为已读（用于冗余key检测）
template <typename T>
[[nodiscard]] std::expected<T, std::string> erase_on_success(
    std::expected<T, std::string>&& value, const toml::table& table, std::string_view key) {
    if (!value) return std::unexpected(value.error());
    // const table 强制转可修改，删除已读取key
    (void)const_cast<toml::table&>(table).erase(key);
    return std::move(*value);
}

/// 可选字段 take 版本，存在值才删除key
template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string> erase_on_success_optional(
    std::expected<std::optional<T>, std::string>&& value, const toml::table& table,
    std::string_view key) {
    if (!value) return std::unexpected(value.error());
    if (*value) {
        (void)const_cast<toml::table&>(table).erase(key);
    }
    return std::move(*value);
}

// ============================================================================
// 通用读取包装器
// ============================================================================
/// 读取必填key，传入自定义解析逻辑
template <typename T, typename ParseFn>
[[nodiscard]] std::expected<T, std::string>
    read_required(const toml::table& table, std::string_view key, ParseFn&& parse_fn) {
    const toml::node* node = table.get(key);
    if (!node) return std::unexpected(missing_key_error(key));
    return std::forward<ParseFn>(parse_fn)(*node);
}

/// 读取可选key，不存在返回nullopt
template <typename T, typename ParseFn>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    read_optional_value(const toml::table& table, std::string_view key, ParseFn&& parse_fn) {
    const toml::node* node = table.get(key);
    if (!node) return std::optional<T>{};
    return as_optional(std::forward<ParseFn>(parse_fn)(*node));
}

// ============================================================================
// 自定义ReadFrom结构体解析支撑
// ============================================================================
// 存储解析后的结构体与对应源table，用于后续未读key校验
template <typename T>
struct parsed_table_value {
    T value;
    const toml::table* parsed_table;
};

/// 解析子表为ReadFrom结构体
template <ReadFrom T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_read_from_node(const toml::node& node, std::string_view key) {
    const auto* table = node.as_table();
    if (!table) return std::unexpected(invalid_table_error(key));

    T value;
    auto result = value.read_from(*table);
    if (!result) {
        return std::unexpected(prefixed_error(key, result.error()));
    }
    return parsed_table_value<T>{std::move(value), table};
}

/// 根据key查找子表并解析为ReadFrom结构体
template <ReadFrom T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_read_from_key(const toml::table& table, std::string_view key) {
    const toml::node* node = table.get(key);
    if (!node) return std::unexpected(missing_table_error(key));
    return parse_read_from_node<T>(*node, key);
}

} // namespace detail

} // namespace toml_helper

// ============================================================================
// fmt 格式化支持：让fmt可以直接打印 required<T> 包装器
// ============================================================================
namespace fmt {
template <typename T, typename Char>
struct formatter<toml_helper::required<T>, Char> : formatter<T, Char> {
    template <typename FormatContext>
    auto format(const toml_helper::required<T>& value, FormatContext& ctx) const {
        // 取出内部T再格式化输出
        return formatter<T, Char>::format(value.get(), ctx);
    }
};
} // namespace fmt