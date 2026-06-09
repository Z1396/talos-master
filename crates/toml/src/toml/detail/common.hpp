#pragma once

#include "toml/type_wrappers.hpp"

#include <array>
#include <concepts>
#include <cstdint>
#include <expected>
#include <fmt/format.h>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef MAGIC_ENUM_RANGE_MIN
# define MAGIC_ENUM_RANGE_MIN 0
#endif
#ifndef MAGIC_ENUM_RANGE_MAX
# define MAGIC_ENUM_RANGE_MAX 16
#endif
#include <field_reflection.hpp>
#include <magic_enum.hpp>

#define TOML_HEADER_ONLY 0
#define TOML_EXCEPTIONS  0
#include <toml++/toml.hpp>

namespace toml_helper {

template <typename T>
concept ReadFrom = requires(T& t, const toml::table& table) {
    { t.read_from(table) } -> std::same_as<std::expected<void, std::string>>;
};

template <typename T>
concept TomlScalarValue = std::is_arithmetic_v<T> || std::is_same_v<T, std::string>
                       || std::is_same_v<T, std::string_view> || std::is_same_v<T, toml::date>
                       || std::is_same_v<T, toml::time> || std::is_same_v<T, toml::date_time>;

template <typename T>
concept Reflectable = field_reflection::field_referenceable<std::remove_cvref_t<T>>
                   && field_reflection::field_namable<std::remove_cvref_t<T>>;

template <typename T>
requires(std::is_enum_v<T>) [[nodiscard]] inline std::string enum_options() {
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

namespace detail {

template <typename>
inline constexpr bool always_false_v = false;

template <typename T>
inline constexpr bool is_std_optional_v = false;

template <typename T>
inline constexpr bool is_std_optional_v<std::optional<T>> = true;

template <typename T>
struct optional_value;

template <typename T>
struct optional_value<std::optional<T>> {
    using type = T;
};

template <typename T>
using optional_value_t = typename optional_value<T>::type;

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

template <typename T>
inline constexpr bool is_std_array_v = false;

template <typename T, std::size_t N>
inline constexpr bool is_std_array_v<std::array<T, N>> = true;

template <typename T>
struct array_value;

template <typename T, std::size_t N>
struct array_value<std::array<T, N>> {
    using type                        = T;
    static constexpr std::size_t size = N;
};

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

template <typename T>
using array_value_t = typename array_value<T>::type;

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

[[nodiscard]] inline std::string join_unread_keys(const toml::table& table) {
    std::string out;
    out.reserve(table.size() * 25);

    bool first = true;
    for (const auto& [k, v] : table) {
        if (!first) {
            out += ", ";
        }
        first = false;
        out += fmt::format("'{}'({})", std::string_view{k}, node_type_name(v.type()));
    }
    return out;
}

[[nodiscard]] inline std::expected<void, std::string>
    error_if_unread(const toml::table& table, std::string_view context) noexcept {
    if (table.empty()) {
        return {};
    }
    if (context.empty()) {
        return std::unexpected(fmt::format("Unread keys: {}", join_unread_keys(table)));
    }
    return std::unexpected(fmt::format("{}: unread keys: {}", context, join_unread_keys(table)));
}

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

[[nodiscard]] inline std::string
    prefixed_error(std::string_view context, std::string_view message) {
    if (context.empty()) {
        return std::string{message};
    }
    return fmt::format("{}: {}", context, message);
}

template <typename T>
[[nodiscard]] constexpr const char* expected_type_name() noexcept {
    if constexpr (std::is_same_v<T, bool>) {
        return "bool";
    } else if constexpr (std::is_same_v<T, std::string>) {
        return "string";
    } else if constexpr (std::is_floating_point_v<T>) {
        return "float";
    } else if constexpr (std::is_integral_v<T>) {
        return "integer";
    } else {
        return "value";
    }
}

template <typename T>
[[nodiscard]] std::optional<T> value_with_numeric_cast(const toml::node& node) noexcept {
    if (auto v = node.value<T>()) {
        return *v;
    }

    if constexpr (std::is_floating_point_v<T>) {
        if (auto i = node.value<int64_t>()) {
            return static_cast<T>(*i);
        }
        if (auto u = node.value<uint64_t>()) {
            return static_cast<T>(*u);
        }
    }

    return std::nullopt;
}

template <TomlScalarValue T>
[[nodiscard]] std::expected<T, std::string>
    parse_scalar_node(const toml::node& node, std::string_view key) {
    if (auto val = value_with_numeric_cast<T>(node)) {
        return *val;
    }

    return std::unexpected(invalid_value_error(key, expected_type_name<T>(), node.type()));
}

template <typename T>
requires(std::is_enum_v<T>) [[nodiscard]] std::expected<T, std::string>
    parse_enum_node(const toml::node& node, std::string_view key) {
    const std::string options = enum_options<T>();

    if (auto s = node.value<std::string>()) {
        if (auto enum_val = magic_enum::enum_cast<T>(*s, magic_enum::case_insensitive)) {
            return *enum_val;
        }
        return std::unexpected(
            fmt::format(
                "Invalid enum value for key '{}': '{}', must be one of {}", key, *s, options));
    }

    if (auto i = node.value<int64_t>()) {
        using U = std::underlying_type_t<T>;
        if (auto enum_val = magic_enum::enum_cast<T>(static_cast<U>(*i))) {
            return *enum_val;
        }
        return std::unexpected(
            fmt::format(
                "Invalid enum value for key '{}': {}, must be one of {}", key, *i, options));
    }

    return std::unexpected(
        fmt::format(
            "Invalid value for key '{}': expected enum as string/integer, got {}; options: {}", key,
            node_type_name(node.type()), options));
}

template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    as_optional(std::expected<T, std::string>&& value) {
    if (!value) {
        return std::unexpected(value.error());
    }
    return std::optional<T>{std::move(*value)};
}

template <typename T>
[[nodiscard]] std::expected<T, std::string> erase_on_success(
    std::expected<T, std::string>&& value, const toml::table& table, std::string_view key) {
    if (!value) {
        return std::unexpected(value.error());
    }
    (void)const_cast<toml::table&>(table).erase(key);
    return std::move(*value);
}

template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string> erase_on_success_optional(
    std::expected<std::optional<T>, std::string>&& value, const toml::table& table,
    std::string_view key) {
    if (!value) {
        return std::unexpected(value.error());
    }
    if (*value) {
        (void)const_cast<toml::table&>(table).erase(key);
    }
    return std::move(*value);
}

template <typename T, typename ParseFn>
[[nodiscard]] std::expected<T, std::string>
    read_required(const toml::table& table, std::string_view key, ParseFn&& parse_fn) {
    const toml::node* node = table.get(key);
    if (!node) {
        return std::unexpected(missing_key_error(key));
    }
    return std::forward<ParseFn>(parse_fn)(*node);
}

template <typename T, typename ParseFn>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    read_optional_value(const toml::table& table, std::string_view key, ParseFn&& parse_fn) {
    const toml::node* node = table.get(key);
    if (!node) {
        return std::optional<T>{};
    }
    return as_optional(std::forward<ParseFn>(parse_fn)(*node));
}

template <typename T>
struct parsed_table_value {
    T value;
    const toml::table* parsed_table;
};

template <ReadFrom T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_read_from_node(const toml::node& node, std::string_view key) {
    const auto* table = node.as_table();
    if (!table) {
        return std::unexpected(invalid_table_error(key));
    }

    T value;
    auto result = value.read_from(*table);
    if (!result) {
        return std::unexpected(prefixed_error(key, result.error()));
    }

    return parsed_table_value<T>{std::move(value), table};
}

template <ReadFrom T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_read_from_key(const toml::table& table, std::string_view key) {
    const toml::node* node = table.get(key);
    if (!node) {
        return std::unexpected(missing_table_error(key));
    }
    return parse_read_from_node<T>(*node, key);
}

} // namespace detail

} // namespace toml_helper

namespace fmt {

template <typename T, typename Char>
struct formatter<toml_helper::required<T>, Char> : formatter<T, Char> {
    template <typename FormatContext>
    auto format(const toml_helper::required<T>& value, FormatContext& ctx) const {
        return formatter<T, Char>::format(value.get(), ctx);
    }
};

} // namespace fmt
