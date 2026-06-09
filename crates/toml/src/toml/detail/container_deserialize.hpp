#pragma once

#include <array>
#include <map>
#include <optional>
#include <vector>

#include "toml/detail/core_deserialize.hpp"

namespace toml_helper::detail {

template <typename T>
[[nodiscard]] std::expected<T, std::string>
    parse_value(const toml::node& node, std::string_view context, bool ensure_all_tables_read);

template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string> parse_optional_value(
    const toml::node& node, std::string_view context, bool ensure_all_tables_read) {
    if (node.type() == toml::node_type::none) {
        return std::optional<T>{};
    }

    auto result = parse_value<T>(node, context, ensure_all_tables_read);
    if (!result) {
        return std::unexpected(result.error());
    }
    return std::optional<T>{std::move(*result)};
}

template <typename T>
[[nodiscard]] std::expected<std::vector<T>, std::string>
    parse_vector_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const toml::array* arr = node.as_array();
    if (!arr) {
        return std::unexpected(invalid_value_error(key, "array", node.type()));
    }

    std::vector<T> result;
    result.reserve(arr->size());

    for (size_t i = 0; i < arr->size(); ++i) {
        const toml::node& elem = (*arr)[i];
        auto elem_context      = fmt::format("{}[{}]", key, i);

        auto elem_result = parse_value<T>(elem, elem_context, ensure_all_tables_read);
        if (!elem_result) {
            return std::unexpected(elem_result.error());
        }

        result.push_back(std::move(*elem_result));
    }

    return result;
}

template <typename T, std::size_t N>
[[nodiscard]] std::expected<std::array<T, N>, std::string>
    parse_array_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const toml::array* arr = node.as_array();
    if (!arr) {
        return std::unexpected(invalid_value_error(key, "array", node.type()));
    }

    if (arr->size() != N) {
        return std::unexpected(
            fmt::format(
                "Invalid size for array '{}': expected {} elements, got {}", key, N, arr->size()));
    }

    std::array<T, N> result{};

    for (size_t i = 0; i < N; ++i) {
        const toml::node& elem = (*arr)[i];
        auto elem_context      = fmt::format("{}[{}]", key, i);

        auto elem_result = parse_value<T>(elem, elem_context, ensure_all_tables_read);
        if (!elem_result) {
            return std::unexpected(elem_result.error());
        }

        result[i] = std::move(*elem_result);
    }

    return result;
}

template <typename V>
[[nodiscard]] std::expected<std::map<std::string, V>, std::string>
    parse_map_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const auto* tbl = node.as_table();
    if (!tbl) {
        return std::unexpected(invalid_value_error(key, "table", node.type()));
    }

    std::map<std::string, V> result;

    for (const auto& [k, v] : *tbl) {
        const auto entry_context = fmt::format("{}.{}", key, std::string_view{k});
        auto entry_result        = parse_value<V>(v, entry_context, ensure_all_tables_read);
        if (!entry_result) {
            return std::unexpected(entry_result.error());
        }
        result.emplace(std::string{k}, std::move(*entry_result));
    }

    return result;
}

template <typename T>
[[nodiscard]] std::expected<T, std::string>
    parse_value(const toml::node& node, std::string_view context, bool ensure_all_tables_read) {
    if constexpr (TomlScalarValue<T>) {
        return parse_scalar_node<T>(node, context);
    } else if constexpr (std::is_enum_v<T>) {
        return parse_enum_node<T>(node, context);
    } else if constexpr (TableReadable<T>) {
        auto parsed = parse_object_from_node<T>(node, context);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (ensure_all_tables_read) {
            if (auto unread = error_if_unread(*parsed->parsed_table, context); !unread) {
                return std::unexpected(unread.error());
            }
        }
        return std::move(parsed->value);
    } else if constexpr (is_required_v<T>) {
        auto parsed = parse_value<required_value_t<T>>(node, context, ensure_all_tables_read);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return T{std::move(*parsed)};
    } else if constexpr (is_std_optional_v<T>) {
        auto parsed =
            parse_optional_value<optional_value_t<T>>(node, context, ensure_all_tables_read);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return T{std::move(*parsed)};
    } else if constexpr (is_std_vector_v<T>) {
        return parse_vector_value<vector_value_t<T>>(node, context, ensure_all_tables_read);
    } else if constexpr (is_std_array_v<T>) {
        return parse_array_value<array_value_t<T>, array_value<T>::size>(
            node, context, ensure_all_tables_read);
    } else if constexpr (is_std_map_v<T>) {
        return parse_map_value<map_value_t<T>>(node, context, ensure_all_tables_read);
    } else {
        static_assert(
            always_false_v<T>,
            "Cannot parse element type: no toml_helper::Deserialize specialization available");
        return std::unexpected("Unsupported element type");
    }
}

} // namespace toml_helper::detail

namespace toml_helper {

template <typename T>
struct Deserialize<std::optional<T>, void> {
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_optional_value<T>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::optional<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::optional<T>>{};
        }

        auto result = detail::parse_optional_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::optional<T>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_optional_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::optional<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::optional<T>>{};
        }

        auto result = detail::parse_optional_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::optional<T>>{std::move(*result)};
    }
};

template <typename T>
struct Deserialize<std::vector<T>, void> {
    [[nodiscard]] static std::expected<std::vector<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_vector_value<T>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::vector<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::vector<T>>{};
        }

        auto result = detail::parse_vector_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::vector<T>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::vector<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_vector_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::vector<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::vector<T>>{};
        }

        auto result = detail::parse_vector_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::vector<T>>{std::move(*result)};
    }
};

template <typename T, std::size_t N>
struct Deserialize<std::array<T, N>, void> {
    [[nodiscard]] static std::expected<std::array<T, N>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_array_value<T, N>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::array<T, N>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::array<T, N>>{};
        }

        auto result = detail::parse_array_value<T, N>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::array<T, N>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::array<T, N>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_array_value<T, N>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::array<T, N>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::array<T, N>>{};
        }

        auto result = detail::parse_array_value<T, N>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::array<T, N>>{std::move(*result)};
    }
};

template <typename V>
struct Deserialize<std::map<std::string, V>, void> {
    [[nodiscard]] static std::expected<std::map<std::string, V>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_map_value<V>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::map<std::string, V>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::map<std::string, V>>{};
        }

        auto result = detail::parse_map_value<V>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::map<std::string, V>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::map<std::string, V>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_map_value<V>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::map<std::string, V>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::map<std::string, V>>{};
        }

        auto result = detail::parse_map_value<V>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::map<std::string, V>>{std::move(*result)};
    }
};

} // namespace toml_helper
