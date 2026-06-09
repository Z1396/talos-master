#pragma once

#include "toml/detail/common.hpp"
#include "toml/detail/core_deserialize.hpp"

namespace toml_helper {

template <typename T>
[[nodiscard]] std::expected<T, std::string> read(const toml::table& table, std::string_view key) {
    return Deserialize<T>::read(table, key);
}

template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    read_optional(const toml::table& table, std::string_view key) {
    return Deserialize<T>::read_optional(table, key);
}

[[nodiscard]] inline std::expected<void, std::string>
    ensure_all_read(const toml::table& table, std::string_view context = {}) noexcept {
    return detail::error_if_unread(table, context);
}

template <typename T>
[[nodiscard]] std::expected<T, std::string> take(const toml::table& table, std::string_view key) {
    return Deserialize<T>::take(table, key);
}

template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    take_optional(const toml::table& table, std::string_view key) {
    return Deserialize<T>::take_optional(table, key);
}

template <detail::TableReadable T>
[[nodiscard]] std::expected<void, std::string> read_into(const toml::table& table, T& out) {
    T parsed{};
    auto result = detail::read_object_into(table, parsed);
    if (!result) {
        return std::unexpected(result.error());
    }
    if (auto unread = detail::error_if_unread(table, {}); !unread) {
        return std::unexpected(unread.error());
    }
    out = std::move(parsed);
    return {};
}

template <detail::TableReadable T>
requires(std::default_initializable<T>)
[[nodiscard]] std::expected<T, std::string> from_table(const toml::table& table) {
    T value{};
    auto result = read_into(table, value);
    if (!result) {
        return std::unexpected(result.error());
    }
    return std::move(value);
}

template <detail::TableReadable T>
[[nodiscard]] std::expected<void, std::string>
    read_flatten_into(const toml::table& table, T& out, std::string_view context) noexcept {
    auto result = detail::read_object_into(table, out);
    if (result) {
        return {};
    }
    return std::unexpected(detail::prefixed_error(context, result.error()));
}

namespace detail {

template <typename Field, typename Parsed>
constexpr void assign_read_opt(Field& field, Parsed&& parsed) {
    if constexpr (is_std_optional_v<std::remove_cvref_t<Field>>) {
        field = std::forward<Parsed>(parsed).value_or(std::nullopt);
    } else if (parsed) {
        field = std::move(*parsed);
    }
}

template <typename Field>
[[nodiscard]] std::expected<void, std::string>
    read_field(const toml::table& table, std::string_view key, Field& field) {
    using field_type = std::remove_cvref_t<Field>;

    if constexpr (is_required_v<field_type>) {
        auto parsed = Deserialize<required_value_t<field_type>>::take(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        field = std::move(*parsed);
        return {};
    } else {
        auto parsed = Deserialize<field_type>::take_optional(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (*parsed) {
            field = std::move(**parsed);
        }
        return {};
    }
}

} // namespace detail

#define READ_OPT(tbl, val)                                               \
    do {                                                                 \
        auto res = toml_helper::take_optional<decltype(val)>(tbl, #val); \
        if (!res) {                                                      \
            return std::unexpected(res.error());                         \
        }                                                                \
        toml_helper::detail::assign_read_opt((val), std::move(*res));    \
    } while (0)

#define READ(tbl, val)                                                  \
    do {                                                                \
        auto res = toml_helper::detail::read_field((tbl), #val, (val)); \
        if (!res) {                                                     \
            return std::unexpected(res.error());                        \
        }                                                               \
    } while (0)

#define READ_FLATTEN(tbl, val)                                         \
    do {                                                               \
        auto res = toml_helper::read_flatten_into((tbl), (val), #val); \
        if (!res) {                                                    \
            return std::unexpected(res.error());                       \
        }                                                              \
    } while (0)

[[nodiscard]] inline std::expected<toml::table, std::string>
    merge_configs(const toml::table& base, const toml::table& override_table) noexcept {
    toml::table result = base;

    for (const auto& [key, override_node] : override_table) {
        auto* base_node               = result.get(key);
        const auto* base_subtable     = base_node ? base_node->as_table() : nullptr;
        const auto* override_subtable = override_node.as_table();

        if (base_subtable && override_subtable) {
            auto merged = merge_configs(*base_subtable, *override_subtable);
            if (!merged) {
                return std::unexpected(merged.error());
            }
            result.insert_or_assign(key, std::move(*merged));
            continue;
        }

        result.insert_or_assign(key, override_node);
    }

    return result;
}

} // namespace toml_helper
