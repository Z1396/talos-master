#pragma once

#include "toml/detail/common.hpp"

namespace toml_helper {

template <typename T, typename Enable = void>
struct Deserialize;

namespace detail {

template <typename T>
concept ReflectDeserializeCandidate =
    Reflectable<T> && !ReadFrom<T> && !TomlScalarValue<T> && !std::is_enum_v<T>
    && !is_flatten_v<std::remove_cvref_t<T>> && !is_required_v<std::remove_cvref_t<T>>
    && !is_std_optional_v<std::remove_cvref_t<T>> && !is_std_vector_v<std::remove_cvref_t<T>>
    && !is_std_array_v<std::remove_cvref_t<T>> && !is_std_map_v<std::remove_cvref_t<T>>;

template <typename T>
concept TableReadable = ReadFrom<T> || ReflectDeserializeCandidate<T>;

template <ReadFrom T>
[[nodiscard]] std::expected<void, std::string> read_object_into(const toml::table& table, T& out) {
    return out.read_from(table);
}

template <typename T>
[[nodiscard]] std::expected<void, std::string>
    parse_reflected_field(const toml::table& table, std::string_view field_name, T& field);

template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<void, std::string>
    read_reflected_into(const toml::table& table, T& out) {
    std::expected<void, std::string> result{};
    field_reflection::for_each_field(out, [&](std::string_view field_name, auto&& field) {
        if (!result) {
            // field_reflection::for_each_field has no short-circuit hook; once a
            // field fails, preserve that first error and skip subsequent work.
            return;
        }
        if (auto field_result = parse_reflected_field(table, field_name, field); !field_result) {
            result = std::unexpected(field_result.error());
        }
    });
    return result;
}

template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<void, std::string> read_object_into(const toml::table& table, T& out) {
    return read_reflected_into(table, out);
}

template <typename T>
[[nodiscard]] std::expected<void, std::string>
    parse_reflected_field(const toml::table& table, std::string_view field_name, T& field) {
    using field_type = std::remove_cvref_t<T>;

    if constexpr (is_flatten_v<field_type>) {
        using flattened_type = flatten_value_t<field_type>;
        static_assert(
            TableReadable<flattened_type>,
            "toml_helper::flatten<T> requires T to be reflective or implement read_from");

        auto result = read_object_into(table, field.get());
        if (!result) {
            return std::unexpected(prefixed_error(field_name, result.error()));
        }
        return {};
    } else if constexpr (is_required_v<field_type>) {
        using required_type = required_value_t<field_type>;

        static_assert(
            !is_flatten_v<required_type>,
            "toml_helper::required<toml_helper::flatten<T>> is not supported");

        const toml::node* node = table.get(field_name);
        if (!node) {
            return std::unexpected(missing_key_error(field_name));
        }

        auto result = Deserialize<required_type>::take(table, field_name);
        if (!result) {
            return std::unexpected(result.error());
        }

        field = std::move(*result);
        return {};
    } else if constexpr (is_std_optional_v<field_type>) {
        using optional_type = optional_value_t<field_type>;

        static_assert(
            !is_flatten_v<optional_type>,
            "std::optional<toml_helper::flatten<T>> is not supported");

        const toml::node* node = table.get(field_name);
        if (!node) {
            field.reset();
            return {};
        }

        auto result = Deserialize<optional_type>::take(table, field_name);
        if (!result) {
            return std::unexpected(result.error());
        }

        field = std::move(*result);
        return {};
    } else {
        const toml::node* node = table.get(field_name);
        if (!node) {
            return {};
        }

        auto result = Deserialize<field_type>::take(table, field_name);
        if (!result) {
            return std::unexpected(result.error());
        }

        field = std::move(*result);
        return {};
    }
}

template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_reflect_from_node(const toml::node& node, std::string_view key) {
    static_assert(
        std::default_initializable<T>,
        "Reflection-based TOML parsing requires a default-initializable type");

    const auto* table = node.as_table();
    if (!table) {
        return std::unexpected(invalid_table_error(key));
    }

    T value{};
    auto result = read_reflected_into(*table, value);
    if (!result) {
        return std::unexpected(prefixed_error(key, result.error()));
    }

    return parsed_table_value<T>{std::move(value), table};
}

template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_reflect_from_key(const toml::table& table, std::string_view key) {
    const toml::node* node = table.get(key);
    if (!node) {
        return std::unexpected(missing_table_error(key));
    }
    return parse_reflect_from_node<T>(*node, key);
}

template <TableReadable T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_object_from_node(const toml::node& node, std::string_view key) {
    if constexpr (ReadFrom<T>) {
        return parse_read_from_node<T>(node, key);
    } else {
        return parse_reflect_from_node<T>(node, key);
    }
}

template <TableReadable T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_object_from_key(const toml::table& table, std::string_view key) {
    if constexpr (ReadFrom<T>) {
        return parse_read_from_key<T>(table, key);
    } else {
        return parse_reflect_from_key<T>(table, key);
    }
}

} // namespace detail

template <typename T, typename Enable>
struct Deserialize {
    [[nodiscard]] static std::expected<T, std::string> read(const toml::table&, std::string_view) {
        static_assert(
            detail::always_false_v<T>,
            "No toml_helper::Deserialize<T> specialization available for this type");
        return std::unexpected("No deserializer available");
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }
        return detail::as_optional(read(table, key));
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

template <TomlScalarValue T>
struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        return detail::read_required<T>(table, key, [&](const toml::node& node) {
            return detail::parse_scalar_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        return detail::read_optional_value<T>(table, key, [&](const toml::node& node) {
            return detail::parse_scalar_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

template <typename T>
requires(std::is_enum_v<T>) struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        return detail::read_required<T>(table, key, [&](const toml::node& node) {
            return detail::parse_enum_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        return detail::read_optional_value<T>(table, key, [&](const toml::node& node) {
            return detail::parse_enum_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

template <typename T>
struct Deserialize<required<T>, void> {
    [[nodiscard]] static std::expected<required<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::read(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return required<T>{std::move(*parsed)};
    }

    [[nodiscard]] static std::expected<std::optional<required<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::read_optional(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (!*parsed) {
            return std::optional<required<T>>{};
        }
        return std::optional<required<T>>{required<T>{std::move(**parsed)}};
    }

    [[nodiscard]] static std::expected<required<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::take(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return required<T>{std::move(*parsed)};
    }

    [[nodiscard]] static std::expected<std::optional<required<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::take_optional(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (!*parsed) {
            return std::optional<required<T>>{};
        }
        return std::optional<required<T>>{required<T>{std::move(**parsed)}};
    }
};

template <ReadFrom T>
struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_read_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }

        auto parsed = detail::parse_read_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::optional<T>{std::move(parsed->value)};
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_read_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }

        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }

        auto parsed = detail::parse_read_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }

        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<T>{std::move(parsed->value)};
    }
};

template <detail::ReflectDeserializeCandidate T>
struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_reflect_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }

        auto parsed = detail::parse_reflect_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::optional<T>{std::move(parsed->value)};
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_reflect_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }

        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }

        auto parsed = detail::parse_reflect_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }

        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<T>{std::move(parsed->value)};
    }
};

} // namespace toml_helper
