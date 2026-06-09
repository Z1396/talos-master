#pragma once

#include <Eigen/Core>

#include <vector>

#include "toml/core.hpp"

namespace toml_helper {

template <typename Mat>
requires(Mat::RowsAtCompileTime != Eigen::Dynamic && Mat::ColsAtCompileTime != Eigen::Dynamic)
struct Deserialize<Mat, void> {
private:
    static std::expected<Mat, std::string>
        parse_node(const toml::node& node, std::string_view key) {
        constexpr int expected_size = Mat::RowsAtCompileTime * Mat::ColsAtCompileTime;
        using Scalar                = typename Mat::Scalar;

        std::vector<Scalar> scalar_array;
        scalar_array.reserve(expected_size);

        const auto* arr = node.as_array();
        if (!arr) {
            return std::unexpected(
                fmt::format(
                    "Invalid value for key '{}': expected array, got {}", key,
                    detail::node_type_name(node.type())));
        }

        for (const auto& value_node : *arr) {
            if (const auto scalar = detail::value_with_numeric_cast<Scalar>(value_node)) {
                scalar_array.push_back(*scalar);
                continue;
            }
            return std::unexpected(fmt::format("'{}' must be an array of numbers", key));
        }

        if (scalar_array.size() != static_cast<std::size_t>(expected_size)) {
            return std::unexpected(
                fmt::format("'{}' must have exactly '{}' elements", key, expected_size));
        }

        return Eigen::Map<const Mat>(scalar_array.data());
    }

public:
    [[nodiscard]] static std::expected<Mat, std::string>
        read(const toml::table& table, std::string_view key) {
        return detail::read_required<Mat>(
            table, key, [&](const toml::node& node) { return parse_node(node, key); });
    }

    [[nodiscard]] static std::expected<std::optional<Mat>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        return detail::read_optional_value<Mat>(
            table, key, [&](const toml::node& node) { return parse_node(node, key); });
    }

    [[nodiscard]] static std::expected<Mat, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    [[nodiscard]] static std::expected<std::optional<Mat>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

} // namespace toml_helper
