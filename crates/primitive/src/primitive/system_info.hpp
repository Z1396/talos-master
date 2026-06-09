#pragma once

#include <expected>
#include <string>
#include <string_view>

namespace talos::primitive {

class SystemInfo {
public:
    SystemInfo() = delete;

    using Result = std::expected<std::string_view, std::string>;

    [[nodiscard]] static auto get_username() noexcept -> Result;

    [[nodiscard]] static auto get_hostname() noexcept -> Result;
};

} // namespace talos::primitive
