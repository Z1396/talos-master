#pragma once

#include <string>

namespace talos::scheduler::detail {
[[nodiscard]] std::string demangle(const char* name) noexcept;
} // namespace talos::scheduler::detail
