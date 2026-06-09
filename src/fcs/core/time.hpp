#pragma once

#include <chrono>
#include <cstdint>

namespace fcs::clock {
[[nodiscard]] static uint64_t now_ns() noexcept {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}
} // namespace fcs::clock
