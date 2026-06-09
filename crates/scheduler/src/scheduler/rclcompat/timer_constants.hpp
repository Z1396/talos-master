#pragma once

#include <cstdint>

namespace talos::scheduler::rclcompat {

/**
 * @brief Supported timer frequencies
 *
 * Pre-defined timer frequencies for create_wall_timer.
 * Only these frequencies are allowed - no custom frequencies.
 */
enum class Frequency : std::uint32_t {
    Hz_1    = 1,
    Hz_2    = 2,
    Hz_5    = 5,
    Hz_10   = 10,
    Hz_20   = 20,
    Hz_27   = 27,
    Hz_30   = 30,
    Hz_50   = 50,
    Hz_60   = 60,
    Hz_100  = 100,
    Hz_120  = 120,
    Hz_150  = 150,
    Hz_200  = 200,
    Hz_250  = 250,
    Hz_500  = 500,
    Hz_1000 = 1000,
};

[[nodiscard]] constexpr std::uint32_t to_uint32(Frequency freq) noexcept {
    return static_cast<std::uint32_t>(freq);
}

} // namespace talos::scheduler::rclcompat
