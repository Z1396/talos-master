#pragma once

#include "../system/execution_policy.hpp"
#include "system.hpp"
#include <cstdint>
#include <functional>
#include <string>
#include <utility>

namespace talos::scheduler::rclcompat {

/**
 * @brief Fixed-frequency timer system
 *
 * Runs on a dedicated fixed_rate thread at a fixed frequency.
 * Inherits from RclSubSystemBase to enable Publisher auto-binding mechanism.
 *
 * ## Template parameters
 *
 * - `FrequencyHz`: Execution frequency in Hz
 * - `Affinity`: CPU affinity (-1 = no binding)
 * - `Priority`: Thread priority (0 = default)
 */
template <std::uint32_t FrequencyHz, std::int32_t Affinity = -1, std::int32_t Priority = 0>
class RclTimerSystem : public RclSubSystemBase {
public:
    using Callback = std::function<void()>;
    using Policy   = fixed_rate<FrequencyHz, Affinity, Priority>;

    explicit RclTimerSystem(std::string name, Callback callback) noexcept
        : callback_(std::move(callback)) {
        meta_ = build_meta<Policy>(std::move(name), [](auto&) {
            // Timer systems don't add any channels
        });
    }

    void bind([[maybe_unused]] World& world) noexcept override {
        // Timer typically doesn't need to bind channels
    }

    bool run([[maybe_unused]] World& world) noexcept override {
        // Set thread context to enable Publisher auto-binding
        CallbackContextScope scope(this);
        callback_();

        return false;
    }

private:
    Callback callback_;
};

} // namespace talos::scheduler::rclcompat
