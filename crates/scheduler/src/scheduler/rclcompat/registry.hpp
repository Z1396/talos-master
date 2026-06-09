#pragma once

#include "../system/system_meta.hpp"

#include <concepts>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <type_traits>

namespace talos::scheduler::rclcompat {

// ============================================================================
// OwnershipRegistry: Track channel ownership
// ============================================================================

/**
 * @brief Registry for tracking channel ownership
 *
 * Ensures single-publisher-per-channel constraint at runtime.
 * Thread-safe for registration operations.
 */
class OwnershipRegistry {
public:
    /**
     * @brief Register a channel owner
     *
     * ## Parameters
     *
     * - `key`: channel identifier
     * - `owner`: name of the owning system
     *
     * ## Thread safety
     *
     * Thread-safe (mutex protected)
     */
    void register_owner(system::ChannelKey key, std::string owner) noexcept;

    /**
     * @brief Try to claim a publisher handle
     *
     * A channel can only be claimed once (enforces move-only semantics at runtime).
     *
     * ## Parameters
     *
     * - `key`: channel identifier
     *
     * ## Returns
     *
     * `true` if claim succeeded, `false` if already claimed
     *
     * ## Thread safety
     *
     * Thread-safe (mutex protected)
     */
    [[nodiscard]] bool try_claim(system::ChannelKey key) noexcept;

    /**
     * @brief Release a claimed publisher handle
     *
     * Called when Publisher is moved or destroyed.
     *
     * ## Parameters
     *
     * - `key`: channel identifier
     *
     * ## Thread safety
     *
     * Thread-safe (mutex protected)
     */
    void release_claim(system::ChannelKey key) noexcept;

    /**
     * @brief Assert that a channel is owned by a specific system
     *
     * ## Parameters
     *
     * - `key`: channel identifier
     * - `caller`: name of the caller for error reporting
     *
     * ## Panics
     *
     * Aborts if the channel is not owned by the caller
     */
    void assert_owner(system::ChannelKey key, std::string_view caller) const noexcept;

private:
    mutable std::mutex lock_;
    std::map<system::ChannelKey, std::string> owners_;
    std::set<system::ChannelKey> claimed_;

    /// Helper: Execute callback with lock held (reduces lock boilerplate)
    template <std::invocable F>
    auto with_lock(F&& func) const noexcept(std::is_nothrow_invocable_v<F>) -> decltype(func()) {
        std::lock_guard lock(lock_);
        return func();
    }
};

} // namespace talos::scheduler::rclcompat
