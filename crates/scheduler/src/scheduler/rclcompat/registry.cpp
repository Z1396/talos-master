#include "scheduler/rclcompat/registry.hpp"

#include "scheduler/demangle.hpp"
#include "scheduler/error.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace talos::scheduler::rclcompat {

void OwnershipRegistry::register_owner(const system::ChannelKey key, std::string owner) noexcept {
    with_lock([&] {
        if (owners_.contains(key)) {
            SPDLOG_CRITICAL(
                "Channel {}@{} is already owned by '{}', but '{}' attempted to register",
                talos::scheduler::detail::demangle(key.type.name()),
                talos::scheduler::detail::demangle(key.topic.name()), owners_[key], owner);
            std::abort();
        }
        owners_[key] = std::move(owner);
    });
}

bool OwnershipRegistry::try_claim(const system::ChannelKey key) noexcept {
    return with_lock([&] {
        if (claimed_.contains(key)) {
            return false;
        }
        claimed_.insert(key);
        return true;
    });
}

void OwnershipRegistry::release_claim(const system::ChannelKey key) noexcept {
    with_lock([&] { claimed_.erase(key); });
}

void OwnershipRegistry::assert_owner(
    const system::ChannelKey key, const std::string_view caller) const noexcept {
    with_lock([&] {
        if (const auto it = owners_.find(key); it == owners_.end()) {
            using namespace talos::scheduler;
            panic(
                "Channel {}@{} is not registered, but '{}' attempted to access",
                talos::scheduler::detail::demangle(key.type.name()),
                talos::scheduler::detail::demangle(key.topic.name()), caller);
        }
    });
}

} // namespace talos::scheduler::rclcompat
