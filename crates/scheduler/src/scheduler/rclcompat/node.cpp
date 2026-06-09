#include "scheduler/rclcompat/node.hpp"

#include "scheduler/rclcompat/timer_factory.hpp"

#include <expected>

namespace talos::scheduler::rclcompat {

Node::Node(std::string name, Scheduler& scheduler) noexcept
    : name_(std::move(name))
    , scheduler_(scheduler) {}

void Node::create_wall_timer(const Frequency frequency, std::function<void()> callback) {
    std::string system_name = name_ + "/timer/" + std::to_string(pending_systems_.size());

    auto system = create_timer_system(std::move(system_name), frequency, std::move(callback));
    pending_systems_.push_back(std::move(system));
}

std::string_view Node::name() const noexcept { return name_; }

BuildResult Node::finalize() { return finalize_impl(false); }

BuildResult Node::unsafe_finalize() { return finalize_impl(true); }

OwnershipRegistry& Node::registry() noexcept { return *registry_; }

BuildResult Node::finalize_impl(const bool allow_running_finalize) {
    if (scheduler_.is_running() && !allow_running_finalize) {
        using namespace talos::scheduler;
        panic(
            "Node '{}': finalize() while scheduler is running is unsafe; use "
            "unsafe_finalize() if you need the explicit escape hatch",
            name_);
    }

    std::size_t consumed = 0;
    for (; consumed < pending_systems_.size(); ++consumed) {
        auto& sys = pending_systems_[consumed];
        if (scheduler_.is_running()) {
            if (auto result = scheduler_.unsafe_hot_add_system(std::move(sys)); !result) {
                pending_systems_.erase(
                    pending_systems_.begin(),
                    pending_systems_.begin() + static_cast<std::ptrdiff_t>(consumed + 1));
                return result;
            }
        } else {
            if (auto result = scheduler_.add_system(std::move(sys)); !result) {
                pending_systems_.erase(
                    pending_systems_.begin(),
                    pending_systems_.begin() + static_cast<std::ptrdiff_t>(consumed + 1));
                return std::unexpected(BuildError{result.error()});
            }
        }
    }
    pending_systems_.clear();

    if (!scheduler_.is_running()) {
        return scheduler_.build();
    }

    return {};
}

} // namespace talos::scheduler::rclcompat
