#include "scheduler/world.hpp"

namespace talos::scheduler {

void World::freeze_resource_structure() noexcept {
    resource_structure_frozen_.store(true, std::memory_order_release);
}

bool World::resource_structure_frozen() const noexcept {
    return resource_structure_frozen_.load(std::memory_order_acquire);
}

void World::freeze_resource_identity() noexcept { freeze_resource_structure(); }

bool World::resource_identity_frozen() const noexcept { return resource_structure_frozen(); }

std::weak_ptr<WorldLifetimeToken> World::lifetime_token() const noexcept { return lifetime_token_; }

} // namespace talos::scheduler
