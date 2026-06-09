#include "scheduler/rclcompat/system.hpp"

namespace talos::scheduler::rclcompat {

thread_local CallbackContext g_callback_context;

CallbackContextScope::CallbackContextScope(RclSubSystemBase* current) noexcept
    : ctx_(&g_callback_context)
    , prev_system_(ctx_->current_system) {
    ctx_->current_system = current;
}

CallbackContextScope::~CallbackContextScope() noexcept { ctx_->current_system = prev_system_; }

const SystemMeta& RclSubSystemBase::meta() const noexcept { return meta_; }

} // namespace talos::scheduler::rclcompat
