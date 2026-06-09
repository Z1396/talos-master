#include "scheduler/error.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace talos::scheduler::detail {

void panic_message(std::string message) noexcept {
    SPDLOG_CRITICAL("{}", message);
    std::abort();
}

} // namespace talos::scheduler::detail
