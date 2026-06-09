#include "scheduler/error_formatter.hpp"

#include <magic_enum.hpp>

#include <span>
#include <string>
#include <variant>

namespace talos::scheduler::detail {

static std::string join_strings(std::span<const std::string> strs, std::string_view sep) {
    if (strs.empty()) {
        return "";
    }
    std::string result = strs[0];
    for (std::size_t i = 1; i < strs.size(); ++i) {
        result += sep;
        result += strs[i];
    }
    return result;
}

} // namespace talos::scheduler::detail

namespace fmt {

auto formatter<talos::scheduler::SchedulerError>::format(
    const talos::scheduler::SchedulerError e, format_context& ctx) const
    -> format_context::iterator {
    return formatter<std::string_view>::format(magic_enum::enum_name(e), ctx);
}

auto formatter<talos::scheduler::ChannelKindConflict>::format(
    const talos::scheduler::ChannelKindConflict& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "ChannelKindConflict: channel used as both SPSC and SPMC by '{}' and '{}'",
        e.first_system, e.second_system);
}

auto formatter<talos::scheduler::MultipleWritersError>::format(
    const talos::scheduler::MultipleWritersError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "MultipleWriters: channel has multiple writers: {}",
        talos::scheduler::detail::join_strings(e.writers, ", "));
}

auto formatter<talos::scheduler::MultipleReadersError>::format(
    const talos::scheduler::MultipleReadersError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "MultipleReaders: SPSC channel has multiple readers: {}",
        talos::scheduler::detail::join_strings(e.readers, ", "));
}

auto formatter<talos::scheduler::OrphanedReaderError>::format(
    const talos::scheduler::OrphanedReaderError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "OrphanedReader: reader(s) without writer: {}",
        talos::scheduler::detail::join_strings(e.readers, ", "));
}

auto formatter<talos::scheduler::DependencyCycleError>::format(
    const talos::scheduler::DependencyCycleError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "DependencyCycle: {}", talos::scheduler::detail::join_strings(e.cycle, " -> "));
}

auto formatter<talos::scheduler::TooManyComputeSystemsError>::format(
    const talos::scheduler::TooManyComputeSystemsError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(ctx.out(), "TooManyComputeSystems: {} (max {})", e.count, e.max_count);
}

auto formatter<talos::scheduler::UnreachableComputeSystemsError>::format(
    const talos::scheduler::UnreachableComputeSystemsError& e, format_context& ctx) const
    -> format_context::iterator {
    return fmt::format_to(
        ctx.out(), "UnreachableComputeSystems: {}",
        talos::scheduler::detail::join_strings(e.systems, ", "));
}

auto formatter<talos::scheduler::BuildError>::format(
    const talos::scheduler::BuildError& err, format_context& ctx) const
    -> format_context::iterator {
    return std::visit(
        [&ctx](const auto& e) -> format_context::iterator {
            return fmt::format_to(ctx.out(), "{}", e);
        },
        err);
}

} // namespace fmt
