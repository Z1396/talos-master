#pragma once

/**
 * @file fmt::formatter specializations for scheduler error types
 *
 * Enables all scheduler error types to be formatted with fmt library.
 * This allows errors to be used directly with fmt::format() and logging sinks.
 *
 * Example:
 *   fmt::print("Error: {}\n", error);
 *   logger.error("Build failed: {}", error);
 */

#include "error.hpp"

#include <fmt/format.h>
#include <string_view>

// ============================================================================
// fmt::formatter specializations - must be in fmt namespace
// ============================================================================

namespace fmt {

// Formatter for simple enum types
template <>
struct formatter<talos::scheduler::SchedulerError> : formatter<std::string_view> {
    auto format(talos::scheduler::SchedulerError e, format_context& ctx) const
        -> format_context::iterator;
};

// Formatter base class (eliminates duplicate parse methods)
template <typename T>
struct error_formatter_base {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
};

// Formatter for errors with two system names
template <>
struct formatter<talos::scheduler::ChannelKindConflict>
    : error_formatter_base<talos::scheduler::ChannelKindConflict> {
    auto format(const talos::scheduler::ChannelKindConflict& e, format_context& ctx) const
        -> format_context::iterator;
};

// Formatter for errors with string lists
template <>
struct formatter<talos::scheduler::MultipleWritersError>
    : error_formatter_base<talos::scheduler::MultipleWritersError> {
    auto format(const talos::scheduler::MultipleWritersError& e, format_context& ctx) const
        -> format_context::iterator;
};

template <>
struct formatter<talos::scheduler::MultipleReadersError>
    : error_formatter_base<talos::scheduler::MultipleReadersError> {
    auto format(const talos::scheduler::MultipleReadersError& e, format_context& ctx) const
        -> format_context::iterator;
};

template <>
struct formatter<talos::scheduler::OrphanedReaderError>
    : error_formatter_base<talos::scheduler::OrphanedReaderError> {
    auto format(const talos::scheduler::OrphanedReaderError& e, format_context& ctx) const
        -> format_context::iterator;
};

template <>
struct formatter<talos::scheduler::DependencyCycleError>
    : error_formatter_base<talos::scheduler::DependencyCycleError> {
    auto format(const talos::scheduler::DependencyCycleError& e, format_context& ctx) const
        -> format_context::iterator;
};

template <>
struct formatter<talos::scheduler::TooManyComputeSystemsError>
    : error_formatter_base<talos::scheduler::TooManyComputeSystemsError> {
    auto format(const talos::scheduler::TooManyComputeSystemsError& e, format_context& ctx) const
        -> format_context::iterator;
};

template <>
struct formatter<talos::scheduler::UnreachableComputeSystemsError>
    : error_formatter_base<talos::scheduler::UnreachableComputeSystemsError> {
    auto
        format(const talos::scheduler::UnreachableComputeSystemsError& e, format_context& ctx) const
        -> format_context::iterator;
};

template <>
struct formatter<talos::scheduler::BuildError>
    : error_formatter_base<talos::scheduler::BuildError> {
    auto format(const talos::scheduler::BuildError& err, format_context& ctx) const
        -> format_context::iterator;
};

} // namespace fmt
