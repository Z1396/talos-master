#pragma once

#include <cstddef>
#include <expected>
#include <fmt/format.h>
#include <string>
#include <typeindex>
#include <variant>
#include <vector>

namespace talos::scheduler {

/**
 * @brief Scheduler lifecycle errors
 */
enum class SchedulerError {
    AlreadyRunning, ///< Scheduler is already running
    NotBuilt,       ///< Scheduler was not built before run
};

/**
 * @brief Channel identifier for error reporting
 *
 * Combines data type and tag to uniquely identify a channel.
 */
struct ChannelKeyInfo {
    std::type_index type;
    std::type_index tag;
    bool is_spsc;

    /**
     * @brief Construct a ChannelKeyInfo
     *
     * ## Parameters
     *
     * - `t`: data type index
     * - `tg`: tag type index
     * - `spsc`: true if SPSC channel, false if SPMC
     */
    ChannelKeyInfo(const std::type_index t, const std::type_index tg, const bool spsc)
        : type(t)
        , tag(tg)
        , is_spsc(spsc) {}
};

/**
 * @brief Same channel key used for both SPSC and SPMC
 */
struct ChannelKindConflict {
    ChannelKeyInfo key;
    std::string first_system;
    std::string second_system;
};

/**
 * @brief SPSC or SPMC channel has multiple writers
 */
struct MultipleWritersError {
    ChannelKeyInfo key;
    std::vector<std::string> writers;
};

/**
 * @brief SPSC channel has multiple readers
 */
struct MultipleReadersError {
    ChannelKeyInfo key;
    std::vector<std::string> readers;
};

/**
 * @brief Reader without a corresponding writer
 */
struct OrphanedReaderError {
    ChannelKeyInfo key;
    std::vector<std::string> readers;
};

/**
 * @brief Dependency cycle detected in the graph
 */
struct DependencyCycleError {
    std::vector<std::string> cycle;
};

/**
 * @brief Too many compute systems (max 64 for efficient bitmask scheduling)
 */
struct TooManyComputeSystemsError {
    std::size_t count;
    static constexpr std::size_t max_count = 64;
};

/**
 * @brief Compute systems that can never be triggered by any external source
 */
struct UnreachableComputeSystemsError {
    std::vector<std::string> systems;
};

/**
 * @brief All possible build errors
 */
using BuildError = std::variant<
    SchedulerError, // Runtime/lifecycle errors
    ChannelKindConflict, MultipleWritersError, MultipleReadersError, OrphanedReaderError,
    DependencyCycleError, TooManyComputeSystemsError, UnreachableComputeSystemsError>;

/**
 * @brief Build result type
 */
using BuildResult = std::expected<void, BuildError>;

// ============================================================================
// Panic Utility
// ============================================================================

namespace detail {
[[noreturn]] void panic_message(std::string message) noexcept;
} // namespace detail

/**
 * @brief Log a critical error and abort (panic)
 *
 * Centralizes fatal error handling through the compiled logging backend.
 * Using this utility ensures consistent error reporting and makes it easier
 * to modify fatal error behavior globally.
 *
 * ## Template parameters
 *
 * - `Args`: argument types (deduced)
 *
 * ## Parameters
 *
 * - `fmt`: format string for the error message
 * - `args`: arguments to format into the message
 *
 * ## Behavior
 *
 * - Logs the formatted message at CRITICAL level
 * - Terminates the program after logging
 * - Marked noexcept and [[noreturn]] for optimizer awareness
 *
 */
template <typename... Args>
[[noreturn]] inline void panic(const char* fmt, const Args&... args) noexcept {
    detail::panic_message(fmt::format(fmt::runtime(fmt), args...));
}

} // namespace talos::scheduler
