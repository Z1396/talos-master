#pragma once

/**
 * @file
 * @brief System metadata and type utilities
 *
 * This module provides:
 * - Type traits for extracting channel and resource types from system parameters
 * - Metadata structures for describing systems at runtime
 * - Channel key identification for dependency tracking
 */

#include "components.hpp"
#include "execution_policy.hpp"

#include <cstddef>
#include <functional>
#include <string>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <utility>
#include <vector>

namespace talos::scheduler::system {

// ============================================================================
// Channel Key: (Type, Topic) for unique identification
// ============================================================================

/**
 * @brief Uniquely identifies a channel by (data type, topic) pair
 *
 * Used for tracking channel usage across systems and detecting conflicts.
 */
struct ChannelKey {
    std::type_index type;
    std::type_index topic;

    bool operator==(const ChannelKey&) const noexcept = default;

    // Lexicographic comparison for std::map
    constexpr bool operator<(const ChannelKey& other) const noexcept {
        if (type != other.type) {
            return type < other.type;
        }
        return topic < other.topic;
    }
};

/**
 * @brief Hash function for ChannelKey
 *
 * Uses boost-style hash combine to reduce collisions compared to naive XOR.
 * The magic constant 0x9e3779b9 is derived from 2^32 / φ (golden ratio).
 */
struct ChannelKeyHash {
    constexpr std::size_t operator()(const ChannelKey& k) const noexcept {
        std::size_t seed = std::hash<std::type_index>{}(k.type);
        // Boost-style hash combine
        seed ^= std::hash<std::type_index>{}(k.topic) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

// ============================================================================
// Function Traits
// ============================================================================

namespace detail {

template <typename F>
struct function_traits;

/**
 * @brief Extract function traits from a function pointer
 */
template <typename R, typename... Args>
struct function_traits<R (*)(Args...)> {
    using return_type                  = R;
    using args_tuple                   = std::tuple<Args...>;
    static constexpr std::size_t arity = sizeof...(Args);
};

/**
 * @brief Extract function traits from a member function
 */
template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...)> : function_traits<R (*)(Args...)> {};

template <typename R, typename C, typename... Args>
struct function_traits<R (C::*)(Args...) const> : function_traits<R (*)(Args...)> {};

/**
 * @brief Extract function traits from a lambda/functor
 */
template <typename F>
struct function_traits : function_traits<decltype(&F::operator())> {};

template <component_kind T>
using inner_type_t = T::value_type;

template <component_kind T>
using inner_topic_t = T::topic_type;

// ============================================================================
// Type Predicates
// ============================================================================

/// Helper: unified type kind checker (eliminates 7 duplicate struct definitions)
template <channel_kind Kind>
struct is_channel_kind {
    template <typename T>
    struct trait : std::bool_constant<T::kind == Kind> {};
};

/// Type predicate aliases (1-liners instead of 7 separate structs)
template <typename T>
struct is_spsc_reader : is_channel_kind<channel_kind::spsc_reader>::trait<T> {};

template <typename T>
struct is_spsc_writer : is_channel_kind<channel_kind::spsc_writer>::trait<T> {};

template <typename T>
struct is_spmc_reader : is_channel_kind<channel_kind::spmc_reader>::trait<T> {};

template <typename T>
struct is_spmc_writer : is_channel_kind<channel_kind::spmc_writer>::trait<T> {};

template <typename T>
struct is_res_type : is_channel_kind<channel_kind::res>::trait<T> {};

template <typename T>
struct is_res_mut_type : is_channel_kind<channel_kind::res_mut>::trait<T> {};

template <typename T>
struct is_local_type : is_channel_kind<channel_kind::local>::trait<T> {};

/// Unified writer checker (eliminates: is_spsc_writer || is_spmc_writer pattern)
template <typename T>
struct is_writer : std::bool_constant<is_spsc_writer<T>::value || is_spmc_writer<T>::value> {};

} // namespace detail

// ============================================================================
// System Metadata
// ============================================================================

/// Metadata for a single channel used by a system
struct ChannelMeta {
    std::type_index type;
    std::type_index topic;
    channel_kind kind;
};

/**
 * @brief Runtime metadata for a system
 *
 * Contains information about the system's policy, channels, and resources.
 */
struct SystemMeta {
    std::string name;
    PolicyInfo policy;
    std::vector<ChannelMeta> spsc_channels;
    std::vector<ChannelMeta> spmc_channels;
    std::vector<std::type_index> atomics;
    std::vector<std::type_index> reads;
    std::vector<std::type_index> writes;
};

namespace detail {

/// Extract channel/resource metadata from a single parameter
template <typename T>
constexpr void extract_one_param(SystemMeta& meta) noexcept {
    using Inner = inner_type_t<T>;
    using Topic = inner_topic_t<T>;

    if constexpr (is_spsc_reader<T>::value || is_spsc_writer<T>::value) {
        meta.spsc_channels.emplace_back(ChannelMeta{typeid(Inner), typeid(Topic), T::kind});
    } else if constexpr (is_spmc_reader<T>::value || is_spmc_writer<T>::value) {
        meta.spmc_channels.emplace_back(ChannelMeta{typeid(Inner), typeid(Topic), T::kind});
    } else if constexpr (is_res_type<T>::value) {
        meta.reads.emplace_back(typeid(Inner));
    } else if constexpr (is_res_mut_type<T>::value) {
        meta.writes.emplace_back(typeid(Inner));
    } else if constexpr (is_local_type<T>::value) {
        // Local variables don't participate in dependency analysis
    }
}

} // namespace detail

/**
 * @brief Extract system metadata from a function type
 *
 * Analyzes the function's parameters to determine:
 * - What channels and resources the system uses
 * - Which are read vs write
 * - The execution policy
 *
 * ## Template parameters
 *
 * - `F`: function type (or lambda/functor)
 * - `Policy`: execution policy (defaults to default_policy)
 *
 * ## Parameters
 *
 * - `name`: system name for identification
 *
 * ## Returns
 *
 * SystemMeta describing the system
 */
template <typename F, typename Policy = default_policy>
constexpr SystemMeta extract_system_meta(std::string name) noexcept {
    SystemMeta meta;
    meta.name   = std::move(name);
    meta.policy = make_policy_info<Policy>();

    using traits = detail::function_traits<F>;
    using args   = traits::args_tuple;

    [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (detail::extract_one_param<std::tuple_element_t<Is, args>>(meta), ...);
    }(std::make_index_sequence<traits::arity>{});

    return meta;
}

} // namespace talos::scheduler::system
