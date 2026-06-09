#pragma once

#include "primitive/channel.hpp"
#include <type_traits>
#include <utility>

namespace talos::scheduler::system {

// ============================================================================
// Concepts
// ============================================================================

template <typename T>
concept topic_type = std::is_same_v<T, void> || requires { sizeof(T) == 0; };

template <typename T>
concept component_kind = requires {
    typename T::value_type;
    typename T::topic_type;
    requires !std::is_void_v<typename T::value_type>;
    requires topic_type<typename T::topic_type>;
};

struct DefaultTopic {};

// ============================================================================
// Channel Kind Enum
// ============================================================================

enum class channel_kind {
    spsc_reader, ///< SPSC reader
    spsc_writer, ///< SPSC writer
    spmc_reader, ///< SPMC reader
    spmc_writer, ///< SPMC writer
    res,         ///< Read-only resource
    res_mut,     ///< Mutable resource
    local,       ///< System-local variable
};

// ============================================================================
// Unified Channel Templates
// ============================================================================

/**
 * @brief Unified reader template
 *
 * @tparam T Value type
 * @tparam Topic Topic tag type
 * @tparam UnderlyingChannel Underlying channel reader type
 * @tparam Kind Channel kind enum value
 * @tparam HasNew Whether the reader has has_new() method (SPMC only)
 */
template <
    typename T, typename Topic = DefaultTopic, typename UnderlyingChannel = void,
    channel_kind Kind = static_cast<channel_kind>(0), bool HasNew = false>
struct basic_channel {
    using value_type           = T;
    using topic_type           = Topic;
    static constexpr auto kind = Kind;

    UnderlyingChannel* ptr_ = nullptr; // Framework guarantees validity

    [[nodiscard]] auto read() noexcept { return ptr_->read(); }

    [[nodiscard]] auto read_current() noexcept { return ptr_->read_current(); }

    [[nodiscard]] bool has_new() const noexcept requires HasNew { return ptr_->has_new(); }

    [[nodiscard]] auto last_generation() const noexcept
        requires requires(const UnderlyingChannel& channel) { channel.last_generation(); } {
        return ptr_->last_generation();
    }
};

/**
 * @brief Unified writer template
 *
 * @tparam T Value type
 * @tparam Topic Topic tag type
 * @tparam UnderlyingChannel Underlying channel writer type
 * @tparam Kind Channel kind enum value
 */
template <
    typename T, typename Topic = DefaultTopic, typename UnderlyingChannel = void,
    channel_kind Kind = static_cast<channel_kind>(0)>
struct basic_writer {
    using value_type           = T;
    using topic_type           = Topic;
    static constexpr auto kind = Kind;

    UnderlyingChannel* ptr_ = nullptr;
    bool* written_flag_     = nullptr; // Set by scheduler for selective wake-up

    void write(T value) noexcept {
        ptr_->write(std::move(value));
        if (written_flag_) [[likely]]
            *written_flag_ = true;
    }
};

/**
 * @brief Unified resource template
 *
 * @tparam T Value type
 * @tparam IsMutable True for mutable resource, false for read-only
 * @tparam Kind Channel kind enum value
 */
template <typename T, bool IsMutable, channel_kind Kind>
struct basic_resource {
    using value_type           = T;
    using topic_type           = void; // Resources don't use topics
    static constexpr auto kind = Kind;

    using PtrType = std::conditional_t<IsMutable, T*, const T*>;

    PtrType ptr_ = nullptr;

    // Read-only access (for both const and mutable resources)
    [[nodiscard]] const T& operator*() const noexcept requires(!IsMutable) { return *ptr_; }

    [[nodiscard]] const T* operator->() const noexcept requires(!IsMutable) { return ptr_; }

    // Mutable access (only for res_mut)
    [[nodiscard]] T& operator*() noexcept requires IsMutable { return *ptr_; }

    [[nodiscard]] T* operator->() noexcept requires IsMutable { return ptr_; }

    // Const overloads for mutable resource
    [[nodiscard]] const T& operator*() const noexcept { return *ptr_; }
    [[nodiscard]] const T* operator->() const noexcept { return ptr_; }
};

// ============================================================================
// System-Local Variable
// ============================================================================

/**
 * @brief System-local variable (mutable, per-system instance)
 *
 * Unlike channels and resources, local<T> is NOT shared across systems
 * and does NOT participate in dependency analysis.
 */
template <typename T>
struct local {
    using value_type           = T;
    using topic_type           = void;
    static constexpr auto kind = channel_kind::local;

    T* ptr_ = nullptr; // Framework guarantees validity

    [[nodiscard]] T& operator*() noexcept { return *ptr_; }
    [[nodiscard]] T* operator->() noexcept { return ptr_; }
    [[nodiscard]] const T& operator*() const noexcept { return *ptr_; }
    [[nodiscard]] const T* operator->() const noexcept { return ptr_; }
};

// ============================================================================
// Type Aliases (Public API)
// ============================================================================

// SPSC Channel (Single Producer, Single Consumer)
template <typename T, typename Topic = DefaultTopic>
using spsc = basic_channel<
    T, Topic, typename primitive::SpscChannel<T>::Reader, channel_kind::spsc_reader,
    false // No has_new()
    >;

template <typename T, typename Topic = DefaultTopic>
using spsc_mut =
    basic_writer<T, Topic, typename primitive::SpscChannel<T>::Writer, channel_kind::spsc_writer>;

// SPMC Channel (Single Producer, Multiple Consumers)
template <typename T, typename Topic = DefaultTopic>
using spmc = basic_channel<
    T, Topic, typename primitive::SpmcChannel<T>::Reader, channel_kind::spmc_reader,
    true // Has has_new()
    >;

template <typename T, typename Topic = DefaultTopic>
using spmc_mut =
    basic_writer<T, Topic, typename primitive::SpmcChannel<T>, channel_kind::spmc_writer>;

// Resources
template <typename T>
using res = basic_resource<T, false, channel_kind::res>;

template <typename T>
using res_mut = basic_resource<T, true, channel_kind::res_mut>;

// Convenience aliases
template <typename T, typename Topic = DefaultTopic>
using subscribe = spmc<T, Topic>;

template <typename T, typename Topic = DefaultTopic>
using publish = spmc_mut<T, Topic>;

// ============================================================================
// Static Assertions
// ============================================================================

static_assert(component_kind<spsc<bool>>);
static_assert(component_kind<spsc_mut<bool>>);
static_assert(component_kind<spmc<bool>>);
static_assert(component_kind<spmc_mut<bool>>);
static_assert(component_kind<res<bool>>);
static_assert(component_kind<res_mut<bool>>);
static_assert(component_kind<local<bool>>);

} // namespace talos::scheduler::system
