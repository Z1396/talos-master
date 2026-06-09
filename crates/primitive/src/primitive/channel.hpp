#pragma once

/**
 * @file
 * @brief Channel abstractions for SPSC and SPMC triple buffers
 *
 * This module provides unified channel types that wrap triple buffer
 * implementations with optional flow graph tracking.
 *
 * ## Channel types
 *
 * - `SpscChannel<T>`: Single-producer single-consumer channel
 * - `SpmcChannel<T>`: Single-producer multiple-consumer channel
 * - `Channel<T>`: Alias for `SpmcChannel<T>` (backward compatibility)
 */

#include "spmc_triple_buffer.hpp"
#include "spsc_triple_buffer.hpp"

#include <concepts>
#include <optional>
#include <utility>

namespace talos::primitive {
// ============================================================================
// Concepts: detect buffer type capabilities
// ============================================================================

namespace detail {

/**
 * @brief Checks if a buffer's Read type supports cloning
 *
 * SPMC buffers can clone readers, SPSC buffers cannot.
 */
template <typename Buffer, typename T>
concept HasClone = requires(typename Buffer::template Read<T>& r) {
    { r.clone() } -> std::same_as<typename Buffer::template Read<T>>;
};

/**
 * @brief SPMC-like buffer supports reader cloning
 */
template <typename Buffer, typename T>
concept SpmcLike = HasClone<Buffer, T>;

/**
 * @brief SPSC-like buffer does not support reader cloning
 */
template <typename Buffer, typename T>
concept SpscLike = !HasClone<Buffer, T>;

} // namespace detail

// ============================================================================
// TrackedWriter: wraps any buffer's Write<T>
// ============================================================================

/**
 * @brief Tracked write handle for a channel
 *
 * Wraps a triple buffer's Write handle with location tracking
 * for flow graph construction.
 *
 * ## Thread safety
 *
 * Inherited from the wrapped buffer type.
 *
 * ## Template parameters
 *
 * - `T`: data type passed through the channel
 * - `Buffer`: underlying buffer implementation (SpscTripleBuffer or SpmcTripleBuffer)
 */
template <typename T, typename Buffer>
class TrackedWriter {
    using Inner = Buffer::template Write<T>;
    Inner inner_;

public:
    using value_type  = T;
    using buffer_type = Buffer;

    TrackedWriter(Inner w) noexcept
        : inner_(std::move(w)) {}

    TrackedWriter(const TrackedWriter&)            = delete;
    TrackedWriter& operator=(const TrackedWriter&) = delete;
    TrackedWriter(TrackedWriter&&) noexcept        = default;
    TrackedWriter& operator=(TrackedWriter&&)      = default;

    void write(T data) noexcept { inner_.write(std::move(data)); }

    // SPSC only: borrow_mut + publish
    auto& borrow_mut() noexcept requires requires(Inner& w) { w.borrow_mut(); } {
        return inner_.borrow_mut();
    }

    void publish() noexcept requires requires(Inner& w) { w.publish(); } { inner_.publish(); }

    // SPMC only: generation
    [[nodiscard]] auto generation() const noexcept
        requires requires(const Inner& w) { w.generation(); } {
        return inner_.generation();
    }

    [[nodiscard]] bool valid() const noexcept { return inner_.valid(); }
    explicit operator bool() const noexcept { return valid(); }

    // Access inner (for advanced use)
    [[nodiscard]] Inner& inner() noexcept { return inner_; }
    [[nodiscard]] const Inner& inner() const noexcept { return inner_; }
};

// ============================================================================
// TrackedReader: wraps any buffer's Read<T>
// ============================================================================

/**
 * @brief Tracked read handle for a channel
 *
 * Wraps a triple buffer's Read handle with location tracking
 * for flow graph construction.
 *
 * ## Copy semantics
 *
 * - SPMC: Copyable (each reader maintains its own generation counter)
 * - SPSC: Move-only (only one consumer exists)
 *
 * ## Thread safety
 *
 * Inherited from the wrapped buffer type.
 *
 * ## Template parameters
 *
 * - `T`: data type passed through the channel
 * - `Buffer`: underlying buffer implementation (SpscTripleBuffer or SpmcTripleBuffer)
 */
template <typename T, typename Buffer>
class TrackedReader {
    using Inner = Buffer::template Read<T>;
    Inner inner_;

public:
    using value_type  = T;
    using buffer_type = Buffer;

    TrackedReader(Inner r) noexcept
        : inner_(std::move(r)) {}

    // SPMC: copyable
    TrackedReader(const TrackedReader& other) requires detail::SpmcLike<Buffer, T>
        : inner_(other.inner_) {}

    TrackedReader& operator=(const TrackedReader& other) requires detail::SpmcLike<Buffer, T> {
        if (this != &other) {
            inner_ = other.inner_;
        }
        return *this;
    }

    // SPSC: move-only
    TrackedReader(const TrackedReader&) requires detail::SpscLike<Buffer, T>            = delete;
    TrackedReader& operator=(const TrackedReader&) requires detail::SpscLike<Buffer, T> = delete;

    TrackedReader(TrackedReader&&) noexcept            = default;
    TrackedReader& operator=(TrackedReader&&) noexcept = default;

    [[nodiscard]] std::optional<T> read() noexcept { return inner_.read(); }

    [[nodiscard]] bool has_new() const noexcept { return inner_.has_new(); }

    // SPMC only: read_current, last_generation
    [[nodiscard]] auto read_current() noexcept requires requires(Inner& r) { r.read_current(); } {
        return inner_.read_current();
    }

    [[nodiscard]] auto last_generation() const noexcept
        requires requires(const Inner& r) { r.last_generation(); } {
        return inner_.last_generation();
    }

    [[nodiscard]] bool valid() const noexcept { return inner_.valid(); }
    explicit operator bool() const noexcept { return valid(); }

    // Access inner
    [[nodiscard]] Inner& inner() noexcept { return inner_; }
    [[nodiscard]] const Inner& inner() const noexcept { return inner_; }
};

// ============================================================================
// TrackedChannel: unified wrapper for any triple buffer
// ============================================================================

/**
 * @brief Unified channel wrapper for triple buffers
 *
 * Provides a unified interface for both SPSC and SPMC channels
 * with automatic flow graph tracking.
 *
 * ## Usage
 *
 * ```cpp
 * auto channel = primitive::make_spmc_channel<Frame>();
 * auto [writer, reader] = channel.split();
 *
 * writer.write(frame);
 * if (auto frame = reader.read()) {
 *     // process frame
 * }
 * ```
 *
 * ## Split vs clone
 *
 * - `split()`: Separates the channel into writer and reader handles
 * - `clone_reader()`: Creates a new reader for SPMC channels (SPMC only)
 * - `take_reader()`: Takes ownership of the reader (consumes reader)
 *
 * ## Thread safety
 *
 * Inherited from the wrapped buffer type.
 *
 * ## Template parameters
 *
 * - `T`: data type passed through the channel
 * - `Buffer`: underlying buffer implementation (SpscTripleBuffer or SpmcTripleBuffer)
 */
template <typename T, typename Buffer>
class TrackedChannel {
    using WriteInner = Buffer::template Write<T>;
    using ReadInner  = Buffer::template Read<T>;

    WriteInner writer_;
    ReadInner reader_;

public:
    using value_type  = T;
    using buffer_type = Buffer;
    using Writer      = TrackedWriter<T, Buffer>;
    using Reader      = TrackedReader<T, Buffer>;

    static constexpr bool is_spmc = detail::SpmcLike<Buffer, T>;
    static constexpr bool is_spsc = detail::SpscLike<Buffer, T>;

    TrackedChannel(WriteInner w, ReadInner r) noexcept
        : writer_(std::move(w))
        , reader_(std::move(r)) {}

    TrackedChannel(const TrackedChannel&)            = delete;
    TrackedChannel& operator=(const TrackedChannel&) = delete;
    TrackedChannel(TrackedChannel&&) noexcept        = default;
    TrackedChannel& operator=(TrackedChannel&&)      = default;

    // ========================================================================
    // Writer interface (for convenience, can also use split())
    // ========================================================================

    void write(T data) noexcept { writer_.write(std::move(data)); }

    auto& borrow_mut() noexcept requires is_spsc { return writer_.borrow_mut(); }

    void publish() noexcept requires is_spsc { writer_.publish(); }

    [[nodiscard]] auto generation() const noexcept requires is_spmc { return writer_.generation(); }

    // ========================================================================
    // Clone reader (SPMC only): records edge
    // ========================================================================

    [[nodiscard]] Reader clone_reader() const requires is_spmc { return Reader(reader_.clone()); }

    // ========================================================================
    // Take reader (both): consumes the internal reader, records edge
    // ========================================================================

    [[nodiscard]] Reader take_reader() { return Reader(std::move(reader_)); }

    // ========================================================================
    // Split: returns (Writer, Reader), records edge
    // ========================================================================

    struct SplitResult {
        Writer writer;
        Reader reader;
    };

    [[nodiscard]] SplitResult split() {
        return SplitResult{Writer(std::move(writer_)), Reader(std::move(reader_))};
    }
};

// ============================================================================
// Factory function: tracked_create<Buffer, T>()
// Usage: tracked_create<SpmcTripleBuffer, Frame>()
// ============================================================================

template <template <typename> class Buffer, typename T>
[[nodiscard]] auto tracked_create() -> TrackedChannel<T, Buffer<T>> {
    auto [w, r] = Buffer<T>::create();
    return TrackedChannel<T, Buffer<T>>(std::move(w), std::move(r));
}

// ============================================================================
// Type aliases for convenience
// ============================================================================

template <typename T>
using SpmcChannel = TrackedChannel<T, SpmcTripleBuffer<T>>;

template <typename T>
using SpscChannel = TrackedChannel<T, SpscTripleBuffer<T>>;

// Convenience factory functions
template <typename T>
[[nodiscard]] auto make_spmc_channel() -> SpmcChannel<T> {
    return tracked_create<SpmcTripleBuffer, T>();
}

template <typename T>
[[nodiscard]] auto make_spsc_channel() -> SpscChannel<T> {
    return tracked_create<SpscTripleBuffer, T>();
}

// ============================================================================
// Legacy alias: Channel<T> = SpmcChannel<T> for backward compatibility
// ============================================================================

template <typename T>
using Channel = SpmcChannel<T>;

} // namespace talos::primitive
