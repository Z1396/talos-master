#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <new>
#include <optional>

/**
 * @file
 * @brief Lock-free single-producer single-consumer triple buffer
 *
 * This module provides a lock-free SPSC (Single Producer Single Consumer)
 * triple buffer optimized for minimal latency in a two-thread pipeline.
 *
 * ## Design
 *
 * - **Three-slot design**: write slot, read slot, and intermediate slot
 * - **State machine**: uses bit flags to coordinate handoff
 * - **Move semantics**: data is moved in and out for zero-copy transfer
 *
 * ## Memory ordering
 *
 * Uses `memory_order_acq_rel` for publish operations to ensure
 * visibility of the written data before the consumer sees the flag.
 *
 * ## Thread safety
 *
 * - **Single writer, single reader only**
 * - No locking required
 * - Data race free when used as designed
 */
namespace talos::primitive {

/**
 * @brief Lock-free single-producer single-consumer triple buffer
 *
 * A two-thread communication channel with minimal latency.
 * Uses three slots and a state machine to coordinate handoff.
 *
 * ## Usage
 *
 * ```cpp
 * auto [writer, reader] = SpscTripleBuffer<Frame>::create();
 *
 * // Producer:
 * writer.write(Frame{...});
 *
 * // Consumer:
 * if (auto frame = reader.read()) {
 *     // process frame
 * }
 * ```
 *
 * ## Performance
 *
 * - **Write**: O(1) atomic operations (~20-30ns)
 * - **Read**: O(1) atomic operations (~20-30ns)
 * - **Zero-copy**: data is moved, not copied
 */
template <typename T>
requires(std::movable<T>) class SpscTripleBuffer {
    static constexpr uint8_t FLAG_NEW   = 0x80;
    static constexpr uint8_t INDEX_MASK = 0x03;

    struct State {
        alignas(std::hardware_destructive_interference_size) std::array<T, 3> slots{};
        alignas(std::hardware_destructive_interference_size) uint8_t write_idx{0};
        alignas(std::hardware_destructive_interference_size) std::atomic<uint8_t> shared{1};
        alignas(std::hardware_destructive_interference_size) uint8_t read_idx{2};
    };

public:
    template <typename U>
    class Write;
    template <typename U>
    class Read;

    /**
     * @brief Write handle for the producer
     *
     * Move-only. Owned by the single producer thread.
     */
    template <typename U>
    class Write {
    public:
        Write(const Write&)                = delete;
        Write& operator=(const Write&)     = delete;
        Write(Write&&) noexcept            = default;
        Write& operator=(Write&&) noexcept = default;

        /**
         * @brief Write data by moving it into the buffer
         *
         * The data is moved into the write slot, then published
         * to make it visible to the consumer.
         *
         * ## Thread safety
         *
         * Must only be called from the single producer thread.
         */
        void write(U data) noexcept {
            state_->slots[state_->write_idx] = std::move(data);
            publish();
        }

        /**
         * @brief Borrow mutable reference to write slot
         *
         * Allows direct construction of data in the buffer,
         * avoiding a move operation.
         *
         * ## Thread safety
         *
         * Must only be called from the producer thread.
         *
         * ## Returns
         *
         * Reference to the write slot. The caller is responsible
         * for ensuring the data is properly initialized before
         * calling `publish()`.
         */
        [[nodiscard]] U& borrow_mut() noexcept { return state_->slots[state_->write_idx]; }

        /**
         * @brief Publish the current write slot
         *
         * Atomically signals to the consumer that new data is available.
         *
         * ## Thread safety
         *
         * Must only be called from the producer thread.
         */
        void publish() noexcept {
            const auto old =
                state_->shared.exchange(state_->write_idx | FLAG_NEW, std::memory_order_acq_rel);
            state_->write_idx = old & INDEX_MASK;
        }

        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        friend class SpscTripleBuffer;
        explicit Write(std::shared_ptr<State> s)
            : state_(std::move(s)) {}
        std::shared_ptr<State> state_;
    };

    /**
     * @brief Read handle for the consumer
     *
     * Move-only. Owned by the single consumer thread.
     */
    template <typename U>
    class Read {
    public:
        Read(const Read&)                = delete;
        Read& operator=(const Read&)     = delete;
        Read(Read&&) noexcept            = default;
        Read& operator=(Read&&) noexcept = default;

        /**
         * @brief Try to read new data
         *
         * Attempts to read data from the buffer. If new data is
         * available (since the last read), it is moved out and
         * returned.
         *
         * ## Thread safety
         *
         * Must only be called from the single consumer thread.
         *
         * ## Returns
         *
         * - `std::optional<U>` containing the data if new data is available
         * - `std::nullopt` if no new data since last read
         */
        [[nodiscard]] std::optional<U> read() noexcept {
            auto expected = state_->shared.load(std::memory_order_acquire);
            if (!(expected & FLAG_NEW)) {
                return std::nullopt;
            }

            auto ready_idx = expected & INDEX_MASK;
            if (!state_->shared.compare_exchange_strong(
                    expected, state_->read_idx, std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                if (!(expected & FLAG_NEW)) {
                    return std::nullopt;
                }
                ready_idx = expected & INDEX_MASK;
                if (!state_->shared.compare_exchange_strong(
                        expected, state_->read_idx, std::memory_order_acq_rel,
                        std::memory_order_relaxed)) {
                    return std::nullopt;
                }
            }

            state_->read_idx = ready_idx;
            return std::move(state_->slots[state_->read_idx]);
        }

        // has_new() removed - TOCTOU hazard makes it fundamentally unsafe.
        // Use read() and check the return value instead.

        // current() removed - UAB hazard (reference may point to slot being written).
        // Use read() for ownership transfer instead.

        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        friend class SpscTripleBuffer;
        explicit Read(std::shared_ptr<State> s)
            : state_(std::move(s)) {}
        std::shared_ptr<State> state_;
    };

    SpscTripleBuffer()                                   = delete;
    SpscTripleBuffer(const SpscTripleBuffer&)            = delete;
    SpscTripleBuffer& operator=(const SpscTripleBuffer&) = delete;

    /**
     * @brief Create buffer, returns {Write<T>, Read<T>}
     */
    [[nodiscard]] static std::pair<Write<T>, Read<T>> create() {
        auto state = std::make_shared<State>();
        return {Write<T>(state), Read<T>(state)};
    }
};

} // namespace talos::primitive
