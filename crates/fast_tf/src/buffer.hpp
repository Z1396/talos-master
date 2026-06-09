#pragma once

#include "euler.hpp"
#include "groups/SO3.hpp"
#include "matrix.hpp"
#include "types.hpp"
#include "validation.hpp"
#include <array>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <expected>
#include <fmt/format.h>
#include <optional>
#include <tbb/spin_rw_mutex.h>
#include <type_traits>
#include <utility>

namespace fast_tf {

using sec            = std::size_t;
using sample_per_sec = std::size_t;

struct exact_t {};
struct nearest_t {};
struct interpolate_t {};
struct clamped_t {};

inline constexpr exact_t exact{};
inline constexpr nearest_t nearest{};
inline constexpr interpolate_t interpolate{};
inline constexpr clamped_t clamped{};

template <typename Mode>
concept buffer_lookup_mode = std::same_as<Mode, exact_t> || std::same_as<Mode, nearest_t>
                          || std::same_as<Mode, interpolate_t> || std::same_as<Mode, clamped_t>;

template <typename Ops, typename T>
concept buffer_value_ops = requires(const T& value, fp_t ratio) {
    { Ops::identity() } -> std::same_as<T>;
    { Ops::interpolate(value, value, ratio) } -> std::same_as<T>;
    { Ops::is_valid(value) } -> std::same_as<bool>;
};

template <typename T>
struct BufferOps;

template <std::floating_point T, typename From, typename To>
struct BufferOps<TransformMatrix<T, From, To>> {
    using Transform = TransformMatrix<T, From, To>;

    static Transform identity() { return Transform{}; }

    static Transform interpolate(const Transform& a, const Transform& b, fp_t ratio) {
        return Transform::lerp(a, b, static_cast<T>(ratio));
    }

    static bool is_valid(const Transform& value) { return validate_transform(value).has_value(); }

    static Transform with_rotation(const Transform& base, const math_fuxk::Ros2EulerRotd& rot) {
        const auto translation        = base.translation();
        const auto [roll, pitch, yaw] = rot.rpy();
        return Transform::from_rpy(
            static_cast<T>(roll), static_cast<T>(pitch), static_cast<T>(yaw), translation.x(),
            translation.y(), translation.z());
    }
};

template <typename T>
struct BufferOps<group::SO3<T>> {
    static group::SO3<T> identity() { return group::SO3<T>{}; }

    static group::SO3<T> interpolate(const group::SO3<T>& a, const group::SO3<T>& b, fp_t ratio) {
        return a * group::SO3<T>::exp(group::SO3<T>::log(a.inv() * b) * ratio);
    }

    static bool is_valid(const group::SO3<T>& value) {
        (void)value;
        return true;
    }
};

template <typename T>
struct BufferOps<Spherial<T>> {
    static Spherial<T> identity() { return Spherial<T>{}; }

    static Spherial<T> interpolate(const Spherial<T>& a, const Spherial<T>& b, fp_t ratio) {
        return Spherial<T>::lerp(a, b, ratio);
    }

    static bool is_valid(const Spherial<T>& value) {
        (void)value;
        return true;
    }
};

template <typename T, sec Range, sample_per_sec Density, typename Ops = BufferOps<T>>
class Buffer {
    static_assert(Density > 0, "Density must be greater than 0");
    static_assert(buffer_value_ops<Ops, T>, "Buffer ops are not defined for this value type");
    static_assert(
        Range > 0 || Density == 1, "Static Buffer only supports Range == 0 && Density == 1");

public:
    struct Timestamped {
        T value;
        timestamp_ns_t timestamp;
    };

    static constexpr bool is_static             = Range == 0;
    static constexpr std::size_t capacity_value = is_static ? 1 : Range * Density;
    using mutex_type                            = tbb::spin_rw_mutex;

    Buffer() noexcept requires(!is_static) = default;

    explicit Buffer(const T& value) noexcept requires(is_static)
        : buffer_{
              StoredSample{value, 0}
    } {}

    Buffer(const Buffer&)            = delete;
    Buffer& operator=(const Buffer&) = delete;

    Buffer(Buffer&& other) noexcept(std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        move_from(std::move(other));
    }

    Buffer&
        operator=(Buffer&& other) noexcept(std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        if (this != &other) {
            move_assign_from(std::move(other));
        }
        return *this;
    }

    void push(timestamp_ns_t timestamp_ns, const T& value) noexcept requires(!is_static) {
        if (!Ops::is_valid(value)) {
            return;
        }

        mutex_type::scoped_lock lock(mutex_);
        if (size_ != 0 && timestamp_ns <= newest_ts_) {
            return;
        }

        buffer_[head_] = StoredSample{value, timestamp_ns};

        if (size_ == 0) {
            oldest_ts_ = timestamp_ns;
        }
        newest_ts_ = timestamp_ns;

        head_ = (head_ + 1) % capacity_value;

        if (size_ < capacity_value) {
            ++size_;
        } else {
            oldest_ts_ = buffer_[head_].timestamp;
        }
    }

    template <typename U = T>
    void push_rotate_only(timestamp_ns_t timestamp_ns, const math_fuxk::Ros2EulerRotd rot) noexcept
        requires(!is_static && requires(const U& value, const math_fuxk::Ros2EulerRotd& rotation) {
            { Ops::with_rotation(value, rotation) } -> std::same_as<U>;
        }) {
        T base = Ops::identity();
        if (auto latest_result = latest(); latest_result) {
            base = latest_result->value;
        }
        push(timestamp_ns, Ops::with_rotation(base, rot));
    }

    [[nodiscard]] std::expected<Timestamped, std::string> latest() const noexcept {
        if constexpr (is_static) {
            return Timestamped{buffer_[0].value, 0};
        }

        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return std::unexpected(
                fmt::format("Buffer::latest(): buffer is empty (capacity={})", capacity_value));
        }

        return materialize(at_logical(size_ - 1));
    }

    template <buffer_lookup_mode Mode>
    [[nodiscard]] std::expected<Timestamped, std::string>
        lookup(timestamp_ns_t ts, Mode) const noexcept {
        if constexpr (is_static) {
            return Timestamped{buffer_[0].value, ts};
        }

        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return std::unexpected(
                fmt::format(
                    "Buffer::lookup({}ns): buffer is empty (capacity={})", ts, capacity_value));
        }

        const auto& oldest = at_logical(0);
        const auto& newest = at_logical(size_ - 1);

        if constexpr (std::same_as<Mode, clamped_t>) {
            if (ts < oldest.timestamp) {
                return materialize(oldest);
            }
            if (ts > newest.timestamp) {
                return materialize(newest);
            }
        } else {
            if (ts < oldest.timestamp) {
                return std::unexpected(
                    fmt::format(
                        "Buffer::lookup({}ns): past extrapolation required, "
                        "buffer range=[{}, {}], size={}, query_ts={}",
                        ts, oldest.timestamp, newest.timestamp, size_, ts));
            }
            if (ts > newest.timestamp) {
                return std::unexpected(
                    fmt::format(
                        "Buffer::lookup({}ns): future extrapolation required, "
                        "buffer range=[{}, {}], size={}, query_ts={}",
                        ts, oldest.timestamp, newest.timestamp, size_, ts));
            }
        }

        const std::size_t lower_idx = find_floor_from_latest(ts);
        const auto& lower           = at_logical(lower_idx);
        if (lower.timestamp == ts) {
            return materialize(lower);
        }

        if constexpr (std::same_as<Mode, exact_t>) {
            return std::unexpected(
                fmt::format(
                    "Buffer::lookup({}ns, exact): no exact match, "
                    "nearest floor={}ns, buffer range=[{}, {}]",
                    ts, lower.timestamp, oldest.timestamp, newest.timestamp));
        } else if constexpr (std::same_as<Mode, nearest_t>) {
            if (lower_idx + 1 >= size_) {
                return materialize(lower);
            }

            if (lower.timestamp > ts) {
                return materialize(oldest);
            }

            const auto& next = at_logical(lower_idx + 1);

            const auto prev_delta = ts - lower.timestamp;
            const auto next_delta = next.timestamp - ts;
            return materialize(prev_delta <= next_delta ? lower : next);
        } else {
            if (lower.timestamp > ts) {
                return materialize(oldest);
            }
            if (lower_idx + 1 >= size_) {
                return materialize(lower);
            }

            const auto& next = at_logical(lower_idx + 1);
            const auto dt    = next.timestamp - lower.timestamp;
            if (dt == 0) {
                return materialize(lower);
            }

            const fp_t ratio = static_cast<fp_t>(ts - lower.timestamp) / static_cast<fp_t>(dt);
            return Timestamped{Ops::interpolate(lower.value, next.value, ratio), ts};
        }
    }

    [[nodiscard]] std::optional<std::pair<timestamp_ns_t, timestamp_ns_t>>
        time_range() const noexcept {
        if constexpr (is_static) {
            return std::make_pair(timestamp_ns_t{0}, timestamp_ns_t{0});
        }

        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return std::nullopt;
        }
        return std::make_pair(oldest_ts_, newest_ts_);
    }

    [[nodiscard]] std::size_t size() const noexcept {
        if constexpr (is_static) {
            return 1;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        return size_;
    }

    [[nodiscard]] constexpr std::size_t capacity() const noexcept { return capacity_value; }

    [[nodiscard]] bool is_full() const noexcept {
        if constexpr (is_static) {
            return true;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        return size_ == capacity_value;
    }

    [[nodiscard]] bool is_empty() const noexcept {
        if constexpr (is_static) {
            return false;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        return size_ == 0;
    }

    [[nodiscard]] bool contains_time(timestamp_ns_t ts) const noexcept {
        if constexpr (is_static) {
            (void)ts;
            return true;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return false;
        }
        return ts >= oldest_ts_ && ts <= newest_ts_;
    }

    void clear() noexcept requires(!is_static) {
        mutex_type::scoped_lock lock(mutex_);
        head_      = 0;
        size_      = 0;
        oldest_ts_ = 0;
        newest_ts_ = 0;
    }

private:
    struct StoredSample {
        T value{};
        timestamp_ns_t timestamp = 0;
    };

    void move_from(Buffer&& other) noexcept(std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        typename mutex_type::scoped_lock lock(other.mutex_);
        buffer_    = std::move(other.buffer_);
        head_      = other.head_;
        size_      = other.size_;
        oldest_ts_ = other.oldest_ts_;
        newest_ts_ = other.newest_ts_;
        reset_moved_from(other);
    }

    void move_assign_from(Buffer&& other) noexcept(
        std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        typename mutex_type::scoped_lock this_lock(mutex_);
        typename mutex_type::scoped_lock other_lock(other.mutex_);
        buffer_    = std::move(other.buffer_);
        head_      = other.head_;
        size_      = other.size_;
        oldest_ts_ = other.oldest_ts_;
        newest_ts_ = other.newest_ts_;
        reset_moved_from(other);
    }

    static void reset_moved_from(Buffer& other) noexcept {
        if constexpr (!is_static) {
            other.head_      = 0;
            other.size_      = 0;
            other.oldest_ts_ = 0;
            other.newest_ts_ = 0;
        }
    }

    [[nodiscard]] Timestamped materialize(const StoredSample& sample) const {
        return Timestamped{sample.value, sample.timestamp};
    }

    [[nodiscard]] const StoredSample& at_logical(std::size_t idx) const {
        const std::size_t physical = (head_ + capacity_value - size_ + idx) % capacity_value;
        return buffer_[physical];
    }

    [[nodiscard]] std::size_t find_floor_from_latest(timestamp_ns_t ts) const {
        std::size_t idx = size_ - 1;
        while (idx > 0 && at_logical(idx).timestamp > ts) {
            --idx;
        }
        return idx;
    }

    std::array<StoredSample, capacity_value> buffer_{};
    std::size_t head_         = 0;
    std::size_t size_         = 0;
    timestamp_ns_t oldest_ts_ = 0;
    timestamp_ns_t newest_ts_ = 0;
    mutable mutex_type mutex_;
};

template <typename T, typename Ops = BufferOps<T>>
using StaticBuffer = Buffer<T, 0, 1, Ops>;

} // namespace fast_tf
