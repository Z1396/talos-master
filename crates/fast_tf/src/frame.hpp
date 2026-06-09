#pragma once

#include "buffer.hpp"
#include "euler.hpp"
#include "matrix.hpp"
#include "types.hpp"
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace stdx {

// ---------- copy const/volatile ----------
template <class From, class To>
struct copy_cv {
    using type = To;
};

template <class From, class To>
struct copy_cv<const From, To> {
    using type = const To;
};

template <class From, class To>
struct copy_cv<volatile From, To> {
    using type = volatile To;
};

template <class From, class To>
struct copy_cv<const volatile From, To> {
    using type = const volatile To;
};

template <class From, class To>
using copy_cv_t = typename copy_cv<From, To>::type;

// ---------- copy reference ----------
template <class From, class To>
struct copy_ref {
    using type = To;
};

template <class From, class To>
struct copy_ref<From&, To> {
    using type = To&;
};

template <class From, class To>
struct copy_ref<From&&, To> {
    using type = To&&;
};

template <class From, class To>
using copy_ref_t = typename copy_ref<From, To>::type;

// ---------- combine ----------
template <class From, class To>
using forward_like_t =
    copy_ref_t<From, copy_cv_t<std::remove_reference_t<From>, std::remove_reference_t<To>>>;

// ---------- forward_like ----------
template <class Like, class T>
constexpr forward_like_t<Like, T&&> forward_like(T&& x) noexcept {
    return static_cast<forward_like_t<Like, T&&>>(x);
}

} // namespace stdx
namespace fast_tf {

#define DECL_ROOT(name)                                     \
    struct name##_fuxk_frame {                              \
        static constexpr std::string_view frame_id = #name; \
        using ancestor                             = void;  \
    };

#define DECL(name, parent)                                                \
    struct name##_fuxk_frame {                                            \
        static constexpr std::string_view frame_id = #name;               \
        using ancestor                             = parent##_fuxk_frame; \
    };

#define DECL_WITH_ID(name, parent, id)                                    \
    struct name##_fuxk_frame {                                            \
        static constexpr std::string_view frame_id = id;                  \
        using ancestor                             = parent##_fuxk_frame; \
    };

DECL_ROOT(world);
DECL(odom, world);
DECL(gimbal_yaw, odom);
DECL(gimbal_pitch, gimbal_yaw);
DECL(camera_link, gimbal_pitch);
DECL_WITH_ID(camera_optical, camera_link, "camera_optical_frame");
DECL(muzzle_link, gimbal_pitch);

// 防止外部使用
#undef DECL_ROOT
#undef DECL
#undef DECL_WITH_ID

using world          = world_fuxk_frame;
using odom           = odom_fuxk_frame;
using gimbal         = gimbal_yaw_fuxk_frame;
using gimbal_pitch   = gimbal_pitch_fuxk_frame;
using camera         = camera_link_fuxk_frame;
using camera_optical = camera_optical_fuxk_frame;
using muzzle         = muzzle_link_fuxk_frame;

template <typename T>
concept frame = requires {
    { T::frame_id } -> std::convertible_to<std::string_view>;
    typename T::ancestor;
};

template <typename T>
concept root_frame = frame<T> && std::is_void_v<typename T::ancestor>;

template <typename T>
concept non_root_frame = frame<T> && !std::is_void_v<typename T::ancestor>;

template <frame Child, frame Ancestor>
constexpr bool is_descendant_of() noexcept {
    if constexpr (std::is_same_v<Child, Ancestor>) {
        return true;
    } else if constexpr (std::is_void_v<typename Child::ancestor>) {
        return false;
    } else {
        return is_descendant_of<typename Child::ancestor, Ancestor>();
    }
}

template <frame Coordinate, typename T>
struct [[nodiscard]] Fuck {
    explicit Fuck(T t)
        : val(std::move(t)) {}
    explicit Fuck() = default;

    // ========================================================================
    // Transparent Dereference Operators (C++23 Deducing This)
    // ========================================================================
    // Single implementation handles all value categories:
    // - Fuck&         → T&
    // - const Fuck&   → const T&
    // - Fuck&&        → T&&
    // - const Fuck&&  → const T&

    template <typename Self>
    [[nodiscard]] constexpr auto operator*(this Self&& self) noexcept -> decltype(auto) {
        // stdx::forward_like preserves the value category:
        // If self is an lvalue reference, returns lvalue reference to val
        // If self is an rvalue reference, returns rvalue reference to val
        return stdx::forward_like<Self>(self.val);
    }

    template <typename Self>
    [[nodiscard]] constexpr auto operator->(this Self&& self) noexcept -> decltype(auto) {
        // std::addressof works for both const and non-const
        // Return type is automatically deduced as pointer-to-T
        // (with const-ness matching Self's const-ness)
        return std::addressof(self.val);
    }

    // ========================================================================
    // Coordinate-Preserving Binary Operators
    // ========================================================================
    // Result retains the left-hand side's coordinate system

    template <typename Self>
    [[nodiscard]] constexpr auto operator*(this Self&& self, const T& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * other)> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * other)>(
            stdx::forward_like<Self>(self.val) * other);
    }

    template <typename Self, frame OtherCoord>
    [[nodiscard]] constexpr auto
        operator*(this Self&& self, const Fuck<OtherCoord, T>& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * (*other))> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * (*other))>(
            stdx::forward_like<Self>(self.val) * (*other));
    }

    template <typename Self, frame OtherCoord>
    requires std::is_same_v<Coordinate, OtherCoord> [[nodiscard]] constexpr auto
        operator+(this Self&& self, const Fuck<OtherCoord, T>& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) + (*other))> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) + (*other))>(
            stdx::forward_like<Self>(self.val) + (*other));
    }

    template <typename Self, frame OtherCoord>
    requires std::is_same_v<Coordinate, OtherCoord> [[nodiscard]] constexpr auto
        operator-(this Self&& self, const Fuck<OtherCoord, T>& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) - (*other))> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) - (*other))>(
            stdx::forward_like<Self>(self.val) - (*other));
    }

    // ========================================================================
    // Transparent Function Call Operator
    // ========================================================================
    // Perfectly forwards arguments to the wrapped callable

    template <typename Self, typename... Args>
    [[nodiscard]] constexpr auto operator()(this Self&& self, Args&&... args) noexcept(
        noexcept(stdx::forward_like<Self>(self.val)(std::forward<Args>(args)...)))
        -> decltype(stdx::forward_like<Self>(self.val)(std::forward<Args>(args)...)) {
        return stdx::forward_like<Self>(self.val)(std::forward<Args>(args)...);
    }

private:
    T val;
};

template <typename Coordinate, typename T>
inline auto fucked(T&& t) {
    return Fuck<Coordinate, T>(std::forward<T>(t));
}

template <frame A, frame B>
using FrameTransform = TransformMatrixd<A, B>;

template <frame Coordinate>
using IdentityTransform = FrameTransform<Coordinate, Coordinate>;

/// Edge transform: from `Layer`'s ancestor into `Layer` (passive: re-expresses Layer in ancestor).
template <non_root_frame Layer>
using EdgeTransform = FrameTransform<typename Layer::ancestor, Layer>;

template <frame Coordinate>
using FuckDouble = Fuck<Coordinate, double>;

template <frame Coordinate>
using FuckFloat = Fuck<Coordinate, float>;

template <frame Coordinate>
using FuckIntrinsicEulerRotd = Fuck<Coordinate, math_fuxk::Ros2EulerRotd>;

template <frame Coordinate>
using FuckIntrinsicEulerRotf = Fuck<Coordinate, math_fuxk::Ros2EulerRotf>;

template <typename T>
using EphemeralBuffer = Buffer<EdgeTransform<T>, 5, 500>;

using CoordinateSystem = std::tuple<
    EphemeralBuffer<odom_fuxk_frame>, EphemeralBuffer<gimbal_yaw_fuxk_frame>,
    EphemeralBuffer<gimbal_pitch_fuxk_frame>, EphemeralBuffer<camera_link_fuxk_frame>,
    EphemeralBuffer<camera_optical_fuxk_frame>, EphemeralBuffer<muzzle_link_fuxk_frame>>;

template <typename Layer>
struct buffer_coord_of;

template <typename Layer>
using buffer_coord_t = buffer_coord_of<Layer>::type;

template <typename Scalar, typename A, typename B, sec Range, sample_per_sec Density>
struct buffer_coord_of<Buffer<TransformMatrix<Scalar, A, B>, Range, Density>> {
    using type = B;
};

template <non_root_frame Layer, typename System, size_t I = 0>
[[nodiscard]] constexpr decltype(auto) buffer_of(System&& system) noexcept {
    using tuple_t = std::remove_cvref_t<System>;

    static_assert(I < std::tuple_size_v<tuple_t>, "Layer not found in CoordinateSystem");

    using buffer_coord =
        typename buffer_coord_of<std::remove_cvref_t<std::tuple_element_t<I, tuple_t>>>::type;

    if constexpr (std::is_same_v<Layer, buffer_coord>) {
        return std::get<I>(std::forward<System>(system));
    } else {
        return buffer_of<Layer, System, I + 1>(std::forward<System>(system));
    }
}

template <typename System>
struct coordinate_of;

template <typename... Buffers>
struct coordinate_of<std::tuple<Buffers...>> {
    using type = std::tuple<typename buffer_coord_of<std::remove_cvref_t<Buffers>>::type...>;
};

template <typename System>
using coordinate_of_t = typename coordinate_of<std::remove_cvref_t<System>>::type;

template <typename System, typename Fn>
constexpr void for_each_coordinate_buffer(System&& system, Fn&& fn) {
    std::apply(
        [&](auto&&... xs) {
            (fn(std::type_identity<
                    typename buffer_coord_of<std::remove_cvref_t<decltype(xs)>>::type>{},
                std::forward<decltype(xs)>(xs)),
             ...);
        },
        std::forward<System>(system));
}

template <non_root_frame Layer>
constexpr void update(
    CoordinateSystem& system, const EdgeTransform<Layer>& transform, timestamp_ns_t ns) noexcept {
    buffer_of<Layer>(system).push(ns, transform);
}

template <non_root_frame Layer>
constexpr void update_rotate_only(
    CoordinateSystem& system, math_fuxk::Ros2EulerRotd rot, timestamp_ns_t ns) noexcept {
    buffer_of<Layer>(system).push_rotate_only(ns, rot);
}

/// Lookup transform: build T<Target, Source> (re-express Source in Target's frame).
///
/// Internal recursive step. `acc` accumulates the chain from Source upward.
template <frame Target, frame Source, frame CurrentFrame>
constexpr auto lookup(
    const CoordinateSystem& system, TransformMatrixd<CurrentFrame, Source> acc,
    timestamp_ns_t ns) noexcept -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    static_assert(is_descendant_of<Source, Target>(), "Source must be a descendant of Target");
    if constexpr (std::is_same_v<CurrentFrame, Target>) {
        return acc;
    } else if constexpr (root_frame<CurrentFrame>) {
        return std::unexpected(
            fmt::format(
                "lookup({} -> {}, {}ns): traversed past root without finding target, "
                "stopped at root frame '{}'",
                Source::frame_id, Target::frame_id, ns, CurrentFrame::frame_id));
    } else {
        const auto& buffer = buffer_of<CurrentFrame>(system);
        auto result        = buffer.lookup(ns, interpolate);
        if (!result) {
            return std::unexpected(
                fmt::format(
                    "lookup({} -> {}, {}ns): failed at edge {} -> {}: {}", Source::frame_id,
                    Target::frame_id, ns, CurrentFrame::ancestor::frame_id, CurrentFrame::frame_id,
                    result.error()));
        }
        // edge: T<CurrentFrame::ancestor, CurrentFrame>
        // acc:  T<CurrentFrame, Source>
        // edge * acc → T<ancestor, Source>
        return lookup<Target, Source>(system, result->value * acc, ns);
    }
}

/// Lookup transform: T<Target, Source> (re-express Source in Target's frame).
template <frame Target, frame Source>
constexpr auto lookup(const CoordinateSystem& system, timestamp_ns_t ns) noexcept
    -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    return lookup<Target, Source>(system, I<Source>(), ns);
}

/// Lookup transform using clamped interpolation.
///
/// This never returns `FutureExtrapolationRequired`/`PastExtrapolationRequired` as long as the
/// buffer has data; it clamps the query time to the buffer's time range (equivalent to TF2
/// `TimePointZero` semantics for "latest available" when `ns` is newer than the newest sample).
template <frame Target, frame Source, frame CurrentFrame>
constexpr auto lookup_clamped(
    const CoordinateSystem& system, TransformMatrixd<CurrentFrame, Source> acc,
    timestamp_ns_t ns) noexcept -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    static_assert(is_descendant_of<Source, Target>(), "Source must be a descendant of Target");
    if constexpr (std::is_same_v<CurrentFrame, Target>) {
        return acc;
    } else if constexpr (root_frame<CurrentFrame>) {
        return std::unexpected(
            fmt::format(
                "lookup_clamped({} -> {}, {}ns): traversed past root without finding target, "
                "stopped at root frame '{}'",
                Source::frame_id, Target::frame_id, ns, CurrentFrame::frame_id));
    } else {
        const auto& buffer = buffer_of<CurrentFrame>(system);
        auto result        = buffer.lookup(ns, clamped);
        if (!result) {
            return std::unexpected(
                fmt::format(
                    "lookup_clamped({} -> {}, {}ns): failed at edge {} -> {}: {}", Source::frame_id,
                    Target::frame_id, ns, CurrentFrame::ancestor::frame_id, CurrentFrame::frame_id,
                    result.error()));
        }
        // edge: T<CurrentFrame::ancestor, CurrentFrame>
        // acc:  T<CurrentFrame, Source>
        // edge * acc → T<ancestor, Source>
        return lookup_clamped<Target, Source>(system, result->value * acc, ns);
    }
}

template <frame Target, frame Source>
constexpr auto lookup_clamped(const CoordinateSystem& system, timestamp_ns_t ns) noexcept
    -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    return lookup_clamped<Target, Source, Source>(system, I<Source>(), ns);
}

} // namespace fast_tf
