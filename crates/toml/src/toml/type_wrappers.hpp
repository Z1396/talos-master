#pragma once

#include <optional>
#include <type_traits>
#include <utility>

namespace toml_helper {

// ============================================================================
// Lightweight type wrappers — separated from the heavy TOML deserialization
// machinery so that config struct headers can declare flatten<T> / required<T>
// members without pulling in <toml++/toml.hpp>.
// ============================================================================

template <typename T>
struct flatten {
    T value{};

    [[nodiscard]] constexpr T& get() noexcept { return value; }
    [[nodiscard]] constexpr const T& get() const noexcept { return value; }

    [[nodiscard]] constexpr T* operator->() noexcept { return &value; }
    [[nodiscard]] constexpr const T* operator->() const noexcept { return &value; }

    [[nodiscard]] constexpr T& operator*() noexcept { return value; }
    [[nodiscard]] constexpr const T& operator*() const noexcept { return value; }

    constexpr operator T&() & noexcept { return value; }
    constexpr operator const T&() const& noexcept { return value; }
    constexpr operator T&&() && noexcept { return std::move(value); }
    constexpr operator const T&&() const&& noexcept { return std::move(value); }

    constexpr flatten& operator=(const T& rhs) {
        value = rhs;
        return *this;
    }

    constexpr flatten& operator=(T&& rhs) noexcept(std::is_nothrow_move_assignable_v<T>) {
        value = std::move(rhs);
        return *this;
    }
};

template <typename T>
struct required {
    std::optional<T> value{};

    constexpr required() = default;
    constexpr required(const T& rhs)
        : value(rhs) {}
    constexpr required(T&& rhs) noexcept(std::is_nothrow_move_constructible_v<T>)
        : value(std::move(rhs)) {}

    [[nodiscard]] constexpr bool has_value() const noexcept { return value.has_value(); }

    [[nodiscard]] constexpr T& get() & noexcept(false) { return value.value(); }
    [[nodiscard]] constexpr const T& get() const& noexcept(false) { return value.value(); }
    [[nodiscard]] constexpr T&& get() && noexcept(false) { return std::move(value).value(); }
    [[nodiscard]] constexpr const T&& get() const&& noexcept(false) {
        return std::move(value).value();
    }

    [[nodiscard]] constexpr T* operator->() noexcept(false) {
        if (!value.has_value()) {
            throw std::bad_optional_access{};
        }
        return value.operator->();
    }
    [[nodiscard]] constexpr const T* operator->() const noexcept(false) {
        if (!value.has_value()) {
            throw std::bad_optional_access{};
        }
        return value.operator->();
    }

    [[nodiscard]] constexpr T& operator*() & { return get(); }
    [[nodiscard]] constexpr const T& operator*() const& { return get(); }
    [[nodiscard]] constexpr T&& operator*() && { return std::move(*this).get(); }
    [[nodiscard]] constexpr const T&& operator*() const&& { return std::move(*this).get(); }

    constexpr operator T&() & { return get(); }
    constexpr operator const T&() const& { return get(); }
    constexpr operator T&&() && { return std::move(get()); }
    constexpr operator const T&&() const&& { return std::move(get()); }

    constexpr required& operator=(const T& rhs) {
        value = rhs;
        return *this;
    }

    constexpr required& operator=(T&& rhs) noexcept(std::is_nothrow_move_assignable_v<T>) {
        value = std::move(rhs);
        return *this;
    }
};

namespace detail {

template <typename>
inline constexpr bool is_flatten_v = false;

template <typename T>
inline constexpr bool is_flatten_v<flatten<T>> = true;

template <typename T>
struct flatten_value;

template <typename T>
struct flatten_value<flatten<T>> {
    using type = T;
};

template <typename T>
using flatten_value_t = typename flatten_value<T>::type;

template <typename>
inline constexpr bool is_required_v = false;

template <typename T>
inline constexpr bool is_required_v<required<T>> = true;

template <typename T>
struct required_value;

template <typename T>
struct required_value<required<T>> {
    using type = T;
};

template <typename T>
using required_value_t = typename required_value<T>::type;

} // namespace detail

} // namespace toml_helper
