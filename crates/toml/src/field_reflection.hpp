#pragma once

// field_reflection compatibility shim — backed by Boost.PFR
//
// Drop-in replacement for the original yosh-matsuda/field_reflection library.
// Same namespace-level API, drastically reduced compile times by delegating to
// Boost.PFR's heavily optimized reflection engine.

#include <boost/pfr.hpp>

#include <cstddef>
#include <string_view>
#include <type_traits>
#include <utility>

namespace field_reflection {

// Concept: true if T is an aggregate whose fields can be individually referenced.
// Guard with std::is_aggregate_v before touching PFR — PFR's internals use
// static_assert which is NOT SFINAE-friendly, so we must filter non-aggregates
// first to prevent hard errors in concept evaluation.
template <typename T>
concept field_referenceable = std::is_aggregate_v<std::remove_cvref_t<T>>;

// Concept: true if field names can be extracted from T at compile time.
template <typename T>
concept field_namable = std::is_aggregate_v<std::remove_cvref_t<T>>
                     && requires { boost::pfr::names_as_array<std::remove_cvref_t<T>>(); };

namespace detail {

template <typename T, typename F, std::size_t... Is>
constexpr void for_each_field_impl(T&& obj, F&& func, std::index_sequence<Is...>) {
    using Clean          = std::remove_cvref_t<T>;
    constexpr auto names = boost::pfr::names_as_array<Clean>();
    (func(names[Is], boost::pfr::get<Is>(std::forward<T>(obj))), ...);
}

} // namespace detail

// Iterate over all fields of an aggregate, calling func(name, field) for each.
template <typename T, typename F>
constexpr void for_each_field(T&& obj, F&& func) {
    using Clean = std::remove_cvref_t<T>;
    detail::for_each_field_impl(
        std::forward<T>(obj), std::forward<F>(func),
        std::make_index_sequence<boost::pfr::tuple_size_v<Clean>>{});
}

} // namespace field_reflection
