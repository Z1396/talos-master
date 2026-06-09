#pragma once

#include <tuple>
#include <utility>

namespace talos::primitive {

template <typename T, typename... PArgs>
struct lazy {
private:
    std::tuple<std::decay_t<PArgs>...> args_;

public:
    explicit constexpr lazy(PArgs&&... p_args) noexcept
        : args_(std::forward<PArgs>(p_args)...) {}

    /// Construct T with the stored pre-args (no additional args allowed)
    [[nodiscard]] std::unique_ptr<T> operator()() noexcept requires(sizeof...(PArgs) > 0) {
        return std::apply(
            [](auto&&... args) {
                return std::make_unique<T>(std::forward<decltype(args)>(args)...);
            },
            args_);
    }

    /// Construct T with additional runtime args (no pre-args stored)
    template <typename... Args>
    [[nodiscard]] std::unique_ptr<T> operator()(Args&&... args) noexcept
        requires(sizeof...(PArgs) == 0) {
        return std::make_unique<T>(std::forward<Args>(args)...);
    }
};

template <typename T, typename... PArgs>
constexpr lazy<T> make_lazy(PArgs&&... args) {
    return lazy<T, PArgs...>(args...);
}

} // namespace talos::primitive