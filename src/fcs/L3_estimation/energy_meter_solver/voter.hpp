#pragma once

#include <cstddef>
#include <optional>

namespace energy_meter {

class Voter {
public:
    struct Decision {
        bool is_big{true};
        int big_votes{0};
        int small_votes{0};
        int frames{0};
    };

    auto update(std::size_t target_count) -> std::optional<Decision>;
    void reset();

    [[nodiscard]] auto determined() const -> bool { return determined_; }
    [[nodiscard]] auto is_big() const -> bool { return is_big_; }

private:
    [[nodiscard]] auto decision() const -> Decision;

    bool determined_{false};
    bool is_big_{true};
    int big_votes_{0};
    int small_votes_{0};
    int frames_{0};
};

} // namespace energy_meter
