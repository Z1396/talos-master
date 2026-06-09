#include "L3_estimation/energy_meter_solver/voter.hpp"

namespace energy_meter {

namespace {

constexpr int VOTE_MIN_VALID  = 8;
constexpr int VOTE_MAX_FRAMES = 20;

} // namespace

auto Voter::update(std::size_t target_count) -> std::optional<Decision> {
    if (determined_) {
        return std::nullopt;
    }

    ++frames_;

    if (target_count == 2) {
        ++big_votes_;
    } else if (target_count == 1) {
        ++small_votes_;
    }

    const int total_votes = big_votes_ + small_votes_;
    const bool vote_ready = (total_votes >= VOTE_MIN_VALID) || (frames_ >= VOTE_MAX_FRAMES);

    if (!vote_ready) {
        return std::nullopt;
    }

    determined_ = true;
    is_big_     = big_votes_ > small_votes_;

    return decision();
}

void Voter::reset() {
    determined_  = false;
    is_big_      = true;
    big_votes_   = 0;
    small_votes_ = 0;
    frames_      = 0;
}

auto Voter::decision() const -> Decision {
    return Decision{
        .is_big      = is_big_,
        .big_votes   = big_votes_,
        .small_votes = small_votes_,
        .frames      = frames_,
    };
}

} // namespace energy_meter
