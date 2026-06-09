#pragma once

#include "L4_planning/aimer/types.hpp"
#include "L4_planning/config.hpp"

#include <cmath>

namespace fcs::L4 {

inline void advance_armor_aim_phase(
    const AimerConfig& cfg, double abs_v_yaw, bool target_jumped, ArmorAimPhase& phase,
    int& overflow_count) noexcept {
    if (!target_jumped) {
        phase          = ArmorAimPhase::SingleArmor;
        overflow_count = 0;
        return;
    }

    switch (phase) {
    case ArmorAimPhase::SingleArmor:
        overflow_count = (abs_v_yaw > cfg.single_whole_up) ? overflow_count + 1 : 0;
        if (overflow_count > cfg.transfer_thresh) {
            phase          = ArmorAimPhase::WholeCarArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarArmor:
        if (abs_v_yaw > cfg.whole_pair_up) {
            ++overflow_count;
        } else if (abs_v_yaw < cfg.single_whole_down) {
            --overflow_count;
        } else {
            overflow_count = 0;
        }
        if (std::abs(overflow_count) > cfg.transfer_thresh) {
            phase = (overflow_count > 0) ? ArmorAimPhase::WholeCarPair : ArmorAimPhase::SingleArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarPair:
        if (abs_v_yaw > cfg.pair_center_up) {
            ++overflow_count;
        } else if (abs_v_yaw < cfg.whole_pair_down) {
            --overflow_count;
        } else {
            overflow_count = 0;
        }
        if (std::abs(overflow_count) > cfg.transfer_thresh) {
            phase =
                (overflow_count > 0) ? ArmorAimPhase::WholeCarCenter : ArmorAimPhase::WholeCarArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarCenter:
        overflow_count = (abs_v_yaw < cfg.pair_center_down) ? overflow_count + 1 : 0;
        if (overflow_count > cfg.transfer_thresh) {
            phase          = ArmorAimPhase::WholeCarPair;
            overflow_count = 0;
        }
        break;
    }
}

} // namespace fcs::L4
