#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

namespace fcs::L1 {

// ============================================================================
// Frame - 输入帧 (外部 → MissionComputer)
// ============================================================================

struct Frame {
    uint64_t seq{0};
    uint64_t timestamp_ns{0};
    cv::Mat image;

    static Frame from_mat(const uint64_t seq, const uint64_t ts, cv::Mat mat) {
        return Frame{seq, ts, std::move(mat)};
    }
};

} // namespace fcs::L1
