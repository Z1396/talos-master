#pragma once

#include "core/armor_types.hpp"
#include "ldm_config.hpp"
#include "types.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <expected>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace fcs::L2::ldm {

class LdmDetector {
public:
    explicit LdmDetector(LdmDetectorConfig config) noexcept;
    ~LdmDetector() = default;

    std::expected<std::optional<LdmDetection>, DetectorError>
        detect(const cv::Mat& image) const noexcept;
    std::expected<std::optional<LdmDetection>, DetectorError>
        detect(const cv::Mat& image, ArmorColor color) const noexcept;

private:
    LdmDetectorConfig config_;
};

} // namespace fcs::L2::ldm
