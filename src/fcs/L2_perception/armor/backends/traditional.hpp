#pragma once

#include "../backend.hpp"
#include "../config.hpp"
#include "traditional_classifier.hpp"
#include "traditional_types.hpp"

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace fcs::L2 {

// ============================================================================
// AT Legacy Traditional Backend Implementation
// ============================================================================

class TraditionalBackend : public DetectorBackendBase<TraditionalBackend> {
public:
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;
    using Config          = ArmorTraditionalConfig;

    /// Factory: construct a fully-initialized backend.
    /// Construction IS initialization — no separate init() needed.
    [[nodiscard]] static std::expected<TraditionalBackend, std::string>
        create(Config config) noexcept;

    ~TraditionalBackend() = default;

    // Move-only
    TraditionalBackend(TraditionalBackend&&) noexcept            = default;
    TraditionalBackend& operator=(TraditionalBackend&&) noexcept = default;
    TraditionalBackend(const TraditionalBackend&)                = delete;
    TraditionalBackend& operator=(const TraditionalBackend&)     = delete;

    /// Detect armors in image
    [[nodiscard]] DetectionResult detect_impl(const cv::Mat& input, ArmorColor color) noexcept;

    /// Get current config
    [[nodiscard]] const Config& get_config() const noexcept { return config_; }

private:
    /// Private constructor — use create() factory
    explicit TraditionalBackend(Config config) noexcept;
    Config config_;
    std::unique_ptr<TraditionalClassifier> classifier_;

    /// Preprocess image to binary
    [[nodiscard]] cv::Mat
        preprocess_image(const cv::Mat& bgr_img, const ArmorTraditionalConfig& cfg) const noexcept;

    /// Find light bars in image
    [[nodiscard]] std::vector<Light> find_lights(
        const cv::Mat& bgr_img, const cv::Mat& binary_img,
        const ArmorTraditionalConfig& cfg) const noexcept;

    /// Check if contour is a valid light bar
    [[nodiscard]] bool
        is_light(const Light& light, const ArmorTraditionalConfig& cfg) const noexcept;

    /// Match light pairs to form armors
    [[nodiscard]] std::vector<ArmorDetection> match_lights(
        std::vector<Light>& lights, const ArmorTraditionalConfig& cfg,
        cv::Mat gray_img) const noexcept;

    /// Check if there's a light between two lights
    [[nodiscard]] static bool
        contains_light(size_t i, size_t j, const std::vector<Light>& lights) noexcept;

    /// Check if two lights form a valid armor
    [[nodiscard]] ArmorType is_armor(
        const Light& light_1, const Light& light_2,
        const ArmorTraditionalConfig& cfg) const noexcept;

    /// Extract number ROI: perspective transform + OTSU binarization
    [[nodiscard]] cv::Mat extract_number(
        const cv::Mat& src, const std::array<cv::Point2f, 4>& lights_vertices,
        ArmorType armor_type) const noexcept;
};

} // namespace fcs::L2
