#pragma once

#include "../backend.hpp"
#include "../config.hpp"
#include "L2_perception/armor/backends/base.hpp"
#include "core/armor_types.hpp"
#include "core/types.hpp"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace fcs::L2 {

// ============================================================================
// AT Legacy NN Backend Implementation
// ============================================================================

/// Neural network detection backend using OnnxRuntime
class OrtBackend : public DetectorBackendBase<OrtBackend> {
public:
    static constexpr int INPUT_W     = 640;
    static constexpr int INPUT_H     = 640;
    static constexpr int NUM_COLORS  = 4;
    static constexpr int NUM_SIZES   = 2;
    static constexpr int NUM_CLASSES = 8; // G, 1, 2, 3, 4, 5, O, B
    static constexpr int NUM_KPTS    = 4; // 4 corner keypoints (x, y)
    using DetectionResult            = std::expected<std::vector<ArmorDetection>, std::string>;
    using Config                     = ArmorOrtConfig;

    /// Factory: construct a fully-initialized backend (load model via OnnxRuntime).
    /// Construction IS initialization — no separate init() needed.
    [[nodiscard]] static std::expected<OrtBackend, std::string> create(Config config) noexcept;

    ~OrtBackend();

    // Move-only (OnnxRuntime resources)
    OrtBackend(OrtBackend&&) noexcept;
    OrtBackend& operator=(OrtBackend&&) noexcept;
    OrtBackend(const OrtBackend&)            = delete;
    OrtBackend& operator=(const OrtBackend&) = delete;

    /// Detect armors in image (synchronous)
    [[nodiscard]] DetectionResult detect_impl(const cv::Mat& image, ArmorColor color) noexcept;

    /// Get execution devices
    [[nodiscard]] std::vector<std::string> execution_devices() const noexcept;

    /// Get current config
    [[nodiscard]] const Config& get_config() const noexcept { return config_; }

private:
    /// Private constructor — use create() factory
    explicit OrtBackend(Config config) noexcept;

    /// Preprocess image for inference (letterbox + normalize)
    struct PreprocContext {
        cv::Mat preprocessed; // Preprocessed input tensor
        float scale_x = 1.0f; // Scale factor for x coordinates
        float scale_y = 1.0f; // Scale factor for y coordinates
        float pad_x   = 0.0f; // Padding offset x
        float pad_y   = 0.0f; // Padding offset y
        int orig_w    = 0;
        int orig_h    = 0;
    };

    [[nodiscard]] PreprocContext
        preprocess(const cv::Mat& image, const ArmorOrtConfig& cfg) const noexcept;

    /// Run inference
    [[nodiscard]] bool infer(const PreprocContext& ctx) noexcept;

    /// Postprocess inference output
    [[nodiscard]] std::vector<ArmorDetection>
        postprocess(const PreprocContext& ctx, const ArmorOrtConfig& cfg) const noexcept;

    /// Transform coordinates from model output to original image
    void transform_coordinates(
        std::vector<ArmorDetection>& detections, const PreprocContext& ctx) const noexcept;

    /// Non-maximum suppression
    [[nodiscard]] std::vector<ArmorDetection>
        nms(std::vector<ArmorDetection>& detections, const ArmorOrtConfig& cfg) const noexcept;

private:
    Config config_;

    // OnnxRuntime resources (PIMPL to avoid header pollution)
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fcs::L2
