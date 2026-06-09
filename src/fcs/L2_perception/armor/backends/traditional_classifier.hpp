#pragma once

#include "../backend.hpp"
#include "core/types.hpp"

#include <expected>
#include <string_view>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace fcs::L2 {

// ============================================================================
// AT Legacy Traditional ONNX Classifier
// ============================================================================

/// OpenCV DNN-based ONNX classifier for armor number classification
/// Supports LeNet and MLP models with typical ONNX format
class TraditionalClassifier {
public:
    using ClassifyResult = std::expected<ArmorName, std::string>;

    /// Factory: load ONNX model and return fully-initialized classifier.
    /// Construction IS initialization — no separate init() needed.
    [[nodiscard]] static std::expected<TraditionalClassifier, std::string>
        create(std::string_view model_path, bool use_softmax = true) noexcept;

    // Move-only (cv::dnn::Net is movable)
    TraditionalClassifier(TraditionalClassifier&&) noexcept            = default;
    TraditionalClassifier& operator=(TraditionalClassifier&&) noexcept = default;
    TraditionalClassifier(const TraditionalClassifier&)                = delete;
    TraditionalClassifier& operator=(const TraditionalClassifier&)     = delete;

    /// Classify a single armor image (preprocessed: 20x28 grayscale OTSU-binary, normalized [0,1])
    /// Returns the predicted ArmorName and updates confidence_out with confidence score
    [[nodiscard]] ClassifyResult
        classify(const cv::Mat& number_img, float& confidence_out) const noexcept;

private:
    TraditionalClassifier(std::string model_path, bool use_softmax, cv::dnn::Net net) noexcept
        : model_path_(std::move(model_path))
        , net_(std::move(net))
        , use_softmax_(use_softmax) {}

    // Helper methods
    [[nodiscard]] ArmorName
        post_process_output(const cv::Mat& output, float& confidence_out) const noexcept;

    /// Map network output index to ArmorName
    [[nodiscard]] static ArmorName index_to_armor_name(int idx) noexcept;

    std::string model_path_;
    mutable cv::dnn::Net net_; // mutable because OpenCV DNN operations are non-const
    bool use_softmax_ = true;  // Whether to apply softmax to network output (for legacy networks)
};

} // namespace fcs::L2
