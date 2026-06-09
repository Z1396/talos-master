#include "L2_perception/armor/backends/traditional_classifier.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================

std::expected<TraditionalClassifier, std::string>
    TraditionalClassifier::create(std::string_view model_path, bool use_softmax) noexcept {
    try {
        auto net = cv::dnn::readNetFromONNX(std::string(model_path));

        if (net.empty()) {
            return std::unexpected(
                "Failed to load ONNX classifier model: " + std::string(model_path));
        }

        net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        return TraditionalClassifier(std::string(model_path), use_softmax, std::move(net));
    } catch (const std::exception& e) {
        SPDLOG_ERROR("[TraditionalClassifier] Model load failed: {}", e.what());
        return std::unexpected("ONNX classifier model load failed: " + std::string(e.what()));
    }
}

// ============================================================================
// Classification
// ============================================================================

TraditionalClassifier::ClassifyResult TraditionalClassifier::classify(
    const cv::Mat& number_img, float& confidence_out) const noexcept {
    try {
        // Normalize image to 0-1 range (matching atvision_ws behavior)
        cv::Mat normalized_img = number_img / 255.0;

        // Create blob from preprocessed image (input is already 28x28 from extract_number)
        cv::Mat blob;
        cv::dnn::blobFromImage(normalized_img, blob);

        // Set input to network
        net_.setInput(blob);

        // Forward inference
        cv::Mat output = net_.forward();

        // Post-process and return result
        return post_process_output(output, confidence_out);
    } catch (const std::exception& e) {
        SPDLOG_ERROR("[TraditionalClassifier] Inference failed: {}", e.what());
        return std::unexpected("Classifier inference failed: " + std::string(e.what()));
    }
}

// ============================================================================
// Output Post-Processing
// ============================================================================

ArmorName TraditionalClassifier::post_process_output(
    const cv::Mat& output, float& confidence_out) const noexcept {
    // Output should be 1D array with class logits or probabilities
    cv::Mat flat_output = output.reshape(1, 1);

    double confidence = 0.0;
    int label_id      = -1;

    if (use_softmax_) {
        // Legacy networks output logits, need to apply softmax (matching atvision_ws)
        float max_val = *std::max_element(flat_output.begin<float>(), flat_output.end<float>());
        cv::Mat exp_scores;
        cv::exp(flat_output - max_val, exp_scores);
        float sum = static_cast<float>(cv::sum(exp_scores)[0]);
        exp_scores /= sum;

        cv::Point class_id_point;
        cv::minMaxLoc(exp_scores, nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    } else {
        // New networks already output probabilities, directly take max
        cv::Point class_id_point;
        cv::minMaxLoc(flat_output, nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    }

    confidence_out = static_cast<float>(confidence);

    if (label_id < 0) {
        return ArmorName::Invalid;
    }

    return index_to_armor_name(label_id);
}

// ============================================================================
// Helper Methods
// ============================================================================

ArmorName TraditionalClassifier::index_to_armor_name(int idx) noexcept {
    switch (idx) {
    case 0: return ArmorName::One;
    case 1: return ArmorName::Two;
    case 2: return ArmorName::Three;
    case 3: return ArmorName::Four;
    case 4: return ArmorName::Five;
    case 5: return ArmorName::Outpost;
    case 6: return ArmorName::Sentry;
    case 7: return ArmorName::Base;
    default: return ArmorName::Invalid;
    }
}

} // namespace fcs::L2
