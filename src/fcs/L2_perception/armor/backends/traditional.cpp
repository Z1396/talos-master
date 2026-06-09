#include "L2_perception/armor/backends/traditional.hpp"

#include "L2_perception/armor/config.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <ranges>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================

std::expected<TraditionalBackend, std::string> TraditionalBackend::create(Config config) noexcept {
    TraditionalBackend backend(std::move(config));
    const auto& cfg = backend.get_config();

    // Validate: classifier model path must be provided
    if (cfg.classifier_model_path.empty()) {
        return std::unexpected("[TraditionalBackend::create] Classifier model path is empty");
    }

    // Create classifier via factory
    auto classifier_result =
        TraditionalClassifier::create(cfg.classifier_model_path, cfg.classifier_use_softmax);
    if (!classifier_result) {
        return std::unexpected(
            "[TraditionalBackend::create] Failed to create ONNX classifier for '"
            + cfg.classifier_model_path + "': " + classifier_result.error());
    }

    backend.classifier_ = std::make_unique<TraditionalClassifier>(std::move(*classifier_result));
    return backend;
}

TraditionalBackend::TraditionalBackend(Config config) noexcept
    : config_{std::move(config)} {}

// ============================================================================
// Detection Entry Point
// ============================================================================

TraditionalBackend::DetectionResult
    TraditionalBackend::detect_impl(const cv::Mat& input, ArmorColor color) noexcept {
    if (input.empty()) {
        return std::unexpected("Empty image passed to traditional backend");
    }

    const auto& cfg = get_config();

    // 1. Preprocess the image
    cv::Mat binary = preprocess_image(input, cfg);

    // 2. Find lights
    auto lights = find_lights(input, binary, cfg);

    // Save gray image for corner correction (precompute for efficiency)
    cv::Mat gray_img;
    cv::cvtColor(input, gray_img, cv::COLOR_BGR2GRAY);

    // 3. Match lights to armors
    auto detections = match_lights(lights, cfg, gray_img);

    // 5. Classify armors if classifier is available
    for (size_t d = 0; d < detections.size(); ++d) {
        auto& detection = detections[d];

        // Extract number image: perspective transform + OTSU binarization
        cv::Mat number_img = extract_number(input, detection.corners, detection.type);

        if (!number_img.empty()) {
            float confidence = 0.0f;
            auto result      = classifier_->classify(number_img, confidence);

            if (result) {
                detection.name       = *result;
                detection.confidence = confidence;
            } else {
                // Classification failed, mark as invalid
                detection.name       = ArmorName::Invalid;
                detection.confidence = 0.0f;
            }
        }

        // 5. Filter out invalid detections
        detections.erase(
            std::remove_if(
                detections.begin(), detections.end(),
                [&cfg](const ArmorDetection& detection) {
                    // Filter by confidence threshold
                    if (detection.confidence < cfg.classifier_confidence_threshold) {
                        return true;
                    }

                    // Filter by name validity
                    if (detection.name == ArmorName::Invalid) {
                        return true;
                    }

                    // Filter by type-class matching (when type filtering
                    // enabled)
                    if (cfg.classifier_enable_type_filtering) {
                        if (detection.type == ArmorType::Large) {
                            // Large armor cannot be: outpost, 2, sentry
                            if (detection.name == ArmorName::Outpost
                                || detection.name == ArmorName::Two
                                || detection.name == ArmorName::Sentry) {
                                return true;
                            }
                        } else if (detection.type == ArmorType::Small) {
                            // Small armor cannot be: 1, base
                            if (detection.name == ArmorName::One
                                || detection.name == ArmorName::Base
                                || detection.name == ArmorName::BaseLarge) {
                                return true;
                            }
                        }
                    }

                    return false; // Keep this detection
                }),
            detections.end());
    }

    return detections;
}
// Preprocessing
// ============================================================================

cv::Mat TraditionalBackend::preprocess_image(
    const cv::Mat& bgr_img, const ArmorTraditionalConfig& cfg) const noexcept {
    if (!cfg.advanced_binary) {
        cv::Mat gray;
        cv::cvtColor(bgr_img, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, gray, cfg.binary_threshold, 255, cv::THRESH_BINARY);
        return gray;
    }
    std::vector<cv::Mat> ch;
    cv::split(bgr_img, ch);

    cv::Mat max01, min01, maxc, minc, chroma;
    cv::max(ch[0], ch[1], max01);
    cv::max(max01, ch[2], maxc);

    cv::min(ch[0], ch[1], min01);
    cv::min(min01, ch[2], minc);

    cv::subtract(maxc, minc, chroma);

    cv::Mat mask;
    // Option A: Dynamically adjust the baseline (recommended for use in competitions)
    // Calculate the average brightness of the image, and the baseline fluctuates with the
    // environmental brightness
    double min_val, max_val;
    cv::minMaxLoc(chroma, &min_val, &max_val);
    int dynamic_thresh = static_cast<int>(
        max_val
        * cfg.dark_percentage); // Use half of the maximum brightness as the dynamic baseline
    // Alternatively, use the threshold from the configuration as the lower bound to prevent
    // misjudgment in completely dark environments
    dynamic_thresh = std::max(dynamic_thresh, cfg.binary_threshold);
    cv::threshold(chroma, mask, dynamic_thresh, 255, cv::THRESH_BINARY);

    return mask;
}

// ============================================================================
// Light Detection
// ============================================================================

std::vector<Light> TraditionalBackend::find_lights(
    const cv::Mat& bgr_img, const cv::Mat& binary_img,
    const ArmorTraditionalConfig& cfg) const noexcept {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<Light> lights;
    lights.reserve(contours.size());

    for (const auto& contour : contours) {
        if (contour.size() < 6)
            continue;

        Light light(contour);

        if (is_light(light, cfg)) {
            // Determine color from contour pixels
            int sum_r = 0, sum_b = 0;
            for (const auto& point : contour) {
                const auto& pixel = bgr_img.at<cv::Vec3b>(point.y, point.x);
                sum_r += pixel[2]; // R channel (BGR order)
                sum_b += pixel[0]; // B channel
            }

            const int avg_diff = std::abs(sum_r - sum_b) / static_cast<int>(contour.size());
            // Only set color if difference exceeds threshold (matching armor_detector_ov)
            if (avg_diff > cfg.light_color_diff_thresh) {
                light.color = sum_r > sum_b ? ArmorColor::Red : ArmorColor::Blue;
            }
            // If threshold not met, light.color remains Neutral (WHITE)
            lights.emplace_back(light);
        }
    }

    // Sort by x coordinate
    std::ranges::sort(lights.begin(), lights.end(), [](const Light& l1, const Light& l2) {
        return l1.center.x < l2.center.x;
    });

    return lights;
}

bool TraditionalBackend::is_light(
    const Light& light, const ArmorTraditionalConfig& cfg) const noexcept {
    // Check width/height ratio
    const float ratio   = static_cast<float>(light.width) / static_cast<float>(light.length);
    const bool ratio_ok = (cfg.light_min_ratio < ratio) && (ratio < cfg.light_max_ratio);

    // Check tilt angle
    const bool angle_ok = light.tilt_angle < cfg.light_max_angle;

    return ratio_ok && angle_ok;
}

// ============================================================================
// Armor Matching
// ============================================================================

std::vector<ArmorDetection> TraditionalBackend::match_lights(
    std::vector<Light>& lights, const ArmorTraditionalConfig& cfg,
    [[maybe_unused]] const cv::Mat gray_img) const noexcept {
    // Pre-allocate to avoid repeated reallocation in nested loop
    // Worst case: n*(n-1)/2 pairs where n = lights.size()
    std::vector<ArmorDetection> detections;
    detections.reserve(lights.size() * (lights.size() - 1) / 2 + 16);

    // NOTE: Color filtering removed - detect all colors (RBGP)
    // Colors are still detected and preserved in the output for downstream use

    // Loop all pairs of lights
    for (auto it1 = lights.begin(); it1 != lights.end(); ++it1) {
        // No color filtering - all colors allowed

        const double max_iter_width = it1->length * cfg.armor_max_large_center_distance;

        for (auto it2 = it1 + 1; it2 != lights.end(); ++it2) {
            // No color filtering - all colors allowed

            // Check if there's a light between them
            const auto i = static_cast<size_t>(std::distance(lights.begin(), it1));
            const auto j = static_cast<size_t>(std::distance(lights.begin(), it2));
            if (contains_light(i, j, lights))
                continue;

            // Early termination if too far apart
            if (it2->center.x - it1->center.x > max_iter_width)
                break;

            // Check if this pair forms a valid armor
            const auto type = is_armor(*it1, *it2, cfg);
            if (type == ArmorType::Invalid) {
                continue;
            }

            // NOTE: armor_detector_ov does NOT use corner_corrector
            // corner_corrector_ is available but disabled to match behavior

            // Create detection from lights
            auto detection = make_detection_from_lights(*it1, *it2);
            detection.type = type;
            detections.emplace_back(detection);
        }
    }

    return detections;
}

bool TraditionalBackend::contains_light(
    size_t i, size_t j, const std::vector<Light>& lights) noexcept {
    const auto& light_1 = lights[i];
    const auto& light_2 = lights[j];

    // Bounding rect of the two lights
    std::array<cv::Point2f, 4> points = {light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect                = cv::boundingRect(points);

    const double avg_length = (light_1.length + light_2.length) / 2.0;
    const double avg_width  = (light_1.width + light_2.width) / 2.0;

    // Check lights in between
    for (size_t k = i + 1; k < j; ++k) {
        const auto& test_light = lights[k];

        // Skip if too wide (might be number)
        if (test_light.width > 2 * avg_width)
            continue;

        // Skip if too short (might be noise)
        if (test_light.length < 0.5 * avg_length)
            continue;

        // Check if any key point is inside the bounding rect
        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom)
            || bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

ArmorType TraditionalBackend::is_armor(
    const Light& light_1, const Light& light_2, const ArmorTraditionalConfig& cfg) const noexcept {
    // Check light length ratio
    const float light_length_ratio = (light_1.length < light_2.length)
                                       ? static_cast<float>(light_1.length / light_2.length)
                                       : static_cast<float>(light_2.length / light_1.length);
    if (light_length_ratio <= cfg.armor_min_light_ratio) {
        return ArmorType::Invalid;
    }

    // Check center distance (normalized by light length)
    const float avg_light_length = static_cast<float>((light_1.length + light_2.length) / 2.0);
    const float center_distance =
        static_cast<float>(cv::norm(light_1.center - light_2.center) / avg_light_length);
    const auto center_distance_ok = (cfg.armor_min_small_center_distance <= center_distance
                                     && cfg.armor_max_small_center_distance > center_distance)
                                 || (cfg.armor_min_large_center_distance <= center_distance
                                     && cfg.armor_max_large_center_distance > center_distance);
    if (!center_distance_ok) {
        return ArmorType::Invalid;
    }

    // Check horizontal angle
    const cv::Point2f diff = light_1.center - light_2.center;
    const float angle =
        static_cast<float>(std::abs(std::atan(diff.y / diff.x)) / std::numbers::pi * 180.0);
    if (angle >= cfg.armor_max_angle) {
        return ArmorType::Invalid;
    }

    return center_distance > cfg.armor_min_large_center_distance ? ArmorType::Large
                                                                 : ArmorType::Small;
}

// ============================================================================
// Number Extraction (Perspective Transform + OTSU Binarization)
// ============================================================================

cv::Mat TraditionalBackend::extract_number(
    const cv::Mat& src, const std::array<cv::Point2f, 4>& lights_vertices,
    ArmorType armor_type) const noexcept {
    // Light length in image
    static const int light_length = 12;
    // Image size after warp
    static const int warp_height       = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // Number ROI size
    static const cv::Size roi_size(20, 28);
    // Classifier input size
    static const cv::Size input_size(28, 28);

    const int top_light_y    = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor_type == ArmorType::Small ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    cv::Point2f lights_vertices2[4] = {
        lights_vertices[3], lights_vertices[0], lights_vertices[1], lights_vertices[2]};

    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices2, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    number_image =
        number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Convert to grayscale and binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_BGR2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Output is already 20x28, no resize needed
    return number_image;
}

} // namespace fcs::L2
