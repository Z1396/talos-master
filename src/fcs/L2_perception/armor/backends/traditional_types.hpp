#pragma once

#include "core/types.hpp"

#include <algorithm>
#include <numbers>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ranges>
#include <vector>

namespace fcs::L2 {

// ============================================================================
// AT Legacy Light Structure (for Traditional Backend)
// ============================================================================

/// Light bar detected in image (灯条)
struct Light : public cv::RotatedRect {
    Light() = default;

    /// Construct from contour
    explicit Light(const std::vector<cv::Point>& contour) noexcept
        : cv::RotatedRect(cv::minAreaRect(contour))
        , color(ArmorColor::Neutral) {
        if (contour.empty())
            return;

        // Calculate center from contour points
        center = std::accumulate(
            contour.begin(), contour.end(), cv::Point2f(0, 0),
            [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                return a + cv::Point2f(static_cast<float>(b.x) / n, static_cast<float>(b.y) / n);
            });

        // Get corner points and sort by y
        cv::Point2f p[4];
        this->points(p);
        std::ranges::sort(
            p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

        // Top is average of two topmost points, bottom is average of two bottommost
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width  = cv::norm(p[0] - p[1]);

        // Calculate axis (normalized direction vector)
        axis = top - bottom;
        if (cv::norm(axis) > 1e-6) {
            axis = axis / static_cast<float>(cv::norm(axis));
        }

        // Calculate tilt angle in degrees
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / std::numbers::pi * 180.0);
    }

    /// Construct from top and bottom points
    explicit Light(const cv::Point2f& top_pt, const cv::Point2f& bottom_pt) noexcept
        : cv::RotatedRect()
        , color(ArmorColor::Neutral) {
        // Ensure top.y < bottom.y
        if (top_pt.y < bottom_pt.y) {
            top    = top_pt;
            bottom = bottom_pt;
        } else {
            top    = bottom_pt;
            bottom = top_pt;
        }
        center = (top + bottom) * 0.5f;

        length = cv::norm(top - bottom);
        if (length <= 1e-6) {
            length = 1e-6;
        }
        width = 1.0;

        // Calculate axis
        axis      = bottom - top;
        double an = cv::norm(axis);
        if (an > 1e-6) {
            axis = axis / static_cast<float>(an);
        } else {
            axis = cv::Point2f(0.f, 1.f);
        }

        // Calculate tilt angle
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / std::numbers::pi * 180.0);

        // Set RotatedRect properties
        float angle_deg = static_cast<float>(
            std::atan2(bottom.y - top.y, bottom.x - top.x) * 180.0 / std::numbers::pi);
        this->size  = cv::Size2f(static_cast<float>(width), static_cast<float>(length));
        this->angle = angle_deg;
    }

    ArmorColor color = ArmorColor::Neutral;
    cv::Point2f top{0, 0};
    cv::Point2f bottom{0, 0};
    cv::Point2f center{0, 0};
    cv::Point2f axis{0, 1}; // Normalized direction vector (top to bottom)
    double length    = 0;
    double width     = 0;
    float tilt_angle = 0;   // Tilt angle in degrees
};

// ============================================================================
// AT Legacy Armor from Lights (for Traditional Backend)
// ============================================================================

/// Construct ArmorDetection from two lights
[[nodiscard]] inline ArmorDetection
    make_detection_from_lights(const Light& left_light, const Light& right_light) noexcept {
    ArmorDetection det;

    // Order: TL, TR, BR, BL (left-top, right-top, right-bottom, left-bottom)
    det.corners[0] = left_light.top;
    det.corners[1] = right_light.top;
    det.corners[2] = right_light.bottom;
    det.corners[3] = left_light.bottom;

    // Calculate rect
    std::vector<cv::Point2f> pts = {det.corners[0], det.corners[1], det.corners[2], det.corners[3]};
    det.rect                     = cv::boundingRect(pts);
    det.area                     = static_cast<int>(cv::contourArea(pts));

    // Set color from lights
    if (left_light.color == ArmorColor::Blue || right_light.color == ArmorColor::Blue) {
        det.color = ArmorColor::Blue;
    } else if (left_light.color == ArmorColor::Red || right_light.color == ArmorColor::Red) {
        det.color = ArmorColor::Red;
    } else {
        det.color = ArmorColor::Neutral;
    }

    // Name and type will be set by number classifier later
    det.name = ArmorName::Invalid;
    det.type = ArmorType::Small; // Default, will be updated based on cls

    return det;
}

} // namespace fcs::L2
