#pragma once

#include <array>
#include <cmath>

#include <opencv2/core.hpp>

namespace fcs::at_legacy::detail {

/// Calculate triangle area.
[[nodiscard]] inline float
    tri_area(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) noexcept {
    return std::fabs(0.5f * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)));
}

/// Calculate quadrilateral area from 4 points.
[[nodiscard]] inline float quad_area(const std::array<cv::Point2f, 4>& p) noexcept {
    return tri_area(p[0], p[1], p[2]) + tri_area(p[0], p[2], p[3]);
}

/// Calculate IoU between two rectangles.
[[nodiscard]] inline float iou(const cv::Rect2f& a, const cv::Rect2f& b) noexcept {
    const float inter = (a & b).area();
    const float uni   = a.area() + b.area() - inter;
    return uni > 0 ? inter / uni : 0.0f;
}

} // namespace fcs::at_legacy::detail
