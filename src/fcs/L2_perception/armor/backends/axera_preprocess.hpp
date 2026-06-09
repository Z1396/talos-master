#pragma once

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace fcs::L2 {

struct AxeraLetterboxGeometry {
    float ratio   = 1.0f;
    int orig_w    = 0;
    int orig_h    = 0;
    int resized_w = 0;
    int resized_h = 0;
    int left      = 0;
    int top       = 0;
    int right     = 0;
    int bottom    = 0;
};

[[nodiscard]] inline AxeraLetterboxGeometry
    compute_axera_letterbox_geometry(int orig_w, int orig_h, int input_w, int input_h) {
    AxeraLetterboxGeometry geometry;
    geometry.orig_w = orig_w;
    geometry.orig_h = orig_h;

    const float ratio_w = static_cast<float>(input_w) / static_cast<float>(orig_w);
    const float ratio_h = static_cast<float>(input_h) / static_cast<float>(orig_h);
    geometry.ratio      = std::min(ratio_w, ratio_h);

    geometry.resized_w = static_cast<int>(std::round(static_cast<float>(orig_w) * geometry.ratio));
    geometry.resized_h = static_cast<int>(std::round(static_cast<float>(orig_h) * geometry.ratio));

    const int pad_w = input_w - geometry.resized_w;
    const int pad_h = input_h - geometry.resized_h;
    geometry.left   = pad_w / 2;
    geometry.right  = pad_w - geometry.left;
    geometry.top    = pad_h / 2;
    geometry.bottom = pad_h - geometry.top;
    return geometry;
}

[[nodiscard]] inline int
    select_axera_resize_interpolation(const AxeraLetterboxGeometry& geometry) noexcept {
    const bool is_downsampling =
        geometry.resized_w < geometry.orig_w || geometry.resized_h < geometry.orig_h;
    return is_downsampling ? cv::INTER_AREA : cv::INTER_LINEAR;
}

inline void preprocess_axera_letterbox_to_rgb(
    const cv::Mat& image, const AxeraLetterboxGeometry& geometry, cv::Mat& dst_rgb,
    cv::Mat& resize_scratch_bgr, const cv::Scalar& pad_color = cv::Scalar(114, 114, 114)) {
    dst_rgb.setTo(pad_color);

    cv::Rect roi(geometry.left, geometry.top, geometry.resized_w, geometry.resized_h);
    cv::Mat roi_rgb = dst_rgb(roi);

    if (image.cols == geometry.resized_w && image.rows == geometry.resized_h) {
        cv::cvtColor(image, roi_rgb, cv::COLOR_BGR2RGB);
        return;
    }

    cv::resize(
        image, resize_scratch_bgr, cv::Size(geometry.resized_w, geometry.resized_h), 0, 0,
        select_axera_resize_interpolation(geometry));
    cv::cvtColor(resize_scratch_bgr, roi_rgb, cv::COLOR_BGR2RGB);
}

} // namespace fcs::L2
