/**
 * @file axera_preprocess.hpp
 * @brief Axera 后端预处理辅助函数（letterbox 模式）
 *
 * 本文件提供 letterbox 预处理相关的辅助函数，用于非 4:3 宽高比输入的场景。
 *
 * Letterbox 原理：
 * - 保持原始宽高比，将图像缩放到目标尺寸内
 * - 用背景色填充空白区域（上下或左右）
 * - 避免图像畸变
 *
 * 与 AxeraBackend 的关系：
 * - AxeraBackend 默认使用直接 resize（STRETCH 模式）
 * - 本文件提供 letterbox 模式的备选实现（未使用，但保留用于未来扩展）
 *
 * 使用场景：
 * - 非 4:3 宽高比输入，需要保持宽高比
 * - 对畸变敏感的检测任务
 */

#pragma once

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace fcs::L2 {

/**
 * @struct AxeraLetterboxGeometry
 * @brief Letterbox 变换几何参数
 *
 * 记录从原始图像到模型输入尺寸的 letterbox 变换参数，
 * 用于坐标映射和可视化。
 */
struct AxeraLetterboxGeometry {
    float ratio   = 1.0f; ///< 缩放比例（取 min(ratio_w, ratio_h)）
    int orig_w    = 0;    ///< 原始图像宽度
    int orig_h    = 0;    ///< 原始图像高度
    int resized_w = 0;    ///< resize 后宽度（不含 padding）
    int resized_h = 0;    ///< resize 后高度（不含 padding）
    int left      = 0;    ///< 左侧 padding 宽度
    int top       = 0;    ///< 顶部 padding 高度
    int right     = 0;    ///< 右侧 padding 宽度
    int bottom    = 0;    ///< 底部 padding 高度
};

/**
 * @brief 计算 letterbox 变换几何参数
 *
 * 算法：
 * 1. 计算宽度和高度方向的缩放比例（ratio_w, ratio_h）
 * 2. 取较小值作为最终缩放比例（保持宽高比）
 * 3. 计算 resize 后尺寸（resized_w, resized_h）
 * 4. 计算 padding 宽度和高度
 * 5. 分配 padding 到左右和上下（居中对齐）
 *
 * 示例：
 * - 输入：1280x720，目标：768x576
 * - ratio_w = 768/1280 = 0.6, ratio_h = 576/720 = 0.8
 * - ratio = min(0.6, 0.8) = 0.6
 * - resized_w = 1280 * 0.6 = 768, resized_h = 720 * 0.6 = 432
 * - pad_w = 0, pad_h = 576 - 432 = 144
 * - left = 0, right = 0, top = 72, bottom = 72
 *
 * @param orig_w 原始图像宽度
 * @param orig_h 原始图像高度
 * @param input_w 目标输入宽度
 * @param input_h 目标输入高度
 * @return Letterbox 几何参数结构
 */
[[nodiscard]] inline AxeraLetterboxGeometry
    compute_axera_letterbox_geometry(int orig_w, int orig_h, int input_w, int input_h) {
    AxeraLetterboxGeometry geometry;
    geometry.orig_w = orig_w;
    geometry.orig_h = orig_h;

    // 计算缩放比例（保持宽高比）
    const float ratio_w = static_cast<float>(input_w) / static_cast<float>(orig_w);
    const float ratio_h = static_cast<float>(input_h) / static_cast<float>(orig_h);
    geometry.ratio      = std::min(ratio_w, ratio_h);

    // 计算 resize 后尺寸
    geometry.resized_w = static_cast<int>(std::round(static_cast<float>(orig_w) * geometry.ratio));
    geometry.resized_h = static_cast<int>(std::round(static_cast<float>(orig_h) * geometry.ratio));

    // 计算 padding（居中对齐）
    const int pad_w = input_w - geometry.resized_w;
    const int pad_h = input_h - geometry.resized_h;
    geometry.left   = pad_w / 2;
    geometry.right  = pad_w - geometry.left;
    geometry.top    = pad_h / 2;
    geometry.bottom = pad_h - geometry.top;

    return geometry;
}

/**
 * @brief 选择 resize 插值方法
 *
 * 根据缩放方向选择最优插值方法：
 * - 下采样（缩小）：使用 INTER_AREA（抗锯齿）
 * - 上采样（放大）：使用 INTER_LINEAR（平滑）
 *
 * @param geometry Letterbox 几何参数
 * @return OpenCV 插值方法常量
 */
[[nodiscard]] inline int
    select_axera_resize_interpolation(const AxeraLetterboxGeometry& geometry) noexcept {
    const bool is_downsampling =
        geometry.resized_w < geometry.orig_w || geometry.resized_h < geometry.orig_h;
    return is_downsampling ? cv::INTER_AREA : cv::INTER_LINEAR;
}

/**
 * @brief 执行 letterbox 预处理（BGR -> RGB）
 *
 * 算法流程：
 * 1. 填充目标缓冲区为背景色（默认灰色 114）
 * 2. 计算有效 ROI 区域（去除 padding）
 * 3. 如果尺寸匹配，直接 BGR2RGB 转换
 * 4. 否则，先 resize 再 BGR2RGB 转换
 *
 * 注意事项：
 * - 输入图像：BGR 格式（OpenCV 默认）
 * - 输出图像：RGB 格式（模型输入）
 * - resize_scratch_bgr：临时缓冲区，避免重复分配
 *
 * @param image 输入图像（BGR 格式）
 * @param geometry Letterbox 几何参数
 * @param dst_rgb 输出缓冲区（RGB 格式，已分配）
 * @param resize_scratch_bgr resize 临时缓冲区（BGR 格式）
 * @param pad_color padding 背景色（默认灰色 114）
 */
inline void preprocess_axera_letterbox_to_rgb(
    const cv::Mat& image, const AxeraLetterboxGeometry& geometry, cv::Mat& dst_rgb,
    cv::Mat& resize_scratch_bgr, const cv::Scalar& pad_color = cv::Scalar(114, 114, 114)) {
    // 填充背景色
    dst_rgb.setTo(pad_color);

    // 计算 ROI 区域
    cv::Rect roi(geometry.left, geometry.top, geometry.resized_w, geometry.resized_h);
    cv::Mat roi_rgb = dst_rgb(roi);

    // 如果尺寸匹配，直接 BGR2RGB
    if (image.cols == geometry.resized_w && image.rows == geometry.resized_h) {
        cv::cvtColor(image, roi_rgb, cv::COLOR_BGR2RGB);
        return;
    }

    // 否则，先 resize 再 BGR2RGB
    cv::resize(
        image, resize_scratch_bgr, cv::Size(geometry.resized_w, geometry.resized_h), 0, 0,
        select_axera_resize_interpolation(geometry));
    cv::cvtColor(resize_scratch_bgr, roi_rgb, cv::COLOR_BGR2RGB);
}

} // namespace fcs::L2
