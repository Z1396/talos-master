// ===========================================================================
// compat/opencv2/core/types.hpp —— OpenCV 最小替身（仅本 stage 使用）
// ---------------------------------------------------------------------------
// crates/fast_tf/src/matrix.hpp include 了 <opencv2/core/types.hpp>，
// 但实际只用了 cv::Vec<T, 3>（from_pnp 的 rvec/tvec 参数类型，
// OpenCV PnP 解算的标准输出格式）。
// 为避免在教学环境拉取/编译整套 OpenCV（几百 MB），这里提供只含
// cv::Vec 的最小实现，API 与 OpenCV 语义一致（固定长度数组 + operator[]）。
// 若机器上装有真实 OpenCV，把本目录从 include 路径中移除即可无缝切回。
// ===========================================================================
#pragma once

#include <cstddef>

namespace cv {

/// 固定长度数值向量（对齐 OpenCV cv::Vec<_Tp, n> 的最小子集）
template <typename _Tp, int n>
struct Vec {
    _Tp val[n]{};

    _Tp operator[](int i) const { return val[i]; }
    _Tp& operator[](int i) { return val[i]; }

    static constexpr int channels = n;
};

} // namespace cv
