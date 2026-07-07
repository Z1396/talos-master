#pragma once
// 定长数组 std::array，存储四边形4个角点
#include <array>
// 数学函数 fabs 浮点绝对值
#include <cmath>

// OpenCV 二维浮点点、浮点矩形基础类型
#include <opencv2/core.hpp>

namespace fcs::at_legacy::detail {

/**
 * @brief 计算三个二维点构成三角形的面积（浮点）
 * 向量叉积法求面积，无分支，计算高效
 * @param a 三角形顶点A
 * @param b 三角形顶点B
 * @param c 三角形顶点C
 * @return 三角形面积（恒为非负数）
 */
[[nodiscard]] inline float
    tri_area(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) noexcept {
    // 叉积公式：S = 0.5 * | (B-A) × (C-A) |
    // (b.x - a.x) 向量AB x分量
    // (c.y - a.y) 向量AC y分量
    // (b.y - a.y) 向量AB y分量
    // (c.x - a.x) 向量AC x分量
    // 叉积 = AB.x * AC.y - AB.y * AC.x
    // fabs 取绝对值保证面积为正，乘以0.5得到真实三角形面积
    return std::fabs(0.5f * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)));
}

/**
 * @brief 四点四边形面积计算（拆分两个三角形求和）
 * 四边形顶点顺序：p0-p1-p2-p3 顺时针/逆时针闭环
 * 拆分：△p0p1p2 + △p0p2p3，两者相加得到四边形总面积
 * @param p 长度为4的定点数组，存储四边形四个角点
 * @return 四边形浮点面积
 */
[[nodiscard]] inline float quad_area(const std::array<cv::Point2f, 4>& p) noexcept {
    return tri_area(p[0], p[1], p[2]) + tri_area(p[0], p[2], p[3]);
}

/**
 * @brief 计算两个浮点矩形的IoU交并比
 * IoU = 交集面积 / 并集面积，目标检测/匹配阈值判断通用指标
 * @param a 矩形A
 * @param b 矩形B
 * @return IoU [0,1]，无交集返回0，完全重合返回1
 */
[[nodiscard]] inline float iou(const cv::Rect2f& a, const cv::Rect2f& b) noexcept {
    // cv::Rect2f 重载&运算符，直接得到相交矩形
    const float inter = (a & b).area();
    // 并集 = A面积 + B面积 - 交集面积（防止重复计算重叠区域）
    const float uni   = a.area() + b.area() - inter;
    // 并集为0说明两个矩形都无面积，直接返回0避免除零NaN
    return uni > 0 ? inter / uni : 0.0f;
}

} // namespace fcs::at_legacy::detail