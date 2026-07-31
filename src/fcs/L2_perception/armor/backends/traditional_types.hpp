/**
 * @file traditional_types.hpp
 * @brief 传统装甲板检测相关的数据类型定义
 *
 * 本文件定义了传统检测算法所需的核心数据结构，主要包括：
 * - Light：灯条几何描述，用于表示检测到的单个灯条
 * - make_detection_from_lights：从灯条对构造装甲板检测结果的辅助函数
 *
 * ## Light 结构体设计原理
 *
 * Light 继承自 cv::RotatedRect（旋转矩形），并扩展了额外的几何信息：
 *
 * **基础几何属性（继承自 RotatedRect）：**
 * - center：旋转矩形的中心点
 * - size：矩形的宽度和高度（width, height）
 * - angle：旋转角度（度数）
 *
 * **扩展几何属性（用于灯条匹配）：**
 * - top：灯条顶部中心点（用于角点定位）
 * - bottom：灯条底部中心点（用于角点定位）
 * - axis：灯条轴线方向向量（单位向量，top → bottom）
 * - length：灯条长度（top 到 bottom 的距离）
 * - width：灯条宽度（短边长度）
 * - tilt_angle：灯条倾斜角度（相对垂直方向的偏离角度）
 * - color：灯条颜色（红色/蓝色/灰色）
 *
 * ## 几何计算原理
 *
 * ### 从轮廓构造 Light
 *
 * 1. **最小外接矩形**：使用 cv::minAreaRect() 拟合轮廓，得到旋转矩形
 * 2. **中心点计算**：通过轮廓点平均值计算更精确的中心点
 * 3. **顶点排序**：获取旋转矩形的4个顶点，按 Y 坐标排序
 * 4. **端点计算**：
 *    - top = 上方两个顶点的中点
 *    - bottom = 下方两个顶点的中点
 * 5. **长度与宽度**：
 *    - length = distance(top, bottom)
 *    - width = distance(上方两顶点)
 * 6. **轴线方向**：
 *    - axis = normalize(bottom - top)
 *    - 用于后续的平行性判断
 * 7. **倾斜角度**：
 *    - tilt_angle = atan2(|Δx|, |Δy|)
 *    - 衡量灯条相对垂直方向的偏离程度
 *
 * ### 从顶点对构造 Light
 *
 * 当已知灯条的 top 和 bottom 端点时，直接构造：
 * - 确保 top.y < bottom.y（坐标系约定）
 * - center = (top + bottom) / 2
 * - axis = normalize(bottom - top)
 * - length = distance(top, bottom)
 * - width 默认为 1.0（未知时）
 *
 * ## 灯条匹配中的应用
 *
 * - **平行性判断**：两灯条的 axis 夹角接近 0° 或 180°
 * - **长度比判断**：length_1 / length_2 接近 1
 * - **间距计算**：两灯条 center 的距离
 * - **角点提取**：使用 top 和 bottom 构造装甲板四角
 *
 * @author Talos Team
 * @date 2024
 */

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

/**
 * @struct Light
 * @brief 灯条结构体：表示检测到的单个灯条及其几何信息
 *
 * Light 是传统检测算法的核心数据结构，用于：
 * - 存储灯条的几何属性（位置、尺寸、方向）
 * - 支持灯条匹配和装甲板构造
 * - 提供几何约束判断的基础信息
 *
 * 继承自 cv::RotatedRect，利用 OpenCV 的旋转矩形功能，
 * 并扩展了 top、bottom、axis 等用于匹配的属性。
 */
struct Light : public cv::RotatedRect {
    /// 默认构造函数：创建未初始化的灯条
    Light() = default;

    /**
     * @brief 从轮廓构造灯条对象
     *
     * 该构造函数是传统检测流程的核心步骤之一，将检测到的轮廓转换为几何描述。
     *
     * **构造流程：**
     *
     * 1. **最小外接矩形拟合**
     *    使用 cv::minAreaRect() 找到覆盖轮廓的最小旋转矩形。
     *    这提供了初始的 center、size、angle 信息。
     *
     * 2. **精确中心点计算**
     *    不直接使用 RotatedRect::center，而是通过轮廓点平均值计算：
     *    ```
     *    center = Σ(contour[i]) / N
     *    ```
     *    这样可以获得更鲁棒的定位，减少噪声影响。
     *
     * 3. **顶点提取与排序**
     *    - 调用 RotatedRect::points(p) 获取4个顶点
     *    - 按 Y 坐标排序：p[0]、p[1] 是上方两点，p[2]、p[3] 是下方两点
     *
     * 4. **端点计算**
     *    - top = (p[0] + p[1]) / 2  // 上方两点的中点
     *    - bottom = (p[2] + p[3]) / 2 // 下方两点的中点
     *
     *    这样处理的好处：
     *    - 鲁棒性：即使旋转矩形拟合有误差，端点仍然相对稳定
     *    - 对称性：灯条上下对称，取中点更合理
     *
     * 5. **长度与宽度计算**
     *    - length = distance(top, bottom)  // 灯条长度（长轴）
     *    - width = distance(p[0], p[1])    // 灯条宽度（短轴）
     *
     * 6. **轴线方向向量**
     *    ```
     *    axis = normalize(top - bottom)
     *    ```
     *    - 单位向量，方向从 top 指向 bottom
     *    - 用于平行性判断：两灯条的 axis 夹角
     *
     * 7. **倾斜角度计算**
     *    ```
     *    tilt_angle = atan2(|top.x - bottom.x|, |top.y - bottom.y|) * 180 / π
     *    ```
     *    - 衡量灯条相对垂直方向的偏离程度
     *    - 0° 表示完全垂直，90° 表示完全水平
     *    - 用于过滤过于倾斜的灯条（物理约束）
     *
     * @param contour 轮廓点集（来自 cv::findContours）
     *
     * @note 如果轮廓为空，构造的 Light 处于未初始化状态
     * @note color 默认为 ArmorColor::Neutral，需要后续设置
     */
    explicit Light(const std::vector<cv::Point>& contour) noexcept
        : cv::RotatedRect(cv::minAreaRect(contour))
        , color(ArmorColor::Neutral) {
        if (contour.empty())
            return;

        // 计算轮廓中心点（更鲁棒的方法）
        // 使用 std::accumulate 累加所有点坐标，然后除以点数
        center = std::accumulate(
            contour.begin(), contour.end(), cv::Point2f(0, 0),
            [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                return a + cv::Point2f(static_cast<float>(b.x) / n, static_cast<float>(b.y) / n);
            });

        // 获取旋转矩形的4个顶点
        cv::Point2f p[4];
        this->points(p);

        // 按 Y 坐标排序（升序），使 p[0]、p[1] 为上方两点
        std::ranges::sort(
            p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

        // 计算灯条端点：上方两点中点为 top，下方两点中点为 bottom
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        // 计算灯条长度（长轴）和宽度（短轴）
        length = cv::norm(top - bottom);
        width  = cv::norm(p[0] - p[1]);

        // 计算轴线方向向量（单位向量，从 top 指向 bottom）
        axis = top - bottom;
        if (cv::norm(axis) > 1e-6) {
            axis = axis / static_cast<float>(cv::norm(axis));
        }

        // 计算倾斜角度（相对垂直方向的偏离角度）
        // 使用 atan2(|Δx|, |Δy|) 而非 atan2(Δy, Δx)，确保角度范围在 [0°, 90°]
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / std::numbers::pi * 180.0);
    }

    /**
     * @brief 从顶点对构造灯条对象
     *
     * 当已知灯条的 top 和 bottom 端点时（例如从其他检测结果转换），
     * 可以直接构造 Light 对象，无需轮廓拟合。
     *
     * **构造流程：**
     *
     * 1. **端点排序**：确保 top.y < bottom.y（图像坐标系约定）
     * 2. **中心点计算**：center = (top + bottom) / 2
     * 3. **长度计算**：length = distance(top, bottom)
     * 4. **轴线方向**：axis = normalize(bottom - top)
     * 5. **倾斜角度**：同上
     * 6. **设置 RotatedRect 属性**：
     *    - size = (width, length)
     *    - angle = 从 top 到 bottom 的角度（标准格式）
     *
     * @param top_pt 灯条顶部端点
     * @param bottom_pt 灯条底部端点
     *
     * @note width 默认设为 1.0（未知时）
     * @note color 默认为 ArmorColor::Neutral
     */
    explicit Light(const cv::Point2f& top_pt, const cv::Point2f& bottom_pt) noexcept
        : cv::RotatedRect()
        , color(ArmorColor::Neutral) {
        // 确保 top.y < bottom.y（图像坐标系，Y 轴向下）
        if (top_pt.y < bottom_pt.y) {
            top    = top_pt;
            bottom = bottom_pt;
        } else {
            top    = bottom_pt;
            bottom = top_pt;
        }

        // 计算中心点
        center = (top + bottom) * 0.5f;

        // 计算长度，避免除零
        length = cv::norm(top - bottom);
        if (length <= 1e-6) {
            length = 1e-6;
        }
        width = 1.0; // 默认宽度（未知时）

        // 计算轴线方向向量（单位向量）
        axis      = bottom - top;
        double an = cv::norm(axis);
        if (an > 1e-6) {
            axis = axis / static_cast<float>(an);
        } else {
            axis = cv::Point2f(0.f, 1.f); // 默认垂直方向
        }

        // 计算倾斜角度
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / std::numbers::pi * 180.0);

        // 设置 RotatedRect 属性
        // 角度：从 top 到 bottom 的角度，使用标准数学坐标系（X 轴向右，Y 轴向上）
        // OpenCV 使用图像坐标系（Y 轴向下），因此需要转换
        float angle_deg = static_cast<float>(
            std::atan2(bottom.y - top.y, bottom.x - top.x) * 180.0 / std::numbers::pi);
        this->size  = cv::Size2f(static_cast<float>(width), static_cast<float>(length));
        this->angle = angle_deg;
    }

    // ============================================================================
    // 灯条属性成员
    // ============================================================================

    /// 灯条颜色（红色/蓝色/灰色），需要在检测后设置
    ArmorColor color = ArmorColor::Neutral;

    /// 灯条顶部中心点（用于构造装甲板角点）
    cv::Point2f top{0, 0};

    /// 灯条底部中心点（用于构造装甲板角点）
    cv::Point2f bottom{0, 0};

    /// 灯条中心点（更鲁棒的中心定位）
    cv::Point2f center{0, 0};

    /// 轴线方向向量（单位向量，从 top 指向 bottom）
    /// 用于平行性判断：两灯条的 axis 夹角接近 0° 或 180°
    cv::Point2f axis{0, 1};

    /// 灯条长度（长轴，top 到 bottom 的距离）
    double length = 0;

    /// 灯条宽度（短轴）
    double width = 0;

    /// 灯条倾斜角度（相对垂直方向的偏离角度，单位：度）
    /// 0° 表示完全垂直，90° 表示完全水平
    /// 用于过滤过于倾斜的灯条（物理约束）
    float tilt_angle = 0;
};

// ============================================================================
// AT Legacy Armor from Lights (for Traditional Backend)
// ============================================================================

/**
 * @brief 从两个灯条构造装甲板检测结果
 *
 * 该函数是传统检测流程的最后一步，将匹配的两灯条转换为 ArmorDetection 对象。
 *
 * **构造流程：**
 *
 * 1. **角点提取**
 *    装甲板的四个角点由两灯条的 top 和 bottom 端点构成：
 *    - corners[0] = left_light.top    // 左上角 (TL)
 *    - corners[1] = right_light.top   // 右上角 (TR)
 *    - corners[2] = right_light.bottom // 右下角 (BR)
 *    - corners[3] = left_light.bottom  // 左下角 (BL)
 *
 *    这种顺序符合装甲板检测的统一约定，便于后续 PnP 解算。
 *
 * 2. **包围盒计算**
 *    使用 cv::boundingRect() 计算四个角点的外包围盒：
 *    - 用于快速判断装甲板是否在图像范围内
 *    - 用于 ROI 提取
 *
 * 3. **面积计算**
 *    使用 cv::contourArea() 计算四边形面积：
 *    - 用于置信度评估
 *    - 用于目标优先级排序
 *
 * 4. **颜色设置**
 *    从灯条颜色推断装甲板颜色：
 *    - 如果任一灯条是蓝色，装甲板为蓝色
 *    - 如果任一灯条是红色，装甲板为红色
 *    - 否则为灰色（Neutral）
 *
 *    这种逻辑确保即使单个灯条颜色识别错误，装甲板颜色仍可能正确。
 *
 * 5. **默认值设置**
 *    - name = ArmorName::Invalid（等待分类器识别）
 *    - type = ArmorType::Small（等待几何判断）
 *
 * @param left_light 左侧灯条
 * @param right_light 右侧灯条
 * @return 构造的 ArmorDetection 对象
 *
 * @note 该函数假设两灯条已通过匹配验证（平行性、间距等）
 * @note name 和 type 需要在后续步骤中更新
 */
[[nodiscard]] inline ArmorDetection
    make_detection_from_lights(const Light& left_light, const Light& right_light) noexcept {
    ArmorDetection det;

    // 角点顺序：TL, TR, BR, BL（左上、右上、右下、左下）
    // 这种顺序符合 PnP 解算的统一约定
    det.corners[0] = left_light.top;
    det.corners[1] = right_light.top;
    det.corners[2] = right_light.bottom;
    det.corners[3] = left_light.bottom;

    // 计算包围盒（用于 ROI 提取和边界检查）
    std::vector<cv::Point2f> pts = {det.corners[0], det.corners[1], det.corners[2], det.corners[3]};
    det.rect                     = cv::boundingRect(pts);

    // 计算面积（用于置信度评估）
    det.area = static_cast<int>(cv::contourArea(pts));

    // 设置颜色（从灯条颜色推断）
    // 使用 OR 逻辑，确保只要有一个灯条颜色正确即可
    if (left_light.color == ArmorColor::Blue || right_light.color == ArmorColor::Blue) {
        det.color = ArmorColor::Blue;
    } else if (left_light.color == ArmorColor::Red || right_light.color == ArmorColor::Red) {
        det.color = ArmorColor::Red;
    } else {
        det.color = ArmorColor::Neutral;
    }

    // name 和 type 将在后续步骤中设置
    // name: 由分类器识别（1-5, 前哨站, 基地）
    // type: 由几何特征判断（大装甲板/小装甲板）
    det.name = ArmorName::Invalid;
    det.type = ArmorType::Small; // 默认值，等待更新

    return det;
}

} // namespace fcs::L2