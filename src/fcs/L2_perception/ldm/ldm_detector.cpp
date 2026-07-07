#include "L2_perception/ldm/ldm_detector.hpp"

// 大符几何工具、三维模型定义
#include "L2_perception/ldm/ldm_geometry.hpp"
// 装甲颜色、Blob、灯对、候选网格、检测结果结构体
#include "core/armor_types.hpp"

// 标准算法容器、数学、固定宽度整数、数值极值、常数、累加器
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
// OpenCV图像处理：轮廓、滤波、颜色空间转换、基础几何
#include <opencv2/imgproc.hpp>
// 日志打印
#include <spdlog/spdlog.h>

namespace fcs::L2::ldm {

namespace { // 内部匿名命名空间，仅本编译单元可见工具函数

/**
 * @brief 获取当前候选网格最低合格分数阈值
 * 规则：<=2组灯对（半符）要求更高打分阈值，>=3组放宽
 * @param candidate 待校验大符网格候选
 * @param config 检测器全局配置
 * @return 最低合格分数float
 */
[[nodiscard]] float
    min_candidate_score(const LdmMeshCandidate& candidate, const LdmDetectorConfig& config) {
    return static_cast<float>(
        (candidate.pair_indices.size() <= 2) ? config.min_preliminary_candidate_score_two_pair
                                             : config.min_preliminary_candidate_score);
}

/**
 * @brief 获取候选所属聚类内总灯对数量
 * 若候选无聚类ID，直接返回自身携带灯对数量；有聚类则统计同cluster全部灯对
 * @param candidate 网格候选
 * @param pairs 全局全部灯对数组
 * @return 同聚类灯对总数
 */
[[nodiscard]] size_t candidate_cluster_pair_count(
    const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) {
    // 无聚类标记，直接使用自身灯对数量
    if (candidate.cluster_id < 0) {
        return candidate.pair_indices.size();
    }
    // 统计所有同cluster_id的灯对
    return static_cast<size_t>(
        std::count_if(pairs.begin(), pairs.end(), [&](const LightPair& pair) {
            return pair.cluster_id == candidate.cluster_id;
        }));
}

/**
 * @brief 校验孤立双灯对横向跨度是否满足最低比例约束
 * 仅当候选恰好2组灯对、聚类内仅2组灯对时生效，过滤狭窄并排残缺灯条
 * @param candidate 网格候选
 * @param pairs 全局灯对数组
 * @param config 检测器配置
 * @return true 满足跨度要求，false 跨度不足丢弃
 */
[[nodiscard]] bool has_min_isolated_two_pair_order_span(
    const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
    const LdmDetectorConfig& config) {
    // 灯对数量不是2，无需校验直接放行
    if (candidate.pair_indices.size() != 2u) {
        return true;
    }
    // 聚类内总灯对不是2，说明存在其他配对，无需校验
    if (candidate_cluster_pair_count(candidate, pairs) != 2u) {
        return true;
    }
    // 配置参数非法直接拦截
    if (!std::isfinite(config.min_isolated_two_pair_order_span_ratio)
        || config.min_isolated_two_pair_order_span_ratio < 0.0) {
        return false;
    }

    // 计算两组灯对纵向层分隔均值
    float mean_pair_layer = 0.0f;
    for (const int pair_idx : candidate.pair_indices) {
        // 索引越界直接判定非法
        if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
            return false;
        }
        mean_pair_layer += pair_layer_separation_px(pairs[static_cast<size_t>(pair_idx)]);
    }
    mean_pair_layer /= static_cast<float>(candidate.pair_indices.size());
    // 层间距过小，数值退化直接拦截
    if (mean_pair_layer <= 1e-3f) {
        return false;
    }

    // 获取两组灯对前后横向坐标差值
    const auto first_pair_idx = static_cast<size_t>(candidate.pair_indices.front());
    const auto last_pair_idx  = static_cast<size_t>(candidate.pair_indices.back());
    const float order_span =
        std::abs(pairs[last_pair_idx].local_order_px - pairs[first_pair_idx].local_order_px);
    // 横向跨度 / 纵向层间距 >= 阈值才合法
    return order_span / mean_pair_layer
        >= static_cast<float>(config.min_isolated_two_pair_order_span_ratio);
}

/**
 * @brief 候选整体检测门限校验，综合多重过滤条件
 * 1. 最少灯对数量校验
 * 2. 基础打分阈值校验
 * 3. 孤立双灯对跨度校验
 * 4. 双灯对竖直高度兜底打分提升校验
 * @param candidate 待过滤网格候选
 * @param pairs 全局灯对数组
 * @param config 检测器配置
 * @return true 通过所有门限，保留候选；false 丢弃
 */
[[nodiscard]] bool candidate_passes_detection_gate(
    const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
    const LdmDetectorConfig& config) {
    // 校验1：灯对数量不满足最低要求
    if (static_cast<int>(candidate.pair_indices.size()) < config.min_pairs_for_detection) {
        return false;
    }
    // 校验2：基础打分低于最低阈值
    if (candidate.preliminary_score < min_candidate_score(candidate, config)) {
        return false;
    }
    // 校验3：孤立双灯对横向跨度不达标
    if (!has_min_isolated_two_pair_order_span(candidate, pairs, config)) {
        return false;
    }

    // 仅2组灯对时额外校验竖直高度
    if (candidate.pair_indices.size() <= 2) {
        double mean_center_dy_px = 0.0;
        int valid_pair_count     = 0;
        for (const int pair_idx : candidate.pair_indices) {
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                continue;
            }
            mean_center_dy_px += pairs[static_cast<size_t>(pair_idx)].center_dy_px;
            ++valid_pair_count;
        }
        // 无有效灯对直接拦截
        if (valid_pair_count == 0) {
            return false;
        }
        mean_center_dy_px /= static_cast<double>(valid_pair_count);
        // 竖直高度不足，需要额外打分余量兜底
        if (mean_center_dy_px < config.min_two_pair_mean_center_dy_px) {
            constexpr float kFlatTwoPairScoreMargin = 0.015f;
            // 打分未达到抬高后的阈值，丢弃
            if (candidate.preliminary_score
                < min_candidate_score(candidate, config) + kFlatTwoPairScoreMargin) {
                return false;
            }
        }
    }

    // 全部校验通过
    return true;
}

/**
 * @brief 保留最优候选关联的全部灯条，删除无交集候选
 * 多候选筛选后，只保留包含最优候选灯条集合的网格，剔除重叠冲突候选
 * @param candidates 全部生成的网格候选数组，原地过滤
 */
void retain_selected_candidate_support(std::vector<LdmMeshCandidate>& candidates) {
    if (candidates.empty()) {
        return;
    }

    // 取出最优候选的灯对索引并排序，用于二分查找匹配
    auto selected_pair_indices = candidates.front().pair_indices;
    std::sort(selected_pair_indices.begin(), selected_pair_indices.end());
    // 删除任意灯对不在最优候选内的网格
    candidates.erase(
        std::remove_if(
            candidates.begin(), candidates.end(),
            [&](const LdmMeshCandidate& candidate) {
                // 当前候选所有灯对必须全部存在于最优候选集合
                return !std::all_of(
                    candidate.pair_indices.begin(), candidate.pair_indices.end(),
                    [&](const int pair_idx) {
                        return std::binary_search(
                            selected_pair_indices.begin(), selected_pair_indices.end(), pair_idx);
                    });
            }),
        candidates.end());
}

/**
 * @brief 计算最大姿态角约束下，灯条投影最小竖直高度比例
 * 用于配对校验，过滤水平歪斜过大、透视畸变严重的灯条配对
 * @param config 检测器配置，读取最大允许姿态角
 * @return 最小竖直投影比例float
 */
[[nodiscard]] float min_projected_pair_vertical_ratio(const LdmDetectorConfig& config) {
    // 读取配置最大姿态角，限制上限89.999°防止cos趋近0
    const double max_pose_angle =
        (std::isfinite(config.max_pose_angle_rad) && config.max_pose_angle_rad > 0.0)
            ? std::min(config.max_pose_angle_rad, std::numbers::pi_v<double> * 0.5 - 1e-3)
            : 0.872664626;
    // 0.75 * cos(最大俯仰角)，下限0.35防止数值过小
    return std::max(0.35f, 0.75f * static_cast<float>(std::cos(max_pose_angle)));
}

/**
 * @brief 获取整型矩形浮点中心坐标
 * @param rect OpenCV整型矩形
 * @return 矩形中心点cv::Point2f
 */
[[nodiscard]] cv::Point2f rect_center(const cv::Rect& rect) {
    return cv::Point2f(
        rect.x + static_cast<float>(rect.width) * 0.5f,
        rect.y + static_cast<float>(rect.height) * 0.5f);
}

/**
 * @brief 整型矩形转浮点矩形，用于高精度像素计算
 * @param rect 输入int矩形
 * @return cv::Rect2f 浮点矩形
 */
[[nodiscard]] cv::Rect2f rect2f_from_rect(const cv::Rect& rect) {
    return cv::Rect2f(
        static_cast<float>(rect.x), static_cast<float>(rect.y), static_cast<float>(rect.width),
        static_cast<float>(rect.height));
}

/**
 * @brief 判断HSV色相是否匹配目标装甲颜色
 * 红：0~30 或 160~180；蓝：90~140；紫：125~165；默认匹配全部三色
 * @param hue 单通道色相值 [0,180]
 * @param color 目标装甲颜色枚举
 * @return true 色相匹配目标颜色
 */
[[nodiscard]] bool hue_matches_target_color(double hue, ArmorColor color) {
    switch (color) {
    case ArmorColor::Blue: return hue >= 90.0 && hue <= 140.0;
    case ArmorColor::Purple: return hue >= 125.0 && hue <= 165.0;
    case ArmorColor::Red: return (hue >= 0.0 && hue <= 30.0) || (hue >= 160.0 && hue <= 180.0);
    default:
        // 无指定颜色，兼容红/蓝/紫全部
        return hue_matches_target_color(hue, ArmorColor::Red)
            || hue_matches_target_color(hue, ArmorColor::Blue)
            || hue_matches_target_color(hue, ArmorColor::Purple);
    }
}

/**
 * @brief 根据目标颜色生成二值掩码，分离灯条区域
 * @param image_bgr 原始BGR图像
 * @param color 目标装甲颜色
 * @param min_value V通道最低亮度阈值，默认80，高亮度掩码传入140
 * @return 单通道二值掩码，灯条白色背景黑色
 */
[[nodiscard]] cv::Mat
    threshold_target_color(const cv::Mat& image_bgr, ArmorColor color, int min_value = 80) {
    cv::Mat hsv;
    // BGR转HSV颜色空间，用于色相分割
    cv::cvtColor(image_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    switch (color) {
    case ArmorColor::Blue:
        // 蓝色色相区间，饱和度下限70，亮度min_value
        cv::inRange(hsv, cv::Scalar(90, 70, min_value), cv::Scalar(140, 255, 255), mask);
        break;
    case ArmorColor::Purple:
        cv::inRange(hsv, cv::Scalar(125, 70, min_value), cv::Scalar(165, 255, 255), mask);
        break;
    case ArmorColor::Red: {
        // 红色两段色相区间，合并掩码
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 80, min_value), cv::Scalar(30, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(160, 80, min_value), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;
        break;
    }
    default: {
        // 全颜色模式，合并三色掩码
        cv::Mat red    = threshold_target_color(image_bgr, ArmorColor::Red, min_value);
        cv::Mat blue   = threshold_target_color(image_bgr, ArmorColor::Blue, min_value);
        cv::Mat purple = threshold_target_color(image_bgr, ArmorColor::Purple, min_value);
        mask           = red | blue | purple;
        break;
    }
    }

    // 清除图像边缘2像素边框，去除HUD、界面边框干扰噪点
    constexpr int border_px = 2;
    if (mask.rows > border_px * 2 && mask.cols > border_px * 2) {
        mask.rowRange(0, border_px).setTo(0);
        mask.rowRange(mask.rows - border_px, mask.rows).setTo(0);
        mask.colRange(0, border_px).setTo(0);
        mask.colRange(mask.cols - border_px, mask.cols).setTo(0);
    }
    return mask;
}

/**
 * @brief 从轮廓生成标准灯条Blob结构体，执行全套阈值过滤
 * @param contour 单条轮廓点集合
 * @param mask 对应二值掩码，用于统计有效像素
 * @param offset 轮廓ROI偏移坐标，还原原图绝对像素
 * @param config 检测器阈值配置
 * @return 合法LightBlob，非法返回std::nullopt
 */
[[nodiscard]] std::optional<LightBlob> make_light_blob_from_contour(
    const std::vector<cv::Point>& contour, const cv::Mat& mask, const cv::Point& offset,
    const LdmDetectorConfig& config) {
    cv::Rect rect = cv::boundingRect(contour);
    // 叠加ROI偏移，映射到原图坐标
    if (offset.x != 0 || offset.y != 0) {
        rect.x += offset.x;
        rect.y += offset.y;
    }
    // 矩形宽高非法直接丢弃
    if (rect.width <= 0 || rect.height <= 0) {
        return std::nullopt;
    }

    // 计算长宽比，过滤过扁/过窄噪点
    const float aspect_ratio =
        static_cast<float>(rect.width) / static_cast<float>(std::max(rect.height, 1));
    if (aspect_ratio < static_cast<float>(config.min_blob_aspect_ratio)
        || aspect_ratio > static_cast<float>(config.max_blob_aspect_ratio)) {
        return std::nullopt;
    }

    // 轮廓面积、掩码内有效像素统计
    const double contour_area   = cv::contourArea(contour);
    const int pixel_count       = cv::countNonZero(mask(rect));
    // 有效面积取轮廓面积/像素数量二者中较大值
    const double effective_area = (contour_area >= static_cast<double>(config.min_blob_area_px))
                                    ? contour_area
                                    : static_cast<double>(pixel_count);
    // 面积低于最小阈值丢弃
    if (effective_area < static_cast<double>(config.min_blob_area_px)) {
        return std::nullopt;
    }

    // 填充率计算：轮廓面积/外接矩形面积；有效像素/外接矩形面积
    const float rect_area      = static_cast<float>(std::max(1, rect.width * rect.height));
    const float fill_ratio_geo = static_cast<float>(contour_area) / rect_area;
    const float fill_ratio_px  = static_cast<float>(pixel_count) / rect_area;
    // 几何填充达标 或 稀疏像素数量达标，二者满足其一保留
    const bool passes_fill     = fill_ratio_geo >= static_cast<float>(config.min_blob_fill_ratio);
    const bool passes_sparse   = pixel_count >= config.min_sparse_blob_pixel_count;
    if (!passes_fill && !passes_sparse) {
        return std::nullopt;
    }

    // 稀疏灯条使用像素填充率，完整灯条使用几何轮廓填充率
    const bool used_sparse_gate = !passes_fill;
    const float fill_ratio      = used_sparse_gate ? fill_ratio_px : fill_ratio_geo;
    return LightBlob{
        .rect         = rect2f_from_rect(rect),
        .center_px    = rect_center(rect),
        .area_px      = static_cast<float>(contour_area),
        .aspect_ratio = aspect_ratio,
        .fill_ratio   = fill_ratio};
}

/**
 * @brief 生成窄长条Yaw专用灯条Blob（特殊极窄灯条，用于远距离大符）
 * 额外长宽、尺寸、色相强约束，区别于标准灯条
 * @param contour 轮廓点
 * @param hsv 原图HSV图像，用于色相校验
 * @param mask 二值掩码
 * @param offset ROI偏移
 * @param config 检测器配置
 * @param color 目标装甲颜色
 * @return 合法窄灯条Blob/nullopt
 */
[[nodiscard]] std::optional<LightBlob> make_narrow_yaw_blob_from_contour(
    const std::vector<cv::Point>& contour, const cv::Mat& hsv, const cv::Mat& mask,
    const cv::Point& offset, const LdmDetectorConfig& config, ArmorColor color) {
    cv::Rect rect = cv::boundingRect(contour);
    if (offset.x != 0 || offset.y != 0) {
        rect.x += offset.x;
        rect.y += offset.y;
    }
    if (rect.width <= 0 || rect.height <= 0) {
        return std::nullopt;
    }

    // 窄灯条固定尺寸约束
    constexpr float kMinNarrowAspect = 0.15f;
    constexpr int kMaxNarrowWidth    = 5;
    constexpr int kMinNarrowHeight   = 10;
    constexpr int kMaxNarrowHeight   = 24;
    constexpr int kMinNarrowPixels   = 36;
    constexpr double kMinNarrowArea  = 20.0;

    const float aspect_ratio =
        static_cast<float>(rect.width) / static_cast<float>(std::max(rect.height, 1));
    // 长宽比、宽高尺寸过滤
    if (aspect_ratio >= static_cast<float>(config.min_blob_aspect_ratio)
        || aspect_ratio < kMinNarrowAspect || rect.width > kMaxNarrowWidth
        || rect.height < kMinNarrowHeight || rect.height > kMaxNarrowHeight) {
        return std::nullopt;
    }
    // ROI越界直接丢弃
    if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > hsv.cols
        || rect.y + rect.height > hsv.rows) {
        return std::nullopt;
    }

    const double contour_area = cv::contourArea(contour);
    const int pixel_count     = cv::countNonZero(mask(rect));
    // 面积、像素数量下限校验
    if (contour_area < kMinNarrowArea || pixel_count < kMinNarrowPixels) {
        return std::nullopt;
    }

    // 取ROI平均色相校验颜色
    const double mean_hue = cv::mean(hsv(rect), mask(rect))[0];
    if (!hue_matches_target_color(mean_hue, color)) {
        return std::nullopt;
    }

    const float rect_area     = static_cast<float>(std::max(1, rect.width * rect.height));
    const float fill_ratio_px = static_cast<float>(pixel_count) / rect_area;
    return LightBlob{
        .rect         = rect2f_from_rect(rect),
        .center_px    = rect_center(rect),
        .area_px      = static_cast<float>(contour_area),
        .aspect_ratio = aspect_ratio,
        .fill_ratio   = fill_ratio_px};
}

/**
 * @brief 拆分粘连合并灯条：单个Blob覆盖左右成对灯条时切分为两个独立Blob
 * 基于几何配置、已生成灯对、网格候选判断是否需要切分
 * @param blobs 原始全部灯条Blob
 * @param pairs 已生成灯对
 * @param mesh_candidates 网格候选
 * @param config 检测器几何配置
 * @return 切分后新Blob数组
 */
[[nodiscard]] std::vector<LightBlob> resolve_merged_light_blobs(
    const std::vector<LightBlob>& blobs, const std::vector<LightPair>& pairs,
    const std::vector<LdmMeshCandidate>& mesh_candidates, const LdmDetectorConfig& config) {
    const double pair_separation_m = config.geometry.pair_center_separation_m;
    const double window_length_m   = config.geometry.window_length_m;
    // 几何参数非法直接返回原Blob不做切分
    if (!std::isfinite(pair_separation_m) || pair_separation_m <= 0.0
        || !std::isfinite(window_length_m) || window_length_m <= 0.0
        || !std::isfinite(config.max_resolved_window_length_fraction)
        || config.max_resolved_window_length_fraction <= 0.0
        || !std::isfinite(config.max_merged_window_pair_separation_px)
        || config.max_merged_window_pair_separation_px <= 0.0) {
        return blobs;
    }

    // 单灯条最大允许宽度比例，超过判定为粘连合并灯条
    const float max_single_window_ratio = static_cast<float>(
        config.max_resolved_window_length_fraction * window_length_m / pair_separation_m);
    // 预存每个Blob对应的配对竖直间距，用于判断是否粘连
    std::vector<float> candidate_pair_separations(blobs.size(), 0.0f);
    for (const auto& candidate : mesh_candidates) {
        for (const int pair_idx : candidate.pair_indices) {
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                continue;
            }
            const auto& pair = pairs[static_cast<size_t>(pair_idx)];
            for (const int blob_idx : {pair.top_blob_index, pair.bottom_blob_index}) {
                if (blob_idx < 0 || static_cast<size_t>(blob_idx) >= blobs.size()) {
                    continue;
                }
                candidate_pair_separations[static_cast<size_t>(blob_idx)] = std::max(
                    candidate_pair_separations[static_cast<size_t>(blob_idx)], pair.center_dy_px);
            }
        }
    }

    std::vector<LightBlob> resolved;
    resolved.reserve(blobs.size() * 2u);
    for (size_t blob_idx = 0; blob_idx < blobs.size(); ++blob_idx) {
        const auto& blob            = blobs[blob_idx];
        const float pair_separation = candidate_pair_separations[blob_idx];
        // 不满足粘连条件，直接保留原Blob
        if (pair_separation <= 1e-3f
            || pair_separation > static_cast<float>(config.max_merged_window_pair_separation_px)
            || blob.rect.width / pair_separation <= max_single_window_ratio) {
            resolved.push_back(blob);
            continue;
        }

        // 左右对半切分
        const float left_width  = std::floor(blob.rect.width * 0.5f);
        const float right_width = blob.rect.width - left_width;
        // 切分后宽度过小，放弃切分保留原Blob
        if (left_width <= 1.0f || right_width <= 1.0f) {
            resolved.push_back(blob);
            continue;
        }

        // 生成左半、右半两个新Blob
        for (int part = 0; part < 2; ++part) {
            LightBlob half  = blob;
            half.rect.x     = blob.rect.x + ((part == 0) ? 0.0f : left_width);
            half.rect.width = (part == 0) ? left_width : right_width;
            half.center_px  = cv::Point2f(
                half.rect.x + half.rect.width * 0.5f, half.rect.y + half.rect.height * 0.5f);
            half.area_px *= 0.5f;
            half.aspect_ratio = half.rect.width / std::max(half.rect.height, 1.0f);
            resolved.push_back(half);
        }
    }
    return resolved;
}

/**
 * @brief 判断窄Yaw灯条是否存在配对伙伴（竖直间距匹配另一窄灯条）
 * @param blob 当前窄灯条
 * @param narrow_blobs 全部窄灯条集合
 * @return true 存在匹配配对，可保留该窄灯条
 */
[[nodiscard]] bool
    has_narrow_yaw_pair_mate(const LightBlob& blob, const std::vector<LightBlob>& narrow_blobs) {
    constexpr float kMaxMateDx = 10.0f;
    constexpr float kMinMateDy = 20.0f;
    constexpr float kMaxMateDy = 60.0f;

    return std::any_of(narrow_blobs.begin(), narrow_blobs.end(), [&](const LightBlob& other) {
        if (&other == &blob) {
            return false;
        }
        const float dx = std::abs(other.center_px.x - blob.center_px.x);
        const float dy = std::abs(other.center_px.y - blob.center_px.y);
        return dx <= kMaxMateDx && dy >= kMinMateDy && dy <= kMaxMateDy;
    });
}

/**
 * @brief 判断灯条周边是否存在足量常规灯条上下文，过滤孤立窄灯条噪点
 * @param blob 待校验窄灯条
 * @param regular_blobs 标准正常灯条集合
 * @return true 周边存在足量灯条上下文，保留
 */
[[nodiscard]] bool has_full_mesh_regular_context(
    const LightBlob& blob, const std::vector<LightBlob>& regular_blobs) {
    constexpr float kContextDx            = 80.0f;
    constexpr float kContextDy            = 80.0f;
    constexpr int kMinRegularContextBlobs = 6;

    int nearby_count = 0;
    for (const auto& regular_blob : regular_blobs) {
        if (std::abs(regular_blob.center_px.x - blob.center_px.x) <= kContextDx
            && std::abs(regular_blob.center_px.y - blob.center_px.y) <= kContextDy) {
            ++nearby_count;
        }
    }
    return nearby_count >= kMinRegularContextBlobs;
}

/**
 * @brief 完整灯条Blob提取流水线：颜色掩码→轮廓提取→标准/窄灯条分类过滤
 * @param image_bgr 输入原图BGR
 * @param config 检测器配置
 * @param color 目标装甲颜色
 * @return 全部合法灯条Blob数组
 */
[[nodiscard]] std::vector<LightBlob> detect_light_blobs(
    const cv::Mat& image_bgr, const LdmDetectorConfig& config, ArmorColor color) {
    if (image_bgr.empty()) {
        return {};
    }

    cv::Mat hsv;
    cv::cvtColor(image_bgr, hsv, cv::COLOR_BGR2HSV);

    // 基础亮度掩码、高亮度核心掩码两套阈值
    cv::Mat mask      = threshold_target_color(image_bgr, color);
    cv::Mat core_mask = threshold_target_color(image_bgr, color, 140);
    std::vector<std::vector<cv::Point>> contours;
    // 提取外层全部轮廓，只取最外层
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBlob> blobs;
    std::vector<LightBlob> narrow_yaw_blobs;
    blobs.reserve(contours.size());
    narrow_yaw_blobs.reserve(contours.size());
    for (const auto& contour : contours) {
        const cv::Rect rect = cv::boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0) {
            continue;
        }

        // 在高亮度核心掩码内提取子轮廓
        std::vector<std::vector<cv::Point>> core_contours;
        cv::Mat core_roi = core_mask(rect);
        cv::findContours(core_roi, core_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<LightBlob> core_blobs;
        core_blobs.reserve(core_contours.size());
        for (const auto& core_contour : core_contours) {
            auto blob = make_light_blob_from_contour(core_contour, core_mask, rect.tl(), config);
            if (blob.has_value()) {
                core_blobs.push_back(*blob);
                continue;
            }
            // 标准Blob失败，尝试窄Yaw灯条
            auto narrow_blob = make_narrow_yaw_blob_from_contour(
                core_contour, hsv, core_mask, rect.tl(), config, color);
            if (narrow_blob.has_value()) {
                narrow_yaw_blobs.push_back(*narrow_blob);
            }
        }
        // 核心掩码内存在>=2个标准灯条，直接加入结果
        if (core_blobs.size() >= 2u) {
            blobs.insert(blobs.end(), core_blobs.begin(), core_blobs.end());
            continue;
        }

        // 核心掩码无足够灯条，使用外层完整轮廓生成标准Blob
        auto blob = make_light_blob_from_contour(contour, mask, {}, config);
        if (blob.has_value()) {
            blobs.push_back(*blob);
        }
    }

    // 筛选合法窄Yaw灯条：存在配对+足量周边上下文
    for (const auto& narrow_blob : narrow_yaw_blobs) {
        if (has_narrow_yaw_pair_mate(narrow_blob, narrow_yaw_blobs)
            && has_full_mesh_regular_context(narrow_blob, blobs)) {
            blobs.push_back(narrow_blob);
        }
    }

    // 按图像X坐标升序排序灯条，方便后续聚类、配对
    std::sort(blobs.begin(), blobs.end(), [](const LightBlob& a, const LightBlob& b) {
        if (a.center_px.x != b.center_px.x) {
            return a.center_px.x < b.center_px.x;
        }
        return a.center_px.y < b.center_px.y;
    });
    return blobs;
}

// 聚类扩张常数
constexpr float kClusterExpandXRatio      = 1.0f;
constexpr float kClusterExpandYRatio      = 3.5f;
constexpr size_t kPcaSeedBlobLimit        = 6;
constexpr size_t kMaxClusterBlobCount     = 20;
constexpr float kMaxPairOrderDeltaRatio   = 1.5f;
constexpr float kPairOrderScoreDeltaRatio = 2.5f;
constexpr float kMinAxisBalance           = 0.4f;

/**
 * @brief 二维点向量点积
 * @param lhs 点1
 * @param rhs 点2
 * @return 点积标量float
 */
[[nodiscard]] float dot(const cv::Point2f& lhs, const cv::Point2f& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

/**
 * @brief 标准化PCA坐标轴方向，统一向量朝向避免正负歧义
 * @param axis 原始特征向量
 * @param prefer_y_positive true优先Y正向；false优先绝对值最大轴正向
 * @return 标准化后向量
 */
[[nodiscard]] cv::Point2f orient_axis(cv::Point2f axis, bool prefer_y_positive) {
    if (prefer_y_positive) {
        if (axis.y < 0.0f) {
            axis *= -1.0f;
        }
        return axis;
    }

    if (std::abs(axis.x) >= std::abs(axis.y)) {
        if (axis.x < 0.0f) {
            axis *= -1.0f;
        }
    } else if (axis.y < 0.0f) {
        axis *= -1.0f;
    }
    return axis;
}

/**
 * @brief 灯条矩形向外扩张生成聚类重叠判定矩形
 * X方向小幅扩张，Y方向大幅扩张，匹配大符竖直长条分布特性
 * @param blob 单灯条Blob
 * @return 扩张后浮点矩形
 */
[[nodiscard]] cv::Rect2f expanded_rect(const LightBlob& blob) {
    const float margin_x = blob.rect.width * kClusterExpandXRatio;
    const float margin_y = std::max(blob.rect.width, blob.rect.height) * kClusterExpandYRatio;
    return cv::Rect2f(
        blob.rect.x - margin_x, blob.rect.y - margin_y, blob.rect.width + 2.0f * margin_x,
        blob.rect.height + 2.0f * margin_y);
}

/**
 * @brief 判断两个浮点矩形是否存在重叠区域
 * @param lhs 矩形A
 * @param rhs 矩形B
 * @return true 存在交集
 */
[[nodiscard]] bool rects_overlap(const cv::Rect2f& lhs, const cv::Rect2f& rhs) {
    const float left   = std::max(lhs.x, rhs.x);
    const float top    = std::max(lhs.y, rhs.y);
    const float right  = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float bottom = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    return right >= left && bottom >= top;
}

/**
 * @brief 基于扩张矩形连通域聚类灯条，同聚类代表同一大符灯条集合
 * @param blobs 全部灯条Blob数组，原地写入cluster_id、local_order_px、local_layer_px
 * @return 每个聚类包含的Blob索引数组
 */
[[nodiscard]] std::vector<std::vector<size_t>> cluster_blob_indices(std::vector<LightBlob>& blobs) {
    std::vector<std::vector<size_t>> clusters;
    if (blobs.empty()) {
        return clusters;
    }

    // 预生成每个Blob扩张矩形
    std::vector<cv::Rect2f> grown_rects;
    grown_rects.reserve(blobs.size());
    for (const auto& blob : blobs) {
        grown_rects.push_back(expanded_rect(blob));
    }

    std::vector<bool> visited(blobs.size(), false);
    // 深度优先连通域聚类
    for (size_t seed_idx = 0; seed_idx < blobs.size(); ++seed_idx) {
        if (visited[seed_idx]) {
            continue;
        }

        std::vector<size_t> cluster;
        std::vector<size_t> stack{seed_idx};
        visited[seed_idx] = true;
        while (!stack.empty()) {
            const size_t idx = stack.back();
            stack.pop_back();
            cluster.push_back(idx);

            // 遍历全部未访问Blob，判断矩形重叠连通
            for (size_t other_idx = 0; other_idx < blobs.size(); ++other_idx) {
                if (visited[other_idx]) {
                    continue;
                }
                if (!rects_overlap(grown_rects[idx], grown_rects[other_idx])) {
                    continue;
                }
                visited[other_idx] = true;
                stack.push_back(other_idx);
            }
        }

        clusters.push_back(std::move(cluster));
    }

    // 聚类按整体X中心从小到大排序
    std::sort(clusters.begin(), clusters.end(), [&](const auto& lhs, const auto& rhs) {
        const auto cluster_center_x = [&](const std::vector<size_t>& cluster) {
            float sum = 0.0f;
            for (const size_t idx : cluster) {
                sum += blobs[idx].center_px.x;
            }
            return sum / static_cast<float>(cluster.size());
        };
        return cluster_center_x(lhs) < cluster_center_x(rhs);
    });

    // 回填每个Blob的聚类ID、局部坐标初始化0
    for (size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
        for (const size_t blob_idx : clusters[cluster_id]) {
            blobs[blob_idx].cluster_id     = static_cast<int>(cluster_id);
            blobs[blob_idx].local_order_px = 0.0f;
            blobs[blob_idx].local_layer_px = 0.0f;
        }
    }
    return clusters;
}

/**
 * @brief PCA轴分割结果结构体，存储分层轴、排序轴、分割阈值、匹配打分
 */
struct AxisSplit {
    cv::Point2f layer_axis{};          // 竖直分层轴（区分上下灯条）
    cv::Point2f order_axis{};          // 水平排序轴（区分前后灯条）
    float split_value{0.0f};           // 分层分割阈值
    float score{-1.0f};                // 分割方案综合打分
    int matched_pair_count{0};         // 该分割下有效灯对数量
    float matched_pair_score{0.0f};    // 全部配对总分和
    int best_candidate_pair_count{0};  // 最优网格候选灯对数量
    float best_candidate_preliminary{0.0f}; // 最优候选基础打分
    float best_candidate_score{0.0f}; // 最优候选综合检测打分
};

/**
 * @brief 配对匹配结果：配对总数、总分
 */
struct MatchResult {
    int pair_count{0};
    float score{0.0f};
};

/**
 * @brief 选中配对局部索引、打分
 */
struct ProjectedPairSelection {
    size_t first_local_idx{0};
    size_t second_local_idx{0};
    float score{0.0f};
};

/**
 * @brief 完整聚类配对匹配输出：全局最优匹配结果、选中配对列表
 */
struct ProjectedMatching {
    MatchResult result{};
    std::vector<ProjectedPairSelection> selected_pairs{};
};

/**
 * @brief 匹配结果优劣比较：优先配对数量，数量相同比总分
 * @param lhs 匹配结果A
 * @param rhs 匹配结果B
 * @return true A优于B
 */
[[nodiscard]] bool better_match(const MatchResult& lhs, const MatchResult& rhs) {
    return lhs.pair_count > rhs.pair_count
        || (lhs.pair_count == rhs.pair_count && lhs.score > rhs.score);
}

/**
 * @brief 配对数量权重打分系数，用于综合候选打分
 * @param pair_count 配对数量
 * @return 权重系数float
 */
[[nodiscard]] float pair_count_priority(int pair_count) {
    switch (pair_count) {
    case 1: return 0.25f;
    case 2: return 0.55f;
    case 3: return 0.82f;
    default: return 0.96f;
    }
}

/**
 * @brief 单组灯对投影匹配打分，过滤非法配对并输出匹配分数
 * @param first 上方/下方灯条1
 * @param first_order 局部排序坐标
 * @param first_layer 局部分层坐标
 * @param second 配对灯条2
 * @param second_order 局部排序坐标
 * @param second_layer 局部分层坐标
 * @param config 检测器配置阈值
 * @return 合法返回分数0~1，非法nullopt
 */
[[nodiscard]] std::optional<float> score_projected_pair(
    const LightBlob& first, float first_order, float first_layer, const LightBlob& second,
    float second_order, float second_layer, const LdmDetectorConfig& config) {
    // 分层必须一正一负（上下分布），同层直接丢弃
    if ((first_layer <= 0.0f) == (second_layer <= 0.0f)) {
        return std::nullopt;
    }

    const float avg_w = (first.rect.width + second.rect.width) * 0.5f;
    const float avg_h = (first.rect.height + second.rect.height) * 0.5f;
    if (avg_w <= 1e-3f || avg_h <= 1e-3f) {
        return std::nullopt;
    }

    // 水平坐标差阈值过滤
    const float order_delta = std::abs(first_order - second_order);
    const float size_scale  = std::max({avg_w, avg_h, 1.0f});
    const float order_limit = kMaxPairOrderDeltaRatio * size_scale;
    if (order_delta > order_limit) {
        return std::nullopt;
    }

    // 竖直分层间距上下限过滤
    const float layer_separation = std::abs(first_layer - second_layer);
    const float min_layer_sep =
        static_cast<float>(config.min_pair_center_dy_ratio) * std::max(avg_h, 1.0f);
    const float max_layer_sep =
        static_cast<float>(config.max_pair_center_dy_ratio) * std::max(avg_h, 1.0f);
    if (layer_separation < min_layer_sep || layer_separation > max_layer_sep) {
        return std::nullopt;
    }

    // 投影竖直高度比例过滤，剔除水平歪斜配对
    const cv::Point2f image_delta = second.center_px - first.center_px;
    const float image_distance =
        std::sqrt(image_delta.x * image_delta.x + image_delta.y * image_delta.y);
    if (image_distance <= 1e-3f) {
        return std::nullopt;
    }
    const float projected_vertical_ratio = std::abs(image_delta.y) / image_distance;
    if (projected_vertical_ratio < min_projected_pair_vertical_ratio(config)) {
        return std::nullopt;
    }

    // 灯条尺寸差异过滤
    const float width_delta =
        std::abs(first.rect.width - second.rect.width) / std::max(avg_w, 1.0f);
    const float height_delta =
        std::abs(first.rect.height - second.rect.height) / std::max(avg_h, 1.0f);
    if (width_delta > static_cast<float>(config.max_pair_size_delta_ratio)
        || height_delta > static_cast<float>(config.max_pair_size_delta_ratio)) {
        return std::nullopt;
    }

    // 多维度加权综合打分
    const float order_score_limit = kPairOrderScoreDeltaRatio * size_scale;
    const float order_score =
        std::max(0.0f, 1.0f - order_delta / std::max(order_score_limit, 1.0f));
    const float width_score  = std::max(0.0f, 1.0f - width_delta);
    const float height_score = std::max(0.0f, 1.0f - height_delta);
    const float fill_score   = 0.5f * (first.fill_ratio + second.fill_ratio);
    const float layer_balance =
        std::abs(std::abs(first_layer) - std::abs(second_layer)) / std::max(layer_separation, 1.0f);
    const float balance_score = std::max(0.0f, 1.0f - layer_balance);

    return std::clamp(
        0.35f * order_score + 0.15f * balance_score + 0.15f * width_score + 0.15f * height_score
            + 0.20f * fill_score,
        0.0f, 1.0f);
}

/**
 * @brief 动态规划搜索聚类内全局最优不重叠配对组合
 * 状态压缩DP，掩码表示已占用Blob，递归搜索最大配对数量+最高总分
 * @param blobs 全局灯条数组
 * @param cluster_indices 当前聚类内Blob局部索引
 * @param order_values 各Blob局部排序坐标
 * @param layer_values 各Blob局部分层坐标
 * @param config 检测器配置
 * @return 最优匹配方案（配对数量、总分、选中配对列表）
 */
[[nodiscard]] ProjectedMatching best_projected_matching(
    const std::vector<LightBlob>& blobs, const std::vector<size_t>& cluster_indices,
    const std::vector<float>& order_values, const std::vector<float>& layer_values,
    const LdmDetectorConfig& config) {
    if (cluster_indices.empty() || cluster_indices.size() > kMaxClusterBlobCount) {
        return {};
    }

    struct Candidate {
        size_t first_local_idx{0};
        size_t second_local_idx{0};
        float score{0.0f};
    };

    std::vector<Candidate> pair_candidates;
    pair_candidates.reserve(cluster_indices.size() * 3);
    std::vector<std::vector<int>> adjacency(cluster_indices.size());
    for (size_t first_local_idx = 0; first_local_idx < cluster_indices.size(); ++first_local_idx) {
        for (size_t second_local_idx = first_local_idx + 1;
             second_local_idx < cluster_indices.size(); ++second_local_idx) {
            const size_t first_blob_idx  = cluster_indices[first_local_idx];
            const size_t second_blob_idx = cluster_indices[second_local_idx];
            auto score                   = score_projected_pair(
                blobs[first_blob_idx], order_values[first_local_idx], layer_values[first_local_idx],
                blobs[second_blob_idx], order_values[second_local_idx],
                layer_values[second_local_idx], config);
            if (!score.has_value()) {
                continue;
            }

            const int candidate_idx = static_cast<int>(pair_candidates.size());
            pair_candidates.push_back(
                Candidate{
                    .first_local_idx  = first_local_idx,
                    .second_local_idx = second_local_idx,
                    .score            = *score,
                });
            adjacency[first_local_idx].push_back(candidate_idx);
            adjacency[second_local_idx].push_back(candidate_idx);
        }
    }
    if (pair_candidates.empty()) {
        return {};
    }

    const uint64_t full_mask = (uint64_t{1} << cluster_indices.size()) - 1u;
    std::vector<bool> cached(static_cast<size_t>(full_mask + 1u), false);
    std::vector<MatchResult> best(static_cast<size_t>(full_mask + 1u));
    std::vector<int> best_choice(static_cast<size_t>(full_mask + 1u), -2);
    const auto solve = [&](auto&& self, uint64_t used_mask) -> MatchResult {
        if (used_mask == full_mask) {
            return {};
        }
        if (cached[static_cast<size_t>(used_mask)]) {
            return best[static_cast<size_t>(used_mask)];
        }

        size_t first_unused = 0;
        while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
            ++first_unused;
        }

        MatchResult best_result = self(self, used_mask | (uint64_t{1} << first_unused));
        int choice              = -1;
        for (const int candidate_idx : adjacency[first_unused]) {
            const auto& candidate    = pair_candidates[static_cast<size_t>(candidate_idx)];
            const size_t other_idx   = (candidate.first_local_idx == first_unused)
                                         ? candidate.second_local_idx
                                         : candidate.first_local_idx;
            const uint64_t other_bit = (uint64_t{1} << other_idx);
            if ((used_mask & other_bit) != 0u) {
                continue;
            }

            auto candidate_result =
                self(self, used_mask | (uint64_t{1} << first_unused) | other_bit);
            candidate_result.pair_count += 1;
            candidate_result.score += candidate.score;
            if (better_match(candidate_result, best_result)) {
                best_result = candidate_result;
                choice      = candidate_idx;
            }
        }

        cached[static_cast<size_t>(used_mask)]      = true;
        best[static_cast<size_t>(used_mask)]        = best_result;
        best_choice[static_cast<size_t>(used_mask)] = choice;
        return best_result;
    };

    ProjectedMatching matching;
    matching.result = solve(solve, 0u);

    uint64_t used_mask = 0u;
    while (used_mask != full_mask) {
        size_t first_unused = 0;
        while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
            ++first_unused;
        }

        const int candidate_idx = best_choice[static_cast<size_t>(used_mask)];
        if (candidate_idx < 0) {
            used_mask |= (uint64_t{1} << first_unused);
            continue;
        }

        const auto& candidate = pair_candidates[static_cast<size_t>(candidate_idx)];
        matching.selected_pairs.push_back(
            ProjectedPairSelection{
                .first_local_idx  = candidate.first_local_idx,
                .second_local_idx = candidate.second_local_idx,
                .score            = candidate.score,
            });
        const size_t other_idx = (candidate.first_local_idx == first_unused)
                                   ? candidate.second_local_idx
                                   : candidate.first_local_idx;
        used_mask |= (uint64_t{1} << first_unused) | (uint64_t{1} << other_idx);
    }
    return matching;
}

[[nodiscard]] float values_stddev(const std::vector<float>& values) {
    if (values.size() <= 1) {
        return 0.0f;
    }

    const float mean =
        std::accumulate(values.begin(), values.end(), 0.0f) / static_cast<float>(values.size());
    float sq_sum = 0.0f;
    for (const float value : values) {
        const float delta = value - mean;
        sq_sum += delta * delta;
    }
    return std::sqrt(sq_sum / static_cast<float>(values.size()));
}

[[nodiscard]] std::optional<AxisSplit> evaluate_axis_split(
    const std::vector<LightBlob>& blobs, const std::vector<size_t>& cluster_indices,
    const cv::Point2f& mean, cv::Point2f layer_axis, cv::Point2f order_axis,
    const LdmDetectorConfig& config) {
    if (cluster_indices.size() < 2) {
        return std::nullopt;
    }

    layer_axis = orient_axis(layer_axis, true);
    order_axis = orient_axis(order_axis, false);

    std::vector<float> projected_values;
    projected_values.reserve(cluster_indices.size());
    for (const size_t blob_idx : cluster_indices) {
        const cv::Point2f centered = blobs[blob_idx].center_px - mean;
        projected_values.push_back(dot(centered, layer_axis));
    }

    std::sort(projected_values.begin(), projected_values.end());
    AxisSplit best_split{
        .layer_axis  = layer_axis,
        .order_axis  = order_axis,
        .split_value = 0.0f,
        .score       = -1.0f,
    };

    for (size_t split = 1; split < projected_values.size(); ++split) {
        const size_t lower_count = split;
        const size_t upper_count = projected_values.size() - split;
        const float balance      = static_cast<float>(std::min(lower_count, upper_count))
                            / static_cast<float>(std::max(lower_count, upper_count));
        if (balance < kMinAxisBalance) {
            continue;
        }

        const float gap = projected_values[split] - projected_values[split - 1];
        if (gap <= 1e-3f) {
            continue;
        }

        std::vector<float> lower(projected_values.begin(), projected_values.begin() + split);
        std::vector<float> upper(
            projected_values.begin() + static_cast<std::ptrdiff_t>(split), projected_values.end());
        const float score =
            gap * (0.55f + 0.45f * balance) / (4.0f + values_stddev(lower) + values_stddev(upper));
        if (score <= best_split.score) {
            continue;
        }

        best_split.split_value = 0.5f * (projected_values[split - 1] + projected_values[split]);
        best_split.score       = score;
    }

    if (best_split.score <= 0.0f) {
        return std::nullopt;
    }

    std::vector<float> order_values;
    std::vector<float> layer_values;
    order_values.reserve(cluster_indices.size());
    layer_values.reserve(cluster_indices.size());
    for (const size_t blob_idx : cluster_indices) {
        const cv::Point2f centered = blobs[blob_idx].center_px - mean;
        order_values.push_back(dot(centered, best_split.order_axis));
        layer_values.push_back(dot(centered, best_split.layer_axis) - best_split.split_value);
    }
    const auto matching =
        best_projected_matching(blobs, cluster_indices, order_values, layer_values, config);
    best_split.matched_pair_count = matching.result.pair_count;
    best_split.matched_pair_score = matching.result.score;

    std::vector<LightPair> projected_pairs;
    projected_pairs.reserve(matching.selected_pairs.size());
    for (const auto& selection : matching.selected_pairs) {
        const size_t first_blob_idx  = cluster_indices[selection.first_local_idx];
        const size_t second_blob_idx = cluster_indices[selection.second_local_idx];
        const auto& first            = blobs[first_blob_idx];
        const auto& second           = blobs[second_blob_idx];
        const size_t top_blob_idx =
            (first.center_px.y <= second.center_px.y) ? first_blob_idx : second_blob_idx;
        const size_t bottom_blob_idx =
            (top_blob_idx == first_blob_idx) ? second_blob_idx : first_blob_idx;
        const auto& top    = blobs[top_blob_idx];
        const auto& bottom = blobs[bottom_blob_idx];
        projected_pairs.push_back(
            LightPair{
                .top_blob_index    = static_cast<int>(top_blob_idx),
                .bottom_blob_index = static_cast<int>(bottom_blob_idx),
                .top_center_px     = top.center_px,
                .bottom_center_px  = bottom.center_px,
                .midpoint_px       = (top.center_px + bottom.center_px) * 0.5f,
                .center_dx_px      = std::abs(bottom.center_px.x - top.center_px.x),
                .center_dy_px      = std::abs(bottom.center_px.y - top.center_px.y),
                .cluster_id        = 0,
                .local_order_px    = 0.5f
                                * (order_values[selection.first_local_idx]
                                   + order_values[selection.second_local_idx]),
                .local_layer_sep_px = std::abs(
                    layer_values[selection.first_local_idx]
                    - layer_values[selection.second_local_idx]),
                .score = selection.score,
            });
    }

    auto preliminary_candidates = build_detection_mesh_candidates(projected_pairs, config);
    for (const auto& candidate : preliminary_candidates) {
        if (!candidate_passes_detection_gate(candidate, projected_pairs, config)) {
            continue;
        }

        const int pair_count = static_cast<int>(candidate.pair_indices.size());
        const float detection_score =
            0.55f * pair_count_priority(pair_count) + 0.45f * candidate.preliminary_score;
        if (pair_count > best_split.best_candidate_pair_count
            || (pair_count == best_split.best_candidate_pair_count
                && detection_score > best_split.best_candidate_score)) {
            best_split.best_candidate_pair_count  = pair_count;
            best_split.best_candidate_preliminary = candidate.preliminary_score;
            best_split.best_candidate_score       = detection_score;
        }
    }
    return best_split;
}

[[nodiscard]] std::optional<AxisSplit> estimate_cluster_axes(
    const std::vector<LightBlob>& blobs, const std::vector<size_t>& cluster_indices,
    const LdmDetectorConfig& config) {
    if (cluster_indices.size() < 2) {
        return std::nullopt;
    }

    std::vector<size_t> seed_indices = cluster_indices;
    std::sort(seed_indices.begin(), seed_indices.end(), [&](size_t lhs, size_t rhs) {
        return blobs[lhs].area_px > blobs[rhs].area_px;
    });
    if (seed_indices.size() > kPcaSeedBlobLimit) {
        seed_indices.resize(kPcaSeedBlobLimit);
    }
    if (seed_indices.size() < 2) {
        return std::nullopt;
    }

    cv::Mat samples(static_cast<int>(seed_indices.size()), 2, CV_32F);
    for (size_t row = 0; row < seed_indices.size(); ++row) {
        samples.at<float>(static_cast<int>(row), 0) = blobs[seed_indices[row]].center_px.x;
        samples.at<float>(static_cast<int>(row), 1) = blobs[seed_indices[row]].center_px.y;
    }

    cv::PCA pca(samples, cv::Mat(), cv::PCA::DATA_AS_ROW);
    const cv::Point2f mean(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));
    const cv::Point2f axis0(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    const cv::Point2f axis1(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1));

    std::vector<std::optional<AxisSplit>> candidates;
    candidates.push_back(evaluate_axis_split(blobs, cluster_indices, mean, axis0, axis1, config));
    candidates.push_back(evaluate_axis_split(blobs, cluster_indices, mean, axis1, axis0, config));
    candidates.push_back(evaluate_axis_split(
        blobs, cluster_indices, mean, cv::Point2f(0.0f, 1.0f), cv::Point2f(1.0f, 0.0f), config));
    candidates.push_back(evaluate_axis_split(
        blobs, cluster_indices, mean, cv::Point2f(1.0f, 0.0f), cv::Point2f(0.0f, 1.0f), config));

    std::optional<AxisSplit> best_candidate;
    for (const auto& candidate : candidates) {
        if (!candidate.has_value()) {
            continue;
        }
        if (!best_candidate.has_value()
            || candidate->best_candidate_score > best_candidate->best_candidate_score
            || (std::abs(candidate->best_candidate_score - best_candidate->best_candidate_score)
                    <= 1e-3f
                && candidate->best_candidate_pair_count > best_candidate->best_candidate_pair_count)
            || (std::abs(candidate->best_candidate_score - best_candidate->best_candidate_score)
                    <= 1e-3f
                && candidate->best_candidate_pair_count == best_candidate->best_candidate_pair_count
                && candidate->matched_pair_count > best_candidate->matched_pair_count)
            || (std::abs(candidate->best_candidate_score - best_candidate->best_candidate_score)
                    <= 1e-3f
                && candidate->best_candidate_pair_count == best_candidate->best_candidate_pair_count
                && candidate->matched_pair_count == best_candidate->matched_pair_count
                && candidate->score > best_candidate->score)) {
            best_candidate = candidate;
        }
    }
    return best_candidate;
}

void annotate_blob_clusters(std::vector<LightBlob>& blobs, const LdmDetectorConfig& config) {
    const auto clusters = cluster_blob_indices(blobs);
    for (const auto& cluster_indices : clusters) {
        auto axes = estimate_cluster_axes(blobs, cluster_indices, config);
        if (!axes.has_value()) {
            continue;
        }

        cv::Point2f mean(0.0f, 0.0f);
        for (const size_t seed_idx : cluster_indices) {
            mean += blobs[seed_idx].center_px;
        }
        mean *= (1.0f / static_cast<float>(cluster_indices.size()));

        // Re-estimate the mean from the seed PCA inputs so local coordinates match the split.
        std::vector<size_t> seed_indices = cluster_indices;
        std::sort(seed_indices.begin(), seed_indices.end(), [&](size_t lhs, size_t rhs) {
            return blobs[lhs].area_px > blobs[rhs].area_px;
        });
        if (seed_indices.size() > kPcaSeedBlobLimit) {
            seed_indices.resize(kPcaSeedBlobLimit);
        }
        if (!seed_indices.empty()) {
            mean = cv::Point2f(0.0f, 0.0f);
            for (const size_t seed_idx : seed_indices) {
                mean += blobs[seed_idx].center_px;
            }
            mean *= (1.0f / static_cast<float>(seed_indices.size()));
        }

        for (const size_t blob_idx : cluster_indices) {
            const cv::Point2f centered     = blobs[blob_idx].center_px - mean;
            blobs[blob_idx].local_order_px = dot(centered, axes->order_axis);
            blobs[blob_idx].local_layer_px = dot(centered, axes->layer_axis) - axes->split_value;
        }
    }
}

struct PairMetrics {
    size_t top_blob_idx{0};
    size_t bottom_blob_idx{0};
    float order_delta{0.0f};
    float layer_separation{0.0f};
    float score{0.0f};
};

[[nodiscard]] std::optional<PairMetrics> score_pair(
    const LightBlob& first, size_t first_idx, const LightBlob& second, size_t second_idx,
    const LdmDetectorConfig& config) {
    if (first.cluster_id < 0 || first.cluster_id != second.cluster_id) {
        return std::nullopt;
    }
    auto score = score_projected_pair(
        first, first.local_order_px, first.local_layer_px, second, second.local_order_px,
        second.local_layer_px, config);
    if (!score.has_value()) {
        return std::nullopt;
    }

    const float order_delta      = std::abs(first.local_order_px - second.local_order_px);
    const float layer_separation = std::abs(first.local_layer_px - second.local_layer_px);

    PairMetrics metrics;
    metrics.top_blob_idx     = (first.center_px.y <= second.center_px.y) ? first_idx : second_idx;
    metrics.bottom_blob_idx  = (metrics.top_blob_idx == first_idx) ? second_idx : first_idx;
    metrics.order_delta      = order_delta;
    metrics.layer_separation = layer_separation;
    metrics.score            = *score;
    return metrics;
}

struct PairCandidate {
    size_t first_local_idx{0};
    size_t second_local_idx{0};
    PairMetrics metrics{};
    float score{0.0f};
};

[[nodiscard]] std::vector<LightPair>
    build_light_pairs(const std::vector<LightBlob>& blobs, const LdmDetectorConfig& config) {
    std::vector<LightPair> pairs;
    if (blobs.size() < 2) {
        return pairs;
    }

    std::vector<int> cluster_ids;
    cluster_ids.reserve(blobs.size());
    for (const auto& blob : blobs) {
        if (blob.cluster_id < 0) {
            continue;
        }
        if (std::find(cluster_ids.begin(), cluster_ids.end(), blob.cluster_id)
            == cluster_ids.end()) {
            cluster_ids.push_back(blob.cluster_id);
        }
    }

    for (const int cluster_id : cluster_ids) {
        std::vector<size_t> cluster_blob_indices;
        cluster_blob_indices.reserve(blobs.size());
        for (size_t blob_idx = 0; blob_idx < blobs.size(); ++blob_idx) {
            if (blobs[blob_idx].cluster_id == cluster_id) {
                cluster_blob_indices.push_back(blob_idx);
            }
        }
        if (cluster_blob_indices.size() < 2 || cluster_blob_indices.size() > kMaxClusterBlobCount) {
            continue;
        }

        std::vector<PairCandidate> pair_candidates;
        pair_candidates.reserve(cluster_blob_indices.size() * 3);
        std::vector<std::vector<int>> adjacency(cluster_blob_indices.size());
        for (size_t first_local_idx = 0; first_local_idx < cluster_blob_indices.size();
             ++first_local_idx) {
            for (size_t second_local_idx = first_local_idx + 1;
                 second_local_idx < cluster_blob_indices.size(); ++second_local_idx) {
                const size_t first_blob_idx  = cluster_blob_indices[first_local_idx];
                const size_t second_blob_idx = cluster_blob_indices[second_local_idx];
                auto metrics                 = score_pair(
                    blobs[first_blob_idx], first_blob_idx, blobs[second_blob_idx], second_blob_idx,
                    config);
                if (!metrics.has_value()) {
                    continue;
                }

                const int candidate_idx = static_cast<int>(pair_candidates.size());
                pair_candidates.push_back(
                    PairCandidate{
                        .first_local_idx  = first_local_idx,
                        .second_local_idx = second_local_idx,
                        .metrics          = *metrics,
                        .score            = metrics->score,
                    });
                adjacency[first_local_idx].push_back(candidate_idx);
                adjacency[second_local_idx].push_back(candidate_idx);
            }
        }
        if (pair_candidates.empty()) {
            continue;
        }

        const uint64_t full_mask = (uint64_t{1} << cluster_blob_indices.size()) - 1u;
        std::vector<bool> cached(static_cast<size_t>(full_mask + 1u), false);
        std::vector<MatchResult> best(static_cast<size_t>(full_mask + 1u));
        std::vector<int> best_choice(static_cast<size_t>(full_mask + 1u), -2);

        const auto solve = [&](auto&& self, uint64_t used_mask) -> MatchResult {
            if (used_mask == full_mask) {
                return {};
            }
            if (cached[static_cast<size_t>(used_mask)]) {
                return best[static_cast<size_t>(used_mask)];
            }

            size_t first_unused = 0;
            while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
                ++first_unused;
            }

            MatchResult best_result = self(self, used_mask | (uint64_t{1} << first_unused));
            int choice              = -1;
            for (const int candidate_idx : adjacency[first_unused]) {
                const auto& candidate    = pair_candidates[static_cast<size_t>(candidate_idx)];
                const size_t other_idx   = (candidate.first_local_idx == first_unused)
                                             ? candidate.second_local_idx
                                             : candidate.first_local_idx;
                const uint64_t other_bit = (uint64_t{1} << other_idx);
                if ((used_mask & other_bit) != 0u) {
                    continue;
                }

                auto candidate_result =
                    self(self, used_mask | (uint64_t{1} << first_unused) | other_bit);
                candidate_result.pair_count += 1;
                candidate_result.score += candidate.score;
                if (better_match(candidate_result, best_result)) {
                    best_result = candidate_result;
                    choice      = candidate_idx;
                }
            }

            cached[static_cast<size_t>(used_mask)]      = true;
            best[static_cast<size_t>(used_mask)]        = best_result;
            best_choice[static_cast<size_t>(used_mask)] = choice;
            return best_result;
        };

        solve(solve, 0u);

        uint64_t used_mask = 0u;
        while (used_mask != full_mask) {
            size_t first_unused = 0;
            while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
                ++first_unused;
            }

            const int candidate_idx = best_choice[static_cast<size_t>(used_mask)];
            if (candidate_idx < 0) {
                used_mask |= (uint64_t{1} << first_unused);
                continue;
            }

            const auto& candidate      = pair_candidates[static_cast<size_t>(candidate_idx)];
            const auto& top            = blobs[candidate.metrics.top_blob_idx];
            const auto& bottom         = blobs[candidate.metrics.bottom_blob_idx];
            const cv::Point2f midpoint = (top.center_px + bottom.center_px) * 0.5f;
            pairs.push_back(
                LightPair{
                    .top_blob_index     = static_cast<int>(candidate.metrics.top_blob_idx),
                    .bottom_blob_index  = static_cast<int>(candidate.metrics.bottom_blob_idx),
                    .top_center_px      = top.center_px,
                    .bottom_center_px   = bottom.center_px,
                    .midpoint_px        = midpoint,
                    .center_dx_px       = std::abs(bottom.center_px.x - top.center_px.x),
                    .center_dy_px       = std::abs(bottom.center_px.y - top.center_px.y),
                    .cluster_id         = cluster_id,
                    .local_order_px     = 0.5f * (top.local_order_px + bottom.local_order_px),
                    .local_layer_sep_px = candidate.metrics.layer_separation,
                    .score              = candidate.score,
                });

            const size_t other_idx = (candidate.first_local_idx == first_unused)
                                       ? candidate.second_local_idx
                                       : candidate.first_local_idx;
            used_mask |= (uint64_t{1} << first_unused) | (uint64_t{1} << other_idx);
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const LightPair& a, const LightPair& b) {
        if (a.cluster_id != b.cluster_id) {
            return a.cluster_id < b.cluster_id;
        }
        return a.local_order_px < b.local_order_px;
    });
    return pairs;
}

/// Use the PCA-derived ordering and vertical-separation profile to assign faces
/// relative to the visible face looking most directly at the camera.
[[nodiscard]] bool assign_face_indices_for_candidate(
    LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) {
    if (candidate.pair_indices.size() < 2) {
        return false;
    }
    if (candidate.pair_indices.size() != candidate.octagon_face_indices.size()) {
        candidate.octagon_face_indices.resize(candidate.pair_indices.size());
    }

    struct Ordered {
        int pair_idx;
        float order;
        float sep;
    };
    std::vector<Ordered> ordered;
    ordered.reserve(pairs.size());

    const int cluster_id = [&]() {
        for (const int pidx : candidate.pair_indices) {
            if (pidx < 0 || static_cast<size_t>(pidx) >= pairs.size()) {
                continue;
            }
            return pairs[static_cast<size_t>(pidx)].cluster_id;
        }
        return -1;
    }();

    for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
        if (cluster_id >= 0 && pairs[pair_idx].cluster_id != cluster_id) {
            continue;
        }
        ordered.push_back(
            {static_cast<int>(pair_idx), pairs[pair_idx].local_order_px,
             pair_layer_separation_px(pairs[pair_idx])});
    }
    if (ordered.size() < 2) {
        return false;
    }
    std::sort(ordered.begin(), ordered.end(), [](const Ordered& a, const Ordered& b) {
        return a.order < b.order;
    });

    size_t front_idx = 0;
    for (size_t i = 1; i < ordered.size(); ++i) {
        if (ordered[i].sep > ordered[front_idx].sep) {
            front_idx = i;
        }
    }

    struct AssignedPair {
        int pair_idx;
        size_t rank;
        int face;
    };
    std::vector<AssignedPair> assigned;
    assigned.reserve(candidate.pair_indices.size());

    for (size_t i = 0; i < candidate.pair_indices.size(); ++i) {
        const auto rank_it = std::find_if(ordered.begin(), ordered.end(), [&](const Ordered& item) {
            return item.pair_idx == candidate.pair_indices[i];
        });
        if (rank_it == ordered.end()) {
            return false;
        }
        const size_t rank = static_cast<size_t>(std::distance(ordered.begin(), rank_it));
        int face          = static_cast<int>(rank) - static_cast<int>(front_idx);
        face %= 8;
        if (face < 0) {
            face += 8;
        }
        assigned.push_back(
            AssignedPair{.pair_idx = candidate.pair_indices[i], .rank = rank, .face = face});
    }

    std::sort(assigned.begin(), assigned.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.rank < rhs.rank;
    });

    candidate.pair_indices.clear();
    candidate.octagon_face_indices.clear();
    candidate.pair_indices.reserve(assigned.size());
    candidate.octagon_face_indices.reserve(assigned.size());
    for (const auto& item : assigned) {
        candidate.pair_indices.push_back(item.pair_idx);
        candidate.octagon_face_indices.push_back(item.face);
    }
    return true;
}

[[nodiscard]] std::optional<LdmDetection>
    detect_laser_module(const cv::Mat& image, const LdmDetectorConfig& config, ArmorColor color) {
    auto blobs = detect_light_blobs(image, config, color);
    annotate_blob_clusters(blobs, config);
    auto pairs = build_light_pairs(blobs, config);
    if (pairs.empty()) {
        return std::nullopt;
    }

    auto mesh_candidates = build_detection_mesh_candidates(pairs, config);
    mesh_candidates.erase(
        std::remove_if(
            mesh_candidates.begin(), mesh_candidates.end(),
            [&](const LdmMeshCandidate& candidate) {
                return !candidate_passes_detection_gate(candidate, pairs, config);
            }),
        mesh_candidates.end());
    if (mesh_candidates.empty()) {
        return std::nullopt;
    }
    retain_selected_candidate_support(mesh_candidates);

    auto resolved_blobs = resolve_merged_light_blobs(blobs, pairs, mesh_candidates, config);
    if (resolved_blobs.size() != blobs.size()) {
        annotate_blob_clusters(resolved_blobs, config);
        auto resolved_pairs      = build_light_pairs(resolved_blobs, config);
        auto resolved_candidates = build_detection_mesh_candidates(resolved_pairs, config);
        resolved_candidates.erase(
            std::remove_if(
                resolved_candidates.begin(), resolved_candidates.end(),
                [&](const LdmMeshCandidate& candidate) {
                    return !candidate_passes_detection_gate(candidate, resolved_pairs, config);
                }),
            resolved_candidates.end());
        if (!resolved_candidates.empty()
            && resolved_candidates.front().pair_indices.size()
                   > mesh_candidates.front().pair_indices.size()) {
            retain_selected_candidate_support(resolved_candidates);
            blobs           = std::move(resolved_blobs);
            pairs           = std::move(resolved_pairs);
            mesh_candidates = std::move(resolved_candidates);
        }
    }

    std::vector<int> selected_pair_indices = mesh_candidates.front().pair_indices;
    std::sort(selected_pair_indices.begin(), selected_pair_indices.end());
    selected_pair_indices.erase(
        std::unique(selected_pair_indices.begin(), selected_pair_indices.end()),
        selected_pair_indices.end());
    if (selected_pair_indices.empty()) {
        return std::nullopt;
    }

    std::vector<LightPair> selected_pairs;
    selected_pairs.reserve(selected_pair_indices.size());
    std::vector<int> selected_blob_indices;
    selected_blob_indices.reserve(selected_pair_indices.size() * 2);
    std::vector<int> pair_remap(pairs.size(), -1);
    for (const int pair_idx : selected_pair_indices) {
        if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
            continue;
        }
        const auto& pair                          = pairs[static_cast<size_t>(pair_idx)];
        pair_remap[static_cast<size_t>(pair_idx)] = static_cast<int>(selected_pairs.size());
        selected_pairs.push_back(pair);
        selected_blob_indices.push_back(pair.top_blob_index);
        selected_blob_indices.push_back(pair.bottom_blob_index);
    }
    if (selected_pairs.empty()) {
        return std::nullopt;
    }

    std::sort(selected_blob_indices.begin(), selected_blob_indices.end());
    selected_blob_indices.erase(
        std::unique(selected_blob_indices.begin(), selected_blob_indices.end()),
        selected_blob_indices.end());

    std::vector<LightBlob> selected_blobs;
    selected_blobs.reserve(selected_blob_indices.size());
    std::vector<int> blob_remap(blobs.size(), -1);
    for (const int old_blob_idx : selected_blob_indices) {
        if (old_blob_idx < 0 || static_cast<size_t>(old_blob_idx) >= blobs.size()) {
            continue;
        }
        blob_remap[static_cast<size_t>(old_blob_idx)] = static_cast<int>(selected_blobs.size());
        selected_blobs.push_back(blobs[static_cast<size_t>(old_blob_idx)]);
    }

    for (auto& pair : selected_pairs) {
        if (pair.top_blob_index >= 0
            && static_cast<size_t>(pair.top_blob_index) < blob_remap.size()) {
            pair.top_blob_index = blob_remap[static_cast<size_t>(pair.top_blob_index)];
        }
        if (pair.bottom_blob_index >= 0
            && static_cast<size_t>(pair.bottom_blob_index) < blob_remap.size()) {
            pair.bottom_blob_index = blob_remap[static_cast<size_t>(pair.bottom_blob_index)];
        }
    }

    for (auto& candidate : mesh_candidates) {
        for (auto& pair_idx : candidate.pair_indices) {
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pair_remap.size()) {
                pair_idx = -1;
                continue;
            }
            pair_idx = pair_remap[static_cast<size_t>(pair_idx)];
        }
        candidate.pair_indices.erase(
            std::remove(candidate.pair_indices.begin(), candidate.pair_indices.end(), -1),
            candidate.pair_indices.end());
    }
    mesh_candidates.erase(
        std::remove_if(
            mesh_candidates.begin(), mesh_candidates.end(),
            [](const LdmMeshCandidate& candidate) { return candidate.pair_indices.empty(); }),
        mesh_candidates.end());
    if (mesh_candidates.empty()) {
        return std::nullopt;
    }

    // Assign face indices relative to the visible face with the largest apparent height.
    for (auto& candidate : mesh_candidates) {
        (void)assign_face_indices_for_candidate(candidate, selected_pairs);
    }

    LdmDetection detection;
    detection.color                  = color;
    detection.blobs                  = std::move(selected_blobs);
    detection.pairs                  = std::move(selected_pairs);
    detection.mesh_candidates        = std::move(mesh_candidates);
    detection.rect                   = bounding_rect_from_pairs(detection.pairs);
    detection.selected_candidate_idx = 0;
    detection.center_image_px        = detection.mesh_candidates.front().estimated_center_image_px;
    return detection;
}

[[nodiscard]] std::pair<size_t, float> best_candidate_signature(const LdmDetection& detection) {
    size_t max_candidate_pairs = 0;
    float max_candidate_score  = 0.0f;
    for (const auto& candidate : detection.mesh_candidates) {
        max_candidate_pairs = std::max(max_candidate_pairs, candidate.pair_indices.size());
        max_candidate_score = std::max(max_candidate_score, candidate.preliminary_score);
    }
    return {max_candidate_pairs, max_candidate_score};
}

[[nodiscard]] bool
    detection_better_than(const LdmDetection& lhs, const std::optional<LdmDetection>& rhs) {
    if (!rhs.has_value()) {
        return true;
    }

    if (lhs.pair_count() != rhs->pair_count()) {
        return lhs.pair_count() > rhs->pair_count();
    }

    const auto [lhs_candidate_pairs, lhs_candidate_score] = best_candidate_signature(lhs);
    const auto [rhs_candidate_pairs, rhs_candidate_score] = best_candidate_signature(*rhs);
    if (lhs_candidate_pairs != rhs_candidate_pairs) {
        return lhs_candidate_pairs > rhs_candidate_pairs;
    }
    return lhs_candidate_score > rhs_candidate_score;
}

} // namespace

LdmDetector::LdmDetector(LdmDetectorConfig config) noexcept
    : config_(config) {}

std::expected<std::optional<LdmDetection>, DetectorError>
    LdmDetector::detect(const cv::Mat& image) const noexcept {
    if (image.empty()) {
        return std::unexpected(DetectorError::InvalidImage);
    }

    std::array<ArmorColor, 4> colors = {
        config_.target_color,
        ArmorColor::Red,
        ArmorColor::Blue,
        ArmorColor::Purple,
    };

    std::optional<LdmDetection> best_detection;
    std::array<bool, 4> tried_colors = {false, false, false, false};
    for (const ArmorColor color : colors) {
        const auto color_index = static_cast<size_t>(color);
        if (color_index >= tried_colors.size() || tried_colors[color_index]) {
            continue;
        }
        tried_colors[color_index] = true;

        auto detection_result = detect_laser_module(image, config_, color);
        if (!detection_result.has_value()) {
            continue;
        }
        if (detection_better_than(*detection_result, best_detection)) {
            best_detection = std::move(*detection_result);
        }
    }
    return best_detection;
}

std::expected<std::optional<LdmDetection>, DetectorError>
    LdmDetector::detect(const cv::Mat& image, ArmorColor color) const noexcept {
    if (image.empty()) {
        return std::unexpected(DetectorError::InvalidImage);
    }
    return detect_laser_module(image, config_, color);
}
} // namespace fcs::L2::ldm
