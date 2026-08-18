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
// 工厂函数模块：创建 TraditionalBackend 传统装甲检测器实例
// ============================================================================

/**
 * @brief 创建传统装甲检测后端实例，工厂静态方法
 * @param config 装甲检测配置参数
 * @return std::expected<TraditionalBackend, std::string>
 *         成功返回检测器对象；失败返回错误字符串
 * @noexcept 不会抛出C++异常，错误全部走expected错误分支
 */
std::expected<TraditionalBackend, std::string> TraditionalBackend::create(Config config) noexcept {
    // 构造后端对象，配置转移进入对象内部，避免拷贝
    TraditionalBackend backend(std::move(config));
    const auto& cfg = backend.get_config();

    // 校验：数字分类器模型文件路径不能为空
    if (cfg.classifier_model_path.empty()) {
        return std::unexpected("[TraditionalBackend::create] Classifier model path is empty");
    }

    // 工厂创建 ONNX 数字分类器实例
    auto classifier_result =
        TraditionalClassifier::create(cfg.classifier_model_path, cfg.classifier_use_softmax);
    // 创建分类器失败，向上返回错误信息
    if (!classifier_result) {
        return std::unexpected(
            "[TraditionalBackend::create] Failed to create ONNX classifier for '"
            + cfg.classifier_model_path + "': " + classifier_result.error());
    }

    // 将分类器移动存入unique_ptr，独占所有权
    backend.classifier_ = std::make_unique<TraditionalClassifier>(std::move(*classifier_result));
    // 构造完成，返回检测器实例
    return backend;
}

/**
 * @brief 构造函数，接收配置，使用移动语义接管配置对象
 * @param config 检测器配置
 * @noexcept
 */
TraditionalBackend::TraditionalBackend(Config config) noexcept
    : config_{std::move(config)} {}

// ============================================================================
// Detection Entry Point
// 检测主入口：输入图像+装甲颜色，输出装甲检测结果
// ============================================================================

/**
 * @brief 传统装甲检测核心实现函数
 * @param input 输入BGR原始图像
 * @param color 目标装甲颜色（本版本不再做颜色过滤，仅保留参数）
 * @return DetectionResult 装甲检测结果数组 / 错误字符串
 * @noexcept
 */
TraditionalBackend::DetectionResult
    TraditionalBackend::detect_impl(const cv::Mat& input, ArmorColor color) noexcept {
    // 图像为空直接返回错误
    if (input.empty()) {
        return std::unexpected("Empty image passed to traditional backend");
    }

    const auto& cfg = get_config();

    // 步骤1：图像预处理，生成二值图，把灯条区域凸显出来
    cv::Mat binary = preprocess_image(input, cfg);

    // 步骤2：从二值图中提取所有候选灯条
    auto lights = find_lights(input, binary, cfg);

    // 预先转灰度图，后续用于角点修正（本代码未启用角点修正，提前预留）
    cv::Mat gray_img;
    cv::cvtColor(input, gray_img, cv::COLOR_BGR2GRAY);

    // 步骤3：灯条两两配对，组合成装甲，输出候选装甲列表
    auto detections = match_lights(lights, cfg, gray_img);

    // 步骤5：对每一块候选装甲做数字分类，过滤无效装甲
    for (size_t d = 0; d < detections.size(); ++d) {
        auto& detection = detections[d];

        // 透视变换抠出装甲数字ROI图像
        cv::Mat number_img = extract_number(input, detection.corners, detection.type);

        if (!number_img.empty()) {
            float confidence = 0.0f;
            // ONNX分类器推理：输入数字图，返回装甲编号+置信度
            auto result      = classifier_->classify(number_img, confidence);

            if (result) {
                // 推理成功，写入装甲编号与置信度
                detection.name       = *result;
                detection.confidence = confidence;
            } else {
                // 分类推理失败，标记为无效装甲
                detection.name       = ArmorName::Invalid;
                detection.confidence = 0.0f;
            }
        }

        // 5. 使用 erase‑remove_if 范式批量过滤不合格装甲
        detections.erase(
            std::remove_if(
                detections.begin(), detections.end(),
                [&cfg](const ArmorDetection& detection) {
                    // 过滤1：置信度低于阈值直接丢弃
                    if (detection.confidence < cfg.classifier_confidence_threshold) {
                        return true;
                    }

                    // 过滤2：编号无效直接丢弃
                    if (detection.name == ArmorName::Invalid) {
                        return true;
                    }

                    // 过滤3：开启大小装甲类型校验，过滤逻辑矛盾样本
                    if (cfg.classifier_enable_type_filtering) {
                        if (detection.type == ArmorType::Large) {
                            // 大装甲不可能是：前哨站、2号、哨兵
                            if (detection.name == ArmorName::Outpost
                                || detection.name == ArmorName::Two
                                || detection.name == ArmorName::Sentry) {
                                return true;
                            }
                        } else if (detection.type == ArmorType::Small) {
                            // 小装甲不可能是：1号、基地、大基地
                            if (detection.name == ArmorName::One
                                || detection.name == ArmorName::Base
                                || detection.name == ArmorName::BaseLarge) {
                                return true;
                            }
                        }
                    }

                    return false; // 返回false代表保留该装甲
                }),
            detections.end());
    }

    // 返回过滤完成后的装甲检测列表
    return detections;
}

// Preprocessing
// ============================================================================
/**
 * @brief 图像预处理，生成灯条二值掩码
 * @param bgr_img 原始BGR图像
 * @param cfg 传统检测器配置
 * @return cv::Mat 单通道二值图，灯条为白色255，背景黑色0
 * @noexcept
 */
cv::Mat TraditionalBackend::preprocess_image(
    const cv::Mat& bgr_img, const ArmorTraditionalConfig& cfg) const noexcept {
    // 简易二值模式：直接灰度+固定阈值
    if (!cfg.advanced_binary) {
        cv::Mat gray;
        cv::cvtColor(bgr_img, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, gray, cfg.binary_threshold, 255, cv::THRESH_BINARY);
        return gray;
    }

    // 高级色度二值模式：利用色度（max‑min）分离灯条，抗环境光
    std::vector<cv::Mat> ch;
    cv::split(bgr_img, ch); // 拆分BGR三通道

    cv::Mat max01, min01, maxc, minc, chroma;
    // 求每个像素三通道最大值
    cv::max(ch[0], ch[1], max01);
    cv::max(max01, ch[2], maxc);

    // 求每个像素三通道最小值
    cv::min(ch[0], ch[1], min01);
    cv::min(min01, ch[2], minc);

    // chroma = 最大值‑最小值，代表色彩饱和度，灯条区域色度高
    cv::subtract(maxc, minc, chroma);

    cv::Mat mask;
    // 动态阈值策略：根据图像色度最大值自动调整分割阈值，竞赛环境推荐
    double min_val, max_val;
    cv::minMaxLoc(chroma, &min_val, &max_val);

    // 动态阈值 = 最大色度 * dark_percentage比例
    int dynamic_thresh = static_cast<int>(
        max_val
        * cfg.dark_percentage);
    // 兜底：不能低于配置的基础阈值，防止全黑图片阈值为0
    dynamic_thresh = std::max(dynamic_thresh, cfg.binary_threshold);

    // 色度图二值化得到灯条掩码
    cv::threshold(chroma, mask, dynamic_thresh, 255, cv::THRESH_BINARY);

    return mask;
}

// ============================================================================
// Light Detection
// 灯条检测模块：从二值图轮廓筛选合法灯条
// ============================================================================

/**
 * @brief 寻找图像中所有合法灯条
 * @param bgr_img 原始彩色图，用于判断灯条颜色
 * @param binary_img 预处理后的二值掩码图
 * @param cfg 检测器配置
 * @return std::vector<Light> 灯条对象数组，按X坐标从小到大排序
 * @noexcept
 */
std::vector<Light> TraditionalBackend::find_lights(
    const cv::Mat& bgr_img, const cv::Mat& binary_img,
    const ArmorTraditionalConfig& cfg) const noexcept {
    std::vector<std::vector<cv::Point>> contours;
    // 查找最外层轮廓，不获取内层嵌套轮廓
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<Light> lights;
    lights.reserve(contours.size()); // 预分配内存，减少扩容开销

    for (const auto& contour : contours) {
        // 轮廓点太少，噪声直接跳过
        if (contour.size() < 6)
            continue;

        // 使用轮廓构造Light对象，内部会计算中心、长宽、倾斜角
        Light light(contour);

        // 使用长宽比、倾斜角过滤，判断是否是有效灯条
        if (is_light(light, cfg)) {
            // 沿着轮廓采样像素，统计红蓝通道求和，判断灯条颜色
            int sum_r = 0, sum_b = 0;
            for (const auto& point : contour) {
                const auto& pixel = bgr_img.at<cv::Vec3b>(point.y, point.x);
                sum_r += pixel[2]; // OpenCV BGR顺序：索引2是R
                sum_b += pixel[0]; // 索引0是B
            }

            // 计算红蓝平均差值
            const int avg_diff = std::abs(sum_r - sum_b) / static_cast<int>(contour.size());
            // 差值大于阈值才判定红/蓝；否则保持中性白色Neutral
            if (avg_diff > cfg.light_color_diff_thresh) {
                light.color = sum_r > sum_b ? ArmorColor::Red : ArmorColor::Blue;
            }
            lights.emplace_back(light);
        }
    }

    // 将灯条集合按照画面X坐标从小到大排序，方便后续两两配对
    std::ranges::sort(lights.begin(), lights.end(), [](const Light& l1, const Light& l2) {
        return l1.center.x < l2.center.x;
    });

    return lights;
}

/**
 * @brief 判断单个轮廓是否符合灯条几何特征
 * @param light 灯条实例
 * @param cfg 配置参数
 * @return true=合法灯条 false=噪声
 * @noexcept
 */
bool TraditionalBackend::is_light(
    const Light& light, const ArmorTraditionalConfig& cfg) const noexcept {
    // 灯条宽高比，灯条是细长条状
    const float ratio   = static_cast<float>(light.width) / static_cast<float>(light.length);
    const bool ratio_ok = (cfg.light_min_ratio < ratio) && (ratio < cfg.light_max_ratio);

    // 灯条倾斜角度不能过大
    const bool angle_ok = light.tilt_angle < cfg.light_max_angle;

    // 两个条件同时满足才认为是灯条
    return ratio_ok && angle_ok;
}

// ============================================================================
// Armor Matching
// 灯条配对模块：两个灯条组合成为装甲
// ============================================================================

/**
 * @brief 遍历灯条两两配对，生成候选装甲
 * @param lights 已经按X排好序的灯条列表
 * @param cfg 检测器配置
 * @param gray_img 灰度图（本版本未使用，预留角点修正）
 * @return std::vector<ArmorDetection> 候选装甲列表（未做数字分类过滤）
 * @noexcept
 */
std::vector<ArmorDetection> TraditionalBackend::match_lights(
    std::vector<Light>& lights, const ArmorTraditionalConfig& cfg,
    [[maybe_unused]] const cv::Mat gray_img) const noexcept {
    // 预分配最大可能空间 n*(n‑1)/2，避免vector反复扩容
    std::vector<ArmorDetection> detections;
    detections.reserve(lights.size() * (lights.size() - 1) / 2 + 16);

    // NOTE：本版本移除颜色过滤，允许红‑红、蓝‑蓝、红‑蓝配对；颜色保留给下游使用

    // 双重循环：两两灯条配对
    for (auto it1 = lights.begin(); it1 != lights.end(); ++it1) {
        // 根据左边灯条长度计算两个灯条允许最大X方向距离，超过直接break（灯条按X有序）
        const double max_iter_width = it1->length * cfg.armor_max_large_center_distance;

        // it2从it1下一个开始，避免重复配对（(A,B) (B,A)）
        for (auto it2 = it1 + 1; it2 != lights.end(); ++it2) {

            // 判断两个灯条中间有没有其他灯条，如果中间存在灯条，不组成装甲
            const auto i = static_cast<size_t>(std::distance(lights.begin(), it1));
            const auto j = static_cast<size_t>(std::distance(lights.begin(), it2));
            if (contains_light(i, j, lights))
                continue;

            // 灯条已经按X升序，X间距超过阈值，后面it2只会更远，直接break内层循环剪枝
            if (it2->center.x - it1->center.x > max_iter_width)
                break;

            // 几何校验：判断这一对灯条是否构成合法装甲，返回大装甲/小装甲/无效
            const auto type = is_armor(*it1, *it2, cfg);
            if (type == ArmorType::Invalid) {
                continue;
            }

            // NOTE：这里关闭角点修正，和ov版本行为对齐

            // 根据两个灯条生成装甲检测结构体，填充四个角点坐标
            auto detection = make_detection_from_lights(*it1, *it2);
            detection.type = type;
            detections.emplace_back(detection);
        }
    }

    return detections;
}

/**
 * @brief 判断下标i,j两个灯条之间是否存在其他干扰灯条
 * @param i 左边灯条下标
 * @param j 右边灯条下标 j>i
 * @param lights 灯条数组
 * @return true：中间存在灯条，不能配对；false：中间干净，可以配对
 * @noexcept
 */
bool TraditionalBackend::contains_light(
    size_t i, size_t j, const std::vector<Light>& lights) noexcept {
    const auto& light_1 = lights[i];
    const auto& light_2 = lights[j];

    // 取出两个灯条上下顶点，计算包围矩形
    std::array<cv::Point2f, 4> points = {light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect                = cv::boundingRect(points);

    // 两个灯条平均长度、平均宽度
    const double avg_length = (light_1.length + light_2.length) / 2.0;
    const double avg_width  = (light_1.width + light_2.width) / 2.0;

    // 遍历i+1 ~ j‑1之间所有灯条
    for (size_t k = i + 1; k < j; ++k) {
        const auto& test_light = lights[k];

        // 过宽，大概率是数字，跳过
        if (test_light.width > 2 * avg_width)
            continue;
        // 太短，噪声，跳过
        if (test_light.length < 0.5 * avg_length)
            continue;

        // 如果该灯条的顶点/中心点落在包围盒内部，代表中间有灯条
        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom)
            || bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 校验一对灯条几何，判断是大装甲、小装甲还是无效
 * @param light_1 左灯条
 * @param light_2 右灯条
 * @param cfg 配置
 * @return ArmorType::Large / Small / Invalid
 * @noexcept
 */
ArmorType TraditionalBackend::is_armor(
    const Light& light_1, const Light& light_2, const ArmorTraditionalConfig& cfg) const noexcept {
    // 灯条长度比例校验，两个灯条不能长短差距过大
    const float light_length_ratio = (light_1.length < light_2.length)
                                       ? static_cast<float>(light_1.length / light_2.length)
                                       : static_cast<float>(light_2.length / light_1.length);
    if (light_length_ratio <= cfg.armor_min_light_ratio) {
        return ArmorType::Invalid;
    }

    // 灯条中心距离，除以灯条平均长度做归一化，消除距离远近影响
    const float avg_light_length = static_cast<float>((light_1.length + light_2.length) / 2.0);
    const float center_distance =
        static_cast<float>(cv::norm(light_1.center - light_2.center) / avg_light_length);

    // 满足小装甲距离区间 或者大装甲距离区间才算合法
    const auto center_distance_ok = (cfg.armor_min_small_center_distance <= center_distance
                                     && cfg.armor_max_small_center_distance > center_distance)
                                 || (cfg.armor_min_large_center_distance <= center_distance
                                     && cfg.armor_max_large_center_distance > center_distance);
    if (!center_distance_ok) {
        return ArmorType::Invalid;
    }

    // 装甲倾斜角度校验，两个灯条连线不能歪得太厉害
    const cv::Point2f diff = light_1.center - light_2.center;
    const float angle =
        static_cast<float>(std::abs(std::atan(diff.y / diff.x)) / std::numbers::pi * 180.0);
    if (angle >= cfg.armor_max_angle) {
        return ArmorType::Invalid;
    }

    // 根据归一化距离区分大/小装甲
    return center_distance > cfg.armor_min_large_center_distance ? ArmorType::Large
                                                                 : ArmorType::Small;
}

// ============================================================================
// Number Extraction (Perspective Transform + OTSU Binarization)
// 数字抠图模块：透视变换把倾斜装甲矫正为正，抠出数字ROI
// ============================================================================

/**
 * @brief 透视变换矫正装甲，抠出数字区域，输出二值数字图，送入分类器
 * @param src 原始BGR图像
 * @param lights_vertices 装甲四个角点
 * @param armor_type 大/小装甲，决定矫正后画布宽度
 * @return cv::Mat 20×28单通道二值数字图
 * @noexcept
 */
cv::Mat TraditionalBackend::extract_number(
    const cv::Mat& src, const std::array<cv::Point2f, 4>& lights_vertices,
    ArmorType armor_type) const noexcept {
    // 矫正后画布上灯条高度
    static const int light_length = 12;
    // warp变换后画布总高度
    static const int warp_height       = 28;
    // 小装甲、大装甲矫正画布宽度
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // 数字ROI实际截取尺寸
    static const cv::Size roi_size(20, 28);

    // 灯条在矫正画布上Y坐标，上下留空，中间留给数字
    const int top_light_y    = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    // 根据装甲类型选择矫正画布宽度
    const int warp_width = armor_type == ArmorType::Small ? small_armor_width : large_armor_width;

    // 目标矫正后的四个顶点坐标（理想直立装甲）
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };

    cv::Mat number_image;
    // 重排原始装甲四个角点顺序，和target_vertices一一对应
    cv::Point2f lights_vertices2[4] = {
        lights_vertices[3], lights_vertices[0], lights_vertices[1], lights_vertices[2]};

    // 计算透视变换矩阵
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices2, target_vertices);
    // 执行透视矫正，把倾斜的装甲掰正
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // 从矫正完成图中间抠出数字ROI区域
    number_image =
        number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // 转为灰度，使用OTSU自适应阈值做二值化，数字变白背景变黑
    cv::cvtColor(number_image, number_image, cv::COLOR_BGR2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    return number_image;
}

} // namespace fcs::L2
