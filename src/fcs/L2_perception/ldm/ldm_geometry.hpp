#pragma once
// LDM检测器、几何尺寸配置结构体 LdmDetectorConfig / LdmGeometryConfig
#include "ldm_config.hpp"
// 项目自定义业务结构体：LightPair、LdmMeshCandidate、基础类型别名
#include "types.hpp"

// STL通用算法：排序、查找、最大最小值
#include <algorithm>
// 固定长度数组，存储固定数量三维模型点
#include <array>
// 数学函数：cos/sin/sqrt/fabs/clamp 等
#include <cmath>
// C++20 标准数学常量 π
#include <numbers>
// 求和工具 std::accumulate
#include <numeric>
// 动态数组容器，存储灯对、候选网格
#include <vector>

namespace fcs::L2::ldm {

/**
 * @brief 计算八边形单一面中心到世界原点的径向半径（单位：米）
 * 八边形外接圆半径 × cos(π/8)，得到每条边中点径向距离
 * @param geometry 大符物理几何配置
 * @return double 面中心径向半径 m
 */
[[nodiscard]] inline double octagon_face_center_radius_m(const LdmGeometryConfig& geometry) {
    return geometry.octagon_circumradius_m * std::cos(std::numbers::pi_v<double> / 8.0);
}

/**
 * @brief 根据八边形面序号，生成该面对应上下一对灯条三维世界坐标模型点
 * 大符每个八边形立面包含上下一组灯条，输出2个3D点
 * 坐标系约定：X左右、Y上下、Z前后；相机看向-Z方向
 * @param geometry 几何尺寸配置
 * @param face_index 八边形立面编号 0~7
 * @return std::array<cv::Point3f,2> 上灯条3D点、下灯条3D点
 */
[[nodiscard]] inline std::array<cv::Point3f, 2>
    pair_model_points_for_face(const LdmGeometryConfig& geometry, int face_index) {
    // 每个立面间隔45°，旋转角 = 面编号 × π/4
    const double angle      = static_cast<double>(face_index) * (std::numbers::pi_v<double> / 4.0);
    // 获取立面中心径向半径
    const double radius     = octagon_face_center_radius_m(geometry);
    // 灯对上下竖直间距一半，用于Y轴偏移
    const double half_y_sep = geometry.pair_center_separation_m * 0.5;
    // 世界X坐标：r * sinθ
    const float x           = static_cast<float>(radius * std::sin(angle));
    // 世界Z坐标：-r * cosθ，负号保证朝向相机一侧Z为负
    const float z           = static_cast<float>(-radius * std::cos(angle));
    // 上灯条Y坐标（负Y向上）
    const float top_y       = static_cast<float>(-half_y_sep);
    // 下灯条Y坐标（正Y向下）
    const float bottom_y    = static_cast<float>(half_y_sep);
    // 返回上下灯条三维点数组
    return {cv::Point3f(x, top_y, z), cv::Point3f(x, bottom_y, z)};
}

/**
 * @brief 生成完整大符八边形外轮廓16个三维顶点（8个顶面点 + 8个底面点）
 * 用于可视化、PnP绘图、包围盒计算
 * @param geometry 几何尺寸配置
 * @return std::array<cv::Point3f,16> 前8个顶面，后8个底面
 */
[[nodiscard]] inline std::array<cv::Point3f, 16>
    volume_outline_points(const LdmGeometryConfig& geometry) {
    std::array<cv::Point3f, 16> points{};
    // 八边形外接圆半径
    const double radius       = geometry.octagon_circumradius_m;
    // 顶面Y坐标：整体高度一半向上
    const float top_y         = static_cast<float>(-geometry.volume_height_m * 0.5);
    // 底面Y坐标：整体高度一半向下
    const float bottom_y      = static_cast<float>(geometry.volume_height_m * 0.5);
    // 角度偏移 π/8，顶点落在两面中间，而非坐标轴上
    const double angle_offset = std::numbers::pi_v<double> / 8.0;

    // 循环生成8组顶点，每组顶面+底面
    for (int i = 0; i < 8; ++i) {
        // 当前顶点旋转角度
        const double angle =
            angle_offset + static_cast<double>(i) * (std::numbers::pi_v<double> / 4.0);
        const float x                       = static_cast<float>(radius * std::sin(angle));
        const float z                       = static_cast<float>(-radius * std::cos(angle));
        // 存入顶面点（前8位索引 0~7）
        points[static_cast<size_t>(i)]      = cv::Point3f(x, top_y, z);
        // 存入底面点（后8位索引 8~15）
        points[static_cast<size_t>(i) + 8u] = cv::Point3f(x, bottom_y, z);
    }
    return points;
}

/**
 * @brief 根据一组灯对，计算所有灯条中点构成的包围矩形
 * 用于绘制检测框、ROI裁剪
 * @param pairs 输入灯对数组
 * @return cv::Rect2f 浮点包围矩形；无灯对返回空矩形
 */
[[nodiscard]] inline cv::Rect2f bounding_rect_from_pairs(const std::vector<LightPair>& pairs) {
    // 无灯对直接返回空矩形
    if (pairs.empty()) {
        return {};
    }

    std::vector<cv::Point2f> pts;
    pts.reserve(pairs.size() * 2);
    // 收集每一组灯对上下灯条中心点
    for (const auto& pair : pairs) {
        pts.push_back(pair.top_center_px);
        pts.push_back(pair.bottom_center_px);
    }
    // OpenCV自动计算点集最小包围矩形
    return cv::boundingRect(pts);
}

/**
 * @brief 获取灯对竖直分层间距（像素）
 * 优先使用PCA分层坐标差值，无分层数据则使用原始图像竖直距离兜底
 * @param pair 单组灯对
 * @return float 灯对上下竖直间距 px
 */
[[nodiscard]] inline float pair_layer_separation_px(const LightPair& pair) {
    return (pair.local_layer_sep_px > 0.0f) ? pair.local_layer_sep_px : pair.center_dy_px;
}

/**
 * @brief 一段连续灯对组综合打分函数
 * 综合四项指标：单灯对平均匹配分、相邻灯对间隙一致性、竖直分层对齐度、数量加分
 * 分数区间 [0,1]，分数越低代表该段灯对越不匹配大符模型
 * @param pairs 全局全部灯对数组
 * @param ordered_pair_indices 同一聚类内按水平顺序排序的灯对索引列表
 * @param start 本段起始下标
 * @param length 本段包含灯对数量
 * @param config 检测器阈值配置
 * @return float 本段综合匹配分数 0~1
 */
[[nodiscard]] inline float pair_group_score(
    const std::vector<LightPair>& pairs, const std::vector<int>& ordered_pair_indices, size_t start,
    size_t length, const LdmDetectorConfig& config) {
    // 长度为0直接返回0分，无效片段
    if (length == 0) {
        return 0.0f;
    }

    float mean_pair_score = 0.0f;
    float mean_pair_layer = 0.0f;
    // 累加本段所有灯对基础分、竖直间距
    for (size_t i = 0; i < length; ++i) {
        const auto pair_idx = static_cast<size_t>(ordered_pair_indices[start + i]);
        const auto& pair    = pairs[pair_idx];
        mean_pair_score += pair.score;
        mean_pair_layer += pair_layer_separation_px(pair);
    }
    // 求均值
    mean_pair_score /= static_cast<float>(length);
    mean_pair_layer /= static_cast<float>(length);

    // 间隙一致性权重（默认满分1）
    float gap_consistency = 1.0f;
    // 竖直分层对齐度权重（默认满分1）
    float layer_alignment = 1.0f;

    // 灯对数量>=2才计算相邻水平间隙一致性
    if (length >= 2) {
        std::vector<float> gaps;
        gaps.reserve(length - 1);
        // 遍历相邻灯对，计算水平local_order_px差值（间隙）
        for (size_t i = start + 1; i < start + length; ++i) {
            const auto lhs_idx = static_cast<size_t>(ordered_pair_indices[i - 1]);
            const auto rhs_idx = static_cast<size_t>(ordered_pair_indices[i]);
            const float gap    = pairs[rhs_idx].local_order_px - pairs[lhs_idx].local_order_px;
            // 间隙<=0代表顺序错乱，直接判定本段无效返回0分
            if (gap <= 0.0f) {
                return 0.0f;
            }
            gaps.push_back(gap);
        }

        // 间隙平均值
        const float mean_gap =
            std::accumulate(gaps.begin(), gaps.end(), 0.0f) / static_cast<float>(gaps.size());
        // 平均间隙极小，数值退化，无效片段
        if (mean_gap <= 1e-3f) {
            return 0.0f;
        }

        // 计算间隙标准差
        float sq_sum = 0.0f;
        for (const float gap : gaps) {
            const float delta = gap - mean_gap;
            sq_sum += delta * delta;
        }
        const float std_gap = std::sqrt(sq_sum / static_cast<float>(gaps.size()));
        // 变异系数 = 标准差 / 均值，衡量间隙均匀程度
        const float gap_cv  = std_gap / mean_gap;
        // 变异系数超过阈值（间隙不均匀）
        if (gap_cv > static_cast<float>(config.max_gap_cv)) {
            // 仅2组灯对直接丢弃；>=3组保留，大幅降低一致性分数，交给PnP姿态求解器判断
            if (length < 3) {
                return 0.0f;
            }
            // 注释：倾斜立面透视会导致间隙不均匀，不直接过滤，降低权重
            gap_consistency = 0.1f;
        } else {
            // 间隙越均匀，一致性分数越高
            gap_consistency =
                std::max(0.0f, 1.0f - 0.5f * gap_cv / static_cast<float>(config.max_gap_cv));
        }
    }

    // 计算竖直分层对齐度：各组灯对竖直间距不能差异过大
    if (mean_pair_layer > 1e-3f) {
        float max_delta = 0.0f;
        // 找出本段灯对竖直间距与均值的最大差值
        for (size_t i = 0; i < length; ++i) {
            const auto pair_idx = static_cast<size_t>(ordered_pair_indices[start + i]);
            max_delta           = std::max(
                max_delta, std::abs(pair_layer_separation_px(pairs[pair_idx]) - mean_pair_layer));
        }
        // 差值越大，对齐分数越低
        layer_alignment =
            std::max(0.0f, 1.0f - max_delta / std::max(1.0f, mean_pair_layer * 0.35f));
    }

    // 灯对数量加分：超过2组每多一组+0.05分，鼓励完整大符
    const float coverage_bonus = 0.05f * static_cast<float>((length > 2) ? (length - 2) : 0u);
    // 加权求和，限制分数0~1
    return std::clamp(
        0.55f * mean_pair_score + 0.30f * gap_consistency + 0.15f * layer_alignment
            + coverage_bonus,
        0.0f, 1.0f);
}

/**
 * @brief 生成一段连续灯对的网格候选LdmMeshCandidate并加入候选列表
 * 无返回值，指针原地追加候选；分数<=0直接丢弃不加入
 * @param candidates 候选列表指针
 * @param pairs 全局灯对数组
 * @param ordered_pair_indices 聚类内有序灯对索引
 * @param start 片段起始下标
 * @param length 片段灯对数量
 * @param cluster_id 所属聚类ID
 * @param config 检测器配置
 */
inline void append_mesh_candidate(
    std::vector<LdmMeshCandidate>* candidates, const std::vector<LightPair>& pairs,
    const std::vector<int>& ordered_pair_indices, size_t start, size_t length, int cluster_id,
    const LdmDetectorConfig& config) {
    LdmMeshCandidate candidate;
    // 绑定所属聚类ID
    candidate.cluster_id = cluster_id;
    // 截取本段灯对索引存入候选
    candidate.pair_indices.assign(
        ordered_pair_indices.begin() + static_cast<std::ptrdiff_t>(start),
        ordered_pair_indices.begin() + static_cast<std::ptrdiff_t>(start + length));
    // 计算本段综合打分
    candidate.preliminary_score =
        pair_group_score(pairs, ordered_pair_indices, start, length, config);
    // 分数<=0，无效候选直接返回不存入
    if (candidate.preliminary_score <= 0.0f) {
        return;
    }

    // 计算本段灯对图像中心点均值
    cv::Point2f center(0.0f, 0.0f);
    for (const int pidx : candidate.pair_indices) {
        center += pairs[static_cast<size_t>(pidx)].midpoint_px;
    }
    candidate.estimated_center_image_px =
        center * (1.0f / static_cast<float>(candidate.pair_indices.size()));
    // 移动语义存入候选列表，避免拷贝
    candidates->push_back(std::move(candidate));
}

/**
 * @brief 网格候选排序规则：优先灯对数量多，数量相同优先高分
 * 原地排序候选数组
 * @param candidates 候选列表指针
 */
inline void sort_mesh_candidates(std::vector<LdmMeshCandidate>* candidates) {
    std::sort(
        candidates->begin(), candidates->end(),
        [](const LdmMeshCandidate& a, const LdmMeshCandidate& b) {
            // 灯对数量降序
            if (a.pair_indices.size() != b.pair_indices.size()) {
                return a.pair_indices.size() > b.pair_indices.size();
            }
            // 数量相同，综合分数降序
            return a.preliminary_score > b.preliminary_score;
        });
}

/**
 * @brief 生成全部初步网格候选（枚举聚类内所有连续片段，1组、2组、多组全部生成）
 * 用于前期全量候选枚举，后续再过滤不满足检测门限的片段
 * @param pairs 全局灯对数组
 * @param config 检测器配置
 * @return 全量未过滤网格候选数组
 */
[[nodiscard]] inline std::vector<LdmMeshCandidate> build_preliminary_mesh_candidates(
    const std::vector<LightPair>& pairs, const LdmDetectorConfig& config) {
    std::vector<LdmMeshCandidate> candidates;
    // 无灯对直接返回空列表
    if (pairs.empty()) {
        return candidates;
    }

    // 提取所有存在的聚类ID，去重
    std::vector<int> cluster_ids;
    cluster_ids.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.cluster_id < 0) {
            continue;
        }
        if (std::find(cluster_ids.begin(), cluster_ids.end(), pair.cluster_id)
            == cluster_ids.end()) {
            cluster_ids.push_back(pair.cluster_id);
        }
    }

    // 遍历每个聚类单独处理
    for (const int cluster_id : cluster_ids) {
        std::vector<int> ordered_pair_indices;
        ordered_pair_indices.reserve(pairs.size());
        // 收集当前聚类所有灯对索引
        for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
            if (pairs[pair_idx].cluster_id == cluster_id) {
                ordered_pair_indices.push_back(static_cast<int>(pair_idx));
            }
        }
        if (ordered_pair_indices.empty()) {
            continue;
        }

        // 按local_order_px从小到大排序（图像水平前后顺序）
        std::sort(ordered_pair_indices.begin(), ordered_pair_indices.end(), [&](int lhs, int rhs) {
            return pairs[static_cast<size_t>(lhs)].local_order_px
                 < pairs[static_cast<size_t>(rhs)].local_order_px;
        });

        // 枚举所有连续片段：长度从总长度递减至1
        for (size_t length = ordered_pair_indices.size(); length >= 1u; --length) {
            // 遍历所有合法起始位置
            for (size_t start = 0; start + length <= ordered_pair_indices.size(); ++start) {
                append_mesh_candidate(
                    &candidates, pairs, ordered_pair_indices, start, length, cluster_id, config);
            }
            // 长度为1枚举完成后退出循环
            if (length == 1u) {
                break;
            }
        }
    }

    // 按数量、分数排序候选
    sort_mesh_candidates(&candidates);
    return candidates;
}

/**
 * @brief 生成最终用于检测的有效网格候选（仅保留满足最低灯对数量的连续片段）
 * 仅保留>=min_pairs_for_detection的完整段，同时在间隙过大处切分分段候选
 * 过滤单组、不完整片段，输出可用于PnP的有效大符候选
 * @param pairs 全局灯对数组
 * @param config 检测器配置
 * @return 过滤后有效网格候选数组
 */
[[nodiscard]] inline std::vector<LdmMeshCandidate> build_detection_mesh_candidates(
    const std::vector<LightPair>& pairs, const LdmDetectorConfig& config) {
    std::vector<LdmMeshCandidate> candidates;
    if (pairs.empty()) {
        return candidates;
    }

    // 提取全部聚类ID并去重
    std::vector<int> cluster_ids;
    cluster_ids.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.cluster_id < 0) {
            continue;
        }
        if (std::find(cluster_ids.begin(), cluster_ids.end(), pair.cluster_id)
            == cluster_ids.end()) {
            cluster_ids.push_back(pair.cluster_id);
        }
    }

    // 有效候选最低灯对数量：取2和配置最小检测对数的较大值
    const size_t min_candidate_length =
        std::max<size_t>(2u, static_cast<size_t>(std::max(config.min_pairs_for_detection, 0)));
    // 遍历每个聚类
    for (const int cluster_id : cluster_ids) {
        std::vector<int> ordered_pair_indices;
        ordered_pair_indices.reserve(pairs.size());
        // 收集当前聚类所有灯对索引
        for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
            if (pairs[pair_idx].cluster_id == cluster_id) {
                ordered_pair_indices.push_back(static_cast<int>(pair_idx));
            }
        }
        // 聚类内灯对不足最低数量，直接跳过
        if (ordered_pair_indices.size() < min_candidate_length) {
            continue;
        }

        // 按水平顺序排序灯对
        std::sort(ordered_pair_indices.begin(), ordered_pair_indices.end(), [&](int lhs, int rhs) {
            return pairs[static_cast<size_t>(lhs)].local_order_px
                 < pairs[static_cast<size_t>(rhs)].local_order_px;
        });

        // 1. 先添加完整聚类全部灯对作为主候选
        append_mesh_candidate(
            &candidates, pairs, ordered_pair_indices, 0, ordered_pair_indices.size(), cluster_id,
            config);

        // 计算本段灯对平均竖直分层间距，用于判断间隙是否过大
        float mean_pair_layer = 0.0f;
        for (const int pair_idx : ordered_pair_indices) {
            mean_pair_layer += pair_layer_separation_px(pairs[static_cast<size_t>(pair_idx)]);
        }
        mean_pair_layer /= static_cast<float>(ordered_pair_indices.size());
        // 分层间距非法/过小，不做分段切分，直接跳过
        if (!std::isfinite(mean_pair_layer) || mean_pair_layer <= 1e-3f
            || !std::isfinite(config.max_adjacent_face_order_gap_ratio)
            || config.max_adjacent_face_order_gap_ratio <= 0.0) {
            continue;
        }

        // 查找水平间隙超过阈值的分割点
        std::vector<size_t> split_points;
        for (size_t i = 1; i < ordered_pair_indices.size(); ++i) {
            const auto lhs_idx = static_cast<size_t>(ordered_pair_indices[i - 1]);
            const auto rhs_idx = static_cast<size_t>(ordered_pair_indices[i]);
            const float gap    = pairs[rhs_idx].local_order_px - pairs[lhs_idx].local_order_px;
            // 间隙/平均竖直间距 > 阈值，判定为断层，记录分割位置
            if (gap / mean_pair_layer
                > static_cast<float>(config.max_adjacent_face_order_gap_ratio)) {
                split_points.push_back(i);
            }
        }
        // 无断层，无需分段候选
        if (split_points.empty()) {
            continue;
        }

        // 根据分割点切分多段子候选
        size_t segment_start = 0;
        for (const size_t split_point : split_points) {
            const size_t length = split_point - segment_start;
            // 分段长度满足最低数量才生成候选
            if (length >= min_candidate_length) {
                append_mesh_candidate(
                    &candidates, pairs, ordered_pair_indices, segment_start, length, cluster_id,
                    config);
            }
            segment_start = split_point;
        }
        // 处理最后一段尾部
        const size_t tail_length = ordered_pair_indices.size() - segment_start;
        if (tail_length >= min_candidate_length) {
            append_mesh_candidate(
                &candidates, pairs, ordered_pair_indices, segment_start, tail_length, cluster_id,
                config);
        }
    }

    // 统一排序全部有效候选
    sort_mesh_candidates(&candidates);
    return candidates;
}

} // namespace fcs::L2::ldm