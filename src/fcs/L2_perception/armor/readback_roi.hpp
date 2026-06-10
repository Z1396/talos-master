// 头文件保护，避免重复包含导致编译错误
#pragma once

// 依赖头文件
#include "L3_estimation/tracker/types.hpp"  // L3跟踪器输出类型定义
#include "camera_config.hpp"                // 相机内参、标定配置
#include "frame.hpp"                        // 图像帧基础定义

// C++ 标准库
#include <Eigen/Core>                       // Eigen 线性代数库，向量/矩阵运算
#include <Eigen/Geometry>                   // Eigen 姿态、旋转、变换相关
#include <algorithm>                        // 标准算法：最值、范围截断等
#include <array>                            // 固定长度数组
#include <cmath>                            // 数学函数、浮点判断
#include <cstdint>                          // 定长整型（时间戳等）
#include <limits>                           // 数值极限、无穷大定义
#include <mutex>                            // 互斥锁，多线程同步
#include <optional>                         // 可选返回值，表达“有/无结果”
#include <utility>                          // 移动语义、pair 等
#include <vector>                           // 动态数组

// OpenCV 核心模块，图像、矩形、点结构
#include <opencv2/core.hpp>

// L2 感知层命名空间：装甲板 ROI 回读、跟踪器数据缓存、投影相关逻辑
namespace fcs::L2 {

/**
 * @brief 跟踪结果回读 ROI 配置
 * 利用上层跟踪结果在原图上截取感兴趣区域，缩小检测范围、提速降噪
 */
struct ArmorReadbackRoiConfig {
    bool enabled{false};                // 总开关：是否启用跟踪结果引导ROI
    double stale_timeout_s{0.20};       // 数据超时阈值(秒)，超过则认为跟踪数据失效
    double margin_ratio_x{0.10};        // ROI 左右扩展比例
    double margin_ratio_y{0.10};        // ROI 上下扩展比例
    std::array<double, 3> box_size_m{0.8, 0.8, 0.6}; // 目标包围盒物理尺寸 [x,y,z] 单位：米
};

/**
 * @brief 跟踪器数据快照
 * 缓存一帧完整的L3跟踪输出，供L2感知层使用
 */
struct TrackerReadbackSnapshot {
    bool valid{false};                          // 快照是否有效
    uint64_t timestamp_ns{0};                  // 快照对应时间戳(纳秒)
    uint64_t projection_timestamp_ns{0};       // 投影计算使用的时间戳
    int selected_armor_id{0};                   // 当前选中的装甲板ID
    int rough_selected_armor_id{0};             // 粗选装甲板ID
    L3::TrackerOutput tracker{};                // L3 跟踪器完整输出数据
};

/**
 * @brief 多线程安全的跟踪器数据缓存
 * 生产者(L3跟踪线程)写入，消费者(L2感知线程)读取，加锁保证线程安全
 */
class TrackerReadbackCache {
public:
    // 默认构造
    TrackerReadbackCache() noexcept = default;

    /**
     * @brief 拷贝构造：加锁读取源对象快照
     */
    TrackerReadbackCache(const TrackerReadbackCache& other) noexcept {
        std::scoped_lock lock(other.mutex_);
        snapshot_ = other.snapshot_;
    }

    /**
     * @brief 拷贝赋值：双锁保护，防止并发读写
     */
    TrackerReadbackCache& operator=(const TrackerReadbackCache& other) noexcept {
        if (this == &other) {
            return *this;
        }
        std::scoped_lock lock(mutex_, other.mutex_);
        snapshot_ = other.snapshot_;
        return *this;
    }

    /**
     * @brief 移动构造：加锁读取源数据
     */
    TrackerReadbackCache(TrackerReadbackCache&& other) noexcept {
        std::scoped_lock lock(other.mutex_);
        snapshot_ = other.snapshot_;
    }

    /**
     * @brief 移动赋值：双锁保护
     */
    TrackerReadbackCache& operator=(TrackerReadbackCache&& other) noexcept {
        if (this == &other) {
            return *this;
        }
        std::scoped_lock lock(mutex_, other.mutex_);
        snapshot_ = other.snapshot_;
        return *this;
    }

    /**
     * @brief 读取当前缓存快照
     * @return 拷贝一份快照返回
     */
    [[nodiscard]] TrackerReadbackSnapshot load() const noexcept {
        std::scoped_lock lock(mutex_);
        return snapshot_;
    }

    /**
     * @brief 写入新的跟踪快照
     * @param snapshot 待存入的跟踪数据
     */
    void store(const TrackerReadbackSnapshot& snapshot) noexcept {
        std::scoped_lock lock(mutex_);
        snapshot_ = snapshot;
    }

    /**
     * @brief 置为无效，清空缓存
     * @param timestamp_ns 重置后使用的时间戳
     */
    void invalidate(uint64_t timestamp_ns = 0) noexcept {
        std::scoped_lock lock(mutex_);
        snapshot_ = TrackerReadbackSnapshot{.valid = false, .timestamp_ns = timestamp_ns};
    }

private:
    mutable std::mutex mutex_;  // 互斥锁，const方法也可加锁，故用mutable
    TrackerReadbackSnapshot snapshot_; // 缓存的跟踪快照
};

/**
 * @brief 推理后端输入分辨率
 * 记录模型要求的输入宽高，用于ROI宽高比对齐
 */
struct BackendInputResolution {
    int width{0};
    int height{0};

    /**
     * @brief 判断分辨率是否合法
     */
    [[nodiscard]] bool valid() const noexcept { return width > 0 && height > 0; }

    /**
     * @brief 计算宽高比
     */
    [[nodiscard]] double aspect_ratio() const noexcept {
        return valid() ? static_cast<double>(width) / static_cast<double>(height) : 1.0;
    }
};

/**
 * @brief 判断跟踪快照是否“新鲜有效”（未超时）
 * @param snapshot 跟踪快照
 * @param config ROI超时配置
 * @param image_timestamp_ns 当前图像时间戳
 * @return true=数据可用，false=超时/无效
 */
[[nodiscard]] inline bool tracker_snapshot_is_fresh(
    const TrackerReadbackSnapshot& snapshot, const ArmorReadbackRoiConfig& config,
    uint64_t image_timestamp_ns) noexcept {
    // 基础合法性校验
    if (!snapshot.valid || snapshot.timestamp_ns == 0
        || image_timestamp_ns < snapshot.timestamp_ns) {
        return false;
    }
    // 换算最大允许时差：秒 → 纳秒
    const auto max_age_ns = static_cast<uint64_t>(std::max(0.0, config.stale_timeout_s) * 1e9);
    // 判断时间差是否在允许范围内
    return image_timestamp_ns - snapshot.timestamp_ns <= max_age_ns;
}

/**
 * @brief 校验原始ROI矩形是否合法
 * 检查坐标、宽高是否为有限浮点数，且宽高大于0
 */
[[nodiscard]] inline bool is_valid_raw_roi(const cv::Rect2f& roi) noexcept {
    return std::isfinite(roi.x) && std::isfinite(roi.y) && std::isfinite(roi.width)
        && std::isfinite(roi.height) && roi.width > 0.0f && roi.height > 0.0f;
}

/**
 * @brief 按配置比例向外扩展ROI区域
 * @param roi 原始ROI
 * @param config 扩展比例配置
 * @return 扩展后的新ROI
 */
[[nodiscard]] inline cv::Rect2f
    expand_raw_roi(const cv::Rect2f& roi, const ArmorReadbackRoiConfig& config) noexcept {
    if (!is_valid_raw_roi(roi)) {
        return {};
    }
    // 限制比例范围 [0, 10]，防止配置异常
    const float margin_ratio_x =
        static_cast<float>(std::max(0.0, std::min(config.margin_ratio_x, 10.0)));
    const float margin_ratio_y =
        static_cast<float>(std::max(0.0, std::min(config.margin_ratio_y, 10.0)));
    // 计算单边扩展像素
    const float margin_x = roi.width * margin_ratio_x;
    const float margin_y = roi.height * margin_ratio_y;
    // 整体向外扩，左上角偏移，宽高增加两倍边距
    return cv::Rect2f(
        roi.x - margin_x, roi.y - margin_y, roi.width + 2.0f * margin_x,
        roi.height + 2.0f * margin_y);
}

/**
 * @brief 解析最终可用ROI
 * 流程：扩展ROI → 对齐模型输入宽高比 → 边界裁剪 → 转为整数像素矩形
 * @param frame_size 原图尺寸
 * @param raw_roi 原始投影ROI
 * @param config ROI配置
 * @param input_resolution 模型输入分辨率
 * @return 合法整数ROI，失败返回nullopt
 */
[[nodiscard]] inline std::optional<cv::Rect> resolve_readback_roi(
    const cv::Size& frame_size, const cv::Rect2f& raw_roi, const ArmorReadbackRoiConfig& config,
    const BackendInputResolution& input_resolution) noexcept {
    // 前置合法性校验
    if (frame_size.width <= 0 || frame_size.height <= 0 || !input_resolution.valid()
        || !is_valid_raw_roi(raw_roi)) {
        return std::nullopt;
    }

    // 1. 扩展ROI
    const cv::Rect2f expanded = expand_raw_roi(raw_roi, config);
    if (!is_valid_raw_roi(expanded)) {
        return std::nullopt;
    }

    // 2. 保证ROI不小于模型最小输入尺寸
    double final_w      = std::max<double>(expanded.width, input_resolution.width);
    double final_h      = std::max<double>(expanded.height, input_resolution.height);
    const double aspect = input_resolution.aspect_ratio();

    // 3. 强制对齐模型宽高比
    if (final_w / final_h < aspect) {
        final_w = final_h * aspect;
    } else {
        final_h = final_w / aspect;
    }

    // 超出原图尺寸直接放弃
    if (final_w > frame_size.width || final_h > frame_size.height) {
        return std::nullopt;
    }

    // 4. 以扩展框中心为基准，重新计算ROI左上角坐标
    const double cx = expanded.x + expanded.width * 0.5;
    const double cy = expanded.y + expanded.height * 0.5;
    double x        = cx - final_w * 0.5;
    double y        = cy - final_h * 0.5;

    // 5. 限制在图像范围内
    x = std::clamp(x, 0.0, static_cast<double>(frame_size.width) - final_w);
    y = std::clamp(y, 0.0, static_cast<double>(frame_size.height) - final_h);

    // 6. 浮点坐标转整数像素
    const int left   = std::max(0, static_cast<int>(std::floor(x)));
    const int top    = std::max(0, static_cast<int>(std::floor(y)));
    const int right  = std::min(frame_size.width, static_cast<int>(std::ceil(x + final_w)));
    const int bottom = std::min(frame_size.height, static_cast<int>(std::ceil(y + final_h)));

    // 校验宽高是否有效
    if (right <= left || bottom <= top) {
        return std::nullopt;
    }

    return cv::Rect(left, top, right - left, bottom - top);
}

/**
 * @brief 从跟踪结果提取目标包围盒中心(世界坐标系)与偏航角
 * 兼容普通机器人、哨兵两种目标类型
 * @param tracker L3跟踪器输出
 * @return 中心3D坐标 + 偏航角，无目标返回nullopt
 */
[[nodiscard]] inline std::optional<std::pair<Eigen::Vector3d, double>>
    tracker_box_center_and_yaw(const L3::TrackerOutput& tracker) noexcept {
    // 普通机器人目标
    if (const auto* robot = tracker.robot_state()) {
        return std::pair{
            Eigen::Vector3d(
                robot->position.x(), robot->position.y(), 0.5 * (robot->position.z() + robot->z1)),
            robot->yaw};
    }
    // 哨兵/基地类目标
    if (const auto* outpost = tracker.outpost_state()) {
        const double mean_z = (outpost->z[0] + outpost->z[1] + outpost->z[2]) / 3.0;
        return std::pair{
            Eigen::Vector3d(outpost->position.x(), outpost->position.y(), mean_z), outpost->yaw};
    }
    return std::nullopt;
}

/**
 * @brief 从跟踪结果获取所有装甲板位姿
 * @param tracker L3跟踪输出
 * @return 装甲板位姿数组
 */
[[nodiscard]] inline std::vector<Eigen::Vector4d>
    tracker_armor_poses(const L3::TrackerOutput& tracker) noexcept {
    if (const auto* robot = tracker.robot_state()) {
        const auto poses = robot->armor_poses();
        return {poses.begin(), poses.end()};
    }
    if (const auto* outpost = tracker.outpost_state()) {
        const auto poses = outpost->armor_poses();
        return {poses.begin(), poses.end()};
    }
    return {};
}

/**
 * @brief 将世界坐标系下3D包围盒投影到图像，得到像素ROI
 * 3D包围盒八个顶点 → 相机针孔投影 → 外接矩形
 * @param center_odom 包围盒世界坐标中心
 * @param yaw 目标偏航角
 * @param T_odom_camera 世界坐标系 → 相机光学坐标系变换
 * @param camera_config 相机内参
 * @param box_size_m 包围盒物理尺寸(米)
 * @return 投影后浮点ROI，失败返回nullopt
 */
[[nodiscard]] inline std::optional<cv::Rect2f> project_box_to_image(
    const Eigen::Vector3d& center_odom, double yaw,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const CameraConfig& camera_config, const std::array<double, 3>& box_size_m) noexcept {
    // 提取相机内参 fx, fy, cx, cy
    const double fx = camera_config.camera_matrix(0, 0);
    const double fy = camera_config.camera_matrix(1, 1);
    const double cx = camera_config.camera_matrix(0, 2);
    const double cy = camera_config.camera_matrix(1, 2);
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy)
        || fx <= 0.0 || fy <= 0.0) {
        return std::nullopt;
    }

    // 半边长，由整体尺寸换算
    const double hx = std::max(0.0, box_size_m[0]) * 0.5;
    const double hy = std::max(0.0, box_size_m[1]) * 0.5;
    const double hz = std::max(0.0, box_size_m[2]) * 0.5;
    if (hx <= 0.0 || hy <= 0.0 || hz <= 0.0) {
        return std::nullopt;
    }

    // 目标自身偏航旋转
    const Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
    // 世界→相机旋转、平移
    const Eigen::Matrix3d R_odom_camera = T_odom_camera.rotation();
    const Eigen::Vector3d t_odom_camera = T_odom_camera.translation();

    // 内部lambda：单个世界点 → 像素点投影
    auto project_point = [&](const Eigen::Vector3d& point_odom) -> std::optional<cv::Point2f> {
        // 坐标变换：世界点 → 相机坐标系
        const Eigen::Vector3d point_camera = R_odom_camera * point_odom + t_odom_camera;
        // 深度合法性检查（相机前方）
        if (!std::isfinite(point_camera.x()) || !std::isfinite(point_camera.y())
            || !std::isfinite(point_camera.z()) || point_camera.z() <= 1e-3) {
            return std::nullopt;
        }
        // 针孔相机模型投影计算像素坐标
        const double u = fx * point_camera.x() / point_camera.z() + cx;
        const double v = fy * point_camera.y() / point_camera.z() + cy;
        if (!std::isfinite(u) || !std::isfinite(v)) {
            return std::nullopt;
        }
        return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    };

    // 遍历3D包围盒8个顶点，求像素坐标极值
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();

    for (const double sx : {-hx, hx}) {
        for (const double sy : {-hy, hy}) {
            for (const double sz : {-hz, hz}) {
                // 顶点世界坐标 = 中心 + 旋转后的偏移
                const Eigen::Vector3d point_odom =
                    center_odom + yaw_rot * Eigen::Vector3d(sx, sy, sz);
                const auto pixel = project_point(point_odom);
                if (!pixel) {
                    return std::nullopt;
                }
                // 更新四个极值
                min_x = std::min(min_x, pixel->x);
                min_y = std::min(min_y, pixel->y);
                max_x = std::max(max_x, pixel->x);
                max_y = std::max(max_y, pixel->y);
            }
        }
    }

    // 由极值构造外接矩形
    cv::Rect2f roi(min_x, min_y, max_x - min_x, max_y - min_y);
    if (!is_valid_raw_roi(roi)) {
        return std::nullopt;
    }
    return roi;
}

/**
 * @brief 封装接口：直接从跟踪结果投影得到图像ROI
 * @param tracker L3跟踪输出
 * @param T_odom_camera 坐标变换
 * @param camera_config 相机配置
 * @param box_size_m 物理包围盒尺寸
 * @return 投影ROI
 */
[[nodiscard]] inline std::optional<cv::Rect2f> project_tracker_box_to_image(
    const L3::TrackerOutput& tracker,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const CameraConfig& camera_config, const std::array<double, 3>& box_size_m) noexcept {
    // 先提取中心与偏航角
    const auto center_and_yaw = tracker_box_center_and_yaw(tracker);
    if (!center_and_yaw) {
        return std::nullopt;
    }
    // 调用通用投影函数
    return project_box_to_image(
        center_and_yaw->first, center_and_yaw->second, T_odom_camera, camera_config, box_size_m);
}

} // namespace fcs::L2