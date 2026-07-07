// 本模块对外头文件
#include "L2_perception/armor/systems.hpp"

// 装甲感知内部依赖
#include "L2_perception/armor/backend.hpp"    // 检测器后端抽象（ONNX/TensorRT/Axera）
#include "L2_perception/armor/config.hpp"     // 装甲检测、相机、ROI 配置结构体
#include "L2_perception/armor/readback_roi.hpp"// 跟踪器引导 ROI 裁剪逻辑
#include "L2_perception/armor/solver.hpp"     // PnP+BA 位姿求解器
#include "core/math/normalize.hpp"            // 角度归一化数学工具
#include "core/runtime.hpp"                  // 运行时能力标记
#include "core/types.hpp"                    // 全局基础类型定义
#include "frame.hpp"                         // 图像帧结构体 ImageFrame
#include "scheduler/scheduler.hpp"           // ECS 调度器核心 Scheduler
// OpenCV 图像处理
#include <opencv2/imgproc.hpp>
// C++17 可选返回值
#include <optional>
// 日志
#include <spdlog/spdlog.h>

// 通用算法、数学、枚举反射
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <magic_enum.hpp>

// 项目顶层命名空间 fcs，二级感知 L2
namespace fcs::L2 {

// 匿名命名空间：仅本文件内部可见工具函数，不对外暴露
namespace {

/**
 * @brief 装甲四点轮廓按**逆时针角度排序**（标准装甲四点顺序：左上、右上、右下、左下）
 * @param points 网络输出无序四点
 * @return 排序后标准顺序四点数组
 * 原理：计算四点相对于轮廓中心的向量与X轴夹角，从大到小排序实现逆时针排布
 */
std::array<cv::Point2f, 4> sort_corners(const std::array<cv::Point2f, 4>& points) {
    // 1. 计算四点几何中心
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& p : points) {
        center += p;
    }
    center *= 0.25f;

    // 存储点+对应夹角
    struct Item {
        cv::Point2f point;
        float angle;
    };
    std::vector<Item> sorted;
    sorted.reserve(4);

    for (const auto& p : points) {
        // 点相对于中心的偏移向量
        cv::Point2f dir = p - center;

        // 单位化向量
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 1e-6f) {
            dir.x /= len;
            dir.y /= len;
        }

        // 计算向量与X正轴夹角，等价 Rust glam::Vec2::angle_to(Vec2::X)
        // 叉乘 cross = -dir.y，点乘 dot = dir.x
        // atan2(叉乘, 点乘) 得到 [-180,180] 角度
        float angle = std::atan2(-dir.y, dir.x) * 180.0f / static_cast<float>(CV_PI);
        sorted.push_back({p, angle});
    }

    // 角度降序排序：实现逆时针四点排布
    std::sort(sorted.begin(), sorted.end(), [](const Item& a, const Item& b) {
        return a.angle > b.angle;
    });

    // 返回固定顺序四点
    return {sorted[0].point, sorted[1].point, sorted[2].point, sorted[3].point};
}

/**
 * @brief 图像像素点去畸变归一化（相机内参去畸变，输出归一化平面坐标）
 * @param image_point 原图像素坐标
 * @param camera_matrix 相机内参矩阵 3x3
 * @param dist_coeffs 畸变系数
 * @return 归一化平面 (x/z, y/z) 无畸变点，失败返回 nullopt
 * [[nodiscard]] 强制接收返回值，禁止忽略
 * noexcept 无异常抛出
 */
[[nodiscard]] std::optional<cv::Point2f> normalize_image_point(
    const cv::Point2f& image_point, const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) noexcept {
    std::vector<cv::Point2f> norm_points;
    // OpenCV 去畸变归一化，输出归一化平面坐标
    cv::undistortPoints(
        std::vector<cv::Point2f>{image_point}, norm_points, camera_matrix, dist_coeffs);
    // 输出数量异常直接失败
    if (norm_points.size() != 1) {
        return std::nullopt;
    }

    const auto& point = norm_points.front();
    // 校验坐标有效，排除NaN/无穷大
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return std::nullopt;
    }
    return point;
}

/**
 * @brief 校验跟踪器缓存快照与当前检测目标匹配（颜色、装甲类型一致）
 * @param snapshot 上一帧跟踪器缓存
 * @param detection 当前帧检测结果
 * @return true 匹配成功，可以使用跟踪器先验位姿
 */
[[nodiscard]] bool tracker_snapshot_matches_detection(
    const TrackerReadbackSnapshot& snapshot, const ArmorDetection& detection) noexcept {
    return snapshot.valid 
        && snapshot.tracker.target_name == detection.name
        && snapshot.tracker.target_color == detection.color;
}

/**
 * @brief 相机坐标系下三维平移向量 → 转换为 YPD (偏航、俯仰、距离) 球坐标
 * @param translation 相机系下目标三维坐标 XYZ
 * @return [yaw(偏航), pitch(俯仰), distance(直线距离)] 弧度
 */
[[nodiscard]] Eigen::Vector3d
    camera_translation_to_ypd_local(const Eigen::Vector3d& translation) noexcept {
    // 水平平面距离 XZ 平面
    const double horizontal = std::hypot(translation.x(), translation.z());
    return {
        // 偏航 yaw：XZ平面与Z轴夹角 atan2(x,z)
        std::atan2(translation.x(), translation.z()),
        // 俯仰 pitch：竖直方向与水平面夹角 atan2(y, 水平距离)
        std::atan2(translation.y(), horizontal),
        // 直线欧式距离
        translation.norm(),
    };
}

/**
 * @brief YPD球坐标 → 笛卡尔XYZ坐标 雅可比矩阵
 * 用于协方差矩阵传播：YPD空间噪声转XYZ空间噪声
 * @param ypd 相机系YPD向量
 * @return 3×3雅可比矩阵 J=d(XYZ)/d(YPD)
 */
[[nodiscard]] Eigen::Matrix3d camera_ypld_to_xyz_jacobian(const Eigen::Vector3d& ypd) noexcept {
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);

    // 雅可比矩阵硬编码推导结果
    Eigen::Matrix3d J;
    J << distance * cp * cy, -distance * sp * sy, distance * cp * sy,
        0.0, distance * cp, distance * sp,
        -distance * cp * sy, -distance * sp * cy, distance * cp * cy;
    return J;
}

/**
 * @brief 笛卡尔XYZ里程计坐标 → YPD球坐标 雅可比矩阵
 * 协方差传播：里程计XYZ噪声转YPD球坐标噪声
 * @param xyz 里程计系三维坐标
 * @return 3×3雅可比矩阵 J=d(YPD)/d(XYZ)
 */
[[nodiscard]] Eigen::Matrix3d odom_xyz_to_ypld_jacobian(const Eigen::Vector3d& xyz) noexcept {
    const double x     = xyz.x();
    const double y     = xyz.y();
    const double z     = xyz.z();
    const double r2_xy = x * x + y * y; // XZ平面距离平方
    const double r2    = r2_xy + z * z; // 空间距离平方
    const double r_xy  = std::sqrt(r2_xy);
    const double r     = std::sqrt(r2);

    // 距离过近避免除零崩溃，返回单位矩阵
    if (r_xy < 1e-10 || r < 1e-10) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d J;
    // 第一行：dyaw/dx, dyaw/dy, dyaw/dz
    J(0, 0) = -y / r2_xy;
    J(0, 1) = x / r2_xy;
    J(0, 2) = 0.0;

    // 第二行：dpitch/dx, dpitch/dy, dpitch/dz
    const double denom = r2 * r_xy;
    J(1, 0)            = x * z / denom;
    J(1, 1)            = y * z / denom;
    J(1, 2)            = -r_xy / r2;

    // 第三行：ddistance/dx, ddistance/dy, ddistance/dz
    J(2, 0) = x / r2;
    J(2, 1) = y / r2;
    J(2, 2) = z / r2;
    return J;
}

/**
 * @brief 将相机系4维YPDR协方差矩阵 转换到里程计坐标系
 * 协方差传播公式：Cov_odom = J * Cov_cam * J^T
 * @param camera_measurement 相机系PnP测量（含YPDR协方差）
 * @param odom_measurement 里程计系目标位姿
 * @param T_odom_camera 里程计→相机外参变换矩阵
 * @return 4×4里程计坐标系YPDR协方差矩阵，数值异常返回大对角矩阵（高噪声）
 */
[[nodiscard]] Eigen::Matrix4d reframe_camera_pnp_cov_ypdr_to_odom(
    const CameraArmorMeasurement& camera_measurement, const ArmorMeasurement& odom_measurement,
    const fast_tf::FrameTransform<fast_tf::odom, fast_tf::camera_optical>& T_odom_camera) noexcept {
    // 协方差含NaN/无穷，直接返回极大噪声矩阵
    if (!camera_measurement.pnp_cov_ypdr.allFinite()) {
        return Eigen::Matrix4d::Identity() * 1e6;
    }

    // 1. 当前相机系目标YPD球坐标
    const Eigen::Vector3d camera_ypd =
        camera_translation_to_ypd_local(camera_measurement.transform.translation());
    // 2. YPD→XYZ雅可比（相机系）
    const Eigen::Matrix3d J_camera_xyz = camera_ypld_to_xyz_jacobian(camera_ypd);
    // 3. 里程计XYZ→YPD雅可比
    const Eigen::Matrix3d J_odom_ypd =
        odom_xyz_to_ypld_jacobian(odom_measurement.transform.translation());

    // 4. 整体变换雅可比矩阵（4维：YPD+旋转R）
    Eigen::Matrix4d J   = Eigen::Matrix4d::Zero();
    // 前3×3：坐标变换链式雅可比
    J.block<3, 3>(0, 0) = J_odom_ypd * T_odom_camera.rotation() * J_camera_xyz;
    // 旋转维度无变换，单位1
    J(3, 3)             = 1.0;

    // 协方差传播计算
    Eigen::Matrix4d cov = J * camera_measurement.pnp_cov_ypdr * J.transpose();
    // 强制对称，消除数值误差带来的不对称
    cov                 = 0.5 * (cov + cov.transpose());
    // 校验数值有效性
    if (!cov.allFinite()) {
        return Eigen::Matrix4d::Identity() * 1e6;
    }
    return cov;
}

/**
 * @brief 根据跟踪器历史生成PnP求解先验位姿（多候选装甲）
 * @param detection 当前帧检测装甲
 * @param snapshot 跟踪器缓存快照（上一帧所有装甲位姿）
 * @param T_camera_odom 相机→里程计变换
 * @param camera_matrix 相机内参
 * @param dist_coeffs 畸变系数
 * @return 所有匹配装甲的先验位姿列表，包含rvec/tvec/代价分数/装甲ID
 * 作用：给PnP提供多组初始猜测，解决装甲歧义、提升BA优化收敛速度
 */
[[nodiscard]] std::vector<PnPSolver::PosePrior> make_pose_priors(
    const ArmorDetection& detection, const TrackerReadbackSnapshot& snapshot,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) noexcept {
    // 跟踪目标与当前检测不匹配，无先验
    if (!tracker_snapshot_matches_detection(snapshot, detection)) {
        return {};
    }

    // 检测装甲中心去畸变归一化
    const auto det_center_norm =
        normalize_image_point(detection.center(), camera_matrix, dist_coeffs);
    if (!det_center_norm.has_value()) {
        return {};
    }

    // 获取跟踪器缓存所有装甲三维位姿
    const auto armor_poses = tracker_armor_poses(snapshot.tracker);
    if (armor_poses.empty()) {
        return {};
    }

    // 装甲固定俯仰角（装甲板倾斜角度，常量）
    const double armor_pitch_rad = armor_pitch_rad_for(detection.name);
    const double cp              = std::cos(armor_pitch_rad);
    const double sp              = std::sin(armor_pitch_rad);
    // 俯仰旋转矩阵（装甲自身固定倾角）
    Eigen::Matrix3d R_pitch;
    R_pitch << cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp;

    // 里程计→相机旋转、平移
    const Eigen::Matrix3d R_odom_camera = T_odom_camera.rotation();
    const Eigen::Vector3d t_odom_camera = T_odom_camera.translation();

    std::vector<PnPSolver::PosePrior> priors;
    priors.reserve(armor_poses.size());

    for (size_t i = 0; i < armor_poses.size(); ++i) {
        const auto& armor_pose = armor_poses[i];
        // 跟踪器输出装甲偏航角
        const double armor_yaw = armor_pose[3];
        const double cy        = std::cos(armor_yaw);
        const double sy        = std::sin(armor_yaw);
        // 装甲绕Z轴偏航旋转矩阵
        Eigen::Matrix3d R_yaw;
        R_yaw << cy, -sy, 0.0,
            sy, cy, 0.0,
            0.0, 0.0, 1.0;

        // 1. 里程计系下装甲总旋转 = 偏航 * 固定俯仰
        const Eigen::Matrix3d R_odom_armor   = R_yaw * R_pitch;
        // 2. 相机系下装甲旋转
        const Eigen::Matrix3d R_camera_armor = R_odom_camera * R_odom_armor;
        // 3. 相机系下装甲三维平移
        const Eigen::Vector3d t_camera_armor =
            R_odom_camera * Eigen::Vector3d(armor_pose[0], armor_pose[1], armor_pose[2])
            + t_odom_camera;

        // 过滤无效深度（相机Z必须大于0，避免后方目标）
        if (!std::isfinite(t_camera_armor.x()) || !std::isfinite(t_camera_armor.y())
            || !std::isfinite(t_camera_armor.z()) || t_camera_armor.z() <= 1e-3) {
            continue;
        }

        // 先验位姿投影到归一化图像平面
        const cv::Point2f prior_center_norm(
            static_cast<float>(t_camera_armor.x() / t_camera_armor.z()),
            static_cast<float>(t_camera_armor.y() / t_camera_armor.z()));
        if (!std::isfinite(prior_center_norm.x) || !std::isfinite(prior_center_norm.y)) {
            continue;
        }

        // 计算先验与检测中心像素误差平方（代价越小越优）
        const double dx  = static_cast<double>(prior_center_norm.x - det_center_norm->x);
        const double dy  = static_cast<double>(prior_center_norm.y - det_center_norm->y);
        double hint_cost = dx * dx + dy * dy;
        // 当前选中装甲，代价加权降低（优先选用）
        if (static_cast<int>(i) == snapshot.selected_armor_id) {
            hint_cost *= 0.85;
        } else if (static_cast<int>(i) == snapshot.rough_selected_armor_id) {
            hint_cost *= 0.92;
        }

        // 欧拉旋转矩阵转OpenCV Rodrigues旋转向量
        cv::Mat R_cv;
        cv::eigen2cv(R_camera_armor, R_cv);
        cv::Mat rvec;
        cv::Rodrigues(R_cv, rvec);

        // 存入先验列表
        priors.push_back(
            PnPSolver::PosePrior{
                .rvec      = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)),
                .tvec      = cv::Vec3d(t_camera_armor.x(), t_camera_armor.y(), t_camera_armor.z()),
                .hint_cost = hint_cost,
                .armor_id  = static_cast<int>(i),
            });
    }

    return priors;
}

/**
 * @brief ROI 裁剪后，把检测框四点/矩形坐标偏移回原图坐标
 * @param detections 裁剪图内检测结果
 * @param offset ROI左上角原图偏移坐标
 */
void offset_detections(std::vector<ArmorDetection>& detections, const cv::Point& offset) noexcept {
    // 无偏移直接返回
    if (offset.x == 0 && offset.y == 0) {
        return;
    }

    const cv::Point2f offset_f(static_cast<float>(offset.x), static_cast<float>(offset.y));
    for (auto& detection : detections) {
        // 四点坐标偏移
        for (auto& corner : detection.corners) {
            corner += offset_f;
        }
        // 外接矩形偏移
        detection.rect.x += static_cast<float>(offset.x);
        detection.rect.y += static_cast<float>(offset.y);
    }
}

/**
 * @brief 判断当前帧是否是局部ROI裁剪（非全图推理）
 * @param roi 推理裁剪区域
 * @param frame_size 原图分辨率
 * @return true 启用局部ROI推理
 */
[[nodiscard]] inline bool
    is_sub_frame_roi(const cv::Rect& roi, const cv::Size& frame_size) noexcept {
    return roi.x > 0 || roi.y > 0 || roi.width < frame_size.width || roi.height < frame_size.height;
}

// 装甲长宽比上下限
struct AspectRatioBounds {
    double min;
    double max;
};

/**
 * @brief 编译期常量：标准装甲理论长宽比
 * Small 小装甲：135/55 像素
 * Large 大装甲：230/55 像素
 */
[[nodiscard]] constexpr double armor_expected_rect_aspect_ratio(ArmorType type) noexcept {
    switch (type) {
    case ArmorType::Small: return 135.0 / 55.0;
    case ArmorType::Large: return 230.0 / 55.0;
    case ArmorType::Invalid: return 0.0;
    }
    std::abort();
}

/**
 * @brief 编译期常量：装甲长宽比有效区间（±45%~130%浮动）
 */
[[nodiscard]] constexpr AspectRatioBounds armor_rect_aspect_ratio_bounds(ArmorType type) noexcept {
    constexpr double min_scale = 0.45;
    constexpr double max_scale = 1.30;
    // 未知类型，取大小装甲完整区间
    if (type == ArmorType::Invalid) {
        return {
            armor_expected_rect_aspect_ratio(ArmorType::Small) * min_scale,
            armor_expected_rect_aspect_ratio(ArmorType::Large) * max_scale,
        };
    }

    const double expected_ratio = armor_expected_rect_aspect_ratio(type);
    return {expected_ratio * min_scale, expected_ratio * max_scale};
}

/**
 * @brief 校验检测装甲长宽比是否符合真实装甲外形，过滤误检
 * @param detection 单装甲检测结果
 * @return true 长宽比合法，进入PnP解算
 */
[[nodiscard]] bool detection_rect_aspect_ratio_is_valid(const ArmorDetection& detection) noexcept {
    // 遍历四点求包围盒
    float min_x = detection.corners[0].x;
    float max_x = detection.corners[0].x;
    float min_y = detection.corners[0].y;
    float max_y = detection.corners[0].y;

    for (const auto& corner : detection.corners) {
        // 坐标非法直接丢弃
        if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) {
            return false;
        }
        min_x = std::min(min_x, corner.x);
        max_x = std::max(max_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_y = std::max(max_y, corner.y);
    }

    const float width  = max_x - min_x;
    const float height = max_y - min_y;
    // 宽高过小，过滤噪点
    if (width <= 1e-3f || height <= 1e-3f) {
        return false;
    }
    // 装甲必须横置，宽度必须大于高度，竖条直接丢弃
    if (width <= height) {
        return false;
    }

    const double aspect_ratio = static_cast<double>(width) / static_cast<double>(height);
    const auto bounds         = armor_rect_aspect_ratio_bounds(detection.type);
    // 长宽比在允许区间内才算有效检测
    return bounds.min <= aspect_ratio && aspect_ratio <= bounds.max;
}

} // 匿名命名空间结束

/**
 * @brief 注册装甲感知两大核心系统到全局调度器，200Hz固定频率运行
 * @param scheduler talos ECS全局调度器
 * 流程：先插入全局资源，再注册两个System：armor_detector、armor_solver
 */
void register_detection_systems(talos::Scheduler& scheduler) noexcept {
    auto& world = scheduler.world();
    // 插入全局单例资源（全局配置，所有系统共享）
    if (!world.has_resource<ArmorReadbackRoiConfig>()) {
        world.insert_resource(ArmorReadbackRoiConfig{});
    }
    if (!world.has_resource<TrackerReadbackCache>()) {
        world.insert_resource(TrackerReadbackCache{});
    }

    // ====================== 系统1：装甲检测器 200Hz ======================
    scheduler.add_system<talos::fixed_rate<200>>(
        "armor_detector",
        [](
            // 输入：多生产者单消费者图像话题
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            // 输出：多生产者单消费者检测结果话题
            talos::spmc_mut<ArmorDetectionBatch, DetectionChannelTopic> detection_out,
            // 可变全局资源：检测器后端（ONNX推理实例）
            talos::res_mut<std::shared_ptr<DetectorBackend>> backend,
            // 只读全局资源：TF坐标变换系统、相机内参、ROI配置、跟踪缓存
            talos::res<fast_tf::CoordinateSystem> tf_system, 
            talos::res<CameraConfig> camera_config,
            talos::res<ArmorReadbackRoiConfig> readback_roi_config,
            talos::res<TrackerReadbackCache> readback_cache, 
            // 运行时能力标记、当前识别颜色（红/蓝）
            core::capabilities cap,
            core::detecting_color detecting_color_
        ) mutable {
            // 读取一帧图像
            auto frame = image_in.read();
            if (!frame) {
                return;
            }
            // 机器人未开启装甲感知能力，空检测包直接输出
            if (!core::capable(*cap, core::Capability::Armor)) {
                detection_out.write(
                    ArmorDetectionBatch({}, frame->image, frame->timestamp_ns, frame->frame_id));
                return;
            }

            // 当前需要识别的装甲颜色
            ArmorColor detect_color = *detecting_color_;

            // 默认推理区域：全图
            cv::Rect detector_roi(0, 0, frame->image.cols, frame->image.rows);
            cv::Mat detector_input = frame->image;

            // 查询当前帧时间戳对应的里程计→相机外参TF变换
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            // TF查找失败，丢弃当前帧
            if (!tf_lookup) {
                SPDLOG_ERROR(
                    "TF lookup failed for timestamp {}: {}", frame->timestamp_ns,
                    tf_lookup.error());
                return;
            }

            auto T_odom_camera = tf_lookup.value();
            // 开启跟踪引导ROI裁剪功能
            if (readback_roi_config->enabled) {
                const auto snapshot = readback_cache->load();
                // 跟踪缓存数据新鲜有效
                if (tracker_snapshot_is_fresh(
                        snapshot, *readback_roi_config, frame->timestamp_ns)) {
                    // 将跟踪器目标三维框投影到图像，生成局部推理ROI
                    const auto maybe_roi = resolve_readback_roi(
                        frame->image.size(),
                        project_tracker_box_to_image(
                            snapshot.tracker, T_odom_camera.inverse(), *camera_config,
                            readback_roi_config->box_size_m)
                            .value_or(cv::Rect2f{}),
                        *readback_roi_config, backend->get()->input_resolution());
                    // 有效ROI，裁剪局部图像用于推理，降低算力消耗
                    if (maybe_roi) {
                        detector_roi   = *maybe_roi;
                        detector_input = frame->image(detector_roi);
                    }
                }
            }
            // 标记是否使用局部裁剪推理
            const bool has_detector_roi = is_sub_frame_roi(detector_roi, frame->image.size());

            // 执行神经网络推理，输出装甲原始检测框
            auto result = backend->get()->detect(detector_input, detect_color);
            // 推理失败，输出空检测包
            if (!result) {
                detection_out.write(
                    ArmorDetectionBatch{
                        {},
                        frame->image,
                        frame->timestamp_ns,
                        frame->frame_id,
                        has_detector_roi,
                        detector_roi});
                return;
            }

            auto detections = std::move(*result);
            // 所有检测四点逆时针排序
            for (auto& detection : detections) {
                detection.corners = sort_corners(detection.corners);
            }
            // ROI裁剪坐标偏移还原到原图坐标系
            offset_detections(detections, detector_roi.tl());
            // 过滤：只保留目标颜色、长宽比合法的装甲，送入PnP解算
            std::vector<ArmorDetection> detections_for_pnp;
            detections_for_pnp.reserve(detections.size());
            std::copy_if(
                detections.begin(), detections.end(), std::back_inserter(detections_for_pnp),
                [detect_color](const ArmorDetection& det) {
                    return det.color == detect_color && detection_rect_aspect_ratio_is_valid(det);
                });

            // 打包检测结果发送到下一级PnP解算系统
            detection_out.write(
                ArmorDetectionBatch{
                    std::move(detections_for_pnp), frame->image, frame->timestamp_ns,
                    frame->frame_id, has_detector_roi, detector_roi});
        });

    // ====================== 系统2：装甲PnP求解器 200Hz ======================
    scheduler.add_system<talos::fixed_rate<200>>(
        "armor_solver",
        [](
            // 输入：检测器输出的装甲检测列表
            talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> detection_in,
            // 输出：里程计系装甲三维位姿测量结果
            talos::spmc_mut<ArmorMeasurementBatch, MeasurementChannelTopic> measurement_out,
            // 全局资源：PnP求解器实例、TF、相机内参、ROI配置、跟踪缓存
            talos::res<std::shared_ptr<PnPSolver>> solver_ptr,
            talos::res<fast_tf::CoordinateSystem> tf_system, 
            talos::res<CameraConfig> camera_config,
            talos::res<ArmorReadbackRoiConfig> readback_roi_config, 
            core::capabilities cap,
            talos::res<TrackerReadbackCache> readback_cache
        ) mutable {
            // 未开启装甲感知，直接退出
            if (!core::capable(*cap, core::Capability::Armor)) {
                return;
            }
            // 读取检测包，无数据返回
            auto detections = detection_in.read();
            if (!detections) {
                return;
            }

            // 查询当前帧时间戳TF变换
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, detections->timestamp_ns);

            if (!tf_lookup) {
                SPDLOG_ERROR(
                    "TF lookup failed for timestamp {}: {}", detections->timestamp_ns,
                    tf_lookup.error());
                return;
            }

            const auto T_odom_camera     = tf_lookup.value();
            const auto T_camera_odom     = T_odom_camera.inverse();
            const auto R_odom_camera     = T_odom_camera.rotation();
            // 读取上一帧跟踪缓存
            const auto tracker_snapshot  = readback_cache->load();
            const bool snapshot_is_fresh = tracker_snapshot_is_fresh(
                tracker_snapshot, *readback_roi_config, detections->timestamp_ns);

            // 一次性转换相机内参为OpenCV Mat，避免循环内重复分配
            cv::Mat camera_matrix_cv;
            cv::Mat dist_coeffs_cv;
            cv::eigen2cv(camera_config->camera_matrix, camera_matrix_cv);
            cv::eigen2cv(camera_config->distort_coefficient, dist_coeffs_cv);

            std::vector<CameraArmorMeasurement> measurements;
            measurements.reserve(detections->detections.size());
            // 遍历所有有效装甲检测
            for (const auto& detection : detections->detections) {
                // 跟踪缓存新鲜则生成多组先验位姿，否则无先验
                const auto priors = snapshot_is_fresh
                                      ? make_pose_priors(
                                            detection, tracker_snapshot, T_camera_odom,
                                            camera_matrix_cv, dist_coeffs_cv)
                                      : std::vector<PnPSolver::PosePrior>{};
                // 执行PnP+BA图优化求解相机系三维位姿
                auto result =
                    (*solver_ptr)
                        ->solve_with_ba(detection, R_odom_camera, detections->timestamp_ns, priors);
                if (result) {
                    // 特殊逻辑：前哨站装甲角度过滤
                    if (result->name == ArmorName::Outpost) {
                        // 目标转换到里程计坐标系
                        auto target_in_ref = T_odom_camera * result->transform;
                        // 计算目标平面偏航、欧拉旋转
                        auto target_pos_yaw = core::math::xyz2ypd(target_in_ref.translation())[0];
                        auto [roll, pitch, target_yaw] = target_in_ref.euler_rot().rpy();
                        // 角度差超过25度，过滤该前哨站装甲（消除歧义）
                        auto delta_angle = core::math::normalize_angle(target_yaw - target_pos_yaw)
                                         * 180.0 / std::numbers::pi;
                        if (std::abs(delta_angle) > 25) {
                            continue;
                        }
                    }
                    measurements.push_back(std::move(*result));
                }
            }

            // 将相机系测量转换为里程计系测量，并转换协方差矩阵
            std::vector<ArmorMeasurement> odom_measurements;
            odom_measurements.reserve(measurements.size());
            for (const auto& meas : measurements) {
                auto odom_measurement = meas.reframe(T_odom_camera);
                // 协方差坐标系转换
                odom_measurement.pnp_cov_ypdr =
                    reframe_camera_pnp_cov_ypdr_to_odom(meas, odom_measurement, T_odom_camera);
                odom_measurements.push_back(std::move(odom_measurement));
            }

            // 输出里程计系装甲三维测量结果，供给跟踪器/弹道解算模块
            measurement_out.write(
                ArmorMeasurementBatch{
                    std::move(odom_measurements), detections->timestamp_ns, detections->frame_id});
        });
}

/**
 * @brief 创建检测器后端共享句柄（封装推理实例+后端名称）
 * @param config 装甲检测器配置（后端类型、模型路径、分辨率）
 * @return 成功返回句柄，失败返回错误字符串
 */
std::expected<DetectorBackendHandle, std::string>
    create_detector_backend_handle(const ArmorDetectorConfig& config) noexcept {
    // 底层创建推理后端实例（Axera/TensorRT/ONNX）
    auto backend_result = create_detector_backend(config);
    if (!backend_result) {
        return std::unexpected(std::move(backend_result.error()));
    }

    // 封装为共享智能指针
    auto backend_ptr  = std::make_shared<DetectorBackend>(std::move(*backend_result));
    // 后端名称用于日志打印
    auto backend_name = std::string(magic_enum::enum_name(config.backend_type));
    return DetectorBackendHandle{std::move(backend_ptr), std::move(backend_name)};
}

/**
 * @brief 构造PnP求解器共享实例，传入相机内参
 * @param config 相机内参、畸变系数配置
 * @return 求解器共享指针
 */
std::shared_ptr<PnPSolver> create_pnp_solver(const CameraConfig& config) noexcept {
    return std::make_shared<PnPSolver>(config);
}

} // namespace fcs::L2