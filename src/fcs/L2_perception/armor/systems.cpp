// 本模块对外头文件，声明register_detection_systems注册函数，供boot.cc调用
#include "L2_perception/armor/systems.hpp"

// ========== 装甲感知内部依赖头文件 ==========
// 检测器后端抽象层：统一封装Axera/TensorRT/ONNX三种推理引擎，屏蔽底层差异
#include "L2_perception/armor/backend.hpp"
// 装甲检测、相机、ROI裁剪、跟踪器全套配置结构体定义
#include "L2_perception/armor/config.hpp"
// 跟踪器引导ROI裁剪逻辑实现：根据上一帧目标缩小推理区域，降低算力
#include "L2_perception/armor/readback_roi.hpp"
// PnP+BA光束平差位姿求解器：2D装甲角点→3D世界坐标解算核心算法
#include "L2_perception/armor/solver.hpp"
// 角度归一化、弧度转换等通用数学工具函数
#include "core/math/normalize.hpp"
// 全局运行时上下文：功能开关、硬件标记、资源生命周期管理
#include "core/runtime.hpp"
// 项目全局基础类型：Eigen位姿、枚举、状态机、容器别名
#include "core/types.hpp"
// fast_tf静态坐标系枚举：world/odom/gimbal/camera_optical/muzzle
#include "frame.hpp"
// ECS调度器核心类：管理所有System、全局资源、SPMC无锁通道
#include "scheduler/scheduler.hpp"

// OpenCV图像处理库：轮廓、畸变校正、矩阵变换、绘图API
#include <opencv2/imgproc.hpp>
// C++17可选返回值类型std::optional，用于函数失败安全返回，不抛异常
#include <optional>
// 分级日志库，统一全局日志输出，分INFO/WARN/ERROR等级
#include <spdlog/spdlog.h>

// C++标准通用算法库：sort排序、copy_if筛选容器
#include <algorithm>
// 数学库：sqrt/hypot三角函数、平方根
#include <cmath>
// 标准库退出函数，非法枚举分支终止程序
#include <cstdlib>
// magic_enum枚举反射库：编译期获取枚举名字、转换字符串，无运行时开销
#include <magic_enum.hpp>

// 项目顶层根命名空间fcs，二级子命名空间L2感知层
namespace fcs::L2 {

/**
 * @namespace 匿名命名空间（文件私有域）
 * 仅当前.cc文件内部函数、结构体可见，不会导出全局符号，避免命名冲突
 * 存放装甲感知内部数学、坐标、PnP辅助工具，外部无法调用
 */
namespace {

/**
 * @brief 将网络输出无序装甲四点，按逆时针标准顺序重排
 * 标准装甲四点固定顺序：左上 → 右上 → 右下 → 左下
 * @param points 神经网络推理输出未排序四点cv::Point2f数组
 * @return 按逆时针排布标准化四点数组std::array<cv::Point2f,4>
 * 核心原理：
 * 1. 计算四点几何中心；
 * 2. 每个点相对中心求向量与X轴夹角；
 * 3. 夹角从大到小降序排序，天然形成逆时针环绕顺序；
 */
std::array<cv::Point2f, 4> sort_corners(const std::array<cv::Point2f, 4>& points) {
    // 步骤1：累加所有点坐标，计算四点几何中心
    cv::Point2f center(0.0f, 0.0f);
    // 范围for遍历四个角点，累加XY
    for (const auto& p : points) {
        center += p;
    }
    // 四个点求平均得到中心点坐标
    center *= 0.25f;

    // 临时结构体：存储单个角点 + 该点相对中心的夹角
    struct Item {
        cv::Point2f point; // 图像像素坐标
        float angle;       // 相对中心点向量与X轴夹角（单位度）
    };
    // 定义动态vector存放四点数据，预分配4个内存，避免扩容开销
    std::vector<Item> sorted;
    sorted.reserve(4);

    // 遍历四个装甲角点，逐点计算夹角存入容器
    for (const auto& p : points) {
        // 步骤2：计算当前点相对几何中心的偏移向量
        cv::Point2f dir = p - center;

        // 计算向量模长，用于单位化归一化
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        // 防止模长接近0（四点重合除零崩溃），阈值过滤极小向量
        if (len > 1e-6f) {
            // 向量单位化，转为单位向量，消除距离对角度计算的影响
            dir.x /= len;
            dir.y /= len;
        }

        // 步骤3：atan2(-y, x) 计算向量与X轴正方向夹角，值域[-180°,180°]
        // 叉乘cross = -y，点乘dot = x，atan2(叉,点)得到平面旋转角
        float angle = std::atan2(-dir.y, dir.x) * 180.0f / static_cast<float>(CV_PI);
        // 当前点与对应夹角存入临时数组
        sorted.push_back({p, angle});
    }

    // 步骤4：自定义排序规则，夹角降序排列，实现逆时针四点顺序
    std::sort(sorted.begin(), sorted.end(), [](const Item& a, const Item& b) {
        // a夹角更大排在前面，逆时针顺序
        return a.angle > b.angle;
    });

    // 按排序结果，依次取出四点返回标准化数组
    return {sorted[0].point, sorted[1].point, sorted[2].point, sorted[3].point};
}

/**
 * @brief 图像像素点去畸变，输出相机归一化平面坐标
 * 相机原始图像存在透镜畸变，需要校正后才能用于PnP解算
 * @param image_point 原始图像像素二维坐标
 * @param camera_matrix 相机3×3内参矩阵（fx/fy/cx/cy）
 * @param dist_coeffs 相机畸变系数（k1/k2/p1/p2/k3）
 * @return std::optional<cv::Point2f> 成功返回归一化点，失败返回std::nullopt
 * @ [[nodiscard]] 强制调用方接收返回值，禁止忽略失败结果
 * @ noexcept 函数不会抛出异常，硬实时调度无异常分支开销
 * 归一化平面定义：相机光心为原点，Z=1平面（x/z,y/z）
 */
[[nodiscard]] std::optional<cv::Point2f> normalize_image_point(
    const cv::Point2f& image_point, const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) noexcept {
    // 构造单元素点容器，传入OpenCV去畸变接口
    std::vector<cv::Point2f> norm_points;
    // OpenCV内置去畸变归一化函数：输入像素点，输出Z=1归一化平面坐标
    cv::undistortPoints(
        std::vector<cv::Point2f>{image_point}, norm_points, camera_matrix, dist_coeffs);
    // 输出容器长度不等于1，代表计算异常，返回空optional
    if (norm_points.size() != 1) {
        return std::nullopt;
    }

    // 取出归一化后的坐标点
    const auto& point = norm_points.front();
    // 校验坐标数值有效性，排除NaN、正负无穷大浮点非法值
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return std::nullopt;
    }
    // 计算合法，返回归一化二维点
    return point;
}

/**
 * @brief 校验跟踪缓存快照与当前检测目标匹配
 * 功能：判断上一帧跟踪目标是否和当前帧装甲为同一类、同颜色
 * @param snapshot 上一帧跟踪器缓存快照（存储历史目标信息）
 * @param detection 当前帧神经网络装甲检测结果
 * @return bool true：目标匹配，可使用跟踪先验优化PnP求解
 * @ noexcept 无异常抛出
 */
[[nodiscard]] bool tracker_snapshot_matches_detection(
    const TrackerReadbackSnapshot& snapshot, const ArmorDetection& detection) noexcept {
    // 三重条件同时满足：快照有效 + 装甲类型一致 + 识别颜色一致
    return snapshot.valid 
        && snapshot.tracker.target_name == detection.name
        && snapshot.tracker.target_color == detection.color;
}

/**
 * @brief 相机坐标系三维平移XYZ → 转换球坐标YPD（偏航、俯仰、直线距离）
 * 相机光学坐标系：Z轴向前（镜头方向）、X向右、Y向下
 * @param translation 相机系目标三维Eigen向量(X,Y,Z)
 * @return Eigen::Vector3d {yaw(rad), pitch(rad), distance(m)}
 * yaw：XZ平面内绕Y轴偏航角；pitch：竖直平面俯仰角；distance：光心到目标直线欧式距离
 */
[[nodiscard]] Eigen::Vector3d
    camera_translation_to_ypd_local(const Eigen::Vector3d& translation) noexcept {
    // 计算XZ平面水平距离（忽略高度Y）
    const double horizontal = std::hypot(translation.x(), translation.z());
    return {
        // 偏航yaw：atan2(横向X, 深度Z)，XZ平面夹角
        std::atan2(translation.x(), translation.z()),
        // 俯仰pitch：atan2(高度Y, XZ水平距离)，上下俯仰角
        std::atan2(translation.y(), horizontal),
        // 三维空间直线距离（目标到相机光心）
        translation.norm(),
    };
}

/**
 * @brief 相机系YPD球坐标 → XYZ笛卡尔坐标 雅可比矩阵
 * 协方差传播专用工具：把YPD空间噪声误差映射到XYZ三维空间
 * @param ypd 相机系{yaw,pitch,distance}三维向量
 * @return Eigen::Matrix3d 3×3雅可比矩阵 J = ∂(XYZ)/∂(YPD)
 */
[[nodiscard]] Eigen::Matrix3d camera_ypld_to_xyz_jacobian(const Eigen::Vector3d& ypd) noexcept {
    // 提取YPD三个分量
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    // 预计算三角函数，重复复用减少计算量
    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);

    // 硬编码推导完成雅可比3×3矩阵
    Eigen::Matrix3d J;
    J << distance * cp * cy, -distance * sp * sy, distance * cp * sy,
        0.0, distance * cp, distance * sp,
        -distance * cp * sy, -distance * sp * cy, distance * cp * cy;
    return J;
}

/**
 * @brief 里程计XYZ笛卡尔 → YPD球坐标 雅可比矩阵
 * 用途：里程计坐标系下三维噪声转换为角度/距离噪声，用于卡尔曼协方差更新
 * @param xyz 里程计系目标三维坐标
 * @return Eigen::Matrix3d J = ∂(YPD)/∂(XYZ)
 */
[[nodiscard]] Eigen::Matrix3d odom_xyz_to_ypld_jacobian(const Eigen::Vector3d& xyz) noexcept {
    // 拆分XYZ三轴分量
    const double x     = xyz.x();
    const double y     = xyz.y();
    const double z     = xyz.z();
    // XZ平面距离平方
    const double r2_xy = x * x + y * y;
    // 三维欧式距离平方
    const double r2    = r2_xy + z * z;
    // XZ平面直线距离
    const double r_xy  = std::sqrt(r2_xy);
    // 三维总距离
    const double r     = std::sqrt(r2);

    // 分母极小值保护，防止除零运算崩溃（目标贴相机）
    if (r_xy < 1e-10 || r < 1e-10) {
        // 数值异常返回单位矩阵，噪声不扩散
        return Eigen::Matrix3d::Identity();
    }

    // 初始化3×3雅可比矩阵
    Eigen::Matrix3d J;
    // 第一行：dyaw/dx, dyaw/dy, dyaw/dz 偏航对XYZ偏导
    J(0, 0) = -y / r2_xy;
    J(0, 1) = x / r2_xy;
    J(0, 2) = 0.0;

    // 第二行：dpitch/dx, dpitch/dy, dpitch/dz 俯仰对XYZ偏导
    const double denom = r2 * r_xy;
    J(1, 0) = x * z / denom;
    J(1, 1) = y * z / denom;
    J(1, 2) = -r_xy / r2;

    // 第三行：ddistance/dx, ddistance/dy, dz 距离对XYZ偏导
    J(2, 0) = x / r2;
    J(2, 1) = y / r2;
    J(2, 2) = z / r2;
    return J;
}

/**
 * @brief 将相机系4维YPDR协方差转换到里程计坐标系
 * 协方差传播公式：Cov_odom = J * Cov_cam * J^T
 * YPDR：yaw/pitch/distance/roll（三轴旋转+距离四维噪声）
 * @param camera_measurement 相机PnP求解的带协方差测量结果
 * @param odom_measurement 转换后里程计系目标位姿
 * @param T_odom_camera fast_tf里程计→相机静态变换矩阵
 * @return Eigen::Matrix4d 4×4里程计坐标系协方差矩阵
 * 数值异常时返回极大对角矩阵，代表高噪声，过滤不可靠观测
 */
[[nodiscard]] Eigen::Matrix4d reframe_camera_pnp_cov_ypdr_to_odom(
    const CameraArmorMeasurement& camera_measurement, const ArmorMeasurement& odom_measurement,
    const fast_tf::FrameTransform<fast_tf::odom, fast_tf::camera_optical>& T_odom_camera) noexcept {
    // 前置校验：协方差矩阵全部数值有限，无NaN/无穷
    if (!camera_measurement.pnp_cov_ypdr.allFinite()) {
        // 数值非法，返回超大对角矩阵，代表测量完全不可信
        return Eigen::Matrix4d::Identity() * 1e6;
    }

    // 步骤1：把相机系平移向量转为YPD球坐标
    const Eigen::Vector3d camera_ypd =
        camera_translation_to_ypd_local(camera_measurement.transform.translation());
    // 步骤2：YPD → XYZ相机空间雅可比矩阵
    const Eigen::Matrix3d J_camera_xyz = camera_ypld_to_xyz_jacobian(camera_ypd);
    // 步骤3：里程计XYZ → YPD球坐标雅可比矩阵
    const Eigen::Matrix3d J_odom_ypd =
        odom_xyz_to_ypld_jacobian(odom_measurement.transform.translation());

    // 构造4维整体变换雅可比矩阵（前三轴YPD，第四轴roll旋转无变换）
    Eigen::Matrix4d J   = Eigen::Matrix4d::Zero();
    // 3×3坐标变换链式雅可比：J_odom_ypd * 外参旋转矩阵 * J_camera_xyz
    J.block<3, 3>(0, 0) = J_odom_ypd * T_odom_camera.rotation() * J_camera_xyz;
    // 第四维roll旋转无坐标变换，单位1
    J(3, 3)             = 1.0;

    // 协方差传播计算：J × 相机协方差 × J转置
    Eigen::Matrix4d cov = J * camera_measurement.pnp_cov_ypdr * J.transpose();
    // 强制对称：数值浮点误差会导致矩阵不对称，取均值修正
    cov                 = 0.5 * (cov + cov.transpose());
    // 再次校验矩阵数值合法性
    if (!cov.allFinite()) {
        return Eigen::Matrix4d::Identity() * 1e6;
    }
    return cov;
}

/**
 * @brief 根据上一帧跟踪快照生成PnP多组先验位姿
 * 作用：给BA光束平差提供多个初始猜测，解决装甲歧义、避免局部最优解
 * @param detection 当前帧装甲检测框
 * @param snapshot 上一帧跟踪缓存快照
 * @param T_camera_odom 相机→里程计坐标变换
 * @param camera_matrix 相机3×3内参
 * @param dist_coeffs 畸变系数
 * @return std::vector<PnPSolver::PosePrior> 多组先验位姿列表（旋转rvec+平移tvec+代价+装甲ID）
 */
[[nodiscard]] std::vector<PnPSolver::PosePrior> make_pose_priors(
    const ArmorDetection& detection, const TrackerReadbackSnapshot& snapshot,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) noexcept {
    // 前置判断：跟踪快照与当前检测目标不匹配，无可用先验，返回空列表
    if (!tracker_snapshot_matches_detection(snapshot, detection)) {
        return {};
    }

    // 将当前装甲检测中心点做去畸变归一化
    const auto det_center_norm =
        normalize_image_point(detection.center(), camera_matrix, dist_coeffs);
    // 归一化失败，无有效投影点，无法生成先验
    if (!det_center_norm.has_value()) {
        return {};
    }

    // 读取跟踪器缓存中所有装甲三维位姿
    const auto armor_poses = tracker_armor_poses(snapshot.tracker);
    // 无任何跟踪目标，直接返回空
    if (armor_poses.empty()) {
        return {};
    }

    // 装甲固定俯仰角常量（装甲板自身倾斜角度，编译期常量）
    const double armor_pitch_rad = armor_pitch_rad_for(detection.name);
    const double cp              = std::cos(armor_pitch_rad);
    const double sp              = std::sin(armor_pitch_rad);
    // 装甲固定俯仰旋转矩阵（仅绕Y轴俯仰）
    Eigen::Matrix3d R_pitch;
    R_pitch << cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp;

    // 提取里程计→相机旋转、平移变换
    const Eigen::Matrix3d R_odom_camera = T_odom_camera.rotation();
    const Eigen::Vector3d t_odom_camera = T_odom_camera.translation();

    // 预分配先验位姿容器，容量等于装甲数量
    std::vector<PnPSolver::PosePrior> priors;
    priors.reserve(armor_poses.size());

    // 遍历所有跟踪装甲位姿，逐个生成PnP先验
    for (size_t i = 0; i < armor_poses.size(); ++i) {
        const auto& armor_pose = armor_poses[i];
        // 取出跟踪得到装甲偏航角
        const double armor_yaw = armor_pose[3];
        const double cy        = std::cos(armor_yaw);
        const double sy        = std::sin(armor_yaw);
        // 装甲绕Z轴偏航旋转矩阵
        Eigen::Matrix3d R_yaw;
        R_yaw << cy, -sy, 0.0,
            sy, cy, 0.0,
            0.0, 0.0, 1.0;

        // 步骤1：里程计系装甲总旋转 = 偏航 × 固定俯仰
        const Eigen::Matrix3d R_odom_armor   = R_yaw * R_pitch;
        // 步骤2：转换到相机坐标系下装甲旋转矩阵
        const Eigen::Matrix3d R_camera_armor = R_odom_camera * R_odom_armor;
        // 步骤3：转换到相机坐标系三维平移向量
        const Eigen::Vector3d t_camera_armor =
            R_odom_camera * Eigen::Vector3d(armor_pose[0], armor_pose[1], armor_pose[2])
            + t_odom_camera;

        // 过滤无效深度：相机Z坐标必须大于0（目标在镜头前方，排除后方镜像）
        if (!std::isfinite(t_camera_armor.x()) || !std::isfinite(t_camera_armor.y()) || !std::isfinite(t_camera_armor.z()) || t_camera_armor.z() <= 1e-3) {
            continue;
        }

        // 将相机三维点投影到归一化图像平面，生成预测中心点
        const cv::Point2f prior_center_norm(
            static_cast<float>(t_camera_armor.x() / t_camera_armor.z()),
            static_cast<float>(t_camera_armor.y() / t_camera_armor.z()));
        // 投影坐标非法直接跳过该组先验
        if (!std::isfinite(prior_center_norm.x) || !std::isfinite(prior_center_norm.y)) {
            continue;
        }

        // 计算预测中心与检测中心像素误差平方（代价越小，先验越可靠）
        const double dx  = static_cast<double>(prior_center_norm.x - det_center_norm->x);
        const double dy  = static_cast<double>(prior_center_norm.y - det_center_norm->y);
        double hint_cost = dx * dx + dy * dy;
        // 当前选中装甲代价加权降低，优先使用
        if (static_cast<int>(i) == snapshot.selected_armor_id) {
            hint_cost *= 0.85;
        } else if (static_cast<int>(i) == snapshot.rough_selected_armor_id) {
            hint_cost *= 0.92;
        }

        // Eigen旋转矩阵转为OpenCV Rodrigues旋转向量rvec
        cv::Mat R_cv;
        cv::eigen2cv(R_camera_armor, R_cv);
        cv::Mat rvec;
        cv::Rodrigues(R_cv, rvec);

        // 封装一组先验位姿存入容器
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
 * @brief ROI裁剪坐标偏移还原
 * 推理使用局部裁剪小图，检测框坐标是裁剪图局部坐标，需要叠加ROI左上角偏移还原到原图像素坐标系
 * @param detections 裁剪图内检测结果数组（原地修改）
 * @param offset ROI区域左上角原图XY偏移点
 * @ noexcept 无异常
 */
void offset_detections(std::vector<ArmorDetection>& detections, const cv::Point& offset) noexcept {
    // 偏移为0，全图推理无需转换，直接返回
    if (offset.x == 0 && offset.y == 0) {
        return;
    }

    // 转换偏移为浮点型，匹配点坐标类型
    const cv::Point2f offset_f(static_cast<float>(offset.x), static_cast<float>(offset.y));
    // 遍历所有装甲检测，四点、矩形全部叠加偏移
    for (auto& detection : detections) {
        // 四个角点逐个加偏移
        for (auto& corner : detection.corners) {
            corner += offset_f;
        }
        // 外接矩形左上角叠加偏移
        detection.rect.x += static_cast<float>(offset.x);
        detection.rect.y += static_cast<float>(offset.y);
    }
}

/**
 * @brief 判断推理是否使用局部ROI裁剪（非全图）
 * @param roi 推理裁剪矩形区域
 * @param frame_size 原图完整分辨率宽高
 * @return true 启用局部裁剪，算力优化；false 整张图推理
 */
[[nodiscard]] inline bool
    is_sub_frame_roi(const cv::Rect& roi, const cv::Size& frame_size) noexcept {
    // 左上角有偏移 / 宽度高度小于原图任意一种，代表局部裁剪
    return roi.x > 0 || roi.y > 0 || roi.width < frame_size.width || roi.height < frame_size.height;
}

// 存储装甲长宽比上下限最小/最大值结构体
struct AspectRatioBounds {
    double min; // 合法最小长宽比
    double max; // 合法最大长宽比
};

/**
 * @brief 编译期常量函数，返回标准装甲理论长宽比
 * Small小装甲：标准图纸135像素 / 55像素
 * Large大装甲：标准图纸230像素 / 55像素
 * @param type 装甲类型枚举 Small/Large/Invalid
 * @return double 理论长宽比
 * @ constexpr 编译期求值，无运行计算开销
 * @ noexcept 无异常
 */
[[nodiscard]] constexpr double armor_expected_rect_aspect_ratio(ArmorType type) noexcept {
    switch (type) {
    case ArmorType::Small: return 135.0 / 55.0;
    case ArmorType::Large: return 230.0 / 55.0;
    case ArmorType::Invalid: return 0.0;
    }
    // 非法枚举分支直接终止程序
    std::abort();
}

/**
 * @brief 编译期常量，返回对应装甲长宽比合法浮动区间
 * 允许上下浮动45%~130，过滤远距离畸变、误检矩形
 * @param type 装甲类型
 * @return AspectRatioBounds 最小、最大合法比值
 */
[[nodiscard]] constexpr AspectRatioBounds armor_rect_aspect_ratio_bounds(ArmorType type) noexcept {
    // 缩放系数下限、上限
    constexpr double min_scale = 0.45;
    constexpr double max_scale = 1.30;
    // 类型无效，取大小装甲完整区间
    if (type == ArmorType::Invalid) {
        return {
            armor_expected_rect_aspect_ratio(ArmorType::Small) * min_scale,
            armor_expected_rect_aspect_ratio(ArmorType::Large) * max_scale,
        };
    }

    // 取出当前装甲标准长宽比，乘浮动系数得到区间
    const double expected_ratio = armor_expected_rect_aspect_ratio(type);
    return {expected_ratio * min_scale, expected_ratio * max_scale};
}

/**
 * @brief 校验单装甲检测框长宽比是否符合真实装甲外形，过滤噪点/误检
 * @param detection 单装甲检测结果
 * @return true 长宽比合法，送入PnP解算；false 误检直接丢弃
 */
[[nodiscard]] bool detection_rect_aspect_ratio_is_valid(const ArmorDetection& detection) noexcept {
    // 初始化包围盒极值，取第一个点作为初始值
    float min_x = detection.corners[0].x;
    float max_x = detection.corners[0].x;
    float min_y = detection.corners[0].y;
    float max_y = detection.corners[0].y;

    // 遍历四个角点，求取包围盒最大最小XY
    for (const auto& corner : detection.corners) {
        // 坐标存在NaN无穷大，直接判定非法检测
        if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) {
            return false;
        }
        min_x = std::min(min_x, corner.x);
        max_x = std::max(max_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_y = std::max(max_y, corner.y);
    }

    // 包围盒宽、高
    const float width  = max_x - min_x;
    const float height = max_y - min_y;
    // 宽高过小，噪点过滤
    if (width <= 1e-3f || height <= 1e-3f) {
        return false;
    }
    // 装甲必须横置，宽度大于高度，竖条直接判定误检
    if (width <= height) {
        return false;
    }

    // 计算实际长宽比
    const double aspect_ratio = static_cast<double>(width) / static_cast<double>(height);
    // 获取当前装甲合法区间
    const auto bounds = armor_rect_aspect_ratio_bounds(detection.type);
    // 比值在区间内才合法
    return bounds.min <= aspect_ratio && aspect_ratio <= bounds.max;
}

} // 匿名命名空间结束（内部工具函数全部完毕）

/**
 * @brief 向全局ECS调度器注册装甲感知两大核心System
 * 系统1：armor_detector 神经网络推理检测，固定200Hz周期运行
 * 系统2：armor_solver PnP+BA三维位姿解算，固定200Hz周期运行
 * @param scheduler talos::Scheduler& ECS全局调度器引用，修改内部资源与任务列表
 * @ noexcept 函数无异常抛出
 */
void register_detection_systems(talos::Scheduler& scheduler) noexcept {
    // 获取调度器内置ECS全局资源容器World
    auto& world = scheduler.world();
    // 全局ROI裁剪配置资源不存在则原地构造
    if (!world.has_resource<ArmorReadbackRoiConfig>()) {
        world.insert_resource(ArmorReadbackRoiConfig{});
    }
    // 全局跟踪缓存资源不存在则原地构造
    if (!world.has_resource<TrackerReadbackCache>()) {
        world.insert_resource(TrackerReadbackCache{});
    }

    // ====================== 第一个固定频率System：装甲检测器 200Hz ======================
    scheduler.add_system<talos::fixed_rate<200>>(
        "armor_detector", // 系统唯一名称，日志/调试标识
        // 系统执行lambda函数，捕获所有依赖资源与通道
        [](
            // 只读SPMC输入通道：图像帧生产者（L1相机），多生产者单消费者
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            // 可写SPMC输出通道：装甲检测结果，下发PnP求解系统
            talos::spmc_mut<ArmorDetectionBatch, DetectionChannelTopic> detection_out,
            // 可变全局资源：推理后端实例（ONNX/TensorRT，需要修改内部状态）
            talos::res_mut<std::shared_ptr<DetectorBackend>> backend,
            // 只读全局资源：TF坐标系管理实例
            talos::res<fast_tf::CoordinateSystem> tf_system, 
            // 只读相机内参、畸变配置
            talos::res<CameraConfig> camera_config,
            // ROI裁剪全局配置
            talos::res<ArmorReadbackRoiConfig> readback_roi_config,
            // 上一帧跟踪缓存快照
            talos::res<TrackerReadbackCache> readback_cache, 
            // 运行时功能能力掩码（全局只读）
            core::capabilities cap,
            // 当前需要识别敌方颜色（红/蓝）
            core::detecting_color detecting_color_
        ) mutable {
            // 读取一帧图像，无新图像直接返回
            auto frame = image_in.read();
            if (!frame) {
                return;
            }
            // 全局能力标记未开启装甲识别，输出空检测包直接返回
            if (!core::capable(*cap, core::Capability::Armor)) {
                detection_out.write(
                    ArmorDetectionBatch({}, frame->image, frame->timestamp_ns, frame->frame_id));
                return;
            }

            // 取出当前需要识别的装甲队伍颜色
            ArmorColor detect_color = *detecting_color_;

            // 默认推理区域：整张图像，左上角(0,0)宽高等于原图
            cv::Rect detector_roi(0, 0, frame->image.cols, frame->image.rows);
            // 默认推理图像为原图完整画面
            cv::Mat detector_input = frame->image;

            // 根据当前帧时间戳，查询里程计→相机静态TF变换
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            // TF坐标查询失败，日志打印错误丢弃当前帧
            if (!tf_lookup) {
                SPDLOG_ERROR(
                    "TF lookup failed for timestamp {}: {}", frame->timestamp_ns,
                    tf_lookup.error());
                return;
            }

            // 取出查询到的静态变换矩阵
            auto T_odom_camera = tf_lookup.value();
            // 判断是否开启跟踪引导ROI裁剪功能
            if (readback_roi_config->enabled) {
                // 加载上一帧跟踪快照
                const auto snapshot = readback_cache->load();
                // 跟踪快照时间戳新鲜有效（未超时）
                if (tracker_snapshot_is_fresh(snapshot, *readback_roi_config, frame->timestamp_ns)) {
                    // 将跟踪三维框投影到图像平面，得到预估ROI区域
                    const auto maybe_roi = resolve_readback_roi(
                        frame->image.size(),
                        project_tracker_box_to_image(
                            snapshot.tracker, T_odom_camera.inverse(), *camera_config,
                            readback_roi_config->box_size_m)
                            .value_or(cv::Rect2f{}),
                        *readback_roi_config, backend->get()->input_resolution());
                    // ROI计算有效，裁剪局部图像用于推理，减少计算量
                    if (maybe_roi) {
                        detector_roi   = *maybe_roi;
                        detector_input = frame->image(detector_roi);
                    }
                }
            }
            // 标记当前帧是否使用局部裁剪推理
            const bool has_detector_roi = is_sub_frame_roi(detector_roi, frame->image.size());

            // 调用推理后端执行神经网络检测，输出原始装甲四点
            auto result = backend->get()->detect(detector_input, detect_color);
            // 推理执行失败，输出空检测包直接退出
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

            // 移动语义接管推理输出检测列表，避免拷贝
            auto detections = std::move(*result);
            // 遍历所有检测，四点逆时针标准化排序
            for (auto& detection : detections) {
                detection.corners = sort_corners(detection.corners);
            }
            // 若使用局部ROI，将所有检测坐标偏移还原到原图坐标系
            offset_detections(detections, detector_roi.tl());
            // 过滤检测：只保留目标颜色、长宽比合法装甲
            std::vector<ArmorDetection> detections_for_pnp;
            detections_for_pnp.reserve(detections.size());
            // 拷贝符合条件的检测进入PnP输入容器
            std::copy_if(
                detections.begin(), detections.end(), std::back_inserter(detections_for_pnp),
                [detect_color](const ArmorDetection& det) {
                    return det.color == detect_color && detection_rect_aspect_ratio_is_valid(det);
                });

            // 打包检测批量数据写入SPMC通道，下发PnP求解System
            detection_out.write(
                ArmorDetectionBatch{
                    std::move(detections_for_pnp), frame->image, frame->timestamp_ns,
                    frame->frame_id, has_detector_roi, detector_roi});
        });

    // ====================== 第二个固定频率System：PnP位姿求解器 200Hz ======================
    scheduler.add_system<talos::fixed_rate<200>>(
        "armor_solver", // 系统标识
        [](
            // SPMC输入：检测器输出的装甲批量检测结果
            talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> detection_in,
            // SPMC输出：里程计系三维装甲测量结果（供给L3跟踪层）
            talos::spmc_mut<ArmorMeasurementBatch, MeasurementChannelTopic> measurement_out,
            // 可变全局资源：PnP求解器单例
            talos::res<std::shared_ptr<PnPSolver>> solver_ptr,
            // 只读TF坐标系
            talos::res<fast_tf::CoordinateSystem> tf_system, 
            // 只读相机内参畸变
            talos::res<CameraConfig> camera_config,
            // ROI配置只读资源
            talos::res<ArmorReadbackRoiConfig> readback_roi_config, 
            // 全局功能掩码
            core::capabilities cap,
            // 上一帧跟踪缓存只读资源
            talos::res<TrackerReadbackCache> readback_cache
        ) mutable {
            // 未开启装甲识别功能，直接退出不执行计算
            if (!core::capable(*cap, core::Capability::Armor)) {
                return;
            }
            // 读取一帧检测批量数据，无数据返回
            auto detections = detection_in.read();
            if (!detections) {
                return;
            }

            // 根据帧时间戳查询里程计→相机静态TF变换
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, detections->timestamp_ns);

            // TF查询失败，日志报错丢弃当前帧
            if (!tf_lookup) {
                SPDLOG_ERROR(
                    "TF lookup failed for timestamp {}: {}", detections->timestamp_ns,
                    tf_lookup.error());
                return;
            }

            // 取出正反坐标变换（里程计↔相机）
            const auto T_odom_camera     = tf_lookup.value();
            const auto T_camera_odom     = T_odom_camera.inverse();
            const auto R_odom_camera     = T_odom_camera.rotation();
            // 加载上一帧跟踪快照
            const auto tracker_snapshot  = readback_cache->load();
            // 判断跟踪缓存是否新鲜未超时
            const bool snapshot_is_fresh = tracker_snapshot_is_fresh(
                tracker_snapshot, *readback_roi_config, detections->timestamp_ns);

            // Eigen矩阵转OpenCV Mat，避免循环重复转换
            cv::Mat camera_matrix_cv;
            cv::Mat dist_coeffs_cv;
            cv::eigen2cv(camera_config->camera_matrix, camera_matrix_cv);
            cv::eigen2cv(camera_config->distort_coefficient, dist_coeffs_cv);

            // 存储相机系PnP求解结果容器
            std::vector<CameraArmorMeasurement> measurements;
            measurements.reserve(detections->detections.size());
            // 遍历每一个合法装甲检测，执行PnP+BA优化
            for (const auto& detection : detections->detections) {
                // 根据跟踪缓存是否有效生成多组先验位姿
                const auto priors = snapshot_is_fresh
                                      ? make_pose_priors(
                                            detection, tracker_snapshot, T_camera_odom,
                                            camera_matrix_cv, dist_coeffs_cv)
                                      : std::vector<PnPSolver::PosePrior>{};
                // 调用PnP求解器，传入多先验光束平差优化
                auto result =
                    (*solver_ptr)
                        ->solve_with_ba(detection, R_odom_camera, detections->timestamp_ns, priors);
                // 求解成功才存入结果容器
                if (result) {
                    // 前哨站装甲特殊角度过滤逻辑，消除目标歧义
                    if (result->name == ArmorName::Outpost) {
                        // 将相机系位姿转换到里程计坐标系
                        auto target_in_ref = T_odom_camera * result->transform;
                        // 分解平移向量得到平面偏航角
                        auto target_pos_yaw = core::math::xyz2ypd(target_in_ref.translation())[0];
                        // 取出目标自身欧拉旋转
                        auto [roll, pitch, target_yaw] = target_in_ref.euler_rot().rpy();
                        // 角度差超过25度判定为歧义目标，直接过滤丢弃
                        auto delta_angle = core::math::normalize_angle(target_yaw - target_pos_yaw)
                                         * 180.0 / std::numbers::pi;
                        if (std::abs(delta_angle) > 25) {
                            continue;
                        }
                    }
                    measurements.push_back(std::move(*result));
                }
            }

            // 将所有相机系测量转换为里程计坐标系，并同步转换协方差矩阵
            std::vector<ArmorMeasurement> odom_measurements;
            odom_measurements.reserve(measurements.size());
            for (const auto& meas : measurements) {
                // 坐标变换
                auto odom_measurement = meas.reframe(T_odom_camera);
                // 协方差矩阵雅可比传播转换
                odom_measurement.pnp_cov_ypdr =
                    reframe_camera_pnp_cov_ypdr_to_odom(meas, odom_measurement, T_odom_camera);
                odom_measurements.push_back(std::move(odom_measurement));
            }

            // 批量测量结果写入SPMC通道，输出给L3跟踪系统
            measurement_out.write(
                ArmorMeasurementBatch{
                    std::move(odom_measurements), detections->timestamp_ns, detections->frame_id});
        });
}

// 函数签名说明：
// 返回值 std::expected<DetectorBackendHandle, std::string>
//  ✅ 成功：返回 DetectorBackendHandle 探测器后端句柄
//  ❌ 失败：返回 std::unexpected + string 类型错误信息
// noexcept：本函数**不会抛出C++异常**，适合机器人实时链路，避免异常打乱调度时序
std::expected<DetectorBackendHandle, std::string>
create_detector_backend_handle(const ArmorDetectorConfig& config) noexcept {
    // 调用底层工厂函数，根据配置创建真正的推理后端实例
    // 内部自动根据 config.backend_type 选择：Axera / TensorRT / ONNX Runtime
    auto backend_result = create_detector_backend(config);

    // 判断：推理后端实例创建失败（比如模型文件找不到、硬件NPU打不开、TensorRT引擎加载失败）
    if (!backend_result) {
        // 把底层的错误信息move转移所有权，封装进unexpected向上返回
        // std::move 避免字符串拷贝，减少内存开销
        return std::unexpected(std::move(backend_result.error()));
    }

    // backend_result.value() 是 DetectorBackend 裸实例，用shared_ptr包装
    // 目的：多地方共享同一个推理后端、自动生命周期管理，不用手动delete
    auto backend_ptr  = std::make_shared<DetectorBackend>(std::move(*backend_result));

    // magic_enum：编译期反射枚举，把 backend_type 枚举值转成可读字符串（如"Axera" / "TensorRT"）
    // 编译期完成转换，运行时无开销，专门用来打日志、调试Foxglove面板
    auto backend_name = std::string(magic_enum::enum_name(config.backend_type));

    // 构造句柄结构体返回：持有shared_ptr推理实例 + 后端名字串
    // DetectorBackendHandle本质就是一个轻量包装，对外屏蔽底层shared_ptr细节
    return DetectorBackendHandle{std::move(backend_ptr), std::move(backend_name)};
}

/**
 * @brief 创建PnP光束平差求解器共享智能指针
 * @param config 相机内参、畸变系数配置
 * @return std::shared_ptr<PnPSolver> 求解器全局单例
 */
std::shared_ptr<PnPSolver> create_pnp_solver(const CameraConfig& config) noexcept {
    // 构造求解器，传入相机标定参数，返回共享指针
    return std::make_shared<PnPSolver>(config);
}

} // 顶层L2感知命名空间结束