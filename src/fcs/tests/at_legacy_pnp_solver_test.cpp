// Google Test 单元测试框架头文件，提供 TEST / ASSERT / EXPECT 系列断言宏
#include <gtest/gtest.h>

// C++标准容器
#include <array>
#include <vector>
// 数学库：sqrt、基础浮点运算
#include <cmath>
// C++20 标准数学常量：π
#include <numbers>

// 项目全局基础类型定义（ArmorDetection、CameraConfig、CameraArmorMeasurement、枚举等）
#include "core/types.hpp"

// Eigen 线性代数库：矩阵、旋转、位姿运算
#include <Eigen/Core>
#include <Eigen/Geometry>
// OpenCV 标定/PnP、基础矩阵、投影函数
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// L2感知层：装甲PnP求解器核心实现，本次测试目标模块
#include "L2_perception/armor/solver.hpp"
// L3估计层：运动模型（仅头文件依赖，本用例不直接调用）
#include "L3_estimation/tracker/new_motion_model.hpp"
// 相机内参、畸变参数结构体定义
#include "camera_config.hpp"

// 匿名命名空间：隔离测试内部工具函数，不会对外导出符号，避免链接冲突
namespace {

/**
 * @brief 构造一套标准化测试相机标定参数（仿真相机内参+畸变系数）
 * @return fcs::CameraConfig 填充好内参、畸变、分辨率的相机配置对象
 * 作用：所有PnP测试统一使用同一套相机参数，消除相机差异带来的测试波动
 */
fcs::CameraConfig make_test_camera_config() {
    fcs::CameraConfig config;
    // 3x3相机内参矩阵 K
    // fx  0  cx
    // 0  fy  cy
    // 0   0   1
    config.camera_matrix << 
        820.0, 0.0, 720.0,
        0.0, 815.0, 540.0,
        0.0, 0.0, 1.0;
    // 5阶径向+切向畸变系数 [k1,k2,p1,p2,k3]
    config.distort_coefficient << -0.085, 0.012, 0.0015, -0.0008, 0.0;
    // 图像分辨率 1440*1080
    config.width  = 1440;
    config.height = 1080;
    return config;
}

/**
 * @brief 获取装甲标准三维模型点（物体坐标系下4个灯条顶点，单位米）
 * @param type 装甲类型：大装甲 / 小装甲
 * @return std::vector<cv::Point3f> 4个3D特征点，构成装甲矩形框
 * 坐标系约定：装甲物体坐标系
 * X：垂直装甲面向外；Y：装甲水平左右；Z：装甲竖直上下
 */
std::vector<cv::Point3f> armor_model_points(fcs::ArmorType type) {
    // 尺寸单位 mm 转 m
    constexpr double kSmallWidth  = 135.0 / 1000.0;  // 小装甲横向宽度
    constexpr double kSmallHeight = 55.0 / 1000.0;   // 装甲灯条竖直高度
    constexpr double kLargeWidth  = 230.0 / 1000.0; // 大装甲横向宽度
    constexpr double kLargeHeight = 55.0 / 1000.0;

    // 根据装甲类型选择尺寸
    const double width  = (type == fcs::ArmorType::Large) ? kLargeWidth : kSmallWidth;
    const double height = (type == fcs::ArmorType::Large) ? kLargeHeight : kSmallHeight;

    // 半宽、半高，用于生成四角坐标
    const float hw = static_cast<float>(width * 0.5);
    const float hh = static_cast<float>(height * 0.5);
    // 四个顶点：右上、左上、左下、右下（对应灯条四角顺序）
    return {
        cv::Point3f(0.0f, hw, hh),
        cv::Point3f(0.0f, -hw, hh),
        cv::Point3f(0.0f, -hw, -hh),
        cv::Point3f(0.0f, hw, -hh),
    };
}

/**
 * @brief 生成装甲真实旋转矩阵（Z偏航 + Y俯仰复合旋转）
 * @param yaw 装甲绕自身Z轴偏航角 rad
 * @param name 装甲编号，用于查表获取该型号装甲固定俯仰倾斜角
 * @return Eigen::Matrix3d 物体坐标系旋转矩阵 R_obj
 * 旋转顺序：先俯仰tilt(Y轴)，再yaw(Z轴)，符合赛场装甲物理倾斜特性
 */
Eigen::Matrix3d armor_rotation(double yaw, fcs::ArmorName name) {
    // 根据装甲编号获取固定俯仰倾斜角（不同型号装甲安装倾角不同）
    const double tilt = fcs::L2::armor_pitch_rad_for(name);
    // Z轴偏航旋转
    const Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
    // Y轴俯仰倾斜旋转
    const Eigen::AngleAxisd pitch_rot(tilt, Eigen::Vector3d::UnitY());
    // 复合旋转：先tilt，后yaw，返回旋转矩阵
    return (yaw_rot * pitch_rot).toRotationMatrix();
}

/**
 * @brief Eigen旋转矩阵 → OpenCV Rodrigues旋转向量rvec转换工具
 * @param rotation Eigen3x3旋转矩阵
 * @return cv::Mat 3行1列double类型rvec（OpenCV solvePnP标准输入格式）
 */
cv::Mat eigen_rotation_to_rvec(const Eigen::Matrix3d& rotation) {
    cv::Mat rvec;
    cv::Mat rotation_cv(3, 3, CV_64F);
    // 矩阵逐元素拷贝 Eigen → OpenCV
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            rotation_cv.at<double>(r, c) = rotation(r, c);
        }
    }
    // Rodrigues公式：旋转矩阵 ↔ 旋转向量
    cv::Rodrigues(rotation_cv, rvec);
    return rvec;
}

/**
 * @brief 仿真生成带检测偏差的装甲检测结果（正向生成仿真数据，PnP测试输入）
 * @param name 装甲编号
 * @param color 红蓝颜色
 * @param rotation_gt 真实装甲旋转矩阵
 * @param translation_gt 真实装甲平移向量(相机坐标系，单位m)
 * @param camera_config 相机标定参数
 * @return fcs::ArmorDetection 仿真图像四角检测结果
 * 逻辑流程：
 * 1. 取装甲3D模型点
 * 2. 真实位姿R/t转OpenCV rvec/tvec
 * 3. projectPoints 3D点投影到2D像素
 * 4. 人为添加检测尺度偏差：模拟真实检测器四角向内收缩/外扩误差
 * 5. 构造ArmorDetection检测包
 */
fcs::ArmorDetection make_detection(
    fcs::ArmorName name, fcs::ArmorColor color, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation, const fcs::CameraConfig& camera_config) {
    // 通过编号得到大/小装甲类型
    const auto type = fcs::cls_to_armor_type(name);
    // 获取装甲3D模型点
    const auto obj  = armor_model_points(type);
    // 真实旋转转rvec
    const auto rvec = eigen_rotation_to_rvec(rotation);
    // Eigen平移向量转为OpenCV tvec矩阵
    const cv::Mat tvec =
        (cv::Mat_<double>(3, 1) << translation.x(), translation.y(), translation.z());

    // 拷贝相机内参矩阵 K
    cv::Mat camera_matrix(3, 3, CV_64F);
    cv::Mat dist_coeffs(1, 5, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            camera_matrix.at<double>(r, c) = camera_config.camera_matrix(r, c);
        }
    }
    // 拷贝5个畸变系数
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(0, i) = camera_config.distort_coefficient(i);
    }

    // 3D模型点 + 真实位姿投影到图像2D像素点
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs, projected);
    // 投影必须输出4个角点
    EXPECT_EQ(projected.size(), 4u);

    // 转定长数组存储四角
    std::array<cv::Point2f, 4> corners{};
    std::copy(projected.begin(), projected.end(), corners.begin());

    // 计算装甲图像中心
    const auto center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
    // 仿真检测器尺度偏差：四角向中心点收缩0.65像素，模拟真实检测框偏小系统误差
    constexpr float kSyntheticDetectorScaleBiasPx = 0.65f;
    for (auto& corner : corners) {
        const cv::Point2f delta = center - corner;
        const float norm        = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (norm > 1e-6f) {
            // 沿中心点方向偏移，制造尺度相关偏差
            corner += (kSyntheticDetectorScaleBiasPx / norm) * delta;
        }
    }
    // 构造仿真检测结果，置信度强制1.0（仿真无噪声）
    return fcs::ArmorDetection(corners, name, color, 1.0f);
}

/**
 * @brief 给检测四角施加指定像素向内偏移偏差，用于测试去偏差算法鲁棒性
 * @param detection 原始无偏检测
 * @param bias_px 向内收缩像素值
 * @return 四角收缩后的新检测结构体
 */
fcs::ArmorDetection
    bias_detection_towards_center(const fcs::ArmorDetection& detection, float bias_px) {
    auto corners      = detection.corners;
    const auto center = detection.center();

    for (auto& corner : corners) {
        const cv::Point2f delta = center - corner;
        const float norm        = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (norm > 1e-6f) {
            corner += (bias_px / norm) * delta;
        }
    }

    return fcs::ArmorDetection(corners, detection.name, detection.color, detection.confidence);
}

/**
 * @brief 对带尺度偏差的检测四角执行去偏差矫正，得到矫正后角点
 * @param detection 存在尺度系统偏差的原始检测
 * @return 偏差补偿后的ArmorDetection，用于重投影误差计算
 * 对应业务函数：debias_correlated_corner_scale 解决检测器框缩放带来的PnP距离漂移
 */
fcs::ArmorDetection debiased_detection_for_projection(const fcs::ArmorDetection& detection) {
    // 调用感知层偏差补偿函数，输出矫正后2D角点
    const auto debiased = fcs::L2::debias_correlated_corner_scale(detection.image_points());
    std::array<cv::Point2f, 4> corners{};
    std::copy(debiased.begin(), debiased.end(), corners.begin());
    return fcs::ArmorDetection(corners, detection.name, detection.color, detection.confidence);
}

/**
 * @brief 计算PnP解算位姿对应的重投影RMSE（像素均方根误差）
 * @param measurement PnP输出的相机坐标系装甲测量结果（估计位姿）
 * @param detection 图像检测四角
 * @param camera_config 相机标定参数
 * @return double 重投影误差 单位像素
 * 用途：量化PnP求解精度，误差越小说明估计位姿越贴合图像观测
 */
double reprojection_rmse_px(
    const fcs::CameraArmorMeasurement& measurement, const fcs::ArmorDetection& detection,
    const fcs::CameraConfig& camera_config) {
    // 当前装甲对应3D模型点
    const auto obj_points = armor_model_points(detection.type);
    // 取出PnP求解得到的旋转、平移
    const auto rotation   = measurement.transform.rotation();
    const auto t          = measurement.transform.translation();
    const auto rvec       = eigen_rotation_to_rvec(rotation);
    const cv::Mat tvec    = (cv::Mat_<double>(3, 1) << t.x(), t.y(), t.z());

    // 拷贝相机内参与畸变
    cv::Mat camera_matrix(3, 3, CV_64F);
    cv::Mat dist_coeffs(1, 5, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            camera_matrix.at<double>(r, c) = camera_config.camera_matrix(r, c);
        }
    }
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(0, i) = camera_config.distort_coefficient(i);
    }

    // 用PnP估计位姿把3D模型点重新投影回图像
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs, projected);

    // 逐点计算像素误差平方和
    double sum_sq = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const cv::Point2f d = projected[i] - detection.corners[i];
        sum_sq += static_cast<double>(d.x) * static_cast<double>(d.x)
                + static_cast<double>(d.y) * static_cast<double>(d.y);
    }
    // 均方根RMSE
    return std::sqrt(sum_sq / static_cast<double>(projected.size()));
}

/**
 * @brief 计算估计旋转矩阵与真值旋转矩阵之间的旋转角度误差 rad
 * @param measurement PnP输出估计位姿
 * @param rotation_gt 仿真生成的真实旋转矩阵
 * @return double 两旋转之间最小旋转角（角度误差）
 * 原理：delta = R_est * R_gt^T，delta为真值到估计值的旋转偏差，取旋转轴角度作为误差
 */
double rotation_error_rad(
    const fcs::CameraArmorMeasurement& measurement, const Eigen::Matrix3d& rotation_gt) {
    const Eigen::Matrix3d delta = measurement.transform.rotation() * rotation_gt.transpose();
    return Eigen::AngleAxisd(delta).angle();
}

// ====================== GTest 测试用例 ======================
/**
 * @brief 测试用例1：无先验信息下，带镜头畸变场景中小装甲位姿可正确恢复
 * 场景：小装甲、无历史位姿先验、相机存在畸变、检测带轻微尺度偏差
 * 校验指标：平移误差、旋转角度误差、重投影RMSE
 */
TEST(PnPSolver, RecoversSmallArmorPoseWithoutPriorUnderDistortion) {
    // 获取标准测试相机参数
    const auto camera_config = make_test_camera_config();
    // 实例化PnP求解器，注入相机标定
    fcs::L2::PnPSolver solver(camera_config);

    // 仿真真值
    const double yaw_gt               = 0.42;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::Three);
    const Eigen::Vector3d translation_gt(0.14, -0.08, 4.2);
    // 生成带尺度偏差的仿真检测四角
    const auto detection = make_detection(
        fcs::ArmorName::Three, fcs::ArmorColor::Blue, rotation_gt, translation_gt, camera_config);

    // 执行BA优化PnP求解，无位姿先验，时间戳1234
    const auto result = solver.solve_with_ba(detection, Eigen::Matrix3d::Identity(), 1234);
    // 必须求解成功，否则直接测试失败
    ASSERT_TRUE(result.has_value());

    const auto& measurement = *result;
    // 平移误差小于2毫米
    EXPECT_LT((measurement.transform.translation() - translation_gt).norm(), 2e-3);
    // 旋转误差小于0.003rad
    EXPECT_LT(rotation_error_rad(measurement, rotation_gt), 3e-3);
    // 使用去偏差矫正后的角点计算重投影误差 <0.1像素
    EXPECT_LT(
        reprojection_rmse_px(
            measurement, debiased_detection_for_projection(detection), camera_config),
        0.1);
}

/**
 * @brief 测试用例2：存在带噪声位姿先验、镜头畸变下大装甲位姿求解
 * 场景：大装甲，传入带偏差的历史先验位姿，验证BA能融合先验约束并收敛到真值
 */
TEST(PnPSolver, RecoversLargeArmorPoseWithPosePriorUnderDistortion) {
    const auto camera_config = make_test_camera_config();
    fcs::L2::PnPSolver solver(camera_config);

    // 真实位姿
    const double yaw_gt               = -0.31;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::One);
    const Eigen::Vector3d translation_gt(-0.18, 0.06, 5.4);
    const auto detection = make_detection(
        fcs::ArmorName::One, fcs::ArmorColor::Red, rotation_gt, translation_gt, camera_config);

    // 构造有偏差的先验位姿（yaw+0.06，平移三轴带偏移）
    const Eigen::Matrix3d rotation_prior    = armor_rotation(yaw_gt + 0.06, fcs::ArmorName::One);
    const Eigen::Vector3d translation_prior = translation_gt + Eigen::Vector3d(0.03, -0.02, 0.10);
    const auto prior_rvec                   = eigen_rotation_to_rvec(rotation_prior);

    // 组装先验约束结构体
    std::vector<fcs::L2::PnPSolver::PosePrior> priors;
    priors.push_back({
        .rvec = cv::Vec3d(prior_rvec.at<double>(0), prior_rvec.at<double>(1), prior_rvec.at<double>(2)),
        .tvec      = cv::Vec3d(translation_prior.x(), translation_prior.y(), translation_prior.z()),
        .hint_cost = 1e-4,    // 先验约束权重/代价
        .armor_id  = 0,
    });

    // 带先验执行BA-PnP
    const auto result = solver.solve_with_ba(detection, Eigen::Matrix3d::Identity(), 5678, priors);
    ASSERT_TRUE(result.has_value());

    const auto& measurement = *result;
    EXPECT_LT((measurement.transform.translation() - translation_gt).norm(), 2e-3);
    EXPECT_LT(rotation_error_rad(measurement, rotation_gt), 3e-3);
    EXPECT_LT(
        reprojection_rmse_px(
            measurement, debiased_detection_for_projection(detection), camera_config),
        0.1);
}

/**
 * @brief 测试用例3：强尺度偏差四角+先验约束，验证求解器保持装甲俯仰流形约束
 * 业务背景：不同型号装甲俯仰角固定，PnP优化时强制约束俯仰仅允许装甲固有倾角，只优化yaw和平移
 * 校验：优化后旋转仅存在yaw自由度，俯仰严格贴合装甲固有tilt
 */
TEST(PnPSolver, KeepsConstrainedPoseManifoldWithBiasedCornersAndPrior) {
    const auto camera_config = make_test_camera_config();
    fcs::L2::PnPSolver solver(camera_config);

    // 真实位姿
    const double yaw_gt               = 0.57;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::Five);
    const Eigen::Vector3d translation_gt(0.09, -0.03, 4.8);
    const auto detection_gt = make_detection(
        fcs::ArmorName::Five, fcs::ArmorColor::Blue, rotation_gt, translation_gt, camera_config);
    // 额外施加1.2像素向内尺度偏差，放大检测系统误差
    const auto detection = bias_detection_towards_center(detection_gt, 1.2f);

    // 构造带误差先验
    const Eigen::Matrix3d rotation_prior    = armor_rotation(yaw_gt - 0.08, fcs::ArmorName::Five);
    const Eigen::Vector3d translation_prior = translation_gt + Eigen::Vector3d(-0.02, 0.02, 0.12);
    const auto prior_rvec                   = eigen_rotation_to_rvec(rotation_prior);

    std::vector<fcs::L2::PnPSolver::PosePrior> priors;
    priors.push_back({
        .rvec = cv::Vec3d(prior_rvec.at<double>(0), prior_rvec.at<double>(1), prior_rvec.at<double>(2)),
        .tvec      = cv::Vec3d(translation_prior.x(), translation_prior.y(), translation_prior.z()),
        .hint_cost = 5e-4,
        .armor_id  = 0,
    });

    const auto result = solver.solve_with_ba(detection, Eigen::Matrix3d::Identity(), 9012, priors);
    ASSERT_TRUE(result.has_value());

    const auto& measurement = *result;
    // 从求解得到的旋转中提取yaw
    const double yaw_est    = fcs::L2::extract_yaw_from_rotation(
        measurement.transform.rotation(), Eigen::Matrix3d::Identity());
    // 使用当前装甲固定俯仰重构标准旋转矩阵
    const Eigen::Matrix3d constrained_rotation = armor_rotation(yaw_est, detection.name);
    // 估计旋转与约束旋转的偏差
    const Eigen::Matrix3d delta =
        measurement.transform.rotation() * constrained_rotation.transpose();

    // 偏差旋转角极小，证明俯仰被约束死，仅yaw可变（流形约束生效）
    EXPECT_LT(Eigen::AngleAxisd(delta).angle(), 1e-6);
    // 整体旋转误差小于0.03rad
    EXPECT_LT(rotation_error_rad(measurement, rotation_gt), 3e-2);
    // 深度z必须大于1米，过滤异常近距离解
    EXPECT_GT(measurement.transform.translation().z(), 1.0);
}

/**
 * @brief 测试用例4：验证尺度相关角点偏差会放大深度方向协方差（不确定性）
 * 业务逻辑：检测器框向内收缩会导致PnP估计距离偏远；同时求解器输出的协方差矩阵深度方差变大，反映不确定性提升
 * 校验两点：
 * 1. 带偏差解算出来的深度大于真实深度
 * 2. 偏差场景下深度协方差显著高于无偏差干净检测
 */
TEST(PnPSolver, CorrelatedCornerScaleBiasInflatesLogDistanceCovariance) {
    const auto camera_config = make_test_camera_config();
    fcs::L2::PnPSolver solver(camera_config);

    // 真实位姿
    const double yaw_gt               = 0.18;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::Three);
    const Eigen::Vector3d translation_gt(0.0, -0.04, 6.0);
    const auto detection_gt = make_detection(
        fcs::ArmorName::Three, fcs::ArmorColor::Blue, rotation_gt, translation_gt, camera_config);
    // 施加1像素尺度偏差
    const auto biased_detection = bias_detection_towards_center(detection_gt, 1.0f);

    // 分别求解干净无偏检测、带偏差检测
    const auto clean_result = solver.solve_with_ba(detection_gt, Eigen::Matrix3d::Identity(), 3456);
    const auto biased_result =
        solver.solve_with_ba(biased_detection, Eigen::Matrix3d::Identity(), 3457);

    ASSERT_TRUE(clean_result.has_value());
    ASSERT_TRUE(biased_result.has_value());

    // 取出深度维度对数方差（协方差矩阵Z轴分量）
    const double clean_log_var   = clean_result->pnp_cov_ypdr(2, 2);
    const double biased_log_var  = biased_result->pnp_cov_ypdr(2, 2);
    // 估计距离模长 & 真实距离模长
    const double biased_distance = biased_result->transform.translation().norm();
    const double gt_distance     = translation_gt.norm();

    // 带偏差检测算出的距离比真实远5%以上
    EXPECT_GT(biased_distance, gt_distance * 1.05);
    // 偏差场景深度方差更大
    EXPECT_GT(biased_log_var, clean_log_var);
    // 深度标准差至少0.08，量化不确定性放大效果
    EXPECT_GT(std::sqrt(std::max(0.0, biased_log_var)), 0.08);
}

} // namespace