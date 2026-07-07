#include "calibration/handeye_calibrator.hpp"
// 欧拉角、旋转矩阵转换工具 math_fuxk::rpy
#include "euler.hpp"

// 数学常量、三角函数、弧度角度换算
#include <cmath>
// 文件写入，存储手眼标定结果TOML
#include <fstream>
// 输出精度控制 std::setprecision
#include <iomanip>
// 圆周率 π 常量
#include <numbers>
// OpenCV手眼标定、旋转向量矩阵转换
#include <opencv2/calib3d.hpp>
// OpenCV Eigen矩阵互转
#include <opencv2/core/eigen.hpp>
// spdlog日志打印标定结果
#include <spdlog/spdlog.h>

namespace fcs::calibration {

/**
 * @brief 手眼标定求解器构造函数
 * @param capture 采集策略配置（最大/最小样本、姿态差异阈值）
 * @param handeye 手眼求解算法配置（Tsai/Park等）
 * @ noexcept 无抛异常，仅预分配样本容器内存
 */
HandEyeCalibrator::HandEyeCalibrator(
    const CaptureConfig& capture, const HandeyeConfig& handeye) noexcept
    : capture_config_(capture)
    , handeye_config_(handeye)
    , samples_() {
    // 预分配最大样本容量，避免动态扩容
    samples_.reserve(capture.max_samples);
}

/**
 * @brief OpenCV PnP输出 rvec/tvec → ROS标准 camera_link 到标定板变换
 * OpenCV坐标系：相机光心Z向前；ROS相机坐标系Z向前、X向右、Y向下，需要旋转修正
 * @param rvec OpenCV旋转向量
 * @param tvec OpenCV平移向量
 * @return fast_tf变换矩阵 camera_link → calibration_board_frame
 */
fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>
    HandEyeCalibrator::opencv_to_ros(const cv::Vec3d& rvec, const cv::Vec3d& tvec) noexcept {
    // 1. 旋转向量转3×3旋转矩阵
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    // OpenCV矩阵转Eigen
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R_cv, R_eigen);

    Eigen::Vector3d t_eigen(tvec[0], tvec[1], tvec[2]);

    // 4×4齐次变换矩阵
    Eigen::Matrix4d T_opencv   = Eigen::Matrix4d::Identity();
    T_opencv.block<3, 3>(0, 0) = R_eigen;
    T_opencv.block<3, 1>(0, 3) = t_eigen;

    // 固定坐标系转换：opencv光学坐标系 → ROS camera_link坐标系
    // 旋转 rpy[-π/2, 0, -π/2] 对齐坐标轴方向
    constexpr double pi = std::numbers::pi;
    auto T_optical_to_link =
        fast_tf::TransformMatrixd<fast_tf::camera, fast_tf::camera_optical>::from_rpy(
            -pi / 2.0, 0.0, -pi / 2.0, 0.0, 0.0, 0.0);

    // PnP输出：board → camera_optical，左乘转换矩阵得到 board → camera_link
    fast_tf::TransformMatrixd<fast_tf::camera_optical, calibration_board_frame> T_opencv_tf(
        T_opencv);
    return T_optical_to_link * T_opencv_tf;
}

/**
 * @brief 判断当前机器人末端姿态与已有样本是否足够差异（过滤重复姿态样本）
 * @param gimbal_pose odom→云台末端变换矩阵
 * @return true 姿态差异达标，可以采集；false 姿态重复丢弃
 */
bool HandEyeCalibrator::is_diverse_enough(
    const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose)
    const noexcept {
    // 无历史样本，直接允许采集
    if (samples_.empty()) {
        return true;
    }

    // 当前帧旋转、平移
    const Eigen::Matrix3d R_new = gimbal_pose.rotation();
    const Eigen::Vector3d t_new = gimbal_pose.translation();
    auto euler_new              = math_fuxk::rpy(R_new);

    // 配置阈值转弧度
    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    const double min_angle_rad  = capture_config_.min_angle_diff * deg_to_rad;
    const double min_trans      = capture_config_.min_translation_diff;

    // 遍历所有历史样本对比
    for (const auto& sample : samples_) {
        const Eigen::Matrix3d R_old = sample.gimbal_pose.rotation();
        const Eigen::Vector3d t_old = sample.gimbal_pose.translation();
        auto euler_old              = math_fuxk::rpy(R_old);

        // 欧拉角差值总和
        double roll_diff  = std::abs(euler_new.roll - euler_old.roll);
        double pitch_diff = std::abs(euler_new.pitch - euler_old.pitch);
        double yaw_diff   = std::abs(euler_new.yaw - euler_old.yaw);
        double angle_diff = roll_diff + pitch_diff + yaw_diff;

        // 平移向量欧氏距离
        double trans_diff = (t_new - t_old).norm();

        // 角度、平移同时小于阈值 → 姿态重复，拒绝采集
        if (angle_diff < min_angle_rad && trans_diff < min_trans) {
            return false;
        }
    }

    // 所有样本均满足差异要求
    return true;
}

/**
 * @brief 添加一组手眼配对样本（机器人末端位姿 + 标定板相对相机位姿）
 * @param gimbal_pose odom→云台末端变换
 * @param board_pose camera→标定板变换
 * @param ts 图像纳秒时间戳
 * @return 成功空expected；失败携带错误字符串
 */
std::expected<void, std::string> HandEyeCalibrator::add_sample(
    const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose,
    const fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>& board_pose,
    timestamp_ns_t ts) noexcept {
    // 达到最大样本上限，拒绝新增
    if (samples_.size() >= capture_config_.max_samples) {
        return std::unexpected("Maximum sample count reached");
    }

    // 填充样本结构体
    PoseSample sample;
    sample.gimbal_pose  = gimbal_pose;
    sample.board_pose   = board_pose;
    sample.timestamp_ns = ts;

    samples_.push_back(sample);
    return {};
}

/**
 * @brief 执行手眼标定求解，输出相机→云台末端固定变换结果
 * AX = XB 手眼方程：A=机器人末端世界位姿，B=标定板相机位姿，X=待求外参
 * @return 标定结果结构体或样本不足错误
 */
std::expected<HandEyeResult, std::string> HandEyeCalibrator::calibrate() noexcept {
    // 样本数量不足最低要求，直接返回失败
    if (samples_.size() < capture_config_.min_samples) {
        return std::unexpected(
            fmt::format(
                "Not enough samples: {} < {}", samples_.size(), capture_config_.min_samples));
    }

    // OpenCV calibrateHandEye输入容器：A(gripper2base)、B(target2cam)
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;

    R_gripper2base.reserve(samples_.size());
    t_gripper2base.reserve(samples_.size());
    R_target2cam.reserve(samples_.size());
    t_target2cam.reserve(samples_.size());

    // 遍历所有样本转换为OpenCV矩阵格式
    for (const auto& sample : samples_) {
        // A：odom → gimbal_pitch
        Eigen::Matrix3d R_A = sample.gimbal_pose.rotation();
        Eigen::Vector3d t_A = sample.gimbal_pose.translation();

        cv::Mat R_A_cv, t_A_cv;
        cv::eigen2cv(R_A, R_A_cv);
        cv::eigen2cv(t_A, t_A_cv);
        R_gripper2base.push_back(R_A_cv);
        t_gripper2base.push_back(t_A_cv);

        // B：camera → calibration_board_frame
        Eigen::Matrix3d R_B = sample.board_pose.rotation();
        Eigen::Vector3d t_B = sample.board_pose.translation();

        cv::Mat R_B_cv, t_B_cv;
        cv::eigen2cv(R_B, R_B_cv);
        cv::eigen2cv(t_B, t_B_cv);
        R_target2cam.push_back(R_B_cv);
        t_target2cam.push_back(t_B_cv);
    }

    // 获取OpenCV求解算法枚举
    cv::HandEyeCalibrationMethod method = to_opencv_method(handeye_config_.method);

    // 核心手眼求解：输出 X = camera → gripper
    cv::Mat R_cam2gripper, t_cam2gripper;
    cv::calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper,
        method);

    // OpenCV结果转回Eigen
    Eigen::Matrix3d R_result;
    Eigen::Vector3d t_result;
    cv::cv2eigen(R_cam2gripper, R_result);
    cv::cv2eigen(t_cam2gripper, t_result);

    // 旋转矩阵转欧拉角 rpy
    auto euler = math_fuxk::rpy(R_result);

    // 计算整体RMS重投影误差 AX-XB
    double total_error  = 0.0;
    Eigen::Matrix4d X   = Eigen::Matrix4d::Identity();
    X.block<3, 3>(0, 0) = R_result;
    X.block<3, 1>(0, 3) = t_result;

    for (size_t i = 0; i < samples_.size(); ++i) {
        const auto& sample = samples_[i];
        Eigen::Matrix4d A  = sample.gimbal_pose.matrix();
        Eigen::Matrix4d B  = sample.board_pose.matrix();

        Eigen::Matrix4d AX = A * X;
        Eigen::Matrix4d XB = X * B;

        // 矩阵差范数平方累加
        double diff = (AX - XB).norm();
        total_error += diff * diff;
    }
    // 均方根误差
    double rms = std::sqrt(total_error / static_cast<double>(samples_.size()));

    // 填充输出结构体
    HandEyeResult result;
    result.translation = t_result;
    result.rpy         = Eigen::Vector3d(euler.roll, euler.pitch, euler.yaw);
    result.rms_error   = rms;
    result.num_samples = static_cast<uint32_t>(samples_.size());

    return result;
}

/**
 * @brief 清空全部采集样本，重置求解器
 */
void HandEyeCalibrator::clear() noexcept { samples_.clear(); }

/**
 * @brief 获取当前系统本地时间字符串，用于标定文件头部记录
 */
static auto current_datetime_string() noexcept -> std::string {
    auto now      = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm    = *std::localtime(&t);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);
    return buffer;
}

/**
 * @brief 将手眼标定结果写入TOML文件持久化
 * @param result 求解输出
 * @param path 文件存储路径
 * @param method 使用的求解算法
 * @return 文件写入成功/失败
 */
std::expected<void, std::string> save_handeye_result(
    const HandEyeResult& result, const std::string& path, HandEyeMethod method) noexcept {
    std::ofstream file(path);
    // 文件打开失败
    if (!file) {
        return std::unexpected(fmt::format("Failed to open file for writing: {}", path));
    }

    // 固定小数精度6位
    file << std::fixed << std::setprecision(6);

    // 文件注释头信息
    file << "# Camera Extrinsic Calibration Result (Hand-Eye)\n";
    file << "# RMS Error: " << result.rms_error << " mm\n";
    file << "# Samples: " << result.num_samples << "\n\n";
    file << "# Date: " << current_datetime_string() << "\n";

    // 变换数据段
    file << "[transform]\n";
    file << "# camera_link → gimbal_link\n";
    file << "# Translation: [x, y, z] in mm\n";
    file << "# Rotation: [roll, pitch, yaw] in radians\n";
    file << "translation = [" << result.translation.x() << ", " << result.translation.y() << ", "
         << result.translation.z() << "]\n";
    file << "rotation = [" << result.rpy.x() << ", " << result.rpy.y() << ", " << result.rpy.z()
         << "]\n\n";

    // 标定元信息
    file << "[calibration_info]\n";
    file << "rms_error = " << result.rms_error << "\n";
    file << "num_samples = " << result.num_samples << "\n";
    file << "method = \"" << magic_enum::enum_name(method) << "\"\n";

    return {};
}

/**
 * @brief 控制台打印完整手眼标定结果（日志输出）
 * 单位转换：米转毫米，弧度转角度便于阅读
 */
void print_handeye_result(const HandEyeResult& result) noexcept {
    constexpr double rad_to_deg = 180.0 / std::numbers::pi;

    SPDLOG_INFO("=== Camera Extrinsic Calibration Result ===");
    SPDLOG_INFO("Transform: gimbal_link → camera_link");
    SPDLOG_INFO("");
    SPDLOG_INFO("Translation:");
    SPDLOG_INFO("  X (forward): {:7.2f} mm", result.translation.x() * 1000.0);
    SPDLOG_INFO("  Y (left):    {:7.2f} mm", result.translation.y() * 1000.0);
    SPDLOG_INFO("  Z (up):      {:7.2f} mm", result.translation.z() * 1000.0);
    SPDLOG_INFO("");
    SPDLOG_INFO("Rotation:");
    SPDLOG_INFO("  Roll:  {:7.2f}°", result.rpy.x() * rad_to_deg);
    SPDLOG_INFO("  Pitch: {:7.2f}°", result.rpy.y() * rad_to_deg);
    SPDLOG_INFO("  Yaw:   {:7.2f}°", result.rpy.z() * rad_to_deg);
    SPDLOG_INFO("");
    SPDLOG_INFO("RMS Error: {:.2f} mm", result.rms_error * 1000.0);
    SPDLOG_INFO("Samples: {}", result.num_samples);
}

} // namespace fcs::calibration