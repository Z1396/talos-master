#include "calibration/intrinsic_calibrator.hpp"

// 数学常量、三角函数、弧度角度换算
#include <cmath>
// 文件写入，持久化内参TOML结果
#include <fstream>
// 圆周率 π 常量
#include <numbers>
// OpenCV相机标定、solvePnP、projectPoints核心函数
#include <opencv2/calib3d.hpp>
// OpenCV Eigen矩阵双向转换工具
#include <opencv2/core/eigen.hpp>

namespace fcs::calibration {

/**
 * @brief 相机内参标定求解器实现
 * 负责采集棋盘/ChArUco角点样本、姿态多样性过滤、执行opencv相机标定、重投影计算、结果保存
 */
IntrinsicCalibrator::IntrinsicCalibrator(
    const CaptureConfig& capture, const IntrinsicConfig& intrinsic) noexcept
    : capture_config_(capture)
    , intrinsic_config_(intrinsic)
    , samples_()
    , sample_rvecs_()
    , sample_tvecs_() {
    // 预分配最大样本容量，避免vector动态扩容
    samples_.reserve(capture.max_samples);
    sample_rvecs_.reserve(capture.max_samples);
    sample_tvecs_.reserve(capture.max_samples);
}

/**
 * @brief 判断当前帧角点检测结果姿态是否足够多样，过滤重复相似视角
 * @param detection 单帧图像角点检测结果
 * @return true 视角差异达标可采集；false 姿态重复丢弃
 */
bool IntrinsicCalibrator::is_diverse_enough(const CornerDetection& detection) const noexcept {
    // 无历史样本，直接允许采集
    if (samples_.empty()) {
        return true;
    }

    // 粗略相机矩阵估算，用于临时PnP求解姿态做多样性校验
    cv::Mat camera_matrix          = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = 1000.0;                       // fx 预估焦距
    camera_matrix.at<double>(1, 1) = 1000.0;                       // fy 预估焦距
    camera_matrix.at<double>(0, 2) = 640.0;                        // cx 预估图像中心X
    camera_matrix.at<double>(1, 2) = 360.0;                        // cy 预估图像中心Y
    cv::Mat dist_coeffs            = cv::Mat::zeros(5, 1, CV_64F); // 畸变全部置0临时估算
    cv::Vec3d rvec, tvec;

    // PnP求解当前帧标定板相对相机旋转平移
    bool success = cv::solvePnP(
        detection.object_points, detection.image_points, camera_matrix, dist_coeffs, rvec, tvec);
    if (!success) {
        return false;
    }

    // 角度阈值转弧度
    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    const double min_angle_rad  = capture_config_.min_angle_diff * deg_to_rad;
    const double min_trans      = capture_config_.min_translation_diff;

    // 遍历所有历史样本对比姿态差异
    for (size_t i = 0; i < sample_rvecs_.size(); ++i) {
        // 旋转向量差值范数代表角度差
        cv::Vec3d rvec_diff = rvec - sample_rvecs_[i];
        double angle_diff   = cv::norm(rvec_diff);
        // 平移向量欧氏距离
        cv::Vec3d tvec_diff = tvec - sample_tvecs_[i];
        double trans_diff   = cv::norm(tvec_diff);

        // 角度、平移同时小于阈值，判定为重复视角，拒绝采集
        if (angle_diff < min_angle_rad && trans_diff < min_trans) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 将有效角点检测帧存入样本库，同时缓存该帧姿态rvec/tvec用于多样性校验
 * @param detection 图像角点检测结果
 * @return 成功空expected / 失败错误字符串
 */
std::expected<void, std::string>
    IntrinsicCalibrator::add_sample(const CornerDetection& detection) noexcept {
    // 检测失败，不存入样本
    if (!detection.success) {
        return std::unexpected("Detection was not successful");
    }
    // 达到最大样本上限，拒绝新增
    if (samples_.size() >= capture_config_.max_samples) {
        return std::unexpected("Maximum sample count reached");
    }

    // 粗略内参临时矩阵，用于PnP求解姿态缓存
    cv::Mat camera_matrix          = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = 1000.0;
    camera_matrix.at<double>(1, 1) = 1000.0;
    camera_matrix.at<double>(0, 2) = 640.0;
    camera_matrix.at<double>(1, 2) = 360.0;
    cv::Mat dist_coeffs            = cv::Mat::zeros(5, 1, CV_64F);
    cv::Vec3d rvec, tvec;

    // 求解当前帧姿态用于后续多样性判断
    bool success = cv::solvePnP(
        detection.object_points, detection.image_points, camera_matrix, dist_coeffs, rvec, tvec);
    if (!success) {
        return std::unexpected("Failed to estimate pose for diversity check");
    }

    // 存入样本、同步缓存旋转平移向量
    samples_.push_back(detection);
    sample_rvecs_.push_back(rvec);
    sample_tvecs_.push_back(tvec);
    return {};
}

/**
 * @brief 执行相机内参标定，批量求解camera_matrix、畸变系数、整体RMS误差
 * @param image_size 图像分辨率宽高
 * @return 内参标定结果结构体 / 样本不足错误
 */
std::expected<IntrinsicResult, std::string>
    IntrinsicCalibrator::calibrate(cv::Size image_size) noexcept {
    // 样本数量未达最低要求，无法标定
    if (samples_.size() < capture_config_.min_samples) {
        return std::unexpected(
            fmt::format(
                "Not enough samples: {} < {}", samples_.size(), capture_config_.min_samples));
    }

    // 组装OpenCV标定所需多层向量：全部帧三维点、全部帧二维像素角点
    std::vector<std::vector<cv::Point3f>> object_points_list;
    std::vector<std::vector<cv::Point2f>> image_points_list;
    object_points_list.reserve(samples_.size());
    image_points_list.reserve(samples_.size());
    for (const auto& sample : samples_) {
        object_points_list.push_back(sample.object_points);
        image_points_list.push_back(sample.image_points);
    }

    // 初始化相机矩阵，主点预置图像中心
    cv::Mat camera_matrix          = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 2) = image_size.width / 2.0;
    camera_matrix.at<double>(1, 2) = image_size.height / 2.0;
    cv::Mat dist_coeffs            = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    // 转换配置约束为OpenCV标定flag
    int flags = intrinsic_config_.to_opencv_flags();
    // 核心标定函数，返回全局重投影RMS误差
    double rms = cv::calibrateCamera(
        object_points_list, image_points_list, image_size, camera_matrix, dist_coeffs, rvecs, tvecs,
        flags);

    // OpenCV结果转Eigen存储结构体
    IntrinsicResult result;
    cv::cv2eigen(camera_matrix, result.camera_matrix);
    // 5阶畸变系数 k1 k2 p1 p2 k3 填入Eigen行向量
    for (int i = 0; i < 5; ++i) {
        result.distort_coefficient(0, i) = dist_coeffs.at<double>(i, 0);
    }

    result.rms_error   = rms;
    result.num_samples = static_cast<uint32_t>(samples_.size());
    result.width       = static_cast<uint32_t>(image_size.width);
    result.height      = static_cast<uint32_t>(image_size.height);
    return result;
}

/**
 * @brief 使用已求解内参对单帧角点做重投影，计算预测像素坐标
 * @param detection 单帧原始检测角点
 * @param result 完整相机内参标定结果
 * @return 重投影二维像素点数组，求解失败返回空容器
 */
std::vector<cv::Point2f> IntrinsicCalibrator::reproject(
    const CornerDetection& detection, const IntrinsicResult& result) noexcept {
    std::vector<cv::Point2f> reprojected;

    // Eigen内参转回OpenCV矩阵
    cv::Mat camera_matrix;
    cv::eigen2cv(result.camera_matrix, camera_matrix);
    cv::Mat dist_coeffs(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(0, i) = result.distort_coefficient(0, i);
    }

    // PnP求解当前帧板相对相机姿态
    cv::Vec3d rvec, tvec;
    bool success = cv::solvePnP(
        detection.object_points, detection.image_points, camera_matrix, dist_coeffs, rvec, tvec);
    if (!success) {
        return reprojected;
    }

    // 根据三维点、姿态、内参计算图像像素重投影坐标
    cv::projectPoints(detection.object_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected);
    return reprojected;
}

/**
 * @brief 清空全部采集样本与姿态缓存，重置标定器状态
 */
void IntrinsicCalibrator::clear() noexcept {
    samples_.clear();
    sample_rvecs_.clear();
    sample_tvecs_.clear();
}

/**
 * @brief 获取本地系统当前时间字符串，用于标定文件头部记录时间戳
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
 * @brief 将相机内参标定结果写入TOML文件持久化
 * @param result 内参求解输出结构体
 * @param path 输出文件路径
 * @return 文件写入成功 / 打开失败错误
 */
std::expected<void, std::string>
    save_intrinsic_result(const IntrinsicResult& result, const std::string& path) noexcept {
    std::ofstream file(path);
    if (!file) {
        return std::unexpected(fmt::format("Failed to open file for writing: {}", path));
    }

    // TOML文件头部注释信息
    file << "# Camera Intrinsic Calibration Result\n";
    file << "# RMS Error: " << result.rms_error << " pixels\n";
    file << "# Samples: " << result.num_samples << "\n\n";
    file << "# Date: " << current_datetime_string() << "\n";

    file << "[camera]\n";
    // 3×3相机矩阵行主序平铺数组
    file << "camera_matrix = [";
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            file << result.camera_matrix(i, j);
            if (i < 2 || j < 2)
                file << ", ";
        }
    }
    file << "]\n";

    // 5维畸变系数数组
    file << "distort_coefficient = [";
    for (int i = 0; i < 5; ++i) {
        file << result.distort_coefficient(0, i);
        if (i < 4)
            file << ", ";
    }
    file << "]\n";

    file << "width = " << result.width << "\n";
    file << "height = " << result.height << "\n\n";

    // 标定元信息
    file << "[calibration_info]\n";
    file << "rms_error = " << result.rms_error << "\n";
    file << "num_samples = " << result.num_samples << "\n";
    return {};
}

} // namespace fcs::calibration