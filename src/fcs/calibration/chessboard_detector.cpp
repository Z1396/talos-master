// 棋盘格检测器头文件（类声明、继承CalibrationBoard抽象接口）
#include "calibration/chessboard_detector.hpp"

// OpenCV标定核心：findChessboardCorners、cornerSubPix、drawChessboardCorners
#include <opencv2/calib3d.hpp>
// 图像处理：灰度转换、图像格式转换
#include <opencv2/imgproc.hpp>

namespace fcs::calibration {

/**
 * @brief ChessboardDetector 构造函数
 * 预生成棋盘格全部三维世界坐标点，存储行列尺寸、方格物理边长
 * @param width 棋盘横向内角点数量（列数）
 * @param height 棋盘纵向内角点数量（行数）
 * @param square_size 棋盘方格边长，单位米
 * @ noexcept 无抛异常，坐标计算无分配失败风险
 */
ChessboardDetector::ChessboardDetector(uint32_t width, uint32_t height, double square_size) noexcept
    // 转为int存入成员尺寸
    : board_size_(static_cast<int>(width), static_cast<int>(height))
    , square_size_(square_size)
    , object_points_() {
    // ===================== 预生成标定板三维坐标 =====================
    // 标定板坐标系：原点左上角，所有点Z=0（平面棋盘）
    object_points_.reserve(static_cast<size_t>(width) * height);
    // 遍历行 i 纵向
    for (uint32_t i = 0; i < height; ++i) {
        // 遍历列 j 横向
        for (uint32_t j = 0; j < width; ++j) {
            // X = j * 方格尺寸，Y = i * 方格尺寸，Z固定0
            object_points_.emplace_back(
                static_cast<float>(j * square_size), static_cast<float>(i * square_size), 0.0f);
        }
    }
}

/**
 * @brief 实现CalibrationBoard虚接口：单帧图像棋盘格角点检测
 * @param image 输入原图 BGR彩色 / 灰度单通道
 * @param ts 图像采集纳秒时间戳，绑定检测结果用于TF时序对齐
 * @return std::expected<CornerDetection, std::string>
 *         成功：填充完整角点检测结果；失败：携带错误文本
 */
std::expected<CornerDetection, std::string>
    ChessboardDetector::detect(const cv::Mat& image, timestamp_ns_t ts) noexcept {
    // 初始化检测结果结构体
    CornerDetection result;
    result.timestamp_ns  = ts;
    // 构造阶段预生成的三维点位直接赋值，无需重复计算
    result.object_points = object_points_;
    // 拷贝原图存入结果，供可视化绘制使用
    result.image         = image.clone();

    // 图像预处理：彩色图转灰度，灰度图直接复用
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // 棋盘检测标志：FAST_CHECK快速预检测，提前过滤无棋盘图像，提升速度
    int flags = cv::CALIB_CB_FAST_CHECK;

    // 核心：OpenCV棋盘格粗检测，输出像素角点存入image_points
    result.success = cv::findChessboardCorners(gray, board_size_, result.image_points, flags);

    // 粗检测无棋盘，返回失败
    if (!result.success) {
        return std::unexpected("Chessboard pattern not found");
    }

    // ===================== 亚像素精细化角点 =====================
    // 迭代终止条件：精度0.001像素 或 最大迭代30次
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
    // 11×11窗口亚像素优化，无死区(-1,-1)
    cv::cornerSubPix(gray, result.image_points, cv::Size(11, 11), cv::Size(-1, -1), criteria);

    // 全部流程正常，返回填充完成的检测结果
    return result;
}

/**
 * @brief 实现CalibrationBoard虚接口：获取预生成三维坐标只读引用
 */
const std::vector<cv::Point3f>& ChessboardDetector::object_points() const noexcept {
    return object_points_;
}

/**
 * @brief 实现CalibrationBoard虚接口：在原图绘制棋盘格角点连线可视化
 * @param image 原始输入图像
 * @param detection detect输出的角点检测结果
 * @return 绘制角点与连线的新图像，不修改输入原图
 */
cv::Mat ChessboardDetector::draw_corners(
    const cv::Mat& image, const CornerDetection& detection) const noexcept {
    // 拷贝原图用于绘制
    cv::Mat output = image.clone();
    // 灰度图转为BGR彩色，保证彩色绘制线条可见
    if (output.channels() == 1) {
        cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
    }

    // OpenCV内置工具：绘制角点+顺序连线，success控制是否彩色高亮
    cv::drawChessboardCorners(output, board_size_, detection.image_points, detection.success);

    return output;
}

} // namespace fcs::calibration