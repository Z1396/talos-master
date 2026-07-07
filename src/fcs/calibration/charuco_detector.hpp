#pragma once
// 标定板抽象基类 CalibrationBoard 定义
#include "calibration_board.hpp"

// 仅编译开启OpenCV Aruco模块时引入相关头文件
#ifdef HAVE_OPENCV_ARUCO
// 优先引入OpenCV核心基础类型，保证cv::Size/cv::Mat等定义前置
# include <opencv2/core.hpp>
// Aruco标记字典、检测器基础API
# include <opencv2/aruco.hpp>
// ChArUco复合标定板专用类
# include <opencv2/aruco/charuco.hpp>
#endif

namespace fcs::calibration {

#ifdef HAVE_OPENCV_ARUCO
/**
 * @brief ChArUco标定板检测器实现类，继承CalibrationBoard抽象接口
 * 依赖OpenCV aruco模块，融合棋盘格+ArUco标记，低纹理场景检测鲁棒性更强
 */
class CharucoDetector : public CalibrationBoard {
public:
    /**
     * @brief 构造ChArUco检测器，预生成标定板几何与字典
     * @param width 棋盘格横向方格数量
     * @param height 棋盘格纵向方格数量
     * @param square_size 棋盘方格物理边长，单位米
     * @param marker_size 方格内部ArUco标记边长，单位米
     * @param dictionary ArUco标记字典枚举类型
     * @ noexcept 构造无抛异常，资源初始化失败逻辑在detect返回错误
     */
    CharucoDetector(
        uint32_t width, uint32_t height, double square_size, double marker_size,
        ArucoDictionary dictionary) noexcept;

    /**
     * @brief 虚接口重写：单帧图像检测ChArUco角点
     * @param image 输入图像（BGR彩色/灰度单通道）
     * @param ts 图像采集纳秒时间戳，绑定检测结果用于时序匹配
     * @return expected<CornerDetection, 错误字符串> 检测结果或失败描述
     */
    [[nodiscard]] std::expected<CornerDetection, std::string>
        detect(const cv::Mat& image, timestamp_ns_t ts) noexcept override;

    /**
     * @brief 虚接口重写：获取标定板全部三维世界坐标点
     * @return 只读向量引用，预生成的棋盘格内角点3D点位
     */
    [[nodiscard]] const std::vector<cv::Point3f>& object_points() const noexcept override;

    /**
     * @brief 虚接口重写：在原图绘制检测到的角点用于可视化调试
     * @param image 原始输入图像
     * @param detection detect接口输出的角点检测结果
     * @return 绘制角点后的新图像，不修改原图
     */
    [[nodiscard]] cv::Mat draw_corners(
        const cv::Mat& image, const CornerDetection& detection) const noexcept override;

    /**
     * @brief 虚接口重写：返回当前标定板类型枚举
     * @return BoardType::ChArUco
     */
    [[nodiscard]] BoardType type() const noexcept override { return BoardType::ChArUco; }

private:
    cv::Size board_size_;                  ///< 棋盘格行列尺寸(width, height)
    double square_size_;                   ///< 棋盘方格物理尺寸（米）
    double marker_size_;                   ///< ArUco标记方格尺寸（米）
    cv::aruco::Dictionary dictionary_;     ///< OpenCV内置标记字典实例
    cv::aruco::CharucoBoard charuco_board_;///< OpenCV ChArUco标定板对象，存储几何规则
    cv::aruco::DetectorParameters detector_params_; ///< ArUco标记检测算法参数
    std::vector<cv::Point3f> object_points_; ///< 预加载全部棋盘格三维标准点位
};

#else
/**
 * @brief Aruco模块未编译启用时的空占位Stub实现
 * 保证编译通过，对外接口完全对齐，调用detect直接返回不支持错误
 */
class CharucoDetector : public CalibrationBoard {
public:
    // 构造函数参数占位，无实际初始化逻辑
    CharucoDetector(uint32_t, uint32_t, double, double, ArucoDictionary) noexcept {}

    /**
     * @brief 空实现detect，直接返回ChArUco功能未启用错误
     */
    [[nodiscard]] std::expected<CornerDetection, std::string>
        detect(const cv::Mat&, timestamp_ns_t) noexcept override {
        return std::unexpected("ChArUco support not available (OpenCV aruco module not found)");
    }

    /**
     * @brief 空实现三维点位，返回静态空向量
     */
    [[nodiscard]] const std::vector<cv::Point3f>& object_points() const noexcept override {
        static std::vector<cv::Point3f> empty;
        return empty;
    }

    /**
     * @brief 空实现绘制，直接返回原图拷贝，无任何绘制逻辑
     */
    [[nodiscard]] cv::Mat
        draw_corners(const cv::Mat& image, const CornerDetection&) const noexcept override {
        return image.clone();
    }

    /**
     * @brief 类型标识仍返回ChArUco，保持多态类型一致
     */
    [[nodiscard]] BoardType type() const noexcept override { return BoardType::ChArUco; }
};

#endif // HAVE_OPENCV_ARUCO

} // namespace fcs::calibration