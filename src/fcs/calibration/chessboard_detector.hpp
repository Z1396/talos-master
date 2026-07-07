#pragma once
// 标定板抽象基类 CalibrationBoard 头文件，提供统一虚接口定义
#include "calibration_board.hpp"

namespace fcs::calibration {

/**
 * @brief 标准黑白棋盘格标定板检测器，继承 CalibrationBoard 抽象接口
 * 实现纯棋盘格内角点检测逻辑，用于相机内参标定采集
 */
class ChessboardDetector : public CalibrationBoard {
public:
    /**
     * @brief 棋盘格检测器构造函数
     * @param width 宽度方向内角点数量（棋盘横向交点数，不含外边框）
     * @param height 高度方向内角点数量（棋盘纵向交点数）
     * @param square_size 单个方格物理边长，单位米
     * @ noexcept 构造无抛异常，三维坐标预生成无失败分支
     */
    ChessboardDetector(uint32_t width, uint32_t height, double square_size) noexcept;

    /**
     * @brief 重写基类虚接口：单帧图像执行棋盘格角点检测
     * @param image 输入图像（BGR彩色 / 灰度单通道）
     * @param ts 图像采集纳秒级时间戳，绑定检测结果用于TF时序对齐
     * @return std::expected<CornerDetection, std::string>
     *         成功：填充完整角点、三维点位、原图、时间戳；失败：错误描述字符串
     */
    [[nodiscard]] std::expected<CornerDetection, std::string>
        detect(const cv::Mat& image, timestamp_ns_t ts) noexcept override;

    /**
     * @brief 重写基类虚接口：获取标定板预生成三维世界坐标点只读引用
     * @return 存储全部棋盘内角点3D坐标的向量，Z恒为0（平面标定板）
     */
    [[nodiscard]] const std::vector<cv::Point3f>& object_points() const noexcept override;

    /**
     * @brief 重写基类虚接口：在原图绘制检测到的棋盘角点与连线，可视化调试
     * @param image 原始输入图像
     * @param detection detect 输出的角点检测结果结构体
     * @return 绘制角点后的新图像，不修改传入原图
     */
    [[nodiscard]] cv::Mat draw_corners(
        const cv::Mat& image, const CornerDetection& detection) const noexcept override;

    /**
     * @brief 重写基类虚接口：返回当前标定板类型枚举
     * @return BoardType::Chessboard 棋盘格类型标识
     */
    [[nodiscard]] BoardType type() const noexcept override { return BoardType::Chessboard; }

private:
    cv::Size board_size_;               ///< 存储棋盘内角点行列尺寸 (width, height)
    double square_size_;                ///< 方格物理边长（米）
    std::vector<cv::Point3f> object_points_; ///< 预生成全部棋盘三维标准坐标点
};

} // namespace fcs::calibration