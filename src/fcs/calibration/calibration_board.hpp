#pragma once
// 标定配置结构体定义（BoardConfig / CharucoConfig）
#include "calibration_config.hpp"
// 标定基础类型、枚举、别名（timestamp_ns_t、BoardType、CornerDetection）
#include "calibration_types.hpp"

// C++23 标准预期类型，承载成功返回值/错误字符串
#include <expected>
// 智能指针 std::unique_ptr，管理多态派生类实例
#include <memory>
// OpenCV 核心矩阵、点、图像基础类型 cv::Mat / cv::Point3f
#include <opencv2/core.hpp>

namespace fcs::calibration {

/**
 * @brief 标定板检测器抽象接口基类
 * 统一各类标定板（棋盘格、ChArUco、圆形网格）的检测、绘制、点位获取行为
 * 基于多态实现，不同标定板子类实现纯虚接口，工厂函数统一创建实例
 */
class CalibrationBoard {
public:
    /**
     * @brief 虚析构函数，保证派生类析构完整调用，unique_ptr安全释放子类资源
     */
    virtual ~CalibrationBoard() = default;

    /**
     * @brief 在输入图像中检测标定板角点图案
     * @param image 输入图像，支持灰度单通道 / BGR三通道彩色图
     * @param ts 图像采集时间戳，单位纳秒 timestamp_ns_t
     * @return std::expected<CornerDetection, std::string>
     *         成功：返回角点检测结果（图像二维角点、世界三维点位、有效标志位等）
     *         失败：携带人类可读错误描述字符串
     * @ noexcept 无抛出异常保证，内部错误全部通过expected返回，不抛异常中断流程
     * @ [[nodiscard]] 禁止忽略返回值，强制处理检测成功/失败分支
     */
    [[nodiscard]] virtual std::expected<CornerDetection, std::string>
        detect(const cv::Mat& image, timestamp_ns_t ts) noexcept = 0;

    /**
     * @brief 获取标定板坐标系下所有三维标准点位（世界坐标）
     * 棋盘格/ChArUco固定几何点位，用于后续solvePnP、相机内参求解
     * @return 只读引用，三维点数组 std::vector<cv::Point3f>
     */
    [[nodiscard]] virtual const std::vector<cv::Point3f>& object_points() const noexcept = 0;

    /**
     * @brief 在原图上绘制检测到的角点，用于可视化调试
     * @param image 原始输入图像
     * @param detection detect()返回的角点检测结果
     * @return 绘制了角点/连线的新图像矩阵，不修改原图
     */
    [[nodiscard]] virtual cv::Mat
        draw_corners(const cv::Mat& image, const CornerDetection& detection) const noexcept = 0;

    /**
     * @brief 获取当前标定板类型枚举（BoardType：棋盘格/ChArUco/圆点等）
     * @return BoardType 枚举值
     */
    [[nodiscard]] virtual BoardType type() const noexcept = 0;
};

/**
 * @brief 标定板检测器工厂创建函数
 * 根据传入配置自动匹配对应CalibrationBoard派生类，屏蔽子类构造细节
 * @param board_config 通用标定板基础配置（行列尺寸、方格尺寸、类型标识）
 * @param charuco_config ChArUco专用扩展配置（标记字典、标记边长等）
 * @return std::expected<std::unique_ptr<CalibrationBoard>, std::string>
 *         成功：返回管理派生检测器实例的独占智能指针
 *         失败：返回配置非法/类型不支持的错误字符串
 */
[[nodiscard]] std::expected<std::unique_ptr<CalibrationBoard>, std::string>
    create_board(const BoardConfig& board_config, const CharucoConfig& charuco_config) noexcept;

} // namespace fcs::calibration