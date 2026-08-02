#pragma once
// 相机采集配置、内参标定算法参数配置结构体定义
#include "calibration_config.hpp"
// 标定业务数据类型：角点检测结果、内参输出结果等
#include "calibration_types.hpp"

// C++23 标准预期类型，用于安全返回「成功/错误信息」，替代异常抛出
#include <expected>
// OpenCV 基础矩阵、点、尺寸等核心数据结构
#include <opencv2/core.hpp>
// 存储多组标定样本
#include <vector>

namespace fcs::calibration {

/**
 * @brief 相机单目内参标定器
 * 功能：采集棋盘格角点样本、筛选有效多样样本、求解相机内参矩阵+畸变系数、重投影误差计算、结果导出
 * 完整流程：add_sample 采集角点样本 → is_diverse_enough 筛选有效样本 → calibrate 执行标定求解 →
 * save_intrinsic_result 存文件
 */
class IntrinsicCalibrator {
public:
    /**
     * @brief 构造相机内参标定器
     * @param capture 图像采集配置：棋盘格尺寸、方格边长、图像分辨率等采集相关参数
     * @param intrinsic 内参求解配置：标定迭代终止条件、畸变系数求解开关、标定优化标志位
     * @noexcept 构造不会抛出异常
     */
    explicit IntrinsicCalibrator(
        const CaptureConfig& capture, const IntrinsicConfig& intrinsic) noexcept;

    /**
     * @brief 向标定器添加一组棋盘格角点检测样本
     * @param detection 单帧图像棋盘格角点检测结果（图像2D角点、棋盘3D世界坐标）
     * @return std::expected<void, std::string>
     *         成功：返回空void；失败：返回携带错误字符串（角点数量不匹配、数据非法等）
     * @noexcept 内部不会抛出异常，错误通过返回值传递
     * @note 内部会同步保存该帧估算的rvec/tvec旋转平移向量，用于后续样本多样性校验
     */
    [[nodiscard]] std::expected<void, std::string>
        add_sample(const CornerDetection& detection) noexcept;

    /**
     * @brief 校验当前待加入样本和已有样本姿态差异是否足够
     *        避免大量相似角度图片参与标定，导致内参求解退化、精度差
     * @param detection 待新增的一帧角点检测结果
     * @return true 样本姿态足够多样，可以加入；false 姿态重复，应当丢弃该帧
     * @const 不修改类内任何成员
     * @noexcept 无异常抛出
     */
    [[nodiscard]] bool is_diverse_enough(const CornerDetection& detection) const noexcept;

    /**
     * @brief 使用所有已收集的有效样本执行OpenCV单目内参标定算法
     * @param image_size 图像原始分辨率宽高 cv::Size(width, height)
     * @return std::expected<IntrinsicResult, std::string>
     *         成功：返回完整内参结果（内参矩阵K、畸变系数D、平均重投影误差、每帧单独误差）
     *         失败：返回错误信息（样本数量不足、标定迭代不收敛等）
     * @noexcept 无异常抛出
     */
    [[nodiscard]] std::expected<IntrinsicResult, std::string>
        calibrate(cv::Size image_size) noexcept;

    /**
     * @brief 根据标定得到的内参，把棋盘格3D世界点重新投影回2D图像
     *        用于计算单帧重投影误差，评估标定精度
     * @param detection 单帧原始棋盘格角点检测数据（含3D世界坐标）
     * @param result 已求解完成的相机内参标定结果
     * @return std::vector<cv::Point2f> 投影后的2D图像像素坐标
     * @static 静态方法，不依赖实例状态
     * @noexcept 无异常抛出
     */
    [[nodiscard]] static std::vector<cv::Point2f>
        reproject(const CornerDetection& detection, const IntrinsicResult& result) noexcept;

    /**
     * @brief 获取当前已存入标定器的有效样本帧数
     * @return uint32_t 样本数量
     * @const 只读访问
     * @noexcept
     */
    [[nodiscard]] uint32_t sample_count() const noexcept {
        return static_cast<uint32_t>(samples_.size());
    }

    /**
     * @brief 清空全部已采集的标定样本、对应rvec/tvec姿态缓存
     *        用于重新开始新一轮标定采集
     * @noexcept
     */
    void clear() noexcept;

private:
    // 图像采集相关静态配置：棋盘规格、方格尺寸
    CaptureConfig capture_config_;
    // 内参求解算法配置：迭代次数、畸变求解标志、收敛阈值
    IntrinsicConfig intrinsic_config_;
    // 全部有效标定样本集合，每一帧对应一组棋盘格2D/3D角点
    std::vector<CornerDetection> samples_;
    // 缓存每帧样本求解出的旋转向量 rvec，用于样本姿态多样性判断
    std::vector<cv::Vec3d> sample_rvecs_;
    // 缓存每帧样本求解出的平移向量 tvec，用于样本姿态多样性判断
    std::vector<cv::Vec3d> sample_tvecs_;
};

/**
 * @brief 将相机内参标定结果持久化保存到TOML配置文件
 * @param result 完整相机内参输出结构体（K矩阵、畸变系数、重投影误差、标定参数）
 * @param path 输出文件完整路径
 * @return std::expected<void, std::string>
 *         成功：无返回值；失败：返回文件打开/写入失败等错误文本
 * @noexcept 不抛异常
 */
[[nodiscard]] std::expected<void, std::string>
    save_intrinsic_result(const IntrinsicResult& result, const std::string& path) noexcept;

} // namespace fcs::calibration