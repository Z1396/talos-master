#pragma once
// 采集策略、手眼求解算法配置结构体定义
#include "calibration_config.hpp"
// 标定基础枚举、PoseSample、HandEyeResult、时间戳别名等类型
#include "calibration_types.hpp"
// 矩阵/欧拉角工具（内部Eigen封装）
#include "matrix.hpp"

// C++23 预期类型，承载成功返回值/错误字符串
#include <expected>
// OpenCV核心向量、矩阵基础类型 rvec/tvec/cv::Mat
#include <opencv2/core.hpp>
// 动态数组，存储手眼配对样本序列
#include <vector>

namespace fcs::calibration {

/**
 * @brief 眼在手上（Eye-in-Hand）手眼标定求解器抽象类
 * 数学模型 AX = XB：
 * A：云台末端(gimbal_link) → 世界odom坐标系（机器人TF/IMU输出）
 * B：标定板 → 相机光学坐标系（PnP求解后转换为ROS标准camera_link）
 * X：待求固定外参 camera_link → gimbal_link（最终标定输出）
 */
class HandEyeCalibrator {
public:
    /**
     * @brief 手眼标定求解器构造函数
     * @param capture 样本采集过滤、数量限制配置（最小/最大样本、姿态差异阈值）
     * @param handeye 手眼求解算法配置（Tsai/Park/Horaud等求解方法）
     * @ noexcept 构造无抛异常，仅拷贝配置、初始化空样本容器
     */
    explicit HandEyeCalibrator(const CaptureConfig& capture, const HandeyeConfig& handeye) noexcept;

    /**
     * @brief 新增一组手眼配对样本（机器人末端位姿 + 标定板相对相机位姿）
     * @param gimbal_pose odom → gimbal_pitch 机器人末端变换矩阵（A矩阵）
     * @param board_pose camera → calibration_board_frame 标定板变换矩阵（B矩阵）
     * @param ts 图像采集纳秒时间戳，用于时序对齐
     * @return std::expected<void, std::string> 空成功值 / 错误描述（样本已满）
     * @ [[nodiscard]] 强制处理返回结果，不可忽略样本添加失败
     */
    [[nodiscard]] std::expected<void, std::string> add_sample(
        const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose,
        const fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>& board_pose,
        timestamp_ns_t ts) noexcept;

    /**
     * @brief 校验当前机器人末端姿态与已有样本是否具备足够多样性
 * 过滤平移/角度高度重复的帧，避免标定矩阵退化、求解精度差
     * @param gimbal_pose 待校验的新云台末端位姿
     * @return true 姿态差异达标，允许采集；false 姿态重复，丢弃该帧
     */
    [[nodiscard]] bool is_diverse_enough(
        const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose)
        const noexcept;

    /**
     * @brief 使用全部已采集样本执行手眼标定求解
     * @return std::expected<HandEyeResult, std::string>
     *         成功：完整手眼外参结果（平移、欧拉角、RMS误差、样本数）
     *         失败：样本不足等错误文本
     */
    [[nodiscard]] std::expected<HandEyeResult, std::string> calibrate() noexcept;

    /**
     * @brief 获取当前已存储有效配对样本数量
     * @return uint32_t 当前样本计数
     */
    [[nodiscard]] uint32_t sample_count() const noexcept {
        return static_cast<uint32_t>(samples_.size());
    }

    /**
     * @brief 清空所有采集样本，重置求解器状态，可重新开始采集
     */
    void clear() noexcept;

    /**
     * @brief 静态工具函数：OpenCV PnP输出旋转/平移向量 → ROS标准相机坐标系变换矩阵
     * 坐标系差异说明：
     * OpenCV光学坐标系：X右、Y下、Z前
     * ROS camera_link坐标系：X前、Y左、Z上
     * 转换逻辑：叠加固定旋转变换 rpy[-π/2, 0, -π/2]，将optical转为camera_link
     * @param rvec OpenCV旋转向量
     * @param tvec OpenCV平移向量
     * @return fast_tf变换矩阵 camera → calibration_board_frame
     */
    [[nodiscard]] static fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>
        opencv_to_ros(const cv::Vec3d& rvec, const cv::Vec3d& tvec) noexcept;

private:
    CaptureConfig capture_config_; ///< 采集规则配置（样本阈值、姿态过滤阈值）
    HandeyeConfig handeye_config_; ///< 手眼求解算法配置
    std::vector<PoseSample> samples_; ///< 存储全部A/B配对位姿样本序列
};

/**
 * @brief 将手眼标定结果写入TOML磁盘文件持久化存储
 * @param result 标定求解输出结构体
 * @param path 输出文件磁盘路径
 * @param method 本次使用的手眼求解算法枚举
 * @return std::expected<void, std::string> 文件写入成功 / 打开失败错误
 */
[[nodiscard]] std::expected<void, std::string> save_handeye_result(
    const HandEyeResult& result, const std::string& path, HandEyeMethod method) noexcept;

/**
 * @brief 控制台打印人类可读格式的手眼标定结果（日志输出）
 * 单位自动转换：米→毫米、弧度→角度，方便调试查看
 * @param result 标定结果结构体
 */
void print_handeye_result(const HandEyeResult& result) noexcept;

} // namespace fcs::calibration