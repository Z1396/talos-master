#pragma once
// Eigen线性代数库，矩阵、向量存储相机内参、位姿
#include <Eigen/Core>
// 固定宽度无符号整数类型 uint8_t
#include <cstdint>
// OpenCV标定核心函数、手眼标定枚举常量
#include <opencv2/calib3d.hpp>
// OpenCV基础矩阵、点类型
#include <opencv2/core.hpp>
// Eigen与OpenCV矩阵互相转换工具
#include <opencv2/core/eigen.hpp>
// 取消全局HAVE_OPENCV_ARUCO宏，避免外部冲突
#undef HAVE_OPENCV_ARUCO
// 仅开启Aruco时引入标记字典相关头文件
#ifdef HAVE_OPENCV_ARUCO
# include <opencv2/aruco.hpp>
#endif
// 标准动态数组，存储角点、样本序列
#include <vector>

// 项目全局基础类型别名（timestamp_ns_t等）
#include "core/types.hpp"

namespace fcs::calibration {

/**
 * @brief 标定板帧坐标标识空结构体，用作TF变换模板标签
 * 用于fast_tf位姿变换：camera → calibration_board_frame
 */
struct calibration_board_frame {};

// ============================================================================
// 消息通道Tag标记（无实体，仅用于区分多生产者单消费者通道）
// ============================================================================

/// @brief 通道标识：角点检测结果输出通道
struct CalibrationCornerChannelTopic {};

/// @brief 通道标识：标定状态机更新消息通道
struct CalibrationStatusChannelTopic {};

// ============================================================================
// 枚举类定义（magic_enum反射自动解析TOML字符串与枚举互转）
// ============================================================================

/**
 * @brief 标定板类型枚举
 * magic_enum自动匹配TOML字符串字面量，大小写严格对应
 */
enum class BoardType : uint8_t {
    Chessboard,  ///< 标准黑白棋盘格，TOML配置字符串 "Chessboard"
    ChArUco,     ///< 融合ArUco标记的复合标定板，TOML "ChArUco"
    CirclesGrid, ///< 圆形网格标定板，TOML "CirclesGrid"
};

/**
 * @brief 标定运行模式，区分单步/全流程标定
 */
enum class CalibrationMode : uint8_t {
    Intrinsic, ///< 仅相机内参标定，求解fx/fy/cx/cy/畸变系数
    Handeye,   ///< 仅手眼外参标定，必须提前完成内参求解
    Full,      ///< 完整流程：先采集内参样本，再采集手眼样本联合求解
};

/**
 * @brief 手眼标定求解算法枚举，一一映射OpenCV底层常量
 */
enum class HandEyeMethod : uint8_t {
    Tsai,       ///< Tsai-Lenz两步解析法，对应 cv::CALIB_HAND_EYE_TSAI
    Park,       ///< Park迭代法 cv::CALIB_HAND_EYE_PARK
    Horaud,     ///< Horaud非线性优化 cv::CALIB_HAND_EYE_HORAUD
    Andreff,    ///< Andreff线性求解 cv::CALIB_HAND_EYE_ANDREFF
    Daniilidis, ///< Daniilidis双四元数法 cv::CALIB_HAND_EYE_DANIILIDIS
};

/**
 * @brief ChArUco板使用的ArUco标记字典规格
 * 数字代表标记尺寸、标记库容量，用于生成/识别标记
 */
enum class ArucoDictionary : uint8_t {
    DICT_4X4_50,
    DICT_4X4_100,
    DICT_4X4_250,
    DICT_5X5_50,
    DICT_5X5_100,
    DICT_5X5_250,
    DICT_6X6_50,
    DICT_6X6_100,
    DICT_6X6_250,
};

/**
 * @brief 标定业务状态机，控制采集、求解流程分支
 */
enum class CalibrationState : uint8_t {
    Idle,        ///< 空闲待机，等待启动指令
    Capturing,   ///< 采集中：持续收集有效图像样本
    Calibrating, ///< 求解中：执行内参/手眼标定算法
    Completed,   ///< 标定成功完成，可读取结果
    Failed,      ///< 标定流程失败，携带错误信息
};

// ============================================================================
// 枚举 → OpenCV底层常量转换工具函数（constexpr编译期求值）
// ============================================================================

/// @brief 将自定义手眼算法枚举转为OpenCV标定函数所需掩码常量
[[nodiscard]] constexpr cv::HandEyeCalibrationMethod to_opencv_method(HandEyeMethod m) noexcept {
    switch (m) {
    case HandEyeMethod::Tsai: return cv::CALIB_HAND_EYE_TSAI;
    case HandEyeMethod::Park: return cv::CALIB_HAND_EYE_PARK;
    case HandEyeMethod::Horaud: return cv::CALIB_HAND_EYE_HORAUD;
    case HandEyeMethod::Andreff: return cv::CALIB_HAND_EYE_ANDREFF;
    case HandEyeMethod::Daniilidis: return cv::CALIB_HAND_EYE_DANIILIDIS;
    }
    // 开启-Wswitch-enum全分支校验后不会走到此处，兜底返回默认Tsai
    return cv::CALIB_HAND_EYE_TSAI;
}

#ifdef HAVE_OPENCV_ARUCO
/// @brief 将自定义ArUco字典枚举转为OpenCV预定义字典常量
[[nodiscard]] constexpr cv::aruco::PredefinedDictionaryType
    to_opencv_dict(ArucoDictionary d) noexcept {
    switch (d) {
    case ArucoDictionary::DICT_4X4_50: return cv::aruco::DICT_4X4_50;
    case ArucoDictionary::DICT_4X4_100: return cv::aruco::DICT_4X4_100;
    case ArucoDictionary::DICT_4X4_250: return cv::aruco::DICT_4X4_250;
    case ArucoDictionary::DICT_5X5_50: return cv::aruco::DICT_5X5_50;
    case ArucoDictionary::DICT_5X5_100: return cv::aruco::DICT_5X5_100;
    case ArucoDictionary::DICT_5X5_250: return cv::aruco::DICT_5X5_250;
    case ArucoDictionary::DICT_6X6_50: return cv::aruco::DICT_6X6_50;
    case ArucoDictionary::DICT_6X6_100: return cv::aruco::DICT_6X6_100;
    case ArucoDictionary::DICT_6X6_250: return cv::aruco::DICT_6X6_250;
    }
    return cv::aruco::DICT_6X6_250;
}
#endif

// ============================================================================
// 核心数据存储结构体
// ============================================================================

/**
 * @brief 单帧图像标定板角点检测结果
 * 角点采集系统核心输出数据载体
 */
struct CornerDetection {
    std::vector<cv::Point2f> image_points;  ///< 图像像素坐标系下检测到的二维角点
    std::vector<cv::Point3f> object_points; ///< 标定板局部坐标系对应三维标准点位
    cv::Mat image;                          ///< 原始输入图像（用于可视化绘制角点）
    timestamp_ns_t timestamp_ns{0};         ///< 图像采集纳秒时间戳，用于TF时间匹配
    bool success{false};                    ///< 检测成功标志，false代表无有效角点
};

/**
 * @brief 相机内参标定求解输出结果
 * 存储相机矩阵、畸变系数、重投影误差等标定核心输出
 */
struct IntrinsicResult {
    // 3×3相机内参矩阵，行主序存储，默认单位矩阵初始化
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix{
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity()};
    // 1×5畸变系数 [k1, k2, p1, p2, k3]，默认全0无畸变
    Eigen::Matrix<double, 1, 5> distort_coefficient{Eigen::Matrix<double, 1, 5>::Zero()};
    double rms_error{0.0};   ///< 标定整体重投影均方根误差，单位像素，越小精度越高
    uint32_t num_samples{0}; ///< 参与求解的有效样本帧数
    uint32_t width{0};       ///< 图像宽度像素
    uint32_t height{0};      ///< 图像高度像素
};

/**
 * @brief 手眼标定输出结果：云台末端 → 相机固定变换
 */
struct HandEyeResult {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()}; ///< 平移向量 [x,y,z] 单位米
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};         ///< 旋转欧拉角 [roll,pitch,yaw] 单位弧度
    double rms_error{0.0};                                ///< 手眼求解均方根误差，单位米
    uint32_t num_samples{0};                              ///< 参与求解的位姿配对样本数量
};

/**
 * @brief 标定运行状态结构体，上位机/可视化实时监控用
 */
struct CalibrationStatus {
    CalibrationState state{CalibrationState::Idle}; ///< 当前状态机
    uint32_t sample_count{0};                      ///< 当前已采集有效样本数
    uint32_t target_samples{30};                   ///< 完成标定所需最低样本阈值
    double current_error{0.0};                     ///< 当前实时重投影误差
    std::string message;                           ///< 状态提示/错误文本
};

/**
 * @brief 单帧手眼配对样本：机器人末端位姿 + 标定板相对相机位姿
 * 手眼标定求解输入最小单元
 */
struct PoseSample {
    // 世界odom坐标系 → 云台gimbal_pitch末端变换矩阵
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch> gimbal_pose;
    // 相机坐标系 → 标定板局部坐标系变换矩阵（PnP求解得到）
    fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame> board_pose;
    timestamp_ns_t timestamp_ns; ///< 帧时间戳，用于时序对齐
};

} // namespace fcs::calibration