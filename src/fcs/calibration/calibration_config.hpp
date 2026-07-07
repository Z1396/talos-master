#pragma once
// 标定基础枚举、类型别名（BoardType、ArucoDictionary、CalibrationMode、HandEyeMethod等）
#include "calibration_types.hpp"
// 相机硬件/成像参数配置结构体
#include "camera_config.hpp"
// Foxglove可视化、Mcap录包相关配置
#include "foxglove_config.hpp"
// TOML反射序列化工具，from_table 结构体自动解析
#include "toml_helper.hpp"

// 固定宽度整数类型 uint32_t
#include <cstdint>
// C++23 预期类型，承载成功配置/错误字符串
#include <expected>
// 标准字符串，文件路径存储
#include <string>

namespace fcs::calibration {

/**
 * @brief 标定板几何配置结构体
 * 统一棋盘格、ChArUco公用基础尺寸参数
 */
struct BoardConfig {
    // 标定板类型：棋盘格/ChArUco/圆点网格，默认棋盘格
    BoardType type{BoardType::Chessboard};
    uint32_t width{11};       ///< 宽度方向内角点数量（棋盘格内部交点，不含外边框）
    uint32_t height{8};       ///< 高度方向内角点数量
    double square_size{0.02}; ///< 方格物理边长，单位米
};

/**
 * @brief ChArUco复合标定板专属扩展配置
 * ChArUco = 棋盘格 + ArUco微型标记，用于低纹理、远距离检测
 */
struct CharucoConfig {
    double marker_size{0.015}; ///< 内部ArUco标记正方形边长，单位米
    // ArUco标记字典，决定标记ID编码规则，默认6x6 250库
    ArucoDictionary dictionary{ArucoDictionary::DICT_6X6_250};
};

/**
 * @brief 样本自动采集策略配置
 * 控制标定时机器人移动、图像自动抓拍筛选逻辑
 */
struct CaptureConfig {
    uint32_t min_samples{30};          ///< 标定完成最低有效样本帧数，不足不执行求解
    uint32_t max_samples{100};         ///< 最大采集样本上限，到达停止采集
    double min_angle_diff{15.0};       ///< 前后两帧相机姿态最小角度差（度），过滤近似重复姿态
    double min_translation_diff{0.05}; ///< 前后两帧相机最小平移距离（米），过滤重复位置
    bool auto_capture{false};          ///< 开启自动抓拍，无需手动触发保存样本
    uint32_t capture_interval_ms{500}; ///< 自动抓拍最小间隔毫秒，防止短时间重复采集
};

/**
 * @brief 相机内参标定优化约束标志配置
 * 映射OpenCV calibrateCamera求解flag，控制哪些内参/畸变固定不参与优化
 */
struct IntrinsicConfig {
    bool fix_aspect_ratio{false};    ///< 固定fx/fy焦距比值为1，仅优化单焦距
    bool fix_principal_point{false}; ///< 固定主点（光心）在图像中心，不优化cx/cy
    bool zero_tangent_dist{false};   ///< 切向畸变系数置0，仅求解径向畸变k1/k2/k3

    /**
     * @brief 将布尔约束转换为OpenCV标定函数二进制flag整数
     * @return cv::calibrateCamera 所需flags掩码
     */
    [[nodiscard]] int to_opencv_flags() const noexcept {
        int flags = 0;
        // 固定长宽比
        if (fix_aspect_ratio)
            flags |= cv::CALIB_FIX_ASPECT_RATIO;
        // 固定主点
        if (fix_principal_point)
            flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
        // 切向畸变清零
        if (zero_tangent_dist)
            flags |= cv::CALIB_ZERO_TANGENT_DIST;
        return flags;
    }
};

/**
 * @brief 手眼标定求解算法配置
 * 手眼标定：求解相机与机器人末端之间固定变换矩阵
 */
struct HandeyeConfig {
    // 求解方法，默认Tsai-Lenz两步解析法，鲁棒、计算速度快
    HandEyeMethod method{HandEyeMethod::Tsai};
};

/**
 * @brief Foxglove可视化界面绘图开关配置
 * 实时预览标定检测、重投影误差、相机坐标系
 */
struct VisualizationConfig {
    bool show_corners{true};      ///< 图像叠加绘制检测到的角点
    bool show_reprojection{true}; ///< 绘制重投影误差线段（真实角点 vs 预测角点）
    bool show_pose{true};         ///< 绘制标定板三维坐标轴，直观展示相机位姿
};

/**
 * @brief 标定结果输出文件路径配置
 */
struct OutputConfig {
    std::string intrinsic_path{"camera_intrinsic.toml"}; ///< 相机内参输出路径
    std::string extrinsic_path{"camera_extrinsic.toml"}; ///< 手眼外参/相机位姿输出路径
};

/**
 * @brief 图像输入源配置：真实硬件相机 / 仿真模拟器
 */
struct InputConfig {
    bool daedalus{false}; ///< true 使用Daedalus仿真器虚拟图像；false 真实硬件相机
};

/**
 * @brief 顶层完整标定总配置结构体
 * 聚合所有子模块配置，TOML [calibration] 根表完整映射
 */
struct CalibrationConfig {
    // 标定模式：仅内参 / 手眼联合标定
    CalibrationMode mode{CalibrationMode::Intrinsic};
    // 相机硬件成像参数（分辨率、像素尺寸、曝光等）
    CameraProfileConfig profile{};
    uint32_t width{};  ///< 图像宽度像素
    uint32_t height{}; ///< 图像高度像素
    // 标定板基础尺寸
    BoardConfig board{};
    // ChArUco扩展参数
    CharucoConfig charuco{};
    // 样本自动采集过滤规则
    CaptureConfig capture{};
    // 内参优化约束标志
    IntrinsicConfig intrinsic{};
    // 手眼标定求解算法
    HandeyeConfig handeye{};
    // 实时可视化绘图开关
    VisualizationConfig visualization{};
    // 结果文件输出路径
    OutputConfig output{};
    // Foxglove可视化/Mcap录包配置
    FoxgloveConfig foxglove{};
    // 图像输入源（硬件/仿真）
    InputConfig input{}; ///< Input source (hardware/daedalus)

    /**
     * @brief 静态工厂方法：从TOML文件加载并反射解析完整标定配置
     * @param path TOML配置文件磁盘路径
     * @return expected<CalibrationConfig, 错误字符串>
     *         成功：填充完整配置结构体；失败：携带文件解析/校验错误信息
     * @ noexcept 无抛出异常，全部错误通过expected返回
     */
    [[nodiscard]] static std::expected<CalibrationConfig, std::string>
        load_from_file(const std::string& path) noexcept {
        // 解析磁盘TOML文件
        auto result = toml::parse_file(path);
        // 文件语法解析失败，返回错误描述
        if (!result) {
            return std::unexpected(
                fmt::format("Failed to parse {}: {}", path, result.error().description()));
        }

        // 获取TOML根表
        const auto& tbl = result.table();
        // 查找顶层 [calibration] 子表
        if (auto calib_tbl = tbl["calibration"].as_table()) {
            // 反射自动解析子表到CalibrationConfig结构体
            auto config = toml_helper::from_table<CalibrationConfig>(*calib_tbl);
            // 结构体字段解析失败（必填缺失、类型不匹配、数组长度错误）
            if (!config) {
                return std::unexpected(config.error());
            }
            // 业务逻辑二次校验：Mcap录包模式必须配置mcap存储路径
            if (config->foxglove.transport == FoxgloveTransport::Mcap
                && config->foxglove.mcap_path.empty()) {
                return std::unexpected(
                    "calibration.foxglove.mcap_path is required when "
                    "calibration.foxglove.transport=\"Mcap\"");
            }
            // 校验全部通过，移动返回配置
            return std::move(*config);
        }

        // TOML文件缺少顶层 [calibration] 子表，配置非法
        return std::unexpected("Missing [calibration] section in config file");
    }
};

} // namespace fcs::calibration