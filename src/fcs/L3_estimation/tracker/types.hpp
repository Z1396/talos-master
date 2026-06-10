// 头文件保护，防止重复包含引发编译重定义
#pragma once

// 第三方/标准库头文件
#include <Eigen/Core>         // Eigen 矩阵、向量基础运算
#include <Eigen/Geometry>     // Eigen 姿态、旋转相关计算
#include <array>              // 固定长度数组
#include <cstdint>            // 定长整型，用于状态枚举、时间戳
#include <limits>             // 数值极值、无穷大定义
#include <optional>           // 可选值类型，表示字段有无有效值
#include <variant>            // 多态变体，统一承载不同目标状态
#include <vector>             // 动态数组

// 项目内部头文件
#include "core/channel_topics.hpp"  // 通信话题定义
#include "core/types.hpp"           // 项目基础通用类型
#include <fmt/core.h>               // 格式化打印库
#include <magic_enum.hpp>           // 枚举字符串反射库
#include <numbers>                  // C++标准数学常数(圆周率等)

// L3 估计/跟踪层命名空间
namespace fcs::L3 {

// ============================================================================
// 跟踪器整体状态枚举
// ============================================================================
enum class TrackerStatus : uint8_t {
    Idle      = 0, // 空闲：未开始跟踪，等待首次检测结果
    Detecting = 1, // 检测中：短暂检测阶段，等待结果确认
    Tracking  = 2, // 稳定跟踪：正常持续跟踪目标
    TempLost  = 3, // 临时丢失：短暂跟丢，依靠模型预测维持状态
};

// ============================================================================
// 滤波器收敛状态枚举（主要针对EKF扩展卡尔曼滤波）
// ============================================================================
enum class FilterConvergenceStatus : uint8_t {
    Unknown    = 0, // 状态未知
    Converging = 1, // 正在收敛
    Converged  = 2, // 已收敛，滤波状态稳定
    Diverging  = 3, // 发散，滤波结果不可信
};

/**
 * @brief 滤波器收敛性状态结构体
 * 用于监测EKF滤波是否稳定，辅助判断跟踪质量
 */
struct FilterConvergenceState {
    FilterConvergenceStatus status{FilterConvergenceStatus::Unknown};
    // 归一化新息平方：反映观测与预测偏差，越小越稳定
    double normalized_innovation_squared{std::numeric_limits<double>::infinity()};
    // 协方差矩阵对角元最大值：表征状态不确定度
    double max_covariance_diag{std::numeric_limits<double>::infinity()};
    // 连续收敛更新次数
    int consecutive_converged_updates{0};
    // 连续发散更新次数
    int consecutive_diverged_updates{0};
};

// ============================================================================
// 普通机器人目标状态（11维EKF状态模型）
// 适用于常规四轮/步兵机器人，共4块装甲板
// ============================================================================
struct RobotTargetState {
    // 世界坐标系(odom)下中心位置
    Eigen::Vector3d position{0, 0, 0}; // [xc, yc, z0] 中心三维坐标
    Eigen::Vector3d velocity{0, 0, 0}; // [vx, vy, vz] 三维速度

    // 旋转姿态与角速度
    double yaw{0};     // 0号装甲板偏航角
    double v_yaw{0};   // 整体偏航角速度

    // 几何尺寸参数
    double radius0{0.22}; // 0、2号装甲板外接半径
    double radius1{0.22}; // 1、3号装甲板外接半径
    double z1{0};         // 1、3号装甲板高度 (z0 + 高度差)

    int armors_num{4};    // 装甲板总数：固定4块

    // EKF内部矩阵（仅用于可视化/调试，业务逻辑一般不使用）
    Eigen::MatrixXd P; // 后验状态协方差矩阵 11×11
    Eigen::MatrixXd K; // 卡尔曼增益矩阵 11×4
    Eigen::MatrixXd Q; // 过程噪声协方差矩阵 11×11
    Eigen::MatrixXd R; // 观测噪声协方差矩阵 4×4

    FilterConvergenceState convergence{}; // 滤波收敛状态

    /**
     * @brief 计算4块装甲板各自位姿
     * @return 数组：每个元素 [x, y, z, yaw] 坐标+偏航角
     */
    [[nodiscard]] std::array<Eigen::Vector4d, 4> armor_poses() const noexcept;
};

// ============================================================================
// 哨兵/基地类目标状态（7维EKF状态模型）
// 适用于哨兵、基地等3块装甲板的目标
// ============================================================================
struct OutpostTargetState {
    Eigen::Vector2d position{0, 0};    // 中心二维位置 [xc, yc]
    Eigen::Vector3d velocity{0, 0, 0}; // 三维速度 [vx, vy, vz]
    double yaw{0};                     // 整体偏航角
    double v_yaw{0};                   // 偏航角速度
    std::array<double, 3> z{0, 0, 0};  // 三块装甲板各自高度 z0,z1,z2

    static constexpr double radius  = 0.2765; // 统一外接半径
    static constexpr int armors_num = 3;      // 装甲板总数：固定3块

    // EKF内部矩阵（调试/可视化用）
    Eigen::MatrixXd P; // 后验协方差 7×7
    Eigen::MatrixXd K; // 卡尔曼增益 7×4
    Eigen::MatrixXd Q; // 过程噪声 7×7
    Eigen::MatrixXd R; // 观测噪声 4×4

    FilterConvergenceState convergence{}; // 滤波收敛状态

    /**
     * @brief 计算3块装甲板各自位姿
     * @return 数组：每个元素 [x, y, z, yaw]
     */
    [[nodiscard]] std::array<Eigen::Vector4d, 3> armor_poses() const noexcept;
};

// ============================================================================
// 统一跟踪器输出结构体
// 兼容机器人、哨兵两类目标，对外提供统一接口
// ============================================================================
struct TrackerOutput {
    uint64_t timestamp_ns{0};                     // 数据时间戳(纳秒)
    TrackerStatus status{TrackerStatus::Idle};    // 跟踪器状态
    ArmorName target_name{ArmorName::Invalid};    // 目标名称/标识
    ArmorColor target_color{ArmorColor::Neutral};// 目标颜色
    bool target_jumped{false};                    // 目标是否发生跳变(瞬移、切换目标)
    std::optional<int> last_armor_id{};            // 上一帧选中的装甲板ID
    // 上一帧目标中心在图像中的像素距离
    double last_image_center_distance_px{std::numeric_limits<double>::infinity()};
    uint64_t last_observation_timestamp_ns{0};     // 最后一次有效观测时间戳

    // 变体类型：空状态 / 机器人目标 / 哨兵目标，实现多目标统一封装
    std::variant<std::monostate, RobotTargetState, OutpostTargetState> state;

    /**
     * @brief 判断是否处于有效跟踪状态
     */
    [[nodiscard]] bool is_tracking() const noexcept {
        return status == TrackerStatus::Tracking || status == TrackerStatus::TempLost;
    }

    /**
     * @brief 判断当前目标是否为普通机器人
     */
    [[nodiscard]] bool is_robot() const noexcept {
        return std::holds_alternative<RobotTargetState>(state);
    }

    /**
     * @brief 判断当前目标是否为哨兵/基地
     */
    [[nodiscard]] bool is_outpost() const noexcept {
        return std::holds_alternative<OutpostTargetState>(state);
    }

    /**
     * @brief 安全获取机器人状态指针，无则返回nullptr
     */
    [[nodiscard]] const RobotTargetState* robot_state() const noexcept {
        return std::get_if<RobotTargetState>(&state);
    }

    /**
     * @brief 安全获取哨兵状态指针，无则返回nullptr
     */
    [[nodiscard]] const OutpostTargetState* outpost_state() const noexcept {
        return std::get_if<OutpostTargetState>(&state);
    }
};

// 批量跟踪输出：支持同时跟踪多个目标
using TrackerOutputs = std::vector<TrackerOutput>;

// ============================================================================
// 通信话题别名（方便外部使用）
// 话题定义在 core/channel_topics.hpp，此处重导出
// ============================================================================
using ::fcs::TrackerOutputChannelTopic;

// ============================================================================
// 成员函数实现
// ============================================================================

/**
 * @brief 机器人目标：计算4块装甲板位姿
 * 原理：以机体中心为原点，按圆周均匀分布，结合半径、高度、偏航角计算世界坐标
 */
inline std::array<Eigen::Vector4d, 4> RobotTargetState::armor_poses() const noexcept {
    std::array<Eigen::Vector4d, 4> poses;
    // 单块装甲板角度步长：2π / 4
    constexpr double angle_step = 2.0 * std::numbers::pi / 4.0;

    for (int i = 0; i < 4; ++i) {
        // 单块装甲板全局偏航角
        const double armor_yaw = yaw + static_cast<double>(i) * angle_step;
        double r, z;

        // 1、3号装甲板：使用radius1、z1高度
        if (i == 1 || i == 3) {
            r = radius1;
            z = z1;
        } else {
            // 0、2号装甲板：使用radius0、机体中心高度
            r = radius0;
            z = position.z();
        }

        // 极坐标转直角坐标，计算装甲板中心世界坐标
        const double x = position.x() - r * std::cos(armor_yaw);
        const double y = position.y() - r * std::sin(armor_yaw);

        poses[i] = {x, y, z, armor_yaw};
    }

    return poses;
}

/**
 * @brief 哨兵目标：计算3块装甲板位姿
 * 圆周均匀分布，每块装甲板独立高度
 */
inline std::array<Eigen::Vector4d, 3> OutpostTargetState::armor_poses() const noexcept {
    std::array<Eigen::Vector4d, 3> poses;
    // 角度步长：2π / 3
    constexpr double angle_step = 2.0 * std::numbers::pi / 3.0;

    for (int i = 0; i < 3; ++i) {
        const double armor_yaw = yaw + static_cast<double>(i) * angle_step;
        const double x         = position.x() - radius * std::cos(armor_yaw);
        const double y         = position.y() - radius * std::sin(armor_yaw);

        // 赋值坐标、高度、偏航角
        poses[i] = {x, y, z[i], armor_yaw};
    }

    return poses;
}

} // namespace fcs::L3

// ============================================================================
// fmt 格式化特化：支持直接打印枚举名称
// 配合 magic_enum 将枚举值转为字符串，日志/调试更直观
// ============================================================================
namespace fmt {

template <>
struct formatter<fcs::L3::TrackerStatus> : formatter<std::string_view> {
    auto format(const fcs::L3::TrackerStatus s, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(s), ctx);
    }
};

} // namespace fmt