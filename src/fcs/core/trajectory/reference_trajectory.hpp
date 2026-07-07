#pragma once
// Eigen线性代数库，动态矩阵存储多步预测状态
#include <Eigen/Core>
// 动态数组，存储每一步射程、飞行时间
#include <vector>
// std::abs
#include <cmath>

namespace fcs::core::trajectory {

/**
 * @brief 武器控制参考轨迹结构体
 * 上层L4规划层生成，下层L5武器执行层消费
 * 存储多步时域预测窗口内的姿态、角速度、射程、飞行时间序列
 * 核心设计：以中心步为基准瞄准点，yaw做相对偏移归一化
 */
struct ReferenceTrajectory {
    /// 状态矩阵别名：4行 × 可变时域步数列
    using StateMatrix = Eigen::Matrix<double, 4, Eigen::Dynamic>;

    /**
     * @brief 4行时域状态矩阵，行数定义：
     * Row 0：相对yaw偏航角（基于yaw_origin基准做归一化）
     * Row 1：yaw角速度 rad/s
     * Row 2：俯仰角 pitch rad
     * Row 3：俯仰角速度 pitch_rate rad/s
     * 每一列对应一个时域预测步长
     */
    StateMatrix state;

    /// 每个时域步对应的目标水平距离，单位米，长度与state列数一致
    std::vector<double> distances;

    /// 每个时域步子弹飞行总时长，单位秒，长度与state列数一致
    std::vector<double> time_of_flights;

    /// yaw基准原点：轨迹中心步原始绝对偏航角，所有state(0,k)为相对该值的偏移量
    double yaw_origin{0.0};

    /**
     * @brief 获取预测时域总步数（矩阵列数）
     * @return 窗口horizon长度
     */
    [[nodiscard]] int horizon() const noexcept { return static_cast<int>(state.cols()); }

    /**
     * @brief 轨迹中心瞄准点结果子结构体
     * 存储最优基准瞄准点的yaw/pitch/射程
     */
    struct AimPoint {
        double yaw{0.0};      ///< 绝对偏航角
        double pitch{0.0};    ///< 俯仰角 rad
        double distance{0.0}; ///< 对应水平距离 m
    };

    /**
     * @brief 自动查找相对yaw偏移最小的时域步，返回基准瞄准点
     * 该点是L4规划原始计算的当前目标瞄准点
     * @return AimPoint 基准瞄准三要素
     */
    [[nodiscard]] AimPoint center_aim_point() const noexcept {
        const int n = horizon();
        // 无预测窗口，返回空默认值
        if (n <= 0)
            return {};
        // 初始假定第0列为中心点
        int center = 0;
        // 遍历所有时域步，寻找相对yaw绝对值最小的索引
        for (int k = 1; k < n; ++k)
            if (std::abs(state(0, k)) < std::abs(state(0, center)))
                center = k;
        // 组装瞄准点：yaw使用基准原点，pitch、距离取自最优步
        return AimPoint{
            .yaw      = yaw_origin,
            .pitch    = state(2, center),
            .distance = distances[center],
        };
    }
};

} // namespace fcs::core::trajectory