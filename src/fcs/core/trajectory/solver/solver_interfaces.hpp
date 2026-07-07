#pragma once
// 弹道物理模型抽象基类 BallisticModel
#include "core/trajectory/model/ballistic_model.hpp"
// Eigen三维向量，目标世界坐标存储
#include <Eigen/Core>

// C++23 预期类型，承载求解结果/错误文本
#include <expected>
// 标准字符串，错误描述
#include <string>
// 只读字符串视图，无内存拷贝，返回求解器名称
#include <string_view>
// std::pair，存储轨迹采样点(x,z)
#include <utility>
// 动态数组，存储整条轨迹离散采样点序列
#include <vector>

namespace fcs::core::trajectory::solver {

/**
 * @brief 弹道瞄准求解完整输出结果结构体
 * 纯值对象，存储收敛后的有效瞄准解；std::expected有值代表求解收敛合法
 */
struct AimSolution {
    double yaw{0.0};            ///< 发射水平方位角（偏航角，单位弧度）
    double pitch{0.0};          ///< 发射俯仰角（高低角，单位弧度）
    double time_of_flight{0.0}; ///< 子弹飞行总时长（秒）
    int iterations{0};          ///< 迭代求解器迭代次数（解析解模型固定为0）
};

/**
 * @brief 弹道求解器无状态抽象接口基类
 * 设计特点：无内部状态，枪口初速度v0作为入参传入，不缓存，避免const_cast可变成员
 * 统一对外两套核心能力：1. 反向求解瞄准角；2. 正向生成轨迹采样点用于可视化
 * 多态实现：DirectSolver查表求解器、迭代数值求解器均继承此类
 */
class TrajectorySolver {
public:
    /**
     * @brief 虚析构函数，保证派生求解器析构完整执行，unique_ptr多态释放安全
     */
    virtual ~TrajectorySolver() = default;

    /**
     * @brief 反向求解：命中目标所需发射方位角+俯仰角
     * @param target_pos 世界坐标系下目标三维坐标（米）
     * @param v0 枪口初速度 m/s
     * @return std::expected<AimSolution, std::string>
     *         成功：收敛有效的瞄准角、飞行时间、迭代次数
     *         失败：可读错误字符串（无解、射程超限、初速度非法等）
     * @ noexcept 内部不抛异常，所有故障通过expected返回
     * @ [[nodiscard]] 强制处理返回值，不可忽略求解失败
     */
    [[nodiscard]] virtual std::expected<AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept = 0;

    /**
     * @brief 正向生成弹道离散采样点，用于Foxglove可视化绘制抛物线
     * @param pitch 发射俯仰角（弧度）
     * @param v0 枪口初速度 m/s
     * @param max_distance 最大水平采样距离（米）
     * @return std::vector<std::pair<double, double>>
     *         每一组pair：(水平距离x, 高度z)，按飞行距离升序排列
     */
    [[nodiscard]] virtual std::vector<std::pair<double, double>>
        generate_trajectory(double pitch, double v0, double max_distance) const noexcept = 0;

    /**
     * @brief 获取求解器可读名称，日志、调试区分求解器类型
     * @return 零拷贝字符串视图，无堆内存分配
     */
    [[nodiscard]] virtual std::string_view solver_name() const noexcept = 0;

    /**
     * @brief 获取底层绑定的弹道物理模型只读指针
     * 替代不安全dynamic_cast，提供编译期安全的模型访问，用于查表求解器构建表格
     * @return BallisticModel基类只读指针（IdealModel / LinearDragModel实现实例）
     */
    [[nodiscard]] virtual const model::BallisticModel* get_model() const noexcept = 0;
};

} // namespace fcs::core::trajectory::solver