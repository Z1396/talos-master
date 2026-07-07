#pragma once
// Eigen线性代数库浮点向量、矩阵基础类型
#include <Eigen/Core>
// C++17 std::optional 可空返回值（承载成功结果/计算失败）
#include <optional>
// std::string_view 只读字符串视图，无内存拷贝
#include <string>

namespace fcs::core::trajectory::model {

/**
 * @brief 给定水平射程下子弹落点计算结果结构体
 * 存储弹道求解后的垂直位置、飞行时间、实际水平射程
 */
struct ImpactResult {
    double z{0.0};   ///< 落点垂直坐标（单位米，向上为正）
    double tof{0.0}; ///< 子弹飞行总时长（单位秒 Time of Flight）
    double x{0.0};   ///< 实际抵达的水平射程（单位米，受空气阻力影响会小于目标range）
};

/**
 * @brief 弹道求解抽象基类（策略模式顶层接口）
 * 隔离物理模型与求解算法，支持多实现替换：理想无阻力、线性空气阻力、二次空气阻力等模型
 * 搭配不同迭代求解器/查表求解器，上层业务无感知切换弹道物理模型
 */
class BallisticModel {
public:
    /**
     * @brief 虚析构函数，保证派生类析构函数完整执行，多态unique_ptr安全释放子类资源
     */
    virtual ~BallisticModel() = default;

    /**
     * @brief 计算指定水平射程对应的子弹落点弹道结果
     * @param range 目标水平射程（单位米）
     * @param pitch 发射俯仰角（相对水平面，单位弧度，向上为正）
     * @param v0 枪口初速度（单位 m/s）
     * @return std::optional<ImpactResult>
     *         有值：求解成功，返回落点三维信息与飞行时间
     *         std::nullopt：弹道计算失败（无解、初速度非法、射程超出模型求解范围等）
     * @ noexcept 函数内部不抛出C++异常，失败统一通过空optional返回
     * @ [[nodiscard]] 强制业务代码处理返回值，禁止忽略求解失败场景
     */
    [[nodiscard]] virtual std::optional<ImpactResult>
        compute_impact(double range, double pitch, double v0) const noexcept = 0;

    /**
     * @brief 获取当前弹道模型可读名称，用于日志打印、调试区分模型类型
     * @return std::string_view 只读字符串视图，无堆内存分配，高效轻量
     */
    [[nodiscard]] virtual std::string_view model_name() const noexcept = 0;
};

} // namespace fcs::core::trajectory::model