// ===========================================================================
// model_types.hpp - 状态/观测索引枚举 + 角度工具函数
// 照搬来源: src/fcs/L3_estimation/tracker/util.hpp（逐字复制，未改动任何定义）
//
// 说明：真实 util.hpp 还包含状态标签(magic_enum)、可见性判断、xyz2ypd 等工具，
//       本文件只保留运动模型(motion_model.hpp)编译所需的最小集合，
//       枚举成员、顺序、注释与真实项目完全一致。
// ===========================================================================

#pragma once   //头文件保护，防止重复include

#include <cmath>     // std::remainder 浮点数取余，用于角度归一化
#include <cstdint>   // uint8_t 无符号1字节整数，枚举底层存储，节省内存
#include <numbers>   // C++20 标准库，std::numbers::pi π常量

namespace fcs::L3 {

/**
 * @brief 机器人状态空间索引（11维）
 *
 * 状态向量：[xc, vx, yc, vy, z0, vz, yaw, v_yaw, log_r0, log_r1, h]
 * 对应卡尔曼滤波的状态X，EKF的状态向量，一共11个维度。
 *
 * 状态建模：
 * - **匀速运动**：xc/vx、yc/vy、z0/vz 分别为三维位置和速度
 *      xc,yc：机器人底盘中心平面坐标；vx vy 底盘中心XY方向匀速速度
 *      z0：0、2号装甲板高度；vz：Z轴竖直方向速度
 * - **旋转状态**：yaw是装甲板0的朝向角，v_yaw是角速度
 *      机器人整体旋转，yaw=装甲板0法向的偏航角；v_yaw 旋转角速度 rad/s
 * - **半径参数化**：log_r0和log_r1是对数化半径，保证半径恒为正（r = exp(log_r)）
 *      👉 重点技巧：滤波状态不能直接存半径r；一旦滤波迭代r变成负数，物理无意义。
 *      存log(r)，不管log(r)输出是多大负数，exp之后r永远>0，约束物理合理性。
 *
 * 几何关系：
 * 四块装甲板位置全部由【底盘中心 + yaw + r0/r1 + h】推导计算，不直接估计装甲板坐标！
 * - 装甲板0：位于 (xc - r0*cos(yaw), yc - r0*sin(yaw), z0)
 * - 装甲板1：位于 (xc - r1*cos(yaw+π/2), yc - r1*sin(yaw+π/2), z0+h)
 * - 装甲板2：位于 (xc - r0*cos(yaw+π), yc - r0*sin(yaw+π), z0)
 * - 装甲板3：位于 (xc - r1*cos(yaw+3π/2), yc - r1*sin(yaw+3π/2), z0+h)
 *
 * @note 11维状态向量适用于4装甲板机器人目标跟踪
 */
enum RoboState : uint8_t {
    XC,         ///< 中心X坐标（米），底盘中心X
    VX,         ///< X方向速度（米/秒）
    YC,         ///< 中心Y坐标（米），底盘中心Y
    VY,         ///< Y方向速度（米/秒）
    Z0,         ///< 装甲板0和2的Z坐标（米）
    VZ,         ///< Z方向速度（米/秒）
    YAW,        ///< 装甲板0的yaw角度（弧度）机器人整体偏航
    V_YAW,      ///< 角速度（弧度/秒）机器人旋转角速度
    LOG_R0,     ///< 装甲板0和2的对数半径（log(米)），r0 = exp(state[LOG_R0])
    LOG_R1,     ///< 装甲板1和3的对数半径（log(米)），r1 = exp(state[LOG_R1])
    H,          ///< 高度差：z1 - z0（米），表示装甲板1和3的相对高度
    STATE_MAX,  ///< 状态维度（枚举计数用），值等于11，可以用来开辟数组/矩阵大小
};

/**
 * @brief 前哨站状态空间索引（7维）
 *
 * 状态向量：[xc, yc, yaw, v_yaw, z0, z1, z2]
 * 前哨站特殊：不会平移运动，只有原地旋转，三块装甲板高度互相独立
 *
 * 状态建模：
 * - **固定半径**：前哨站为固定结构，半径已知（不估计），写死在参数，不进滤波状态
 * - **固定速度**：前哨站静止，无速度状态，没有vx vy vz
 * - **旋转状态**：只有yaw和v_yaw，表示旋转
 * - **独立高度**：三个装甲板高度独立估计
 *
 * @note 7维状态向量适用于3装甲板前哨站目标跟踪
 */
enum OutpostState : uint8_t {
    O_XC,        ///< 中心X坐标（米）前哨站中心X
    O_YC,        ///< 中心Y坐标（米）前哨站中心Y
    O_YAW,       ///< 装甲板0的yaw角度（弧度）旋转朝向
    O_VYAW,      ///< 角速度（弧度/秒）原地旋转角速度
    O_Z0,        ///< 装甲板0的Z坐标（米）
    O_Z1,        ///< 装甲板1的Z坐标（米）
    O_Z2,        ///< 装甲板2的Z坐标（米）
    O_STATE_MAX, ///< 状态维度（枚举计数用），等于7
};

/**
 * @brief 测量空间索引（4维）
 *
 * 测量向量Z：[yaw, pitch, log(distance), armor_yaw]
 * 视觉模块检测到**单块装甲板**之后输出的观测，送入EKF更新步骤。
 *
 * 测量建模：
 * - **球坐标观测**：使用yaw/pitch/distance，避免距离对角度的数值耦合
 *      不用直角坐标xyz做观测！视觉输出本身就是球坐标，减小EKF非线性。
 * - **对数距离**：log(distance)，提高近距离精度，避免距离主导
 *      距离数值范围波动巨大；取log压缩数值区间，防止观测方差被距离支配。
 * - **装甲板朝向**：armor_yaw是装甲板法向量的yaw角，用于可见性判断
 *      判断这块装甲板朝向是否朝向我方；背向的装甲板观测直接丢弃。
 *
 * @note 4维测量向量适用于单装甲板观测
 */
enum Measure : uint8_t {
    ARMOR_YAW,       ///< 装甲板方位角（yaw）：atan2(y, x)（弧度），装甲板在相机坐标系下的方位
    ARMOR_PITCH,     ///< 装甲板俯仰角（pitch）：atan2(-z, sqrt(x²+y²))（弧度）
    ARMOR_DISTANCE,  ///< 装甲板距离（对数）：log(sqrt(x²+y²+z²))（log(米)）
    ARMOR_YAW_ARMOR, ///< 装甲板朝向角：装甲板法向量的yaw（弧度）装甲板自身法线角度
    MEASURE_MAX,     ///< 测量维度（枚举计数用），等于4
};

/**
 * @brief 归一化角度到[-π, π)
 *
 * 算法：使用std::remainder实现高精度归一化
 * std::remainder(a, 2π)：浮点数余数，输出范围 [-π , +π]
 * 注意remainder有可能输出等于 -π，而滤波约定角度区间 [-π,π)，所以特殊处理把-π改成+π
 *
 * @param a 输入角度（弧度）可以是无限大，比如100π、‑200π
 * @return 归一化角度（弧度），范围[-π, π)
 *
 * @note 比手动 while(a>2π) a‑=2π 减法更精确，避免浮点数累积误差
 */
[[nodiscard]] inline double normalize_rad(double a) noexcept {
    double r = std::remainder(a, 2.0 * std::numbers::pi);

    // 处理边界情况：remainder可能返回-π，需转换为π
    // 滤波统一约定：‑π 等价 +π，我们统一映射成 +π，保证区间左闭右开 [-π, π)
    if (r <= -std::numbers::pi)
        r += 2.0 * std::numbers::pi;

    return r;
}

/**
 * @brief 计算最短角度差：to - from，结果在(-π, π]
 *
 * 应用场景：
 * - 计算角度误差（如新息innovation，EKF更新的时候角度残差！！最核心）
 * - 避免角度跳变（如从179°到-179°差值应为2°而非-358°）
 *      例子 from=179°(≈π‑0.02), to=-179°(≈‑π+0.02)
 *      to‑from直接算数等于‑358°；最短角度差是 +2°
 *
 * @param from 起始角度（弧度）旧角度
 * @param to 目标角度（弧度）新角度
 * @return 最短角度差（弧度）
 */
[[nodiscard]] inline double shortest_rad(double from, double to) noexcept {
    return normalize_rad(to - from);
}

/**
 * @brief 归一化角度（单参数版本）重载，兼容老代码调用
 * @param angle 输入角度（弧度）
 * @return 归一化角度（弧度）
 */
[[nodiscard]] inline double shortest_rad(double angle) noexcept { return normalize_rad(angle); }

} // namespace fcs::L3
