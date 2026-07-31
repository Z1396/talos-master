/**
 * @file acceleration_motion_model.hpp
 * @brief 级联加速度扩展卡尔曼滤波器（EKF）运动模型
 *
 * 本文件实现了级联加速度估计器的核心运动模型，用于从主EKF的速度估计中
 * 平滑地提取线加速度和角加速度状态。
 *
 * 核心算法原理：
 * - **恒定加速度模型（CA Model）**：假设加速度在短时间内缓慢变化（随机游走）
 * - **状态方程**：v_{k+1} = v_k + a_k * dt, a_{k+1} = a_k（加速度为随机游走）
 * - **过程噪声**：使用jerk（加速度的导数）作为驱动噪声，更符合物理直觉
 * - **协方差传播**：采用离散时间白噪声加速度模型的解析形式，精度高且计算高效
 *
 * 数学推导：
 * 对于状态向量 [v, a]，在白噪声加速度驱动下，过程噪声协方差Q的解析解为：
 * Q = [[σ²*dt⁴/4, σ²*dt³/2],
 *      [σ²*dt³/2, σ²*dt²]]
 * 其中σ²是jerk噪声方差，而非加速度噪声方差。
 *
 * 应用场景：
 * - 平滑主EKF输出的速度估计
 * - 为弹道预测提供加速度信息
 * - 为MPC控制器提供状态导数信息
 *
 * 设计决策：
 * - 使用8维状态向量（而非独立的CA模型），保证状态相关性
 * - 分离XY/Z/YAW的噪声参数，适应机器人运动特性（XY运动剧烈，Z方向稳定）
 * - 提供R_diag()接口，支持对角测量噪声假设，提升计算效率
 */

#pragma once

#include <Eigen/Core>
#include <cmath>

namespace fcs::L3 {

/**
 * @brief 级联加速度EKF状态空间索引（8维）
 *
 * 状态向量：[vx, vy, vz, ax, ay, az, v_yaw, alpha_yaw]
 * - 前3维：线速度（从主EKF测量得到）
 * - 中3维：线加速度（本模型估计）
 * - 后2维：角速度和角加速度
 */
enum AccelState : uint8_t {
    VX_A,            ///< 线速度X分量（m/s）
    VY_A,            ///< 线速度Y分量（m/s）
    VZ_A,            ///< 线速度Z分量（m/s）
    AX,              ///< 线加速度X分量（m/s²）
    AY,              ///< 线加速度Y分量（m/s²）
    AZ,              ///< 线加速度Z分量（m/s²）
    VYAW_A,          ///< 角速度（rad/s）
    ALPHA_YAW,       ///< 角加速度（rad/s²）
    ACCEL_STATE_MAX, ///< 状态维度（枚举计数用）
};

/**
 * @brief 级联加速度EKF测量空间索引（4维）
 *
 * 测量向量：[vx, vy, vz, v_yaw]
 * 直接观测速度状态，加速度通过状态估计间接获得。
 */
enum AccelMeasure : uint8_t {
    AM_VX,            ///< 线速度X测量（m/s）
    AM_VY,            ///< 线速度Y测量（m/s）
    AM_VZ,            ///< 线速度Z测量（m/s）
    AM_VYAW,          ///< 角速度测量（rad/s）
    ACCEL_MEASURE_MAX ///< 测量维度（枚举计数用）
};

/**
 * @brief 级联加速度EKF运动模型
 *
 * 核心功能：
 * - 将主机器人EKF输出的速度估计作为测量输入
 * - 估计平滑的线加速度和角加速度状态
 * - 为下游模块提供状态导数信息
 *
 * 数学模型：
 * - **状态转移**：恒定加速度模型（CA Model）
 *   - v_{k+1} = v_k + a_k * dt
 *   - a_{k+1} = a_k（加速度为随机游走）
 * - **测量模型**：直接观测速度状态
 *   - z = [vx, vy, vz, v_yaw]
 * - **噪声模型**：
 *   - 过程噪声：jerk（加速度变化率）驱动白噪声
 *   - 测量噪声：主EKF速度估计的不确定性
 *
 * 关键特性：
 * - **级联结构**：依赖主EKF的速度估计，而非原始观测
 * - **分离参数**：XY/Z/YAW使用不同的噪声参数，适应机器人运动特性
 * - **计算高效**：过程噪声协方差Q使用解析解，避免数值积分
 *
 * @note 该模型是主EKF的辅助模型，不直接处理图像观测
 */
struct AccelEkfMotionModel {
    using Scalar                    = double;            ///< 标量类型
    static constexpr int NX         = ACCEL_STATE_MAX;   ///< 状态维度（8）
    static constexpr int NZ         = ACCEL_MEASURE_MAX; ///< 测量维度（4）
    static constexpr int ARMORS_NUM = 1; ///< 装甲板数量（本模型不使用，仅为兼容EkfTargetInfo接口）

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;  ///< 状态向量类型（8维）
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;  ///< 测量向量类型（4维）
    using MatXX = Eigen::Matrix<Scalar, NX, NX>; ///< 状态协方差矩阵类型（8x8）
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>; ///< 测量协方差矩阵类型（4x4）

    static constexpr int kVyawIndex    = VYAW_A; ///< 角速度状态索引
    static constexpr bool kHasLogRadii = false;  ///< 本模型不使用对数半径参数

    /**
     * @brief EKF噪声参数配置
     *
     * 参数调优指南：
     * - **sigma_jerk_xy**：控制XY平面加速度平滑度
     *   - 值越大 → 加速度响应越快，噪声抑制弱
     *   - 值越小 → 加速度平滑，但响应滞后
     * - **sigma_jerk_z**：控制Z方向加速度平滑度
     *   - 通常设为小值（机器人Z方向运动稳定）
     * - **sigma_jerk_yaw**：控制角加速度平滑度
     *   - 陀螺旋转弯时需较大值以跟踪快速转向
     *
     * - **meas_v_xy_var**：主EKF速度估计的信任度
     *   - 值越小 → 更信任主EKF的速度估计
     *   - 值越大 → 更依赖本模型的平滑效果
     */
    struct Params {
        // ===== 过程噪声参数：控制加速度平滑度 =====
        Scalar sigma_jerk_xy  = 14.5; ///< XY平面jerk噪声（m/s³），控制加速度变化率
        Scalar sigma_jerk_z   = 0.5;  ///< Z方向jerk噪声（m/s³），通常较小（机器人Z方向稳定）
        Scalar sigma_jerk_yaw = 10.0; ///< 角加速度jerk噪声（rad/s³），影响转向响应速度

        // ===== 测量噪声参数：对主EKF速度估计的信任度 =====
        Scalar meas_v_xy_var = 0.05; ///< XY速度测量方差（m/s）²
        Scalar meas_v_z_var  = 0.1;  ///< Z速度测量方差（m/s）²
        Scalar meas_vyaw_var = 0.1;  ///< 角速度测量方差（rad/s）²
    };
    Params params{};                 ///< 可配置参数实例

    /**
     * @brief 装甲板ID钳制（兼容接口）
     * @param id 输入ID
     * @return 固定返回0（本模型不处理多装甲板）
     * @note 仅为满足EkfTargetInfo接口要求，实际未使用
     */
    [[nodiscard]] static int clamp_armor_id(int id) noexcept { return 0; }

    /**
     * @brief 状态预测函数（模板化，支持自动微分）
     *
     * 实现恒定加速度模型（CA Model）：
     * - v_{k+1} = v_k + a_k * dt（速度积分）
     * - a_{k+1} = a_k（加速度保持，随机游走）
     *
     * @tparam T 标量类型（double或自动微分类型）
     * @param x 当前状态向量（8维）
     * @param dt 时间步长（秒）
     * @param xp 预测状态向量输出（8维）
     *
     * @note 模板化设计支持Ceres自动微分，用于非线性优化场景
     */
    template <typename T>
    static void predict_state(const T* x, const T& dt, T* xp) noexcept {
        // 线速度预测：v += a * dt
        xp[VX_A] = x[VX_A] + x[AX] * dt;
        xp[VY_A] = x[VY_A] + x[AY] * dt;
        xp[VZ_A] = x[VZ_A] + x[AZ] * dt;

        // 线加速度预测：保持不变（随机游走）
        xp[AX] = x[AX];
        xp[AY] = x[AY];
        xp[AZ] = x[AZ];

        // 角速度预测：v_yaw += alpha_yaw * dt
        xp[VYAW_A] = x[VYAW_A] + x[ALPHA_YAW] * dt;

        // 角加速度预测：保持不变（随机游走）
        xp[ALPHA_YAW] = x[ALPHA_YAW];
    }

    /**
     * @brief 测量预测函数（模板化，支持自动微分）
     *
     * 直接观测速度状态（无变换）：
     * - z = [vx, vy, vz, v_yaw]
     *
     * @tparam T 标量类型
     * @param x 状态向量（8维）
     * @param armor_id 装甲板ID（未使用，仅为兼容接口）
     * @param z 测量向量输出（4维）
     */
    template <typename T>
    static void measure_state(const T* x, int /*armor_id*/, T* z) noexcept {
        // 直接观测速度状态（前4个状态）
        z[AM_VX]   = x[VX_A];
        z[AM_VY]   = x[VY_A];
        z[AM_VZ]   = x[VZ_A];
        z[AM_VYAW] = x[VYAW_A];
    }

    /**
     * @brief 状态转移函数（EKF接口）
     *
     * @param x 当前状态（8维）
     * @param dt 时间步长（秒）
     * @return 预测状态（8维）
     */
    [[nodiscard]] VecX f(const VecX& x, double dt) const noexcept {
        VecX xp = VecX::Zero();
        predict_state<double>(x.data(), dt, xp.data());
        return xp;
    }

    /**
     * @brief 测量函数（EKF接口）
     *
     * @param x 状态向量（8维）
     * @param armor_id 装甲板ID（未使用）
     * @return 预测测量（4维）
     */
    [[nodiscard]] VecZ h(const VecX& x, int armor_id = 0) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    /**
     * @brief 计算过程噪声协方差矩阵Q
     *
     * 使用离散时间白噪声加速度模型的解析形式：
     * 对于状态对 [v, a]，在白噪声jerk驱动下：
     * Q = [[σ²*dt⁴/4, σ²*dt³/2],
     *      [σ²*dt³/2, σ²*dt²]]
     *
     * 数学推导：
     * - 设jerk（加速度导数）为白噪声，方差为σ²
     * - 则加速度为积分白噪声，速度为二重积分
     * - 使用分步积分可得上述协方差解析解
     *
     * @param dt 时间步长（秒）
     * @return 过程噪声协方差矩阵（8x8），零矩阵如果dt <= 0
     *
     * @note 相比数值积分，解析形式精度高且无累积误差
     */
    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX Q = MatXX::Zero();

        // 边界条件：dt <= 0时返回零矩阵
        if (dt <= 0.0) {
            return Q;
        }

        // 预计算时间幂次（避免重复计算）
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        // 预计算噪声方差
        const double sigma_xy2  = params.sigma_jerk_xy * params.sigma_jerk_xy;
        const double sigma_z2   = params.sigma_jerk_z * params.sigma_jerk_z;
        const double sigma_yaw2 = params.sigma_jerk_yaw * params.sigma_jerk_yaw;

        /**
         * @brief 填充恒定速度（CV）模型的协方差块
         *
         * 对于状态对 [velocity, acceleration]，协方差块为：
         * Q(v,a) = [[σ²*dt⁴/4, σ²*dt³/2],
         *           [σ²*dt³/2, σ²*dt²]]
         *
         * @param v_idx 速度状态索引
         * @param a_idx 加速度状态索引
         * @param q jerk噪声方差（σ²）
         */
        auto fill_cv = [&](int v_idx, int a_idx, double q) {
            Q(v_idx, v_idx) = q * dt4 / 4.0; // Var[v]
            Q(v_idx, a_idx) = q * dt3 / 2.0; // Cov[v,a]
            Q(a_idx, v_idx) = q * dt3 / 2.0; // Cov[a,v]（对称）
            Q(a_idx, a_idx) = q * dt2;       // Var[a]
        };

        // 填充XY平面（使用相同噪声参数）
        fill_cv(VX_A, AX, sigma_xy2);
        fill_cv(VY_A, AY, sigma_xy2);

        // 填充Z方向（使用独立噪声参数）
        fill_cv(VZ_A, AZ, sigma_z2);

        // 填充角速度（使用独立噪声参数）
        fill_cv(VYAW_A, ALPHA_YAW, sigma_yaw2);

        return Q;
    }

    /**
     * @brief 计算测量噪声协方差矩阵R
     *
     * @param z 测量向量（未使用，预留接口）
     * @return 测量噪声协方差矩阵（4x4，对角矩阵）
     *
     * @note 对角假设：假设各速度分量独立，简化计算
     */
    [[nodiscard]] MatZZ R(const VecZ& /*z*/) const noexcept {
        (void)params; // 抑制未使用成员警告（防止R变为static时编译报错）
        MatZZ R             = MatZZ::Zero();
        R(AM_VX, AM_VX)     = params.meas_v_xy_var;
        R(AM_VY, AM_VY)     = params.meas_v_xy_var;
        R(AM_VZ, AM_VZ)     = params.meas_v_z_var;
        R(AM_VYAW, AM_VYAW) = params.meas_vyaw_var;
        return R;
    }

    /**
     * @brief 获取测量噪声协方差对角向量（优化接口）
     *
     * 避免构造完整4x4矩阵，直接返回对角元素向量，
     * 用于对角假设的卡尔曼增益计算，提升性能。
     *
     * @param z 测量向量（未使用）
     * @return 测量噪声对角向量（4维）
     */
    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const noexcept {
        Eigen::Matrix<Scalar, NZ, 1> R_diag;
        R_diag << params.meas_v_xy_var, params.meas_v_xy_var, params.meas_v_z_var,
            params.meas_vyaw_var;
        return R_diag;
    }
};

} // namespace fcs::L3
