/**
 * @file invariant_extended_kalman_filter.hpp
 * @brief 不变扩展卡尔曼滤波器（IEKF）- 基于Lie群理论的状态估计
 *
 * @details
 * 本文件实现了不变扩展卡尔曼滤波器（Invariant Extended Kalman Filter, IEKF），
 * 用于非线性系统状态估计，特别适用于机器人姿态和位置跟踪。
 *
 * IEKF vs 传统EKF：
 * - 传统EKF：在欧氏空间中线性化，大角度误差时精度下降
 * - IEKF：在Lie群（矩阵群）上定义状态，使用对数映射（log map）处理误差
 *
 * 核心思想：
 * - 状态空间：Lie群（如SO(3), SE(3)），而非欧氏空间
 * - 误差状态：Lie代数（通过log map），保持几何结构
 * - 状态更新：群运算（左乘/右乘），而非加法
 * - 协方差：在Lie代数上定义，保持物理意义
 *
 * 数学基础：
 * - Lie群G：具有群结构的光滑矩阵群（如旋转矩阵群SO(3)）
 * - Lie代数g：群在单位元处的切空间（如旋转向量so(3)）
 * - 指数映射exp: g -> G（Lie代数到Lie群）
 * - 对数映射log: G -> g（Lie群到Lie代数）
 *
 * 状态表示：
 * - 名义状态（Nominal）：Lie群元素，如旋转矩阵、位姿
 * - 误差状态（Error）：Lie代数元素，如旋转向量、位姿误差
 * - 状态更新：x_new = x_old * exp(dx)，而非x_new = x_old + dx
 *
 * 协方差更新：
 * - 误差状态协方差P在Lie代数上定义
 * - 状态更新后，P需要在新的切空间上重新表示（伴随变换）
 * - 这保证了协方差矩阵的物理意义
 *
 * IEKF优势：
 * - 大角度误差时保持精度（传统EKF会退化）
 * - 状态约束自动满足（如旋转矩阵保持正交）
 * - 数值稳定性更好（避免万向锁）
 * - 参数化无关（不受欧拉角奇异点影响）
 *
 * 典型应用：
 * - 机器人位姿估计（SLAM）
 * - IMU姿态融合
 * - 相机位姿跟踪
 * - 目标运动估计（本项目）
 *
 * @tparam NominalT 名义状态类型，必须满足Lie群接口：
 *                  - Scalar: 标量类型（如double）
 *                  - VectorType: Lie代数向量类型
 *                  - TMatrixType: 误差协方差矩阵类型
 *                  - exp(VectorType): 指数映射
 *                  - log(NominalT): 对数映射
 *                  - inv(): 群逆元
 *                  - operator*(NominalT): 群运算
 *
 * @tparam N_Z 测量向量维度
 *
 * @see ExtendedKalmanFilter
 * @see RobotEkfMotionModel
 *
 * @warning 使用有限差分法线性化（精度依赖步长选择）
 * @warning 需要NominalT类型实现Lie群运算接口
 */

#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <functional>
#include <utility>

namespace fcs::L3 {

/**
 * @class InvariantExtendedKalmanFilter
 * @brief 不变扩展卡尔曼滤波器，基于Lie群理论的状态估计
 *
 * @details
 * 核心特性：
 * - 名义状态在Lie群上（保持几何结构）
 * - 误差状态在Lie代数上（欧氏向量空间）
 * - 使用指数/对数映射进行状态更新
 * - 有限差分法计算过程雅可比矩阵
 *
 * @tparam NominalT 名义状态类型（Lie群元素）
 * @tparam N_Z 测量维度
 */
template <class NominalT, int N_Z>
class InvariantExtendedKalmanFilter {
public:
    using Nominal = NominalT;                 ///< 名义状态类型（Lie群元素）
    using Scalar  = typename Nominal::Scalar; ///< 标量类型（如double）

    // 矩阵类型定义
    using MatrixXX = typename Nominal::TMatrixType; ///< 误差协方差矩阵类型（Lie代数空间）
    using MatrixZX = Eigen::Matrix<Scalar, N_Z, MatrixXX::RowsAtCompileTime>; ///< 测量雅可比矩阵
    using MatrixXZ = Eigen::Matrix<Scalar, MatrixXX::RowsAtCompileTime, N_Z>; ///< 卡尔曼增益
    using MatrixZZ = Eigen::Matrix<Scalar, N_Z, N_Z>;                         ///< 测量协方差矩阵
    using MatrixX1 = typename Nominal::VectorType; ///< Lie代数向量类型（误差状态）

    using PredictFunc = std::function<Nominal(const Nominal&, Scalar)>; ///< 预测函数类型
    using UpdateQFunc = std::function<MatrixXX(Scalar)>;                ///< 过程噪声协方差函数类型

    static constexpr int N_X = MatrixX1::RowsAtCompileTime; ///< 状态维度（Lie代数向量维度）

    InvariantExtendedKalmanFilter() = default;

    /**
     * @brief 构造函数 - 初始化IEKF实例
     *
     * @details
     * 初始化步骤：
     * 1. 存储预测函数f（Lie群上的状态转移）
     * 2. 存储过程噪声协方差函数update_Q
     * 3. 设置有限差分步长（用于线性化）
     * 4. 初始化后验协方差P_post
     *
     * @param f 预测函数（Lie群上的状态转移）
     * @param update_q 过程噪声协方差生成函数
     * @param P0 初始误差协方差矩阵（在Lie代数上定义）
     * @param finite_difference_step 有限差分步长（默认1e-6）
     *
     * @note 有限差分步长需要在数值精度和截断误差间权衡
     * @warning 步长过大会降低线性化精度，过小会引入数值误差
     */
    explicit InvariantExtendedKalmanFilter(
        PredictFunc f, UpdateQFunc update_q, const MatrixXX& P0,
        Scalar finite_difference_step = Scalar(1e-6))
        : f_(std::move(f))
        , update_Q_(std::move(update_q))
        , finite_difference_step_(finite_difference_step)
        , P_post_(P0) {
        F_.setIdentity();
        H_.setZero();
        Q_.setZero();
        R_.setZero();
        K_.setZero();
    }

    /// @brief 设置初始状态（Lie群元素）
    void setState(const Nominal& x0) { x_post_ = x0; }

    /// @brief 获取当前状态（只读）
    [[nodiscard]] const Nominal& X() const noexcept { return x_post_; }
    /// @brief 获取当前协方差矩阵（只读）
    [[nodiscard]] const MatrixXX& P() const noexcept { return P_post_; }

    /**
     * @brief 预测步（时间更新）- 在Lie群上进行状态传播
     *
     * @details
     * 算法步骤：
     * 1. 调用预测函数f，得到预测状态x_pri（Lie群运算）
     * 2. 使用有限差分法计算过程雅可比矩阵F
     * 3. 计算过程噪声协方差Q
     * 4. 更新协方差：P = F * P * F^T + Q（在Lie代数上）
     * 5. 更新状态：x_post = x_pri
     *
     * Lie群状态更新：
     * - 传统EKF：x_pred = f(x_post)（加法）
     * - IEKF：x_pred = f(x_post, dt)（群运算）
     *
     * 协方差更新：
     * - 在Lie代数（切空间）上进行线性化
     * - F矩阵将误差从一个切空间映射到另一个
     *
     * @param dt 时间间隔（秒）
     * @return const Nominal& 预测后的状态引用
     *
     * @note 有限差分法：F_ij ≈ (f(x+εe_j) - f(x-εe_j)) / (2ε)
     * @note 使用中心差分提高精度
     */
    const Nominal& predict(Scalar dt) {
        dt                  = std::max(Scalar(0), dt);
        const Nominal x_pri = f_(x_post_, dt);                        // 调用预测函数（Lie群运算）
        F_                  = linearize_process_(x_post_, x_pri, dt); // 有限差分法计算F
        Q_                  = update_Q_(dt);                          // 获取过程噪声协方差

        // 协方差更新：P = F * P * F^T + Q（在Lie代数上）
        P_post_ = symmetrized_(F_ * P_post_ * F_.transpose() + Q_);
        x_post_ = x_pri;

        return x_post_;
    }

    /**
     * @brief 更新步（测量更新）- 使用新息和测量雅可比矩阵
     *
     * @details
     * 算法步骤：
     * 1. 接收新息（innovation）和测量雅可比矩阵H（由外部计算）
     * 2. 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^{-1}
     * 3. 计算误差状态：dx = K * innovation（Lie代数向量）
     * 4. 更新状态：x_new = x_old * exp(dx)（Lie群运算）
     * 5. 使用Joseph形式更新协方差：P = (I-KH) * P * (I-KH)^T + K * R * K^T
     *
     * 新息定义：
     * - 传统EKF：ν = z - h(x)（测量残差）
     * - IEKF：ν可以是定义在测量空间的任意向量
     *
     * 状态更新：
     * - 传统EKF：x_new = x_old + dx（加法）
     * - IEKF：x_new = x_old * exp(dx)（群运算，保持几何结构）
     *
     * @param innovation 新息向量（测量空间的误差）
     * @param H 测量雅可比矩阵（由外部计算）
     * @param R 测量噪声协方差矩阵
     * @return const Nominal& 更新后的状态引用
     *
     * @note 使用直接求逆（S.inverse()），可能存在数值风险
     * @warning 大维度测量时建议使用LDLT分解代替直接求逆
     */
    const Nominal& update(
        const Eigen::Matrix<Scalar, N_Z, 1>& innovation, const MatrixZX& H, const MatrixZZ& R) {
        H_ = H;
        R_ = R;

        // 计算卡尔曼增益
        const MatrixZZ S = H_ * P_post_ * H_.transpose() + R_;     // 新息协方差
        K_               = P_post_ * H_.transpose() * S.inverse(); // 卡尔曼增益

        // 计算误差状态（Lie代数向量）
        const MatrixX1 dx = K_ * innovation;

        // 状态更新：Lie群运算
        // x_new = x_old * exp(dx)（保持几何结构，如旋转矩阵的正交性）
        x_post_ = x_post_ * Nominal::exp(dx);

        // 协方差更新：Joseph形式（保证正定性）
        const MatrixXX IKH = MatrixXX::Identity() - K_ * H_;
        P_post_ = symmetrized_(IKH * P_post_ * IKH.transpose() + K_ * R_ * K_.transpose());

        return x_post_;
    }

private:
    /**
     * @brief 协方差矩阵对称化（防止数值漂移）
     *
     * @details
     * 数值计算可能导致协方差矩阵失去对称性（P(i,j) ≠ P(j,i)），
     * 通过取平均强制对称化：P_sym = 0.5 * (P + P^T)
     *
     * @param P 待对称化的协方差矩阵
     * @return MatrixXX 对称化后的协方差矩阵
     */
    [[nodiscard]] static MatrixXX symmetrized_(const MatrixXX& P) {
        return Scalar(0.5) * (P + P.transpose());
    }

    /**
     * @brief 使用有限差分法计算过程雅可比矩阵F
     *
     * @details
     * 在Lie代数上进行中心差分：
     * F_ij ≈ [log(x_pred^{-1} * f(x * exp(εe_j), dt)) -
     *         log(x_pred^{-1} * f(x * exp(-εe_j), dt))] / (2ε)
     *
     * 步骤：
     * 1. 在状态x上施加扰动：x_plus = x * exp(εe_i)（Lie群运算）
     * 2. 调用预测函数：y_plus = f(x_plus, dt)
     * 3. 计算误差：e_plus = log(x_pred^{-1} * y_plus)（Lie代数）
     * 4. 重复步骤1-3，但使用-ε扰动
     * 5. 计算差分：F.col(i) = (e_plus - e_minus) / (2ε)
     *
     * 为什么使用Lie群运算？
     * - 传统有限差分：x_plus = x + εe_i（假设欧氏空间）
     * - IEKF有限差分：x_plus = x * exp(εe_i)（在Lie群上）
     * - 优势：保持状态的几何结构（如旋转矩阵的正交性）
     *
     * @param x 当前状态（Lie群）
     * @param x_pred 预测状态（Lie群）
     * @param dt 时间间隔
     * @return MatrixXX 过程雅可比矩阵F
     *
     * @note 复杂度：O(N_X^2)，需要N_X次预测函数调用
     * @warning 步长eps选择：太小->数值误差，太大->截断误差
     */
    [[nodiscard]] MatrixXX
        linearize_process_(const Nominal& x, const Nominal& x_pred, Scalar dt) const {
        MatrixXX F;
        F.setZero();

        const Scalar eps = finite_difference_step_; // 有限差分步长

        // 对每个状态维度进行中心差分
        for (int i = 0; i < N_X; ++i) {
            // 正扰动：x_plus = x * exp(εe_i)
            MatrixX1 d            = MatrixX1::Zero();
            d[i]                  = eps;
            const Nominal x_plus  = x * Nominal::exp(d);                 // Lie群运算
            const Nominal y_plus  = f_(x_plus, dt);                      // 预测函数
            const MatrixX1 e_plus = Nominal::log(x_pred.inv() * y_plus); // Lie代数误差

            // 负扰动：x_minus = x * exp(-εe_i)
            d[i]                   = -eps;
            const Nominal x_minus  = x * Nominal::exp(d);                  // Lie群运算
            const Nominal y_minus  = f_(x_minus, dt);                      // 预测函数
            const MatrixX1 e_minus = Nominal::log(x_pred.inv() * y_minus); // Lie代数误差

            // 中心差分：F.col(i) = (e_plus - e_minus) / (2ε)
            F.col(i) = (e_plus - e_minus) / (Scalar(2) * eps);
        }

        return F;
    }

    // ========================================================================
    // 成员变量
    // ========================================================================

    PredictFunc f_{};                             ///< 预测函数（Lie群上的状态转移）
    UpdateQFunc update_Q_{};                      ///< 过程噪声协方差生成函数
    Scalar finite_difference_step_{Scalar(1e-6)}; ///< 有限差分步长

    MatrixXX F_{MatrixXX::Identity()};            ///< 过程雅可比矩阵（Lie代数空间）
    MatrixZX H_{MatrixZX::Zero()};                ///< 测量雅可比矩阵（由外部提供）
    MatrixXX Q_{MatrixXX::Zero()};                ///< 过程噪声协方差矩阵
    MatrixZZ R_{MatrixZZ::Zero()};                ///< 测量噪声协方差矩阵
    MatrixXX P_post_{MatrixXX::Identity()};       ///< 后验协方差矩阵（Lie代数空间）
    MatrixXZ K_{MatrixXZ::Zero()};                ///< 卡尔曼增益矩阵

    Nominal x_post_{};                            ///< 后验状态（Lie群元素）
};

} // namespace fcs::L3
