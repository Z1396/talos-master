/**
 * @file extended_kalman_filter.hpp
 * @brief 扩展卡尔曼滤波器（EKF）- 使用Ceres Jet自动微分实现
 *
 * @details
 * 本文件实现了扩展卡尔曼滤波器（Extended Kalman Filter, EKF），用于非线性状态估计。
 * 核心特性：
 * - 使用Ceres Jet类型实现自动微分，避免手动推导雅可比矩阵
 * - 实现完整的收敛状态监控机制（Unknown/Converging/Converged/Diverging）
 * - 采用Joseph形式协方差更新，保证正定性
 * - 提供数值稳定性检查和异常处理
 *
 * EKF算法原理：
 * 1. 预测步（时间更新）：
 *    x_pred = f(x_post)         // 状态预测
 *    P_pred = F * P_post * F^T + Q  // 协方差预测
 *    其中F = ∂f/∂x（雅可比矩阵）
 *
 * 2. 更新步（测量更新）：
 *    K = P_pred * H^T * (H * P_pred * H^T + R)^{-1}  // 卡尔曼增益
 *    x_post = x_pred + K * (z - h(x_pred))           // 状态更新
 *    P_post = (I - K*H) * P_pred * (I - K*H)^T + K*R*K^T  // 协方差更新（Joseph形式）
 *    其中H = ∂h/∂x（雅可比矩阵）
 *
 * Ceres Jet自动微分：
 * ceres::Jet<T, N>是值类型，包含：
 *   - T a;     // 标量值（如double）
 *   - T v[N];  // 导数向量（梯度）
 *
 * 使用方式：
 * 1. 将状态向量封装为Jet数组
 * 2. 调用非线性函数f或h
 * 3. 从结果中提取值和导数
 *
 * 收敛性监控：
 * - 归一化新息平方（NIS）：ν^T * S^{-1} * ν，应服从χ²分布
 * - 协方差对角元素：检测发散（过大）或退化（过小）
 * - 连续收敛/发散计数：避免单次抖动误判
 *
 * @tparam N_X 状态向量维度
 * @tparam N_Z 测量向量维度
 * @tparam PredicFunc 预测函数类型（非线性状态转移）
 * @tparam MeasureFunc 测量函数类型（非线性观测）
 *
 * @see ceres::Jet
 * @see ConvergenceStatus
 * @see ConvergenceCriteria
 *
 * @warning 数值稳定性风险：
 * - 协方差矩阵可能失去正定性（需Joseph形式）
 * - 新息协方差S可能奇异（需LDLT分解）
 * - 大协方差值可能导致数值溢出
 *
 * @author Chen Jun, xinyang, Chengfu Zou
 * @copyright MIT License, Apache License 2.0
 */

// Copyright Chen Jun 2023. Licensed under the MIT License.
// Copyright xinyang 2021.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once  // 防止头文件被重复包含

// std 标准库头文件（提供基础功能）
#include <algorithm>   // 提供 std::max 等算法函数（用于数值保险）
#include <cmath>       // 提供 std::isfinite 等数学函数（用于检查 NaN 和无穷大）
#include <cstdint>     // 提供 uint8_t 等标准整数类型（用于枚举状态）
#include <functional>  // 提供 std::function（用于存储用户传入的 f 和 h 函数）
#include <limits>      // 提供 std::numeric_limits（用于获取 inf 和 最小值）
// Eigen 线性代数库（矩阵运算）
#include <Eigen/Dense> // 提供矩阵、向量、LDLT分解等
// ceres 自动微分库
#include <ceres/jet.h> // 提供 ceres::Jet 类型（自动微分的核心）

/**
 * @class ExtendedKalmanFilter
 * @brief 扩展卡尔曼滤波器，支持自动微分和收敛性监控
 *
 * @details
 * 核心特性：
 * - 自动微分：使用Ceres Jet，无需手动推导雅可比矩阵
 * - 收敛监控：基于NIS和协方差的状态诊断
 * - Joseph更新：保证协方差正定性
 * - LDLT分解：稳定求解新息协方差逆
 *
 * @tparam N_X 状态维度
 * @tparam N_Z 测量维度
 * @tparam PredicFunc 预测函数签名：void(const JetX*, JetX*)
 * @tparam MeasureFunc 测量函数签名：void(const JetX*, JetX*)
 */
template <int N_X, int N_Z, class PredicFunc, class MeasureFunc>  // 模板参数：状态维度、观测维度、函数类型
class ExtendedKalmanFilter {  // EKF 类定义
public:
    ExtendedKalmanFilter() = default;  // 默认构造函数（通常不使用，因为依赖 P0 初始化）

    // 矩阵类型别名（Eigen 静态矩阵，编译期固定大小，效率极高）
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>; ///< 状态协方差矩阵类型 (N_X * N_X)
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>; ///< 测量-状态雅可比矩阵类型 (N_Z * N_X)
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>; ///< 卡尔曼增益矩阵类型 (N_X * N_Z)
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>; ///< 测量协方差矩阵类型 (N_Z * N_Z)
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;   ///< 状态向量类型 (N_X * 1)
    using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;   ///< 测量向量类型 (N_Z * 1)

    // 噪声协方差函数类型
    using UpdateQFunc = std::function<MatrixXX()>;                  ///< 过程噪声协方差生成函数（每次预测时调用）
    using UpdateRFunc = std::function<MatrixZZ(const MatrixZ1& z)>; ///< 测量噪声协方差生成函数（每次更新时调用）

    /**
     * @enum ConvergenceStatus
     * @brief 收敛状态枚举，用于监控滤波器健康度（看当前滤波器是好是坏）
     */
    enum class ConvergenceStatus : uint8_t {  // 无符号8位整数存储状态
        Unknown,    ///< 未知状态（滤波器刚创建，什么都没确定）
        Converging, ///< 正在收敛（误差在缩小，但还没达到稳定）
        Converged,  ///< 已收敛（精确跟踪，状态极其稳定）
        Diverging,  ///< 正在发散（误差在无限放大，滤波器已经失效）
    };

    /**
     * @struct ConvergenceCriteria
     * @brief 收敛判据配置（这是用户可以调节的“体检指标”）
     *
     * @details
     * 用于判断滤波器是否收敛或发散的阈值参数。
     * 参数选择基于χ²分布理论和经验值。
     */
    struct ConvergenceCriteria {
        double converged_nis{2.0 * static_cast<double>(N_Z)}; ///< 收敛NIS阈值（约95%置信区间，越小越准）
        double diverged_nis{25.0 * static_cast<double>(N_Z)}; ///< 发散NIS阈值（严重异常，一旦超过说明严重偏离）
        double converged_max_covariance_diag{1e6};            ///< 收敛最大协方差对角元素（绝对信心值，超过说明不够准）
        double diverged_max_covariance_diag{1e12}; ///< 发散最大协方差对角元素（接近数值极限，滤波器算炸了）
        double min_covariance_diag{-1e-9};         ///< 最小协方差对角元素（允许极小负值，检测退化（比如协方差变成负数））
        int converged_updates{3};                  ///< 连续收敛次数阈值（必须连续 3 次达标才算收敛，防抖动）
        int diverged_updates{2};                   ///< 连续发散次数阈值（连续 2 次恶劣算发散）
    };

    /**
     * @brief 构造函数 - 初始化EKF实例
     *
     * @details
     * 初始化步骤：
     * 1. 存储非线性函数f和h（用于自动微分）
     * 2. 存储噪声协方差函数update_Q和update_R
     * 3. 初始化后验协方差P_post
     * 4. 设置收敛判据
     *
     * @param f 预测函数（状态转移）
     * @param h 测量函数（观测模型）
     * @param u_q 过程噪声协方差生成函数
     * @param u_r 测量噪声协方差生成函数
     * @param P0 初始协方差矩阵
     * @param convergence_criteria 收敛判据（可选）
     *
     * @note 构造函数必须noexcept，避免异常
     */
    explicit ExtendedKalmanFilter(  // 显式构造，避免隐式类型转换
        const PredicFunc& f, const MeasureFunc& h, const UpdateQFunc& u_q, const UpdateRFunc& u_r,
        const MatrixXX& P0, ConvergenceCriteria convergence_criteria = {}) noexcept // noexcept: 不抛异常
        : f(f)                        // 存储预测函数 f
        , h(h)                        // 存储测量函数 h
        , update_Q(u_q)               // 存储过程噪声生成函数
        , update_R(u_r)               // 存储测量噪声生成函数
        , P_post(P0)                  // 初始后验协方差 P
        , convergence_criteria_(convergence_criteria) {  // 存下收敛判据
        F = MatrixXX::Zero();    // 初始化预测雅可比矩阵为零矩阵
        H = MatrixZX::Zero();    // 初始化观测雅可比矩阵为零矩阵
        update_covariance_stat_(P_post);  // 计算初始协方差的最大对角线元素（用于后续监控）
    }

    /// @brief 设置初始状态向量
    void setState(const MatrixX1& x0) noexcept { x_post = x0; }  // 直接把初始估计状态填入 x_post

    // ========================================================================
    // 只读访问接口（用于可视化和调试）
    // ========================================================================

    [[nodiscard]] const MatrixX1& X() const noexcept { return x_post; }  ///< 获取当前后验状态向量（最重要的输出）
    [[nodiscard]] const MatrixXX& P() const noexcept { return P_post; }  ///< 获取当前后验协方差矩阵（代表估计的不确定度）
    [[nodiscard]] const MatrixXZ& K_gain() const noexcept { return K_; } ///< 获取卡尔曼增益矩阵（当前观测的权重）
    [[nodiscard]] const MatrixXX& Q_mat() const noexcept { return Q; }   ///< 获取过程噪声协方差（模型有多不靠谱）
    [[nodiscard]] const MatrixZZ& R_mat() const noexcept { return R; }   ///< 获取测量噪声协方差（传感器有多不靠谱）

    /// @brief 获取收敛状态
    [[nodiscard]] ConvergenceStatus convergence_status() const noexcept {  // 返回当前滤波器健康状态
        return convergence_status_;
    }
    [[nodiscard]] bool converging() const noexcept {  // 是否正在收敛
        return convergence_status_ == ConvergenceStatus::Converging;
    }
    [[nodiscard]] bool converged() const noexcept {   // 是否已经收敛（稳定可靠）
        return convergence_status_ == ConvergenceStatus::Converged;
    }
    [[nodiscard]] bool diverging() const noexcept {   // 是否已经发散（要抛弃滤波器结果）
        return convergence_status_ == ConvergenceStatus::Diverging;
    }

    /// @brief 获取归一化新息平方（NIS - 衡量测量和预测的偏差是否在正常统计范围内）
    [[nodiscard]] double normalized_innovation_squared() const noexcept {
        return normalized_innovation_squared_;
    }
    /// @brief 获取最大协方差对角元素（衡量滤波器目前有多大自信）
    [[nodiscard]] double max_covariance_diag() const noexcept { return max_covariance_diag_; }
    /// @brief 获取连续收敛次数
    [[nodiscard]] int consecutive_converged_updates() const noexcept {
        return consecutive_converged_updates_;
    }
    /// @brief 获取连续发散次数
    [[nodiscard]] int consecutive_diverged_updates() const noexcept {
        return consecutive_diverged_updates_;
    }
    /// @brief 获取收敛判据配置
    [[nodiscard]] const ConvergenceCriteria& convergence_criteria() const noexcept {
        return convergence_criteria_;
    }

    /// @brief 设置收敛判据（会重置收敛状态）
    void setConvergenceCriteria(ConvergenceCriteria criteria) noexcept {
        convergence_criteria_ = criteria;  // 更新判据
        resetConvergenceStatus();          // 清零监控计数器，重新开始统计
    }

    /// @brief 重置收敛状态（用于重新初始化）
    void resetConvergenceStatus() noexcept {
        convergence_status_            = ConvergenceStatus::Unknown;  // 状态归零为未知
        consecutive_converged_updates_ = 0;                          // 收敛计数归零
        consecutive_diverged_updates_  = 0;                          // 发散计数归零
        normalized_innovation_squared_ = std::numeric_limits<double>::infinity(); // NIS 设为无穷大
        max_covariance_diag_           = std::numeric_limits<double>::infinity(); // 最大协方差设为无穷大
    }

    /// @brief 设置预测函数（允许运行时更换模型）
    void setPredictFunc(const PredicFunc& f) noexcept { this->f = f; } // 替换预测函数 f

    /// @brief 设置测量函数（允许运行时更换模型）
    void setMeasureFunc(const MeasureFunc& h) noexcept { this->h = h; } // 替换测量函数 h

    /**
     * @brief 预测步（时间更新）- 使用自动微分计算雅可比矩阵
     *
     * @details
     * 算法步骤：
     * 1. 将后验状态x_post封装为Jet数组（自动微分输入）
     * 2. 调用非线性预测函数f，得到预测状态和雅可比矩阵F
     * 3. 从Jet结果中提取预测状态x_pri和雅可比矩阵F
     * 4. 计算预测协方差：P_pri = F * P_post * F^T + Q
     * 5. 更新后验状态：x_post = x_pri
     * 6. 检查预测健康度（数值稳定性）
     *
     * Ceres Jet自动微分原理：
     * - Jet类型：包含标量值a和导数向量v[N]
     * - 初始化：x_e_jet[i].a = x_post[i], x_e_jet[i].v[i] = 1（偏导数初始化）
     * - 计算：f(x_e_jet, x_p_jet)会自动计算导数
     * - 提取：x_p_jet[i].a是预测值，x_p_jet[i].v是雅可比矩阵第i行
     *
     * @return MatrixX1 预测状态向量（x_pri）
     *
     * @note 性能优化：栈分配Jet数组，避免堆分配
     * @note 自动微分避免了手动推导雅可比矩阵的繁琐和错误
     *
     * @warning 如果预测导致数值不稳定（NaN/Inf），会标记为发散状态
     */
    MatrixX1 predict() noexcept {  // 预测阶段
        // ----------------------------------------------------------------------
        // 步骤1-2：自动微分计算预测状态和雅可比矩阵
        // ----------------------------------------------------------------------
        // 创建Jet数组：栈分配，RAII管理（极快，无堆内存开销）
        ceres::Jet<double, N_X> x_e_jet[N_X];  // 声明输入Jet数组

        // 初始化Jet数组：值 = 后验状态，导数 = 单位向量（∂x_i/∂x_i = 1）
        for (int i = 0; i < N_X; ++i) {
            x_e_jet[i].a    = x_post[i];  // 存储状态数值
            x_e_jet[i].v[i] = 1.;         // 自动微分初始化：每个变量对自己求导是1
        }

        // 调用非线性预测函数，自动计算导数（不用手算复杂的雅可比）
        ceres::Jet<double, N_X> x_p_jet[N_X]; // 声明输出Jet数组
        f(x_e_jet, x_p_jet);                  // 用户定义的非线性函数 f

        // ----------------------------------------------------------------------
        // 步骤3：从Jet结果中提取预测状态和雅可比矩阵
        // ----------------------------------------------------------------------
        for (int i = 0; i < N_X; ++i) {
            x_pri[i]              = x_p_jet[i].a;             // 提取预测后的状态值
            F.block(i, 0, 1, N_X) = x_p_jet[i].v.transpose(); // 提取雅可比矩阵第 i 行（导数）
        }

        // ----------------------------------------------------------------------
        // 步骤4-6：计算预测协方差并更新状态
        // ----------------------------------------------------------------------
        Q      = update_Q();                     // 获取当前时刻的过程噪声协方差 Q
        P_pri  = F * P_post * F.transpose() + Q; // 协方差预测公式：P_pri = F * P_post * F^T + Q
        x_post = x_pri;                          // 直接更新后验状态（预测后没有测量，先暂时用预测值）

        // 检查预测健康度（检测数值异常，比如矩阵中出现 NaN 或 Inf）
        update_prediction_health_();

        return x_pri;  // 返回预测状态
    }

    /**
     * @brief 更新步（测量更新）- 使用LDLT分解和Joseph形式保证稳定性
     *
     * @details
     * 算法步骤：
     * 1. 将先验状态x_pri封装为Jet数组
     * 2. 调用非线性测量函数h，得到预测测量和雅可比矩阵H
     * 3. 计算新息（测量残差）：ν = z - z_pri
     * 4. 计算新息协方差：S = H * P_pri * H^T + R
     * 5. 使用LDLT分解求解：K = P_pri * H^T * S^{-1}
     * 6. 计算归一化新息平方：NIS = ν^T * S^{-1} * ν
     * 7. 更新状态：x_post = x_pri + K * ν
     * 8. 使用Joseph形式更新协方差：P = (I-KH) * P_pri * (I-KH)^T + K*R*K^T
     * 9. 检查收敛状态
     *
     * LDLT分解优势：
     * - 数值稳定：避免直接求逆的数值问题
     * - 正定性检查：LDLT分解可以检测矩阵是否正定
     * - 效率高：O(n³)复杂度，但比全分解快
     *
     * Joseph形式：
     * - 保证协方差矩阵对称正定
     * - 避免标准形式P = (I-KH) * P_pri的数值问题
     * - 代价：计算量稍大，但值得
     *
     * @param z 测量向量
     * @return MatrixX1 更新后的状态向量（x_post）
     *
     * @note 如果新息协方差S奇异或非正定，会跳过更新并标记发散
     * @warning 数值风险：大协方差值可能导致LDLT分解失败
     */
    MatrixX1 update(const MatrixZ1& z) noexcept {  // 更新阶段
        // ----------------------------------------------------------------------
        // 步骤1-2：自动微分计算预测测量和雅可比矩阵
        // ----------------------------------------------------------------------
        ceres::Jet<double, N_X> x_p_jet[N_X];  // 输入状态 Jet 数组
        for (int i = 0; i < N_X; i++) {
            x_p_jet[i].a    = x_pri[i];   // 使用预测阶段得到的状态 x_pri
            x_p_jet[i].v[i] = 1;          // 自动微分初始化
        }

        ceres::Jet<double, N_X> z_p_jet[N_Z];  // 输出测量 Jet 数组（维度是观测维度）
        h(x_p_jet, z_p_jet);                   // 调用测量函数 h

        // 提取预测测量和雅可比矩阵
        MatrixZ1 z_pri;  // 预测测量值
        for (int i = 0; i < N_Z; i++) {
            z_pri[i]              = z_p_jet[i].a;             // 提取预测观测值
            H.block(i, 0, 1, N_X) = z_p_jet[i].v.transpose(); // 提取观测雅可比矩阵第 i 行
        }

        // ----------------------------------------------------------------------
        // 步骤3-5：计算卡尔曼增益（使用LDLT分解）
        // ----------------------------------------------------------------------
        R                 = update_R(z);                   // 获取当前测量噪声协方差 R
        const MatrixZ1 nu = z - z_pri;                     // 新息（测量残差）：实际观测值 - 预测观测值
        const MatrixZZ S  = H * P_pri * H.transpose() + R; // 新息协方差

        // LDLT分解（数值稳定，可检测正定性，比直接求逆安全得多）
        Eigen::LDLT<MatrixZZ> ldlt(S);

        // 检查分解成功和正定性（如果矩阵不是正定的，说明传感器数据异常或系统计算崩溃）
        if (!S.allFinite() || ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
            // 数值异常：跳过更新，标记发散（保护系统不崩溃，向外界宣告滤波器失效）
            K_                             = MatrixXZ::Zero();  // 增益设为零（不采纳测量值）
            normalized_innovation_squared_ = std::numeric_limits<double>::infinity(); // NIS 置为无穷大表示异常
            mark_immediate_divergence_();  // 立即标记为发散
            return x_post;                 // 保持原状态返回，不进行任何修正
        }

        // 使用LDLT求解：K^T = S^{-1} * H * P_pri^T
        const MatrixZ1 solved_nu = ldlt.solve(nu);                    // 求解 S^{-1} * ν（用于算 NIS）
        const MatrixZX gain_t    = ldlt.solve(H * P_pri.transpose()); // 求解 S^{-1} * H * P_pri^T（用于算增益）

        // 检查求解结果有效性（防止无穷大/NaN 传播）
        if (!solved_nu.allFinite() || !gain_t.allFinite()) {
            K_                             = MatrixXZ::Zero();  // 同样，直接放弃
            normalized_innovation_squared_ = std::numeric_limits<double>::infinity(); // 异常
            mark_immediate_divergence_();  // 发散
            return x_post;                 // 返回原状态
        }

        // ----------------------------------------------------------------------
        // 步骤6-8：计算NIS、更新状态和协方差
        // ----------------------------------------------------------------------
        // 归一化新息平方（NIS）：用于收敛性监控（它是衡量预测和测量匹配程度的统计指标）
        normalized_innovation_squared_ = std::max(0.0, static_cast<double>(nu.dot(solved_nu)));

        // 卡尔曼增益
        K_ = gain_t.transpose();  // 从上面求得的转置矩阵恢复到标准 K 维度

        // 状态更新：用增益乘以误差，修正状态
        x_post = x_post + K_ * nu;

        // 协方差更新：Joseph形式（保证对称正定）
        // 标准形式：P = (I - K*H) * P_pri（可能失去正定性）
        // Joseph形式：P = (I-KH) * P_pri * (I-KH)^T + K*R*K^T（数学上保证永远稳定正定）
        const MatrixXX IKH = MatrixXX::Identity() - K_ * H;  // 计算 (I - K * H)
        P_post             = IKH * P_pri * IKH.transpose() + K_ * R * K_.transpose(); // 更新协方差

        // ----------------------------------------------------------------------
        // 步骤9：更新收敛状态
        // ----------------------------------------------------------------------
        update_convergence_status_();  // 检查状态，决定是否标记为收敛或发散

        return x_post;  // 返回修正后的后验状态
    }

    /**
     * @brief 解相关：将状态i和j之间的协方差置零
     *
     * @details
     * 用途：在某些模型中，某些状态变量在物理上独立，
     * 但滤波器可能产生虚假相关性（"关联幻觉"）。
     * 通过置零协方差矩阵的非对角元素，可以避免这种问题。
     *
     * 典型应用：
     * - 机器人跟踪中，两个旋转半径r0和r1是独立参数
     * - 如果不解除相关，滤波器可能错误地认为r0和r1相关
     *
     * @param i 状态索引i
     * @param j 状态索引j
     *
     * @note 仅修改P_post的(i,j)和(j,i)元素
     * @warning 过度使用可能破坏协方差矩阵的结构
     */
    void decorrelate(int i, int j) noexcept {  // 人为打破两个状态变量的数学相关性
        P_post(i, j) = 0.0;  // 协方差矩阵中第 i 行第 j 列置零
        P_post(j, i) = 0.0;  // 协方差矩阵中第 j 行第 i 列置零（保持对称）
        update_covariance_stat_(P_post);  // 重新计算统计值（以防万一）
    }

private:  // 以下为私有辅助函数
    /**
     * @brief 检查预测健康度（数值稳定性）
     *
     * @details
     * 检查项：
     * 1. 预测状态x_pri是否包含NaN/Inf
     * 2. 预测协方差P_pri是否包含NaN/Inf
     * 3. 协方差对角元素是否过小（退化）
     * 4. 协方差对角元素是否过大（发散）
     *
     * 状态转换：
     * - 异常 -> 立即标记为Diverging
     * - 协方差异常 -> 记录发散样本
     * - 正常 -> Unknown -> Converging
     * - 已收敛但协方差增大 -> Converged -> Converging
     */
    void update_prediction_health_() noexcept {  // 预测后的体检
        update_covariance_stat_(P_pri);  // 统计当前协方差的最大值

        // 检查数值异常（NaN/Inf/负值）
        if (!x_pri.allFinite() || !P_pri.allFinite()
            || min_covariance_diag(P_pri) < convergence_criteria_.min_covariance_diag) {
            mark_immediate_divergence_();  // 算错了，立刻宣判死刑
            return;
        }

        // 检查协方差是否过大（发散）
        if (max_covariance_diag_ > convergence_criteria_.diverged_max_covariance_diag) {
            register_divergence_sample_();  // 统计一次发散样本
            return;
        }

        // 正常情况：更新收敛状态
        if (convergence_status_ == ConvergenceStatus::Unknown) {
            convergence_status_ = ConvergenceStatus::Converging; // 从未知转为正在收敛
        }
        // 已收敛但协方差增大：重新进入Converging（可能突然遭遇剧烈机动）
        if (convergence_status_ == ConvergenceStatus::Converged
            && max_covariance_diag_ > convergence_criteria_.converged_max_covariance_diag) {
            convergence_status_            = ConvergenceStatus::Converging; // 退出完全收敛状态
            consecutive_converged_updates_ = 0; // 重新开始计数
        }
    }

    /**
     * @brief 更新收敛状态（基于NIS和协方差）
     *
     * @details
     * 收敛判据：
     * - NIS <= converged_nis（约95%置信区间）
     * - max_covariance_diag <= converged_max_covariance_diag
     * - 连续converged_updates次满足上述条件
     *
     * 发散判据：
     * - NIS > diverged_nis（严重异常）
     * - max_covariance_diag > diverged_max_covariance_diag
     * - 连续diverged_updates次满足上述条件
     *
     * 状态机：
     * Unknown -> Converging -> Converged
     *         -> Diverging
     */
    void update_convergence_status_() noexcept {  // 测量更新后的体检
        update_covariance_stat_(P_post);  // 重新统计

        // 检查数值异常
        if (!std::isfinite(normalized_innovation_squared_) || !x_post.allFinite()
            || !P_post.allFinite()
            || min_covariance_diag(P_post) < convergence_criteria_.min_covariance_diag) {
            mark_immediate_divergence_();  // 数学上彻底崩溃了
            return;
        }

        // 检查发散条件
        const bool covariance_diverging =  // 协方差过大
            max_covariance_diag_ > convergence_criteria_.diverged_max_covariance_diag;
        const bool innovation_diverging =  // 测量残差过大（NIS 爆炸）
            normalized_innovation_squared_ > convergence_criteria_.diverged_nis;
        if (covariance_diverging || innovation_diverging) {
            register_divergence_sample_();  // 记录一次发散
            return;
        }

        // 正常情况：清除发散计数
        consecutive_diverged_updates_ = 0;

        // 检查收敛条件
        const bool covariance_converged =  // 协方差足够小
            max_covariance_diag_ <= convergence_criteria_.converged_max_covariance_diag;
        const bool innovation_converged =  // NIS 足够小
            normalized_innovation_squared_ <= convergence_criteria_.converged_nis;
        if (covariance_converged && innovation_converged) {
            ++consecutive_converged_updates_;  // 连续收敛次数+1
            convergence_status_ = (consecutive_converged_updates_
                                   >= std::max(1, convergence_criteria_.converged_updates))  // 达到连续次数阈值
                                    ? ConvergenceStatus::Converged  // 判定完全收敛
                                    : ConvergenceStatus::Converging; // 还没到次数，继续收敛中
            return;
        }

        // 未满足收敛条件：Converging状态
        consecutive_converged_updates_ = 0;  // 收敛计数清零
        convergence_status_            = ConvergenceStatus::Converging; // 维持在收敛中
    }

    /// @brief 记录发散样本（用于连续性检测）
    void register_divergence_sample_() noexcept {  // 多记一次发散警告
        consecutive_converged_updates_ = 0;  // 收敛计数清零
        ++consecutive_diverged_updates_;     // 发散计数+1
        convergence_status_ =
            (consecutive_diverged_updates_ >= std::max(1, convergence_criteria_.diverged_updates)) // 达到阈值
                ? ConvergenceStatus::Diverging  // 判定发散
                : ConvergenceStatus::Converging; // 还没达到阈值，暂时还在收敛中（观察）
    }

    /// @brief 立即标记为发散（用于严重数值异常）
    void mark_immediate_divergence_() noexcept {  // 直接宣判死刑
        consecutive_converged_updates_ = 0;  // 放弃收敛
        consecutive_diverged_updates_  = std::max(1, convergence_criteria_.diverged_updates); // 直接塞满发散计数
        convergence_status_            = ConvergenceStatus::Diverging; // 一律发散
    }

    /// @brief 更新协方差统计信息（最大对角元素）
    void update_covariance_stat_(const MatrixXX& P) noexcept {  // 计算最大对角元素
        max_covariance_diag_ = max_abs_covariance_diag(P);
    }

    /// @brief 计算协方差矩阵最大绝对对角元素
    static double max_abs_covariance_diag(const MatrixXX& P) noexcept {  // 遍历对角线找最大绝对值
        double max_diag = 0.0;
        for (int i = 0; i < N_X; ++i) {
            const double diag = P(i, i);   // 取当前对角线元素
            if (!std::isfinite(diag)) {    // 出现 NaN/Inf 直接返回无穷大
                return std::numeric_limits<double>::infinity();
            }
            max_diag = std::max(max_diag, std::abs(diag)); // 取最大值
        }
        return max_diag;
    }

    /// @brief 计算协方差矩阵最小对角元素（检测退化）
    static double min_covariance_diag(const MatrixXX& P) noexcept {  // 遍历对角线找最小
        double min_diag = std::numeric_limits<double>::infinity();
        for (int i = 0; i < N_X; ++i) {
            const double diag = P(i, i);
            if (!std::isfinite(diag)) {   // 异常值
                return -std::numeric_limits<double>::infinity();
            }
            min_diag = std::min(min_diag, diag); // 找最小
        }
        return min_diag;
    }

    // ========================================================================
    // 成员变量 - 状态与协方差
    // ========================================================================

    PredicFunc f;         ///< 预测函数（非线性状态转移）
    MatrixXX F;           ///< 预测雅可比矩阵 ∂f/∂x

    MeasureFunc h;        ///< 测量函数（非线性观测模型）
    MatrixZX H;           ///< 测量雅可比矩阵 ∂h/∂x

    UpdateQFunc update_Q; ///< 过程噪声协方差生成函数
    MatrixXX Q;           ///< 过程噪声协方差矩阵

    UpdateRFunc update_R; ///< 测量噪声协方差生成函数
    MatrixZZ R;           ///< 测量噪声协方差矩阵

    MatrixXX P_pri;       ///< 先验协方差矩阵（预测后）
    MatrixXX P_post;      ///< 后验协方差矩阵（更新后）

    MatrixXZ K_;          ///< 卡尔曼增益矩阵


    MatrixX1 x_pri;       ///< 先验状态向量（预测后）
    MatrixX1 x_post;      ///< 后验状态向量（更新后）

    // ========================================================================
    // 收敛性监控成员
    // ========================================================================

    ConvergenceCriteria convergence_criteria_{};                          ///< 收敛判据配置
    ConvergenceStatus convergence_status_{ConvergenceStatus::Unknown};    ///< 当前收敛状态
    int consecutive_converged_updates_{0};                                ///< 连续收敛次数
    int consecutive_diverged_updates_{0};                                 ///< 连续发散次数
    double normalized_innovation_squared_{
        std::numeric_limits<double>::infinity()};                         ///< 归一化新息平方
    double max_covariance_diag_{std::numeric_limits<double>::infinity()}; ///< 最大协方差对角元素
};
