/**
 * @file ldm_kinematic_model.hpp
 * @brief LDM（Landing Device Marker）运动学模型 - 基于SE2(3)群的Invariant EKF实现
 *
 * ## 核心算法原理
 *
 * ### 1. SE2(3)群（扩展特殊欧氏群）
 * SE2(3)是SO(3)旋转群与位置、速度的半直积，用于统一表达姿态-速度-位置状态：
 *   - 旋转R ∈ SO(3)：世界坐标系到物体坐标系的变换
 *   - 物体系速度v_body：在物体坐标系下的速度矢量
 *   - 世界系位置p：在世界坐标系下的位置矢量
 *
 * 状态表示为X = (R, v_body, p)，通过李代数右扰动进行状态传播：
 *   X̂₊ = X̂ * exp(ξ)，其中 ξ = (dθ, dv, dp) ∈ se2(3)
 *
 * ### 2. C8对称性处理（八面棱镜的旋转对称群）
 * LDM标志是正八面棱柱，具有C8旋转对称性（绕Y轴旋转n·45°等效）：
 *   - 视觉测量存在8重姿态歧义：R_obs 可跳变n·45°
 *   - 等价类：[R_obs] = {R_obs * Ry(n·π/4) | n=0..7} ⊂ SO(3)/C8
 *   - 使用nearest_lift算法从商空间SO(3)/C8提升回SO(3)
 *
 * ### 3. 最近提升（Nearest-Lift）算法
 * 从8个候选对称代表中选择最接近预测姿态的那个：
 *   R_canon = R_obs * S*
 *   S* = argmin_{S∈C8} ||log(R_refᵀ * R_obs * S)||₂
 *
 * 算法保证：
 *   - 当帧间角位移 << 22.5°时，提升唯一且稳定
 *   - 返回分支置信度（branch_confidence）用于自适应噪声调节
 *
 * ### 4. 观测模型
 * - 旋转残差：在物体坐标系下计算SO(3)李代数残差
 *   - ROT_X/Z：俯仰/横滚，无歧义（由中心轴方向唯一确定）
 *   - ROT_Y：偏航角，存在C8歧义，通过8倍相位投影消除歧义
 * - 方位残差：方位角、俯仰角、距离
 *
 * ### 5. 数值稳定性与潜在风险
 * - 李代数log映射在姿态接近时稳定，大角度时需谨慎
 * - nearest_lift假设帧间运动小，高速翻滚场景可能失效
 * - C8分支边界（22.5°附近）需自适应增大观测噪声
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include "L3_estimation/tracker/util.hpp"
#include "ldm_kinematic_params.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <groups/SEn3.hpp>
#include <limits>
#include <numbers>

namespace fcs::L3::ldm {

/**
 * @brief 位姿更新分量索引
 *
 * 定义了EKF观测向量的6维结构：
 * - ROT_X/Z：物体系X/Z轴旋转残差（俯仰/横滚）
 * - ROT_Y：物体系Y轴旋转残差（偏航），存在C8歧义
 * - BEARING_YAW/PITCH/DISTANCE：方位角、俯仰角、距离残差
 */
enum LdmPoseUpdate : uint8_t {
    ROT_X,            ///< 物体系X轴旋转残差（俯仰角）
    ROT_Y,            ///< 物体系Y轴旋转残差（偏航角，存在C8歧义）
    ROT_Z,            ///< 物体系Z轴旋转残差（横滚角）
    BEARING_YAW,      ///< 方位角残差（球坐标系）
    BEARING_PITCH,    ///< 俯仰角残差（球坐标系）
    BEARING_DISTANCE, ///< 距离残差（球坐标系）
    POSE_UPDATE_MAX   ///< 观测向量维度（=6）
};

/**
 * @brief LDM运动学模型 - Invariant EKF的核心实现
 *
 * 该结构体封装了LDM标志的完整运动学模型，包括：
 * - 状态表示：SE2(3)群元素（姿态、速度、位置）
 * - 过程模型：匀速运动假设下的状态预测
 * - 观测模型：位姿测量残差计算
 * - 噪声模型：过程噪声Q和观测噪声R
 *
 * ## 状态空间结构（SE2(3)）
 * 状态维度 NX = 9：
 *   - dθ (3维)：李代数旋转扰动
 *   - dv (3维)：物体系速度扰动
 *   - dp (3维)：世界系位置扰动
 *
 * ## 观测空间结构
 * 观测维度 NZ = 6：
 *   - 旋转残差 (3维)：物体系下SO(3)李代数残差
 *   - 方位残差 (3维)：方位角、俯仰角、距离
 */
struct LdmKinematic {
    using Scalar            = double;                         ///< 标量类型（双精度）
    static constexpr int NZ = POSE_UPDATE_MAX;                ///< 观测向量维度（=6）

    using Nominal           = group::SEn3<Scalar, 2>;         ///< SE2(3)群类型（SEn3<double, 2>）
    using Xi                = Nominal::VectorType;            ///< 李代数向量类型（9维）
    using CovXi             = Nominal::TMatrixType;           ///< 状态协方差矩阵类型（9×9）
    static constexpr int NX = Xi::RowsAtCompileTime;          ///< 状态向量维度（=9）

    using Innovation         = Eigen::Matrix<Scalar, NZ, 1>;  ///< 观测残差向量类型（6维）
    using CovInnovation      = Eigen::Matrix<Scalar, NZ, NZ>; ///< 观测协方差矩阵类型（6×6）
    using InnovationJacobian = Eigen::Matrix<Scalar, NZ, NX>; ///< 观测雅可比矩阵类型（6×9）

    /// Re-exported from ldm_kinematic_params.hpp for backward compatibility.
    using Params = LdmKinematicParams;

    /**
     * @brief C8对称群 - LDM八面棱柱的旋转对称性
     *
     * LDM是正八面棱柱，其位姿观测存在C₈面索引歧义：
     *   - 绕物体系Y轴旋转n·45°（n=0..7）产生等效的面分配
     *   - 视觉测量不是单一旋转R∈SO(3)，而是等价类[R_obs]
     *
     * 数学表示：
     *   [R_obs] = {R_obs * Ry(n·π/4) | n=0..7} ⊂ SO(3)/C₈
     *
     * ## 物理解释
     * 相机能看到棱柱中心轴指向，但"哪个面是面0"存在n·45°歧义：
     *   R ≈ (中心轴方向, 绕轴旋转角)
     *   绕轴旋转角 ≈ 绕轴旋转角 + n·π/4
     *
     * ## 为什么不是D₈或八面体群O？
     * - D₈包含反射（det=-1），不属于SO(3)，无法表示旋转
     * - 棱柱有唯一中心轴（body Y），其他两轴由面法向定义，存在C₈歧义
     * - 与八面体不同（无特殊轴，24个手性对称）
     *
     * @note 只有旋转子群C₈⊂SO(3)对位姿滤波有效
     */
    static constexpr int kSymmetryCount = 8; ///< C₈对称群元素数量

    /**
     * @brief 获取C8对称群的8个旋转矩阵
     *
     * 生成绕Y轴的8个等距旋转：
     *   S[n] = Ry(n·π/4), n=0..7
     *
     * @return 8个旋转矩阵的数组（静态存储，编译期初始化）
     *
     * @note 使用静态局部变量实现懒加载，避免重复计算
     */
    static const std::array<Eigen::Matrix3d, kSymmetryCount>& symmetry_rotations() noexcept {
        constexpr double kPi4  = std::numbers::pi_v<double> / 4.0;
        static const auto kSym = [] {
            std::array<Eigen::Matrix3d, kSymmetryCount> sym{};
            for (int n = 0; n < kSymmetryCount; ++n) {
                sym[static_cast<size_t>(n)] =
                    Eigen::AngleAxisd(static_cast<double>(n) * kPi4, Eigen::Vector3d::UnitY())
                        .toRotationMatrix();
            }
            return sym;
        }();
        return kSym;
    }

    /**
     * @brief 最近提升算法 - 从SO(3)/C₈商空间提升回SO(3)
     *
     * 核心思想：通过选择最接近预测姿态的C₈代表，将商空间[R_obs]∈SO(3)/C₈
     * 提升回连续的SO(3)轨迹。
     *
     * ## 动机
     * 八面棱柱的8个面完全相同 → PnP面分配任意，R_obs可在帧间跳变n·45°。
     * 与其维护8个EKF分支，我们选择最接近R_ref的代表，利用时间连续性。
     *
     * ## 数学原理
     * R_canon = R_obs · S*
     * S* = argmin_{S∈C₈} ||log(R_refᵀ · R_obs · S)||₂
     *
     * 这不是恢复绝对面索引，而是局部提升：
     *   - 从SO(3)/C₈商空间 → SO(3)全空间
     *   - 使用时间连续性选择代表
     *
     * ## 算法保证
     * 当帧间角位移 << 22.5°（C₈分支安全半径）时，提升唯一且稳定。
     * 这在典型无人机非翻滚运动下成立。
     *
     * @param R_obs 观测旋转矩阵（来自PnP）
     * @param R_ref 参考旋转矩阵（滤波器预测）
     * @return LiftResult 包含：
     *         - R_canon：提升后的规范旋转矩阵
     *         - branch_confidence：分支置信度（最佳与次佳的测地距离差）
     *
     * ## 分支置信度解释
     * - 大（>几度）→ 无歧义提升，滤波器使用基准噪声
     * - 小（≈0）→ 靠近C₈分支边界，滤波器应增大旋转噪声（见R()）
     *
     * @warning 假设帧间运动小，高速翻滚场景可能失效
     * @note 这是局部提升，不保证跨序列的绝对面标签一致性
     */
    struct LiftResult {
        Eigen::Matrix3d R_canon;  ///< 提升后的规范旋转矩阵
        double branch_confidence; ///< 分支置信度（最佳与次佳的测地距离差）
    };

    /**
     * @brief 执行最近提升算法
     *
     * 遍历C₈对称群的8个代表，选择最接近参考姿态的那个。
     *
     * @param R_obs 观测旋转矩阵（来自视觉PnP）
     * @param R_ref 参考旋转矩阵（EKF预测）
     * @return LiftResult 包含规范旋转和分支置信度
     *
     * ## 算法步骤
     * 1. 对每个对称S ∈ C₈，计算候选R_candidate = R_obs * S
     * 2. 计算候选与参考的测地距离 d = ||log(R_refᵀ * R_candidate)||₂
     * 3. 选择最小d对应的R_canon，并记录最佳和次佳距离
     * 4. 返回R_canon和置信度（次佳 - 最佳）
     *
     * @note 使用SO(3)李代数log映射，在姿态接近时数值稳定
     */
    [[nodiscard]] static LiftResult
        nearest_lift(const Eigen::Matrix3d& R_obs, const Eigen::Matrix3d& R_ref) noexcept {
        using SO3 = typename Nominal::SO3Type;

        const auto& sym = symmetry_rotations();

        // 初始化最佳候选为R_obs本身（n=0情况）
        Eigen::Matrix3d R_best = R_obs;
        double best_d          = std::numeric_limits<double>::infinity();
        double second_d        = std::numeric_limits<double>::infinity();

        // 遍历8个对称代表，寻找最接近R_ref的那个
        for (const auto& S : sym) {
            const Eigen::Matrix3d R_candidate = R_obs * S;
            const Eigen::Matrix3d R_err       = R_ref.transpose() * R_candidate;
            const Eigen::Vector3d err         = SO3::log(R_err); // SO(3)李代数残差
            const double d                    = err.norm();      // 测地距离

            // 维护最佳和次佳距离（用于计算置信度）
            if (d < best_d) {
                second_d = best_d;
                best_d   = d;
                R_best   = R_candidate;
            } else if (d < second_d) {
                second_d = d;
            }
        }

        // 计算置信度：次佳距离 - 最佳距离
        // 大值表示R_obs远离分支边界，提升无歧义
        const double confidence = (second_d < std::numeric_limits<double>::infinity())
                                    ? (second_d - best_d)
                                    : std::numeric_limits<double>::infinity();

        return {R_best, confidence};
    }

    /**
     * @brief 位姿测量数据结构
     *
     * 封装视觉PnP算法输出的位姿观测：
     * - 旋转矩阵：世界坐标系到物体坐标系的变换
     * - 位置向量：世界坐标系下的位置
     * - 分支置信度：C₈歧义度量（由nearest_lift填充）
     */
    struct PoseMeasurement {
        Eigen::Matrix3d R_world_body{Eigen::Matrix3d::Identity()}; ///< 世界→物体旋转矩阵
        Eigen::Vector3d p_world_body{Eigen::Vector3d::Zero()};     ///< 世界系位置向量

        /// Branch ambiguity from the last nearest-lift, populated by the
        /// tracker before calling pose_innovation.  Large → unambiguous.
        double branch_confidence{std::numeric_limits<double>::infinity()}; ///< C₈分支置信度
    };

    Params params{}; ///< 模型参数（噪声、初始方差等）

    /**
     * @brief 状态预测 - SE2(3)群上的右扰动传播
     *
     * 基于匀速运动假设，通过SE2(3)李代数右扰动进行状态预测：
     *   X̂₊ = X̂ * exp(ξ)，其中 ξ = (dθ=0, dv=0, dp=v_body·dt)
     *
     * ## 运动模型
     * - 假设：物体系速度恒定（匀速模型）
     * - 状态转移：
     *   - 旋转：R不变（无角速度）
     *   - 速度：v_body不变
     *   - 位置：p₊ = p + R * v_body * dt
     *
     * ## SE2(3)群结构（SEn3<double, 2>）
     * 存储[rotation, body-frame velocity, world position]
     * 右乘dp项积分 p_dot = R * v_body
     *
     * @param X 当前SE2(3)状态（旋转、速度、位置）
     * @param dt 时间步长（秒）
     * @return 预测后的SE2(3)状态
     *
     * @note 这是Invariant EKF的预测步骤，在群上运算保持结构性质
     */
    static Nominal predict_state(const Nominal& X, Scalar dt) noexcept {
        if (dt <= Scalar(0)) {
            return X;
        }

        const auto v = X.v(); // 获取物体系速度

        Xi xi = Xi::Zero();

        // SEn3<*,2> stores [rotation, body-frame velocity, world position].
        // Right-multiplying dp integrates p_dot = R * v_body.
        // 只更新位置分量（dp = v * dt），旋转和速度保持不变
        xi.template segment<3>(6).noalias() = v * dt;

        return X * Nominal::exp(xi); // 群乘法：右扰动
    }

    /**
     * @brief 状态转移函数（供Invariant EKF使用）
     *
     * 封装predict_state，提供函数式接口。
     *
     * @param x 当前状态
     * @param dt 时间步长
     * @return 预测状态
     */
    [[nodiscard]] Nominal f(const Nominal& x, double dt) const noexcept {
        return predict_state(x, dt);
    }

    /**
     * @brief 位姿观测残差计算 - 核心观测模型
     *
     * 计算预测状态与观测位姿之间的残差向量（6维）。
     *
     * ## 残差结构
     * 1. **旋转残差（3维）**：物体系下SO(3)李代数残差
     *    - ROT_X/Z：俯仰/横滚，无C₈歧义（由中心轴方向确定）
     *    - ROT_Y：偏航角，存在C₈歧义，通过8倍相位投影消除
     *
     * 2. **方位残差（3维）**：球坐标系下的方位角、俯仰角、距离
     *
     * ## C₈歧义处理 - 8倍相位投影
     * 棱柱的8个面相同 → 原始偏航误差Δφ存在n·45°歧义。
     * 不选择离散面标签，而是投影通过8倍相位：
     *   δφ = (1/8) · atan2(sin(8·Δφ), cos(8·Δφ))
     *
     * 该投影自然落在[-22.5°, 22.5°]，无分支边界。
     * 前提：R_pred与R_obs轴对齐已接近（|Δφ| << 22.5°），
     * 这是EKF线性化成立的条件，与nearest_lift相同。
     *
     * @param predicted EKF预测状态（SE2(3)群元素）
     * @param observed 观测位姿（旋转、位置、分支置信度）
     * @return Innovation 6维残差向量 [ROT_X, ROT_Y, ROT_Z, BEARING_YAW, BEARING_PITCH,
     * BEARING_DISTANCE]
     *
     * @warning 假设轴对齐接近，大角度误差下线性化失效
     * @note 分支置信度已通过nearest_lift预先计算，用于自适应噪声调节
     */
    [[nodiscard]] static Innovation
        pose_innovation(const Nominal& predicted, const PoseMeasurement& observed) noexcept {
        using SO3 = typename Nominal::SO3Type;

        Innovation nu = Innovation::Zero();

        // SO(3) residual in body frame.
        // 计算物体系下的SO(3)残差
        const Eigen::Matrix3d R_pred  = predicted.R();
        const Eigen::Matrix3d dR_mat  = R_pred.transpose() * observed.R_world_body;
        const Eigen::Vector3d rot_err = SO3::log(SO3(dR_mat)); // 李代数残差

        // ROT_X and ROT_Z (pitch/roll, observable directions) — raw SO(3) residual.
        // The prism's central axis is body Y; pitch/roll around X/Z are determined
        // by the camera view of the axis direction and have no C₈ ambiguity.
        // This component-wise split of the SO(3) log into ROT_X, ROT_Y, ROT_Z
        // is a local approximation near the predicted attitude.  It is valid
        // when axis alignment is close (|Δφ| << 22.5°), which holds under the
        // same safety condition as nearest_lift.
        // 棱柱中心轴是body Y；X/Z轴俯仰/横滚由轴方向唯一确定，无C₈歧义
        nu[ROT_X] = rot_err.x();
        nu[ROT_Z] = rot_err.z();

        // ROT_Y (roll around the prism's central axis) — C₈ latent residual.
        //
        // The 8 faces are identical → the raw yaw error Δφ has an n·45° ambiguity.
        // Instead of picking a face label (discrete branch), we project through
        // the 8-fold phase, which is C₈-invariant and continuous:
        //
        //     δφ = (1/8) · atan2(sin(8·Δφ), cos(8·Δφ))
        //
        // This naturally lands in [-22.5°, 22.5°] with no branch boundary.
        // The R_pred vs R_obs axis alignment must already be close (|Δφ| << 22.5°)
        // for the EKF linearization to hold — this is the same safety condition as
        // the nearest-lift approach, but without the explicit face label selection.
        // C₈歧义处理：8倍相位投影，消除偏航角的n·45°歧义
        nu[ROT_Y] = std::atan2(std::sin(8.0 * rot_err.y()), std::cos(8.0 * rot_err.y())) / 8.0;

        // ── Bearing innovation (unchanged) ──
        // 方位残差：球坐标系下的方位角、俯仰角、距离
        const Eigen::Vector3d predicted_ypd = fcs::L3::xyz2ypd(predicted.p());
        const Eigen::Vector3d observed_ypd  = fcs::L3::xyz2ypd(observed.p_world_body);
        nu[BEARING_YAW]      = fcs::L3::shortest_rad(predicted_ypd.x(), observed_ypd.x());
        nu[BEARING_PITCH]    = fcs::L3::shortest_rad(predicted_ypd.y(), observed_ypd.y());
        nu[BEARING_DISTANCE] = observed_ypd.z() - predicted_ypd.z();
        return nu;
    }

    /**
     * @brief 观测雅可比矩阵计算
     *
     * 计算残差关于状态扰动的雅可比矩阵（6×9维）。
     *
     * ## 雅可比结构
     * H = [∂(rotation_innovation)/∂dθ | 0 | 0]
     *     [0 | 0 | ∂(bearing_innovation)/∂dp]
     *
     * - 旋转残差对姿态扰动的雅可比：单位矩阵（在零残差附近）
     * - 方位残差对位置扰动的雅可比：球坐标变换的雅可比 × 旋转矩阵
     *
     * @param predicted 预测状态（SE2(3)群元素）
     * @return InnovationJacobian 观测雅可比矩阵（6×9）
     *
     * @note 假设残差小，使用一阶近似
     */
    [[nodiscard]] static InnovationJacobian pose_update_H(const Nominal& predicted) noexcept {
        InnovationJacobian H = InnovationJacobian::Zero();
        // 旋转残差对姿态扰动的雅可比：单位矩阵（局部线性化）
        H.template block<3, 3>(ROT_X, 0).setIdentity();
        // 方位残差对位置扰动的雅可比：球坐标雅可比 × 旋转矩阵
        H.template block<3, 3>(BEARING_YAW, 6).noalias() =
            fcs::L3::xyz2ypd_jacobian(predicted.p()) * predicted.R();
        return H;
    }

    /**
     * @brief 过程噪声协方差矩阵计算
     *
     * 基于匀速运动假设，构造过程噪声协方差矩阵Q（9×9维）。
     *
     * ## 噪声模型
     * - 角速度噪声：ω_noise（影响姿态）
     * - 加速度噪声：a_noise（影响速度和位置）
     *
     * ## 协方差结构
     * - dθ噪声：Var(ω) · dt²（角速度积分）
     * - dv噪声：Var(a) · dt²（加速度积分）
     * - dp噪声：Var(a) · dt⁴ / 4（位置二重积分）
     * - dv-dp相关：Var(a) · dt³ / 2（速度-位置耦合）
     *
     * @param dt 时间步长（秒）
     * @return CovXi 过程噪声协方差矩阵（9×9）
     *
     * @note 噪声参数来自LdmKinematicParams
     */
    [[nodiscard]] CovXi Q(double dt) const noexcept {
        CovXi Q = CovXi::Zero();
        if (dt <= Scalar(0)) {
            return Q;
        }

        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

        const Mat3 I3 = Mat3::Identity();

        const Scalar dt2 = dt * dt;
        const Scalar dt3 = dt2 * dt;
        const Scalar dt4 = dt2 * dt2;

        const Scalar omega_var = params.sigma_inert_omega * params.sigma_inert_omega; // (rad/s)^2
        const Scalar accel_var = params.sigma_inert_accel * params.sigma_inert_accel; // (m/s^2)^2

        // Xi = [dtheta, dv, dp]

        // dtheta_noise = omega_noise * dt
        // 角度噪声：角速度噪声的时间积分
        Q.template block<3, 3>(0, 0) = omega_var * dt2 * I3; // rad^2

        // dv_noise = accel_noise * dt
        // 速度噪声：加速度噪声的时间积分
        Q.template block<3, 3>(3, 3) = accel_var * dt2 * I3; // (m/s)^2

        // dp_noise = 0.5 * accel_noise * dt^2
        // 位置噪声：加速度噪声的二重积分
        Q.template block<3, 3>(6, 6) = Scalar(0.25) * accel_var * dt4 * I3; // m^2

        // Cov(dv, dp)
        // 速度-位置相关噪声
        Q.template block<3, 3>(3, 6) = Scalar(0.5) * accel_var * dt3 * I3; // m^2/s

        Q.template block<3, 3>(6, 3) = Q.template block<3, 3>(3, 6).transpose();

        return Q;
    }

    /**
     * @brief 观测噪声协方差矩阵计算 - 自适应噪声调节
     *
     * 构造观测噪声协方差矩阵R（6×6维），并根据C₈分支置信度自适应调节。
     *
     * ## 噪声结构
     * - 旋转噪声（ROT_X/Y/Z）：基准值 + C₈边界自适应调节
     * - 方位噪声（BEARING_YAW/PITCH）：基准值
     * - 距离噪声（BEARING_DISTANCE）：基准值 + 深度/离轴距离相关
     *
     * ## 距离噪声模型
     * σ_distance = σ_min + k_depth · depth + k_planar · planar_offset
     * - 深度分量：距离越远，PnP不确定性增加
     * - 离轴分量：偏离视野中心，测量精度下降
     *
     * ## C₈边界自适应调节
     * 当branch_confidence < 15°（靠近C₈分支边界）时，线性增大旋转噪声：
     *   factor = 1 + 9 * (1 - confidence / 15°)
     *   - confidence = 15° → factor = 1（基准噪声）
     *   - confidence = 0° → factor = 10（噪声放大10倍）
     *
     * 这防止滤波器在分支边界附近过度信任错误的面标签。
     *
     * @param z 观测位姿（包含分支置信度）
     * @return CovInnovation 观测噪声协方差矩阵（6×6）
     *
     * @warning 自适应调节假设branch_confidence已通过nearest_lift正确计算
     * @note 距离模型参数来自LdmKinematicParams
     */
    [[nodiscard]] CovInnovation R(const PoseMeasurement& z) const noexcept {
        using std::abs;
        using std::cos;
        using std::sin;
        using std::sqrt;

        Eigen::Matrix<Scalar, NZ, 1> diag;
        diag.setZero();

        const Eigen::Vector3d ypd = fcs::L3::xyz2ypd(z.p_world_body);

        const Scalar yaw      = ypd.x();
        const Scalar pitch    = ypd.y();
        const Scalar distance = abs(ypd.z());

        // Bearing convention:
        //
        // x = d cos(pitch) cos(yaw)
        // y = d cos(pitch) sin(yaw)
        // z = -d sin(pitch)
        //
        // depth: forward component
        // planar_offset: off-axis component
        // 球坐标变换：计算深度（前向分量）和离轴距离
        const Scalar cp = cos(pitch);
        const Scalar sp = sin(pitch);
        const Scalar cy = cos(yaw);
        const Scalar sy = sin(yaw);

        const Scalar depth = abs(distance * cp * cy);                               // 深度分量

        const Scalar lateral       = distance * cp * sy;
        const Scalar vertical      = -distance * sp;
        const Scalar planar_offset = sqrt(lateral * lateral + vertical * vertical); // 离轴分量

        // 距离噪声：基准值 + 深度相关 + 离轴相关
        const Scalar sigma_distance = params.sigma_distance_min + params.k_distance_depth * depth
                                    + params.k_distance_planar * planar_offset;

        // 基准噪声方差
        diag[ROT_X] = params.sigma_rot_x * params.sigma_rot_x;

        diag[ROT_Y] = params.sigma_rot_y * params.sigma_rot_y;

        diag[ROT_Z] = params.sigma_rot_z * params.sigma_rot_z;

        diag[BEARING_YAW] = params.sigma_r_bearing_yaw * params.sigma_r_bearing_yaw;

        diag[BEARING_PITCH] = params.sigma_r_bearing_pitch * params.sigma_r_bearing_pitch;

        diag[BEARING_DISTANCE] = sigma_distance * sigma_distance;

        // Inflate rotation noise when near the C₈ branch boundary.
        // (branch_confidence small → symmetry boundary is near).
        // C₈边界自适应调节：当靠近分支边界时增大旋转噪声
        if (z.branch_confidence < std::numbers::pi_v<double> / 12.0) { // < 15°
            // Linearly scale noise from baseline (confidence=15°) → 10× (confidence=0)
            // 线性缩放：confidence=15°时为基准，confidence=0°时放大10倍
            const double factor =
                1.0
                + 9.0
                      * std::clamp(
                          1.0 - z.branch_confidence / (std::numbers::pi_v<double> / 12.0), 0.0,
                          1.0);
            diag[ROT_X] *= factor;
            diag[ROT_Y] *= factor;
            diag[ROT_Z] *= factor;
        }

        CovInnovation R = CovInnovation::Zero();
        R.diagonal()    = diag; // 对角矩阵（假设各分量独立）
        return R;
    }
};

} // namespace fcs::L3::ldm
