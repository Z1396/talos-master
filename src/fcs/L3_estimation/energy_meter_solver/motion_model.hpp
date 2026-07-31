/**
 * @file motion_model.hpp
 * @brief 能量机关运动模型定义
 *
 * 本文件定义了能量机关的大小符运动模型，用于扩展卡尔曼滤波（EKF）的状态预测和观测。
 * 能量机关是RoboMaster比赛中的关键目标，分为大符和小符两种类型，具有不同的运动特性。
 *
 * 核心算法原理：
 * 1. 大符（BigRuneModel）：采用正弦运动模型，旋转速度呈周期性变化
 *    θ' = dir · [(a/ω)(cos(ωτ) - cos(ω(τ+dt))) + b·dt]
 *    其中a为振幅，ω为角频率，τ为激活时间，b为常数偏移
 * 2. 小符（SmallRuneModel）：采用恒定速度模型，旋转速度固定
 *    θ' = dir · ω_fixed · dt
 *    其中ω_fixed = π/3 rad/s（固定转速）
 *
 * 状态定义：
 * - 大符状态：8维 [XC, YC, ZC, YAW, THETA, TAU, A, W]
 *   包含中心位置(XC,YC,ZC)、偏航角(YAW)、旋转角(THETA)、
 *   累积时间(TAU)、振幅(A)、角频率(W)
 * - 小符状态：5维 [XC, YC, ZC, YAW, THETA]
 *   包含中心位置(XC,YC,ZC)、偏航角(YAW)、旋转角(THETA)
 *
 * 关键技术点：
 * - 使用Ceres::Jet实现自动微分，避免手动推导雅可比矩阵
 * - 距离依赖的测量噪声模型：近距离噪声小，远距离噪声大
 * - 参数物理约束：振幅a ∈ [0.780, 1.045]，角频率ω ∈ [1.884, 2.000]
 */

#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace energy_meter {

/// 小符固定旋转速度（单位：rad/s）
/// 根据比赛规则，小符转速固定为π/3 rad/s，即每秒旋转60度
inline constexpr double FIXED_RUNE_SPEED = std::numbers::pi / 3.0;

/// 能量机关叶片数量
/// 每个能量机关有5个叶片，每个叶片间隔72度
inline constexpr int ARMORS_NUM = 5;

/**
 * @brief 将装甲板ID限制在有效范围内[0, 4]
 *
 * 由于能量机关有5个叶片，ID范围为[0, 4]，此函数确保ID不超过边界
 *
 * @param id 待限制的装甲板ID
 * @return 限制后的装甲板ID，范围在[0, ARMORS_NUM-1]
 */
inline int clamp_armor_id(int id) noexcept { return std::clamp(id, 0, ARMORS_NUM - 1); }

/// 测量向量索引（大小符模型共享）
/// 包含能量机关中心位置、偏航角和装甲板滚转角
enum Measure : uint8_t {
    CENTER_X,    ///< 0 - 能量机关中心X坐标（相机坐标系）
    CENTER_Y,    ///< 1 - 能量机关中心Y坐标（相机坐标系）
    CENTER_Z,    ///< 2 - 能量机关中心Z坐标（相机坐标系）
    RUNE_YAW,    ///< 3 - 能量机关偏航角（绕相机Z轴）
    ARMOR_ROLL,  ///< 4 - 当前激活装甲板滚转角（绕能量机关中心）
    MEASURE_MAX, ///< 测量向量维度（5维）
};

/**
 * @brief 距离依赖的测量噪声参数
 *
 * 测量噪声随距离增加而增大，符合物理直觉：
 * - 近距离：装甲板在图像中较大，测量精度高
 * - 远距离：装甲板在图像中较小，测量精度低
 */
struct MeasurementNoiseParams {
    double R_scale{0.1};      ///< 测量噪声基础缩放因子
    double meas_dist_k{0.28}; ///< 距离噪声系数，控制噪声随距离增长的速率
};

/**
 * @brief 计算距离依赖的测量噪声协方差矩阵R
 *
 * 该函数根据测量值（位置和角度）计算测量噪声协方差矩阵。
 * 噪声模型考虑两个因素：
 * 1. 距离因素：距离越远，噪声越大（使用对数函数建模）
 * 2. 偏航角因素：偏航角越大，噪声越大（侧面测量精度较低）
 *
 * @param z 测量向量，包含中心位置和角度信息
 * @param p 测量噪声参数
 * @return 测量噪声协方差矩阵（5x5对角阵）
 *
 * @note 该函数使用对数函数建模距离依赖噪声，确保近距离时噪声不会过小
 *
 * @warning 对于距离为0或负数的情况，可能出现数值不稳定
 */
[[nodiscard]] inline Eigen::Matrix<double, MEASURE_MAX, MEASURE_MAX> measurement_R(
    const Eigen::Matrix<double, MEASURE_MAX, 1>& z, const MeasurementNoiseParams& p) noexcept {
    const double x     = z[CENTER_X];
    const double y     = z[CENTER_Y];
    const double z_val = z[CENTER_Z];

    // 计算能量机关中心到相机的距离
    const double distance = std::sqrt(x * x + y * y + z_val * z_val);

    // 计算能量机关中心的偏航角（相机坐标系）
    const double yaw = std::atan2(y, x);

    // 计算测量偏航角与实际偏航角的差值
    // 该差值反映了测量方向的侧面程度，侧面测量精度较低
    const double delta_yaw = [&]() {
        double a = std::fmod(yaw - z[RUNE_YAW] + M_PI, 2.0 * M_PI);
        if (a <= 0.0)
            a += 2.0 * M_PI;
        return a - M_PI;
    }();

    // 距离依赖噪声权重（对数函数，确保近距离噪声不会过小）
    const double K      = p.meas_dist_k;
    const double weight = std::log(K * distance + 1.0);

    // 构造对角噪声协方差矩阵
    // X/Y方向：考虑距离和偏航角因素
    // Z方向：仅考虑距离因素（深度测量相对稳定）
    // 角度：固定噪声值（角度测量精度相对稳定）
    Eigen::Matrix<double, MEASURE_MAX, MEASURE_MAX> Rs =
        Eigen::Matrix<double, MEASURE_MAX, MEASURE_MAX>::Zero();
    Rs.diagonal() << p.R_scale * (weight + std::abs(delta_yaw / 5.0 + 0.5)),
        p.R_scale * (weight + std::abs(delta_yaw / 5.0 + 0.5)), p.R_scale * weight,
        (std::log(std::abs(distance) + 1) / 150.0 + 5e-2), 5e-2;
    return Rs;
}

/**
 * @struct BigRuneModel
 * @brief 大符运动模型（正弦旋转模型）
 *
 * 大符采用正弦运动模型，其旋转速度呈周期性变化。
 * 根据比赛规则，大符激活后的旋转速度满足：
 *   θ' = dir · [(a/ω)(cos(ωτ) - cos(ω(τ+dt))) + b·dt]
 * 其中：
 * - dir: 旋转方向（+1为逆时针，-1为顺时针）
 * - a: 振幅参数，范围 [0.780, 1.045]
 * - b: 常数偏移，满足 a + b = 2.090（规则约束）
 * - ω: 角频率参数，范围 [1.884, 2.000]（对应周期约π秒）
 * - τ: 激活后的累积时间
 *
 * 该模型的特点：
 * 1. 旋转速度呈正弦变化，存在加速和减速阶段
 * 2. 参数a和ω需要在线估计，增加跟踪难度
 * 3. 需要累积时间τ来预测当前状态
 *
 * 状态向量（8维）：
 * - [XC, YC, ZC]: 能量机关中心位置（相机坐标系）
 * - YAW: 能量机关偏航角
 * - THETA: 基准装甲板（ID=0）的滚转角
 * - TAU: 激活后的累积时间（秒）
 * - A: 振幅参数a
 * - W: 角频率参数ω
 */
struct BigRuneModel {
    using Scalar            = double;
    static constexpr int NX = 8;           ///< 状态向量维度（8维）
    static constexpr int NZ = MEASURE_MAX; ///< 测量向量维度（5维）

    /// 状态向量索引
    enum State : uint8_t {
        XC,    ///< 0 - 能量机关中心X坐标（相机坐标系）
        YC,    ///< 1 - 能量机关中心Y坐标（相机坐标系）
        ZC,    ///< 2 - 能量机关中心Z坐标（相机坐标系）
        YAW,   ///< 3 - 能量机关偏航角（绕相机Z轴）
        THETA, ///< 4 - 基准装甲板（ID=0）的滚转角
        TAU,   ///< 5 - 激活后的累积时间（秒）
        A,     ///< 6 - 振幅参数a ∈ [0.780, 1.045]
        W,     ///< 7 - 角频率参数ω ∈ [1.884, 2.000]
    };

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;

    /**
     * @brief 大符模型参数
     *
     * 包含过程噪声协方差参数和几何参数。
     * 过程噪声参数控制EKF对新观测的信任程度。
     */
    struct Params {
        Scalar sigma_xy{2.0};          ///< X/Y方向过程噪声标准差（米）
        Scalar sigma_z{1.5};           ///< Z方向过程噪声标准差（米）
        Scalar sigma_yaw{1e-4};        ///< 偏航角过程噪声标准差（弧度）
        Scalar sigma_theta{0.2};       ///< 滚转角过程噪声标准差（弧度）
        Scalar sigma_a{0.10};          ///< 振幅参数a的过程噪声标准差
        Scalar sigma_w{0.15};          ///< 角频率参数ω的过程噪声标准差

        Scalar radius{0.7};            ///< 能量机关半径（装甲板到中心距离，米）
        MeasurementNoiseParams meas{}; ///< 测量噪声参数
    };

    Params params{};                   ///< 模型参数实例

    /// 振幅常数和约束（根据比赛规则）
    /// 规则规定：a + b = 2.090，其中a为振幅，b为常数偏移
    static constexpr double AMPLITUDE_SUM = 2.090;
    static constexpr double A_LOWER       = 0.780; ///< 振幅参数a最小值
    static constexpr double A_UPPER       = 1.045; ///< 振幅参数a最大值
    static constexpr double W_LOWER       = 1.884; ///< 角频率参数ω最小值
    static constexpr double W_UPPER       = 2.000; ///< 角频率参数ω最大值

    /**
     * @brief 状态预测函数（用于EKF预测步骤）
     *
     * 根据当前状态和时间增量预测下一时刻的状态。
     * 该函数使用Ceres::Jet模板，支持自动微分计算雅可比矩阵。
     *
     * @tparam S 标量类型（double或Ceres::Jet<double, NX>）
     * @param x 当前状态向量（NX维）
     * @param dt 时间增量（秒）
     * @param dir 旋转方向（+1为逆时针，-1为顺时针）
     * @param xp 预测状态向量（输出，NX维）
     *
     * @note 该函数假设dt > 0，若dt <= 0则状态不变
     *
     * @warning 该函数使用cos函数，可能存在数值精度问题（当ω·τ很大时）
     */
    template <typename S>
    static void predict_state(const S* x, const S& dt, int dir, S* xp) noexcept {
        using std::cos;

        // 复制所有状态变量（位置和角度不变）
        for (int i = 0; i < NX; ++i) {
            xp[i] = x[i];
        }

        // 提取关键参数
        const S a = x[A];                 // 振幅参数
        const S w = x[W];                 // 角频率参数
        const S b = S(AMPLITUDE_SUM) - a; // 常数偏移（由规则约束）

        // 更新累积时间
        const S t_new = x[TAU] + dt;
        xp[TAU]       = t_new;

        // 计算滚转角增量（正弦运动模型核心公式）
        // delta_theta = (a/ω)·[cos(ω·τ) - cos(ω·(τ+dt))] + b·dt
        const S delta_theta = (a / w) * (cos(w * x[TAU]) - cos(w * t_new)) + b * dt;

        // 应用旋转方向
        xp[THETA] = x[THETA] + S(static_cast<double>(dir)) * delta_theta;
    }

    /**
     * @brief 观测函数（用于EKF更新步骤）
     *
     * 根据当前状态和装甲板ID计算预测的观测值。
     * 观测值包含能量机关中心位置和装甲板角度。
     *
     * @tparam S 标量类型（double或Ceres::Jet<double, NX>）
     * @param x 当前状态向量（NX维）
     * @param armor_id 装甲板ID（0-4）
     * @param z 预测观测向量（输出，NZ维）
     *
     * @note 装甲板ID通过角度偏移计算：angle_offset = 2π/5 * id
     */
    template <typename S>
    static void measure_state(const S* x, int armor_id, S* z) noexcept {
        const int id         = clamp_armor_id(armor_id);
        const S angle_offset = S(2.0 * M_PI / 5.0 * static_cast<double>(id));

        // 中心位置不变
        z[CENTER_X] = x[XC];
        z[CENTER_Y] = x[YC];
        z[CENTER_Z] = x[ZC];
        // 偏航角不变
        z[RUNE_YAW] = x[YAW];
        // 滚转角 = 基准角 + 装甲板偏移
        z[ARMOR_ROLL] = x[THETA] + angle_offset;
    }

    /**
     * @brief 计算预测观测值（double版本）
     *
     * @param x 状态向量
     * @param armor_id 装甲板ID
     * @return 预测观测向量
     */
    [[nodiscard]] VecZ h(const VecX& x, int armor_id) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    /**
     * @brief 计算过程噪声协方差矩阵Q
     *
     * 过程噪声控制EKF对新观测的信任程度。
     * 较大的过程噪声使EKF更信任观测，较小的过程噪声使EKF更信任预测。
     *
     * @param dt 时间增量（秒）
     * @return 过程噪声协方差矩阵（NX×NX对角阵）
     *
     * @note 累积时间TAU的过程噪声设为0（确定性累积）
     */
    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX q = MatXX::Zero();
        if (dt <= 0.0) {
            return q;
        }

        // 对角元素：σ²·dt（连续时间白噪声模型）
        q(XC, XC)       = params.sigma_xy * params.sigma_xy * dt;
        q(YC, YC)       = params.sigma_xy * params.sigma_xy * dt;
        q(ZC, ZC)       = params.sigma_z * params.sigma_z * dt;
        q(YAW, YAW)     = params.sigma_yaw * params.sigma_yaw * dt;
        q(THETA, THETA) = params.sigma_theta * params.sigma_theta * dt;
        q(TAU, TAU)     = 0.0; // 累积时间确定性累积
        q(A, A)         = params.sigma_a * params.sigma_a * dt;
        q(W, W)         = params.sigma_w * params.sigma_w * dt;

        return q;
    }

    /**
     * @brief 计算测量噪声协方差矩阵R
     *
     * 使用距离依赖的噪声模型。
     *
     * @param z 测量向量
     * @return 测量噪声协方差矩阵（NZ×NZ）
     */
    [[nodiscard]] MatZZ R(const VecZ& z) const noexcept { return measurement_R(z, params.meas); }

    /**
     * @brief 获取测量噪声协方差矩阵的对角元素
     *
     * 用于计算归一化新息（normalized innovation）
     *
     * @param z 测量向量
     * @return 测量噪声协方差矩阵的对角元素（NZ维向量）
     */
    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const {
        return R(z).diagonal();
    }
};

/**
 * @struct SmallRuneModel
 * @brief 小符运动模型（恒定速度旋转模型）
 *
 * 小符采用恒定速度旋转模型，其旋转速度固定不变。
 * 根据比赛规则，小符激活后的旋转速度满足：
 *   θ' = dir · ω_fixed · dt
 * 其中：
 * - dir: 旋转方向（+1为逆时针，-1为顺时针）
 * - ω_fixed: 固定旋转速度 = π/3 rad/s（每秒旋转60度）
 *
 * 该模型的特点：
 * 1. 旋转速度恒定，易于预测和跟踪
 * 2. 不需要在线估计旋转参数（a和ω）
 * 3. 状态向量维度较低（5维），计算量小
 *
 * 状态向量（5维）：
 * - [XC, YC, ZC]: 能量机关中心位置（相机坐标系）
 * - YAW: 能量机关偏航角
 * - THETA: 基准装甲板（ID=0）的滚转角
 *
 * 适用场景：
 * - 小符激活后跟踪
 * - 不需要参数估计的简单场景
 */
struct SmallRuneModel {
    using Scalar            = double;
    static constexpr int NX = 5;           ///< 状态向量维度（5维）
    static constexpr int NZ = MEASURE_MAX; ///< 测量向量维度（5维）

    /// 状态向量索引
    enum State : uint8_t {
        XC,    ///< 0 - 能量机关中心X坐标（相机坐标系）
        YC,    ///< 1 - 能量机关中心Y坐标（相机坐标系）
        ZC,    ///< 2 - 能量机关中心Z坐标（相机坐标系）
        YAW,   ///< 3 - 能量机关偏航角（绕相机Z轴）
        THETA, ///< 4 - 基准装甲板（ID=0）的滚转角
    };

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;

    /**
     * @brief 小符模型参数
     *
     * 包含过程噪声协方差参数和几何参数。
     * 小符的过程噪声参数应设置较小，因为旋转速度恒定。
     */
    struct Params {
        /// 过程噪声标准差（小符转速恒定，过程噪声应设置较小）
        /// 参考值：auto_buff v1使用0.001，表示对恒定速度模型的高度信任
        Scalar sigma_xy{0.01};         ///< X/Y方向过程噪声标准差（米）
        Scalar sigma_z{0.01};          ///< Z方向过程噪声标准差（米）
        Scalar sigma_yaw{1e-4};        ///< 偏航角过程噪声标准差（弧度）
        Scalar sigma_theta{0.001};     ///< 滚转角过程噪声标准差（弧度）

        Scalar radius{0.7};            ///< 能量机关半径（装甲板到中心距离，米）
        MeasurementNoiseParams meas{}; ///< 测量噪声参数
    };

    Params params{};                   ///< 模型参数实例

    /**
     * @brief 状态预测函数（用于EKF预测步骤）
     *
     * 根据当前状态和时间增量预测下一时刻的状态。
     * 小符的滚转角按恒定速度增加。
     *
     * @tparam S 标量类型（double或Ceres::Jet<double, NX>）
     * @param x 当前状态向量（NX维）
     * @param dt 时间增量（秒）
     * @param dir 旋转方向（+1为逆时针，-1为顺时针）
     * @param xp 预测状态向量（输出，NX维）
     *
     * @note 该函数假设dt > 0，若dt <= 0则状态不变
     */
    template <typename S>
    static void predict_state(const S* x, const S& dt, int dir, S* xp) noexcept {
        // 复制所有状态变量（位置和角度不变）
        for (int i = 0; i < NX; ++i) {
            xp[i] = x[i];
        }
        // 滚转角按恒定速度增加
        // theta_new = theta + dir · ω_fixed · dt
        xp[THETA] = x[THETA] + S(static_cast<double>(dir)) * S(FIXED_RUNE_SPEED) * dt;
    }

    /**
     * @brief 观测函数（用于EKF更新步骤）
     *
     * 根据当前状态和装甲板ID计算预测的观测值。
     * 观测值包含能量机关中心位置和装甲板角度。
     *
     * @tparam S 标量类型（double或Ceres::Jet<double, NX>）
     * @param x 当前状态向量（NX维）
     * @param armor_id 装甲板ID（0-4）
     * @param z 预测观测向量（输出，NZ维）
     *
     * @note 装甲板ID通过角度偏移计算：angle_offset = 2π/5 * id
     */
    template <typename S>
    static void measure_state(const S* x, int armor_id, S* z) noexcept {
        const int id         = clamp_armor_id(armor_id);
        const S angle_offset = S(2.0 * M_PI / 5.0 * static_cast<double>(id));

        // 中心位置不变
        z[CENTER_X] = x[XC];
        z[CENTER_Y] = x[YC];
        z[CENTER_Z] = x[ZC];
        // 偏航角不变
        z[RUNE_YAW] = x[YAW];
        // 滚转角 = 基准角 + 装甲板偏移
        z[ARMOR_ROLL] = x[THETA] + angle_offset;
    }

    /**
     * @brief 计算预测观测值（double版本）
     *
     * @param x 状态向量
     * @param armor_id 装甲板ID
     * @return 预测观测向量
     */
    [[nodiscard]] VecZ h(const VecX& x, int armor_id) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    /**
     * @brief 计算过程噪声协方差矩阵Q
     *
     * 小符的过程噪声应设置较小，因为旋转速度恒定。
     *
     * @param dt 时间增量（秒）
     * @return 过程噪声协方差矩阵（NX×NX对角阵）
     */
    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX q = MatXX::Zero();
        if (dt <= 0.0) {
            return q;
        }

        // 对角元素：σ²·dt（连续时间白噪声模型）
        q(XC, XC)       = params.sigma_xy * params.sigma_xy * dt;
        q(YC, YC)       = params.sigma_xy * params.sigma_xy * dt;
        q(ZC, ZC)       = params.sigma_z * params.sigma_z * dt;
        q(YAW, YAW)     = params.sigma_yaw * params.sigma_yaw * dt;
        q(THETA, THETA) = params.sigma_theta * params.sigma_theta * dt;

        return q;
    }

    /**
     * @brief 计算测量噪声协方差矩阵R
     *
     * 使用距离依赖的噪声模型。
     *
     * @param z 测量向量
     * @return 测量噪声协方差矩阵（NZ×NZ）
     */
    [[nodiscard]] MatZZ R(const VecZ& z) const noexcept { return measurement_R(z, params.meas); }

    /**
     * @brief 获取测量噪声协方差矩阵的对角元素
     *
     * 用于计算归一化新息（normalized innovation）
     *
     * @param z 测量向量
     * @return 测量噪声协方差矩阵的对角元素（NZ维向量）
     */
    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const {
        return R(z).diagonal();
    }
};

/**
 * @brief 计算大符的常数偏移参数b
 *
 * 根据比赛规则，大符的振幅参数a和常数偏移b满足：
 *   a + b = 2.090
 * 该函数根据振幅a计算对应的偏移b。
 *
 * @param a 振幅参数（范围[0.780, 1.045]）
 * @return 常数偏移参数b = 2.090 - a
 *
 * @note 该约束来自比赛规则，确保大符的运动特性符合规则要求
 */
inline double to_b(double a) { return BigRuneModel::AMPLITUDE_SUM - a; }

} // namespace energy_meter
