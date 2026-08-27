// ===========================================================================
// motion_model.hpp - EKF 运动模型（机器人 / 前哨站）
// 照搬来源: src/fcs/L3_estimation/tracker/new_motion_model.hpp（逐字复制）
//
// 唯一改动：#include "util.hpp" → #include "armor/model_types.hpp"
// （本教学项目只照搬了 util.hpp 中运动模型需要的枚举和角度工具）
// 其余内容（predict_state / measure_state / Q / R / Params）与真实项目一字不差，
// 读本项目 = 读真实项目。
// ===========================================================================

#pragma once
// 头文件保护，防止多重包含引发重复定义

#include "armor/model_types.hpp"  // 状态/观测索引枚举、shortest_rad 角度工具（照搬自 util.hpp）
#include <Eigen/Core>      // Eigen线性代数库，矩阵、向量运算（滤波核心）
#include <algorithm>       // std::clamp 数值限幅
#include <cmath>           // 数学函数 atan2、cos、sin、hypot、exp、log
#include <numbers>         // C++20 标准圆周率 std::numbers::pi
#include <optional>

namespace fcs::L3 {

/**
 * @brief 约束运动模型必须满足的接口契约 Concept（C++20）
 * 所有能送入 SRUKF / ISRCKF 平方根卡尔曼滤波器的运动模型都必须满足该约束
 * @tparam M 运动模型类型（RobotEkfMotionModel / OutpostEkfMotionModel）
 * requires 内部罗列滤波器调用模型时全部需要的类型、常量、成员函数
 */
template <typename M>
concept MotionModel = requires(M m, typename M::VecX x, typename M::Scalar dt, int id) {
    // 内部必须定义标量类型
    typename M::Scalar;
    // 状态向量类型
    typename M::VecX;
    // 观测向量类型
    typename M::VecZ;

    // 编译期常量：状态维度、观测维度、装甲数量
    { M::NX } -> std::convertible_to<int>;
    { M::NZ } -> std::convertible_to<int>;
    { M::ARMORS_NUM } -> std::convertible_to<int>;

    // 状态转移函数 f(x,dt)：预测下一时刻状态 x' = f(x,dt)
    { m.f(x, dt) } -> std::same_as<typename M::VecX>;
    // 观测方程 h(x) 无装甲id（重载预留）
    { m.h(x) } -> std::same_as<typename M::VecZ>;
    // 观测方程 h(x,id)：给定状态+装甲编号，生成观测值
    { m.h(x, id) } -> std::same_as<typename M::VecZ>;

    // 过程噪声平方根矩阵（平方根滤波专用）
    { m.Q_sqrt(dt) } -> std::convertible_to<Eigen::Matrix<typename M::Scalar, M::NX, M::NX>>;
    // 观测噪声相关接口
    { m.R_sqrt(std::declval<typename M::VecZ>()) };
    { m.R_diag(std::declval<typename M::VecZ>()) };
};

/**
 * @brief 敌方机器人 EKF 运动模型（步兵/英雄哨兵通用）
 * 状态：机器人中心位置、速度、偏航角速度、装甲半径对数、高度偏移等
 * 采用log域半径，保证半径恒大于0，避免滤波出现物理非法负值
 */
struct RobotEkfMotionModel
{
    using Scalar                       = double;
    static constexpr int NX            = STATE_MAX;       // 状态向量长度
    static constexpr int NZ            = MEASURE_MAX;     // 观测向量长度
    static constexpr int ARMORS_NUM    = 4;               // 机器人4块装甲板
    static constexpr int kHighArmorIdA = 1;
    static constexpr int kHighArmorIdB = 3;               // 1、3号是高地装甲
    static constexpr int kVyawIndex    = V_YAW;
    static constexpr bool kHasLogRadii = true;             // 启用对数半径建模

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;   // 状态列向量
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;   // 观测列向量
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;  // NX×NX 状态协方差矩阵
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;  // NZ×NZ 观测噪声矩阵

    /**
     * @brief 滤波器可调参数结构体，会被FCS TOML反射框架加载
     * 过程噪声、观测噪声基底
     */
    struct Params {
        Scalar sigma_a_xy  = 10.0; // XY平面加速度噪声标准差
        Scalar sigma_a_z   = 1.0;  // Z方向加速度噪声标准差
        Scalar sigma_a_yaw = 1.0;  // 偏航角加速度噪声标准差
        Scalar sigma_r0    = 1.0;  // log(R0) 漂移噪声（0、2号装甲半径）
        Scalar sigma_r1    = 1.0;  // log(R1) 漂移噪声（1、3号高地装甲半径）
        Scalar sigma_h     = 0.5;  // 装甲高度偏移H漂移噪声

        // ========== 观测噪声基底（方差）==========
        Scalar meas_yaw_var_floor   = 4e-3;    // 观测水平角度最小方差下限
        Scalar meas_pitch_var_floor = 4e-3;    // 观测俯仰角最小方差下限

        // 距离观测噪声（log距离空间）
        // PnP解算带来的几何噪声在L2层额外叠加，不在此模型内
        Scalar meas_dist_var_floor      = 2.5e-3;
        Scalar meas_dist_delta_angle_k  = 1.0;

        Scalar meas_armor_yaw_var_floor = 9e-2; // 装甲自身偏航观测方差下限
        Scalar meas_armor_yaw_range_k   = 1.0 / 200.0; // 噪声随距离增长系数
    };
    Params params{};

    /// 装甲id限幅，防止非法索引
    [[nodiscard]] static int clamp_armor_id(int id) noexcept {
        return std::clamp(id, 0, ARMORS_NUM - 1);
    }

    /**
     * @brief 通用状态预测迭代函数（模板，支持float/double，无Eigen依赖，高效）
     * @tparam T 浮点类型
     * @param x 当前状态
     * @param dt 时间间隔s
     * @param xp 输出预测状态
     * 匀速模型：位置 = 位置 + 速度*dt；速度保持不变
     */
    template <typename T>
    static void predict_state(const T* x, const T& dt, T* xp) noexcept {
        xp[XC]     = x[XC] + x[VX] * dt;    // X坐标
        xp[VX]     = x[VX];                 // X速度（匀速假设）
        xp[YC]     = x[YC] + x[VY] * dt;    // Y坐标
        xp[VY]     = x[VY];                 // Y速度
        xp[Z0]     = x[Z0] + x[VZ] * dt;    // 低装甲Z坐标
        xp[VZ]     = x[VZ];                 // Z速度
        xp[YAW]    = x[YAW] + x[V_YAW] * dt; // 机体偏航角
        xp[V_YAW]  = x[V_YAW];               // 偏航角速度
        xp[LOG_R0] = x[LOG_R0];              // log(R0) 缓变，预测阶段不变
        xp[LOG_R1] = x[LOG_R1];              // log(R1) 缓变
        xp[H]      = x[H];                   // 高低装甲高度差，缓变
    }

    /**
     * @brief 观测方程：由机器人状态 + 装甲编号 算出视觉观测值
     * 输入：机器人中心状态；输出：这块装甲对应的观测(水平角、俯仰角、log距离、装甲偏航)
     */
    template <typename T>
    static void measure_state(const T* x, int armor_id, T* z) noexcept {
        using std::atan2;
        using std::cos;
        using std::exp;
        using std::hypot;
        using std::log;
        using std::sin;

        const int id       = clamp_armor_id(armor_id);
        const bool is_high = (id == kHighArmorIdA || id == kHighArmorIdB);
        const T angle_step = T(2.0 * std::numbers::pi / static_cast<double>(ARMORS_NUM));
        const T yaw        = x[YAW] + T(id) * angle_step; // 当前装甲全局角度

        // log域半径还原真实半径 exp(logR)
        const T radius  = is_high ? exp(x[LOG_R1]) : exp(x[LOG_R0]);
        const T armor_z = is_high ? (x[Z0] + x[H]) : x[Z0];
        // 计算装甲世界坐标
        const T armor_x = x[XC] - radius * cos(yaw);
        const T armor_y = x[YC] - radius * sin(yaw);

        const T horizontal = hypot(armor_x, armor_y);
        const T distance   = hypot(horizontal, armor_z);

        // 填充观测向量
        z[ARMOR_YAW]       = atan2(armor_y, armor_x);        // 装甲相对于相机的水平角
        z[ARMOR_PITCH]     = atan2(-armor_z, horizontal);    // 俯仰角
        z[ARMOR_DISTANCE]  = log(distance + T(1e-9));        // 距离取对数，防止log(0)
        z[ARMOR_YAW_ARMOR] = yaw;                             // 装甲自身朝向
    }

    /// 对外接口：状态转移 f(x,dt)，返回预测状态向量
    [[nodiscard]] VecX f(const VecX& x, double dt) const noexcept {
        VecX xp = VecX::Zero();
        predict_state<double>(x.data(), dt, xp.data());
        return xp;
    }

    /// 对外观测方程：输入状态+装甲id，输出观测向量
    [[nodiscard]] VecZ h(const VecX& x, int armor_id) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    /**
     * @brief 生成过程噪声协方差矩阵Q
     * 匀速模型标准连续噪声离散化
     */
    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX Q = MatXX::Zero();
        if (dt <= 0.0) {
            return Q;
        }

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        const double sigma_xy2  = params.sigma_a_xy * params.sigma_a_xy;
        const double sigma_z2   = params.sigma_a_z * params.sigma_a_z;
        const double sigma_yaw2 = params.sigma_a_yaw * params.sigma_a_yaw;

        // 连续白噪声离散化：位置-速度二维噪声填充
        auto fill_cv = [&](int idx, double q) {
            Q(idx, idx)         = q * dt4 / 4.0;
            Q(idx, idx + 1)     = q * dt3 / 2.0;
            Q(idx + 1, idx)     = q * dt3 / 2.0;
            Q(idx + 1, idx + 1) = q * dt2;
        };

        fill_cv(XC, sigma_xy2);
        fill_cv(YC, sigma_xy2);
        fill_cv(Z0, sigma_z2);
        fill_cv(YAW, sigma_yaw2);

        // 半径、高度为随机游走模型
        Q(LOG_R0, LOG_R0) = params.sigma_r0 * params.sigma_r0 * dt2;
        Q(LOG_R1, LOG_R1) = params.sigma_r1 * params.sigma_r1 * dt2;
        Q(H, H)           = params.sigma_h * params.sigma_h * dt2;
        return Q;
    }

    /**
     * @brief 观测噪声协方差矩阵 R
     * 噪声**和观测值相关**：距离、装甲角度差会动态改变噪声大小
     */
    [[nodiscard]] MatZZ R(const VecZ& z) const noexcept {
        MatZZ R                  = MatZZ::Zero();
        const double delta_angle = shortest_rad(z[ARMOR_YAW], z[ARMOR_YAW_ARMOR]);
        const double distance    = std::exp(std::clamp(z[ARMOR_DISTANCE], -20.0, 20.0));
        R(0, 0)                  = std::max(1e-8, params.meas_yaw_var_floor);
        R(1, 1)                  = std::max(1e-8, params.meas_pitch_var_floor);
        R(2, 2)                  = std::max(
            1e-8, params.meas_dist_var_floor
                      + params.meas_dist_delta_angle_k * std::log(std::abs(delta_angle) + 1.0));
        R(3, 3) = std::max(
            1e-8, params.meas_armor_yaw_var_floor
                      + params.meas_armor_yaw_range_k * std::log(distance + 1.0));
        return R;
    }

    /// 获取观测噪声对角线向量（部分滤波器仅需要对角元素，提升速度）
    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const noexcept {
        const Scalar delta_angle = shortest_rad(z[ARMOR_YAW], z[ARMOR_YAW_ARMOR]);
        const Scalar distance    = std::exp(std::clamp(z[ARMOR_DISTANCE], -20.0, 20.0));
        Eigen::Matrix<Scalar, NZ, 1> R_dig;
        R_dig << params.meas_yaw_var_floor, params.meas_pitch_var_floor,
            params.meas_dist_var_floor
                + params.meas_dist_delta_angle_k * std::log(std::abs(delta_angle) + 1.0),
            params.meas_armor_yaw_var_floor
                + params.meas_armor_yaw_range_k * std::log(distance + 1.0);
        return R_dig;
    }
};

/**
 * @brief 前哨站 Outpost 运动模型
 * 和机器人模型最大区别：
 * 1. 3块装甲、固定半径；
 * 2. 中心XY坐标**不移动**，只有自身旋转；
 * 3. 三块装甲独立Z高度；
 * 4. 没有半径对数状态 kHasLogRadii=false
 */
struct OutpostEkfMotionModel {
    using Scalar                       = double;
    static constexpr int NX            = O_STATE_MAX;
    static constexpr int NZ            = MEASURE_MAX;
    static constexpr int ARMORS_NUM    = 3;               // 前哨3块装甲
    static constexpr int kVyawIndex    = O_VYAW;
    static constexpr bool kHasLogRadii = false;

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;

    static constexpr Scalar OUTPOST_RADIUS = 0.2765;     // 前哨装甲旋转半径，机械图纸定值
    static constexpr Scalar OUTPOST_V_YAW  = 2.51327412; // 规则固定旋转角速度（常量）
    struct Params {
        Scalar sigma_q_xy  = 10.0;
        Scalar sigma_q_z   = 1.0;
        Scalar sigma_a_yaw = 1.0;

        Scalar meas_yaw_var_floor       = 4e-3;
        Scalar meas_pitch_var_floor     = 4e-3;
        Scalar meas_log_dist_var_floor  = 2.5e-3;
        Scalar meas_dist_k              = 0.43;
        Scalar meas_armor_yaw_var_floor = 9e-2;
        Scalar yaw_log_k                = 0.005;
    };
    Params params{};

    [[nodiscard]] static int clamp_armor_id(int id) noexcept {
        return std::clamp(id, 0, ARMORS_NUM - 1);
    }

    /**
     * @brief 前哨站状态预测
     * 关键点：O_XC、O_YC 不更新！前哨站中心位置固定不动，只有自身旋转
     */
    template <typename T>
    static void predict_state(const T* x, const T& dt, T* xp) noexcept {
        xp[O_XC]   = x[O_XC];
        xp[O_YC]   = x[O_YC];
        xp[O_YAW]  = x[O_YAW] + x[O_VYAW] * dt; // 偏航持续旋转
        xp[O_VYAW] = x[O_VYAW];
        xp[O_Z0]   = x[O_Z0];   // 三块装甲高度独立，随机游走
        xp[O_Z1]   = x[O_Z1];
        xp[O_Z2]   = x[O_Z2];
    }

    /**
     * @brief 前哨观测方程
     * 固定半径，无需exp(logR)；三块装甲各自独立Z坐标
     */
    template <typename T>
    static void measure_state(const T* x, int armor_id, T* z) noexcept {
        using std::atan2;
        using std::cos;
        using std::hypot;
        using std::log;
        using std::sin;

        const int id       = clamp_armor_id(armor_id);
        const T angle_step = T(2.0 * std::numbers::pi / static_cast<double>(ARMORS_NUM));
        const T yaw        = x[O_YAW] + T(id) * angle_step;

        const T armor_x = x[O_XC] - T(OUTPOST_RADIUS) * cos(yaw);
        const T armor_y = x[O_YC] - T(OUTPOST_RADIUS) * sin(yaw);
        const T armor_z = x[O_Z0 + id]; // 装甲0→Z0，装甲1→Z1，装甲2→Z2

        const T horizontal = hypot(armor_x, armor_y);
        const T distance   = hypot(horizontal, armor_z);

        z[ARMOR_YAW]       = atan2(armor_y, armor_x);
        z[ARMOR_PITCH]     = atan2(-armor_z, horizontal);
        z[ARMOR_DISTANCE]  = log(distance + T(1e-9));
        z[ARMOR_YAW_ARMOR] = yaw;
    }

    [[nodiscard]] VecX f(const VecX& x, double dt) const noexcept {
        VecX xp = VecX::Zero();
        predict_state<double>(x.data(), dt, xp.data());
        return xp;
    }

    [[nodiscard]] VecZ h(const VecX& x, int armor_id) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    /// 前哨站过程噪声矩阵Q
    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX Q = MatXX::Zero();
        if (dt <= 0.0) {
            return Q;
        }

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        const double sigma_xy2  = params.sigma_q_xy * params.sigma_q_xy;
        const double sigma_z2   = params.sigma_q_z * params.sigma_q_z;
        const double sigma_yaw2 = params.sigma_a_yaw * params.sigma_a_yaw;

        // XY位置随机游走（前哨理论不动，少量噪声允许微小漂移）
        Q(O_XC, O_XC) = sigma_xy2 * dt2;
        Q(O_YC, O_YC) = sigma_xy2 * dt2;

        // 偏航-角速度匀速模型
        Q(O_YAW, O_YAW)   = sigma_yaw2 * dt4 / 4.0;
        Q(O_YAW, O_VYAW)  = sigma_yaw2 * dt3 / 2.0;
        Q(O_VYAW, O_YAW)  = Q(O_YAW, O_VYAW);
        Q(O_VYAW, O_VYAW) = sigma_yaw2 * dt2;

        // 三块装甲高度随机游走
        Q(O_Z0, O_Z0) = sigma_z2 * dt2;
        Q(O_Z1, O_Z1) = sigma_z2 * dt2;
        Q(O_Z2, O_Z2) = sigma_z2 * dt2;
        return Q;
    }

    /// 观测噪声矩阵R
    [[nodiscard]] MatZZ R(const VecZ& z) const noexcept {
        MatZZ R                  = MatZZ::Zero();
        const double delta_angle = shortest_rad(z[ARMOR_YAW], z[ARMOR_YAW_ARMOR]);
        const double distance    = std::exp(std::clamp(z[ARMOR_DISTANCE], -20.0, 20.0));
        R(0, 0)                  = std::max(1e-8, params.meas_yaw_var_floor);
        R(1, 1)                  = std::max(1e-8, params.meas_pitch_var_floor);
        R(2, 2)                  = std::max(
            1e-8, params.meas_log_dist_var_floor
                      + params.meas_dist_k * std::log(std::abs(delta_angle) + 1.0));
        R(3, 3) = std::max(
            1e-8, params.meas_armor_yaw_var_floor + params.yaw_log_k * std::log(distance + 1.0));
        return R;
    }

    /// 观测噪声对角向量
    Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const {
        const double delta_angle = shortest_rad(z[ARMOR_YAW], z[ARMOR_YAW_ARMOR]);
        const double distance    = std::exp(std::clamp(z[ARMOR_DISTANCE], -20.0, 20.0));
        Eigen::Matrix<Scalar, NZ, 1> R_dig;
        R_dig << std::max(1e-8, params.meas_yaw_var_floor),
            std::max(1e-8, params.meas_pitch_var_floor),
            std::max(
                1e-8, params.meas_log_dist_var_floor
                          + params.meas_dist_k * std::log(std::abs(delta_angle) + 1.0)),
            std::max(
                1e-8,
                params.meas_armor_yaw_var_floor + params.yaw_log_k * std::log(distance + 1.0));
        return R_dig;
    }
};

} // namespace fcs::L3
