/**
 * @file new_tracker.hpp
 * @brief 新版目标跟踪器 - 基于EKF的单目标状态估计与数据关联
 *
 * @details
 * 本文件实现了Talos火控系统的核心跟踪器，负责单个目标的完整生命周期管理。
 * 主要功能：
 * - EKF状态估计：位置、速度、偏航角、旋转半径等
 * - 数据关联：Mahalanobis距离匹配 + 先验约束
 * - 状态机管理：Idle/Detecting/Tracking/TempLost
 * - 双模型支持：机器人目标（RobotEkfMotionModel）和前哨站（OutpostEkfMotionModel）
 *
 * 核心算法：
 * 1. 状态估计（EKF）：
 *    - 状态向量：位置(xc,yc,z)、速度(vx,vy,vz)、偏航角(yaw,v_yaw)、旋转半径(log_r0,log_r1)、高度(h)
 *    - 测量向量：装甲板方位角(yaw,pitch)、距离(log_d)、装甲板朝向(yaw_armor)
 *    - 过程模型：匀速运动 + 匀角速度旋转
 *    - 观测模型：装甲板球坐标系测量
 *
 * 2. 数据关联：
 *    - Mahalanobis距离：d² = (z-h(x))ᵀ R⁻¹ (z-h(x))
 *    - 先验约束：基于角速度vyaw预测装甲板ID的概率分布
 *    - 贪婪匹配：按cost排序，优先匹配最佳候选
 *    - commit gate：仅当观测一致性足够强时才更新last_armor_id
 *
 * 3. 状态机：
 *    - Idle -> Detecting（首次观测）
 *    - Detecting -> Tracking（连续观测超过阈值）
 *    - Tracking -> TempLost（丢失观测）
 *    - TempLost -> Tracking（重新观测）或 -> Idle（超时）
 *
 * 4. 特殊处理：
 *    - 前哨站：三档角速度模式（±ω, 0），独立的运动模型
 *    - 机器人目标：连续角速度，自适应旋转半径估计
 *    - 装甲板ID跳变检测：观测到非零ID时标记target_jumped
 *
 * 设计模式：
 * - Policy-based design：通过ModelT模板参数切换运动模型
 * - RAII：EkfTargetInfo管理EKF生命周期
 * - 状态模式：TrackerNew内部状态机
 *
 * @see ExtendedKalmanFilter
 * @see RobotEkfMotionModel
 * @see OutpostEkfMotionModel
 * @see TrackerManager
 *
 * @warning 数值稳定性风险：
 * - 协方差矩阵可能失去正定性
 * - 大偏航角可能导致观测模型奇异
 * - 旋转半径的对数参数化需要特殊处理（解相关）
 *
 * @author Chengfu Zou
 * @copyright Apache License 2.0
 */

#pragma once

#include "config.hpp"
#include "core/types.hpp"
#include "extended_kalman_filter.hpp"
#include "new_motion_model.hpp"
#include "types.hpp"
#include "util.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <ceres/jet.h>

namespace fcs::L3 {

/**
 * @class EkfTargetInfo
 * @brief EKF目标信息封装类 - 持有单个目标的EKF实例和运动模型
 *
 * @details
 * 核心职责：
 * - 封装EKF实例和运动模型
 * - 提供简化的predict/update接口
 * - 处理对数半径参数化的特殊情况
 * - 管理收敛状态
 *
 * 设计要点：
 * - 模板参数ModelT：运动模型类型（RobotEkfMotionModel或OutpostEkfMotionModel）
 * - 惰性初始化：initialize()时才创建EKF实例
 * - 对数半径解相关：避免r0和r1的虚假相关性
 *
 * @tparam ModelT 运动模型类型，必须提供：
 *                 - NX: 状态维度
 *                 - NZ: 测量维度
 *                 - VecX, VecZ, MatXX, MatZZ: 类型别名
 *                 - predict_state(), measure_state(), Q(), R()
 */
template <typename ModelT>
class EkfTargetInfo {
public:
    using Model             = ModelT;                            ///< 运动模型类型
    static constexpr int NX = Model::NX;                         ///< 状态维度
    static constexpr int NZ = Model::NZ;                         ///< 测量维度

    using VecX   = typename Model::VecX;                         ///< 状态向量类型
    using VecZ   = typename Model::VecZ;                         ///< 测量向量类型
    using MatXX  = typename Model::MatXX;                        ///< 状态协方差矩阵类型
    using MatZZ  = typename Model::MatZZ;                        ///< 测量协方差矩阵类型
    using Params = typename Model::Params;                       ///< 模型参数类型

    using JetX        = ceres::Jet<double, NX>;                  ///< Ceres Jet类型（自动微分）
    using PredictFunc = std::function<void(const JetX*, JetX*)>; ///< 预测函数签名
    using MeasureFunc = std::function<void(const JetX*, JetX*)>; ///< 测量函数签名
    using EKF         = ExtendedKalmanFilter<NX, NZ, PredictFunc, MeasureFunc>; ///< EKF实例类型

    EkfTargetInfo() = default;

    /**
     * @brief 初始化EKF目标信息
     *
     * @details
     * 创建EKF实例并设置初始状态。关键步骤：
     * 1. 存储模型参数（过程噪声、测量噪声等）
     * 2. 创建预测函数f和测量函数h（Lambda表达式，捕获this）
     * 3. 创建噪声协方差函数q和r
     * 4. 初始化EKF实例和状态向量
     *
     * 对数半径处理：
     * - 状态向量中使用log(r0)和log(r1)（避免负值约束）
     * - 过程噪声需要缩放：Q(log_r) /= r²（保持线性空间不确定性）
     * - 原因：d(log r)/dt = (1/r) * dr/dt
     *
     * @param params 模型参数
     * @param x0 初始状态向量
     * @param P0 初始协方差矩阵
     */
    void initialize(const Params& params, const VecX& x0, const MatXX& P0) noexcept {
        model_.params = params;
        dt_           = 0.0;
        armor_id_     = 0;

        // 创建预测函数：Lambda捕获this，调用Model::predict_state
        auto f = [this](const JetX* x, JetX* xp) {
            Model::template predict_state<JetX>(x, JetX(dt_), xp);
        };

        // 创建测量函数：Lambda捕获this，调用Model::measure_state
        auto h = [this](const JetX* x, JetX* z) {
            Model::template measure_state<JetX>(x, armor_id_, z);
        };

        // 创建过程噪声协方差函数：特殊处理对数半径
        auto q = [this]() -> MatXX {
            auto Q = model_.Q(dt_);
            if constexpr (Model::kHasLogRadii) {
                // 对数半径的过程噪声缩放：Q(log_r) /= r²
                // 原因：d(log r)/dt = (1/r) * dr/dt，协方差需乘以(1/r)²
                const double r0 = std::exp(x_[LOG_R0]);
                const double r1 = std::exp(x_[LOG_R1]);
                Q(LOG_R0, LOG_R0) /= (r0 * r0);
                Q(LOG_R1, LOG_R1) /= (r1 * r1);
            }
            return Q;
        };

        // 创建测量噪声协方差函数：支持外部覆盖
        auto r = [this](const VecZ& z) -> MatZZ {
            if (update_R_override_.has_value()) {
                return *update_R_override_; // 使用外部提供的R
            }
            return model_.R(z);             // 使用模型计算的R
        };

        // 初始化EKF实例
        ekf_ = EKF(f, h, q, r, P0);
        ekf_.setState(x0);
        x_      = x0;
        active_ = true;
    }

    /**
     * @brief 预测步（时间更新）
     *
     * @param dt 时间间隔（秒）
     *
     * @note 调用EKF::predict()，内部使用自动微分计算雅可比矩阵
     */
    void predict(double dt) noexcept {
        if (!active_) {
            return;
        }
        dt_ = std::max(0.0, dt);
        x_  = ekf_.predict();
    }

    /**
     * @brief 更新步（测量更新）
     *
     * @param z 测量向量
     * @param armor_id 装甲板ID（用于选择观测哪个装甲板）
     *
     * @note 更新后自动解相关log_r0和log_r1（避免虚假相关性）
     */
    void update(const VecZ& z, int armor_id) noexcept {
        if (!active_) {
            return;
        }
        armor_id_ = Model::clamp_armor_id(armor_id);
        x_        = ekf_.update(z);
        update_R_override_.reset();

        // 解相关独立的对数半径状态（防止"关联幻觉"）
        if constexpr (Model::kHasLogRadii) {
            ekf_.decorrelate(LOG_R0, LOG_R1);
        }
    }

    /**
     * @brief 更新步（带外部测量噪声协方差）
     *
     * @param z 测量向量
     * @param armor_id 装甲板ID
     * @param R 外部测量噪声协方差矩阵（覆盖模型计算的R）
     *
     * @note 用于PNP协方差传递等场景
     */
    void update(const VecZ& z, int armor_id, const MatZZ& R) noexcept {
        if (!active_) {
            return;
        }
        update_R_override_ = R;
        armor_id_          = Model::clamp_armor_id(armor_id);
        x_                 = ekf_.update(z);
        update_R_override_.reset();

        // 解相关独立的对数半径状态
        if constexpr (Model::kHasLogRadii) {
            ekf_.decorrelate(LOG_R0, LOG_R1);
        }
    }

    [[nodiscard]] bool active() const noexcept { return active_; }

    [[nodiscard]] const VecX& x() const noexcept { return x_; }

    [[nodiscard]] const MatXX& P() const noexcept { return ekf_.P(); }

    using MatXZ = Eigen::Matrix<double, NX, NZ>;
    [[nodiscard]] const MatXZ& K_gain() const noexcept { return ekf_.K_gain(); }
    [[nodiscard]] const MatXX& Q_mat() const noexcept { return ekf_.Q_mat(); }
    [[nodiscard]] const MatZZ& R_mat() const noexcept { return ekf_.R_mat(); }
    [[nodiscard]] FilterConvergenceState convergence() const noexcept {
        FilterConvergenceState state;
        switch (ekf_.convergence_status()) {
        case EKF::ConvergenceStatus::Unknown:
            state.status = FilterConvergenceStatus::Unknown;
            break;
        case EKF::ConvergenceStatus::Converging:
            state.status = FilterConvergenceStatus::Converging;
            break;
        case EKF::ConvergenceStatus::Converged:
            state.status = FilterConvergenceStatus::Converged;
            break;
        case EKF::ConvergenceStatus::Diverging:
            state.status = FilterConvergenceStatus::Diverging;
            break;
        }
        state.normalized_innovation_squared = ekf_.normalized_innovation_squared();
        state.max_covariance_diag           = ekf_.max_covariance_diag();
        state.consecutive_converged_updates = ekf_.consecutive_converged_updates();
        state.consecutive_diverged_updates  = ekf_.consecutive_diverged_updates();
        return state;
    }

    void set_state_unsafe(VecX x) noexcept { ekf_.setState(x); }

    [[nodiscard]] const Model& model() const noexcept { return model_; }

    [[nodiscard]] Model& model() noexcept { return model_; }

private:
    bool active_{false};
    double dt_{0.0};
    int armor_id_{0};
    Model model_{};
    EKF ekf_{};
    VecX x_{VecX::Zero()};
    std::optional<MatZZ> update_R_override_{};
};

class TrackerNew {
public:
    TrackerNew() noexcept = default;

    explicit TrackerNew(const TrackerConfig& config) noexcept
        : config_(config) {}

    void predict(double dt) noexcept {
        last_dt_ = std::max(0.0, dt);

        if (status_ == TrackerStatus::TempLost && (lost_time_ + last_dt_) >= lost_threshold_) {
            reset();
            return;
        }

        if (robot_target_.has_value()) {
            robot_target_->predict(last_dt_);
        }

        if (outpost_target_.has_value()) {
            outpost_target_->predict(last_dt_);
        }
    }

    [[nodiscard]] bool update(const ArmorMeasurementBatch& measurements) noexcept {
        measurement_.fill(0.0);

        if (target_name_ == ArmorName::Outpost) {
            if (!outpost_target_.has_value()) {
                return false;
            }
            auto d = update_target_(
                *outpost_target_, measurements, OutpostEkfMotionModel::ARMORS_NUM,
                outpost_last_armor_id_, false);
            if (!outpost_target_.has_value()) {
                return false;
            }
            return d;
        }

        if (!robot_target_.has_value()) {
            return false;
        }
        return update_target_(
            *robot_target_, measurements, RobotEkfMotionModel::ARMORS_NUM, robot_last_armor_id_,
            true);
    }

    [[nodiscard]] std::optional<ArmorName>
        first_meet(const ArmorMeasurementBatch& measurements) noexcept {
        if (measurements.empty()) {
            state_machine(false);
            return std::nullopt;
        }

        int min_idx        = -1;
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(measurements.measurements.size()); ++i) {
            const float dist = measurements.measurements[i].distance_to_image_center;
            if (dist < min_distance) {
                min_distance = dist;
                min_idx      = i;
            }
        }

        if (min_idx < 0) {
            state_machine(false);
            return std::nullopt;
        }

        const auto& target     = measurements.measurements[static_cast<size_t>(min_idx)];
        const auto& pos        = target.transform.translation();
        const double armor_yaw = target.yaw();

        target_name_                   = target.name;
        target_color_                  = target.color;
        last_image_center_distance_px_ = target.distance_to_image_center;
        last_observation_timestamp_ns_ = target.timestamp_ns;
        const bool is_outpost          = (target_name_ == ArmorName::Outpost);

        robot_target_.reset();
        outpost_target_.reset();
        robot_last_armor_id_.reset();
        outpost_last_armor_id_.reset();
        target_jumped_ = false;

        if (is_outpost) {
            const double x0 = pos.x() + OutpostEkfMotionModel::OUTPOST_RADIUS * std::cos(armor_yaw);
            const double y0 = pos.y() + OutpostEkfMotionModel::OUTPOST_RADIUS * std::sin(armor_yaw);
            const double z0 = pos.z();

            OutpostEkfMotionModel::VecX xp0;
            xp0 << x0, y0, armor_yaw, 0.0, z0 - 0.1, z0, z0 + 0.1;
            OutpostEkfMotionModel::MatXX P0 = OutpostEkfMotionModel::MatXX::Identity();

            P0(O_XC, O_XC) = 0.1;
            P0(O_YC, O_YC) = 0.1;

            P0(O_YAW, O_YAW)   = 0.4;
            P0(O_VYAW, O_VYAW) = 100;

            P0(O_Z0, O_Z0) = 1.5;
            P0(O_Z1, O_Z1) = 1.5;
            P0(O_Z2, O_Z2) = 1.5;
            outpost_target_.emplace();
            outpost_target_->initialize(config_.outpost.model, xp0, P0);
            outpost_last_armor_id_ = 0;

            lost_threshold_     = config_.outpost.lost_threshold;
            tracking_threshold_ = config_.outpost.tracking_threshold;
            matcher_gate_       = config_.outpost.matcher_gate;
        } else {
            const double r0_prior = std::clamp(config_.robot_inekf.radius0, 0.10, 0.50);
            const double r1_prior = std::clamp(config_.robot_inekf.radius1, 0.10, 0.50);
            const double h_prior  = std::clamp(config_.robot_inekf.height, -0.20, 0.20);
            const double r_init   = 0.5 * (r0_prior + r1_prior);
            const double x0       = pos.x() + r_init * std::cos(armor_yaw);
            const double y0       = pos.y() + r_init * std::sin(armor_yaw);
            const double z0       = pos.z() - 0.5 * h_prior;

            RobotEkfMotionModel::VecX xp0;
            auto v0 = 0.0;
            xp0 << x0, v0, y0, v0, z0, v0, armor_yaw, v0, std::log(r0_prior), std::log(r1_prior),
                h_prior;
            RobotEkfMotionModel::MatXX P0 = RobotEkfMotionModel::MatXX::Identity();

            P0(XC, XC) = 1;
            P0(YC, YC) = 1;

            P0(VX, VX) = 64;
            P0(VY, VY) = 64;
            P0(VZ, VZ) = 64;

            P0(YAW, YAW)     = 0.4;
            P0(V_YAW, V_YAW) = 100;

            P0(LOG_R0, LOG_R0) = 1e-5 / (r0_prior * r0_prior);
            P0(LOG_R1, LOG_R1) = 1e-5 / (r1_prior * r1_prior);
            P0(H, H)           = 1;

            robot_target_.emplace();
            robot_target_->initialize(config_.robot.model, xp0, P0);
            robot_last_armor_id_.reset();

            lost_threshold_     = config_.robot.lost_threshold;
            tracking_threshold_ = config_.robot.tracking_threshold;
            matcher_gate_       = config_.robot.matcher_gate;
        }

        state_machine(true);
        return target_name_;
    }

    [[nodiscard]] TrackerOutput get_output() const noexcept {
        TrackerOutput output;
        output.status                        = status_;
        output.target_name                   = target_name_;
        output.target_color                  = target_color_;
        output.target_jumped                 = target_jumped_;
        output.last_image_center_distance_px = last_image_center_distance_px_;
        output.last_observation_timestamp_ns = last_observation_timestamp_ns_;

        if (robot_target_.has_value()) {
            const auto& x = robot_target_->x();
            RobotTargetState state;
            state.position       = {x[XC], x[YC], x[Z0]};
            state.velocity       = {x[VX], x[VY], x[VZ]};
            state.yaw            = x[YAW];
            state.v_yaw          = x[V_YAW];
            state.radius0        = std::exp(x[LOG_R0]);
            state.radius1        = std::exp(x[LOG_R1]);
            state.z1             = x[Z0] + x[H];
            state.armors_num     = RobotEkfMotionModel::ARMORS_NUM;
            state.P              = robot_target_->P();
            state.K              = robot_target_->K_gain();
            state.Q              = robot_target_->Q_mat();
            state.R              = robot_target_->R_mat();
            state.convergence    = robot_target_->convergence();
            output.state         = state;
            output.last_armor_id = robot_last_armor_id_;
        } else if (outpost_target_.has_value()) {
            const auto& x = outpost_target_->x();
            OutpostTargetState state;
            state.position       = {x[O_XC], x[O_YC]};
            state.yaw            = x[O_YAW];
            state.v_yaw          = x[O_VYAW];
            state.z              = {x[O_Z0], x[O_Z1], x[O_Z2]};
            state.P              = outpost_target_->P();
            state.K              = outpost_target_->K_gain();
            state.Q              = outpost_target_->Q_mat();
            state.R              = outpost_target_->R_mat();
            state.convergence    = outpost_target_->convergence();
            output.state         = state;
            output.last_armor_id = outpost_last_armor_id_;
        }

        return output;
    }

    void reset() noexcept {
        robot_target_.reset();
        outpost_target_.reset();
        robot_last_armor_id_.reset();
        outpost_last_armor_id_.reset();
        target_jumped_ = false;
        target_name_   = ArmorName::Invalid;
        target_color_  = ArmorColor::Neutral;
        measurement_.fill(0.0);
        last_image_center_distance_px_ = std::numeric_limits<double>::infinity();
        last_observation_timestamp_ns_ = 0;
        detecting_count_               = 0;
        last_dt_                       = 0.0;
        lost_time_                     = 0.0;
        status_                        = TrackerStatus::Idle;
    }

    [[nodiscard]] TrackerStatus status() const noexcept { return status_; }

    [[nodiscard]] ArmorName target_name() const noexcept { return target_name_; }

    [[nodiscard]] const std::array<double, RobotEkfMotionModel::NZ * 2>&
        measurement() const noexcept {
        return measurement_;
    }

private:
    template <typename TargetInfo>
    struct MatchCandidate {
        size_t obs_index{0};
        int armor_id{0};
        double cost{0.0};
        typename TargetInfo::VecZ z_update{};
        typename TargetInfo::MatZZ R_update{};
    };

    template <typename TargetInfo>
    [[nodiscard]] bool update_target_(
        TargetInfo& target, const ArmorMeasurementBatch& measurements, int armors_num,
        std::optional<int>& last_armor_id, bool enable_robot_pair_measure) noexcept {
        using VecZ  = typename TargetInfo::VecZ;
        using MatZZ = typename TargetInfo::MatZZ;

        std::vector<size_t> obs_indices;
        obs_indices.reserve(measurements.measurements.size());
        for (size_t i = 0; i < measurements.measurements.size(); ++i) {
            if (measurements.measurements[i].name == target_name_) {
                obs_indices.push_back(i);
            }
        }

        if (obs_indices.empty()) {
            state_machine(false);
            return false;
        }

        const double angle_step = 2.0 * std::numbers::pi / static_cast<double>(armors_num);

        // -------------------------------------------------------------------------
        // Prior cost.
        //
        // 关键结构：
        //   cost = observation_d2 + prior_cost
        //
        // prior 可以影响当前帧 association 排序；
        // 但后面 last_armor_id commit 不看 prior，只看 observation-only consistency。
        // -------------------------------------------------------------------------
        const auto prior_cost_for_id = [&](int id) noexcept -> double {
            if (!last_armor_id.has_value()) {
                return 0.0;
            }

            constexpr double kMinProb = 1e-6;

            // ---------------------------------------------------------------------
            // Outpost:
            //   GT vyaw 不是自由 CV，而是约等于 {-a, 0, +a}。
            //   所以 matcher prior 单独用三离散模式 mixture。
            // ---------------------------------------------------------------------
            if (target_name_ == ArmorName::Outpost) {
                constexpr double kOutpostVyaw         = 2.51327412;
                constexpr double kOutpostModePhaseStd = 0.45;
                constexpr double kOutpostPriorWeight  = 0.6;

                const std::array<double, 3> mode_vyaws = {
                    -kOutpostVyaw,
                    0.0,
                    +kOutpostVyaw,
                };

                double best_prior_cost = std::numeric_limits<double>::infinity();

                for (const double mode_vyaw : mode_vyaws) {
                    const double mode_phase_shift = mode_vyaw * last_dt_ / angle_step;

                    const auto mode_prior = build_mode_prior_(
                        armors_num, last_armor_id, mode_phase_shift, kOutpostModePhaseStd);

                    const double p = std::clamp(
                        id < static_cast<int>(mode_prior.size())
                            ? mode_prior[static_cast<size_t>(id)]
                            : 1.0,
                        kMinProb, 1.0);

                    best_prior_cost = std::min(best_prior_cost, -2.0 * std::log(p));
                }

                if (!std::isfinite(best_prior_cost)) {
                    return 0.0;
                }

                return kOutpostPriorWeight * best_prior_cost;
            }

            // ---------------------------------------------------------------------
            // Normal robot:
            //   phase prior 来自连续 vyaw，并且由 P(vyaw,vyaw) 控制 prior 尖锐程度。
            // ---------------------------------------------------------------------
            const double phase_shift =
                target.x()[TargetInfo::Model::kVyawIndex] * last_dt_ / angle_step;

            const auto& P = target.P();

            const double vyaw_var = std::max(
                0.0, static_cast<double>(
                         P(TargetInfo::Model::kVyawIndex, TargetInfo::Model::kVyawIndex)));

            const double phase_std_from_p =
                std::sqrt(vyaw_var) * std::abs(last_dt_) / std::max(angle_step, 1e-9);

            const double base_phase_std = std::clamp(std::abs(last_dt_) * 0.5, 0.05, 1.0);

            const double phase_std =
                std::clamp(std::hypot(base_phase_std, phase_std_from_p), 0.08, 1.5);

            const double prior_weight =
                std::clamp(1.0 / (1.0 + phase_std_from_p * phase_std_from_p), 0.0, 1.0);

            const auto prior = build_mode_prior_(armors_num, last_armor_id, phase_shift, phase_std);

            const double p = std::clamp(
                id < static_cast<int>(prior.size()) ? prior[static_cast<size_t>(id)] : 1.0,
                kMinProb, 1.0);

            return prior_weight * (-2.0 * std::log(p));
        };

        std::vector<MatchCandidate<TargetInfo>> candidates;
        candidates.reserve(obs_indices.size() * static_cast<size_t>(armors_num));

        for (const size_t obs_index : obs_indices) {
            const auto& meas = measurements.measurements[obs_index];

            const VecZ z_raw    = measurement_to_z_(meas);
            const MatZZ R_model = target.model().R(z_raw);
            MatZZ R_update      = R_model;
            if (target_name_ == ArmorName::Outpost) {
                R_update = pnp_augmented_measurement_R_<TargetInfo>(R_model, meas.pnp_cov_ypdr);
            }

            for (int id = 0; id < armors_num; ++id) {
                const VecZ z_pred = target.model().h(target.x(), id);
                if (!armor_measurement_visible_from_origin(z_pred)) {
                    continue;
                }

                const VecZ nu   = innovation_(z_pred, z_raw);
                const double d2 = mahalanobis_(nu, R_model);
                if (!std::isfinite(d2) || d2 >= matcher_gate_) {
                    continue;
                }

                const double cost = d2 + prior_cost_for_id(id);

                MatchCandidate<TargetInfo> candidate;
                candidate.obs_index = obs_index;
                candidate.armor_id  = id;
                candidate.cost      = cost;
                candidate.z_update  = wrap_measurement_(z_pred, z_raw);
                candidate.R_update  = R_update;
                candidates.push_back(candidate);
            }
        }

        if (candidates.empty()) {
            state_machine(false);
            return false;
        }

        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return a.cost < b.cost;
        });

        std::vector<bool> used_obs(measurements.measurements.size(), false);
        std::vector<bool> used_id(static_cast<size_t>(armors_num), false);
        std::vector<MatchCandidate<TargetInfo>> selected;
        selected.reserve(static_cast<size_t>(armors_num));

        for (const auto& candidate : candidates) {
            if (used_obs[candidate.obs_index] || used_id[static_cast<size_t>(candidate.armor_id)]) {
                continue;
            }
            used_obs[candidate.obs_index]                    = true;
            used_id[static_cast<size_t>(candidate.armor_id)] = true;
            selected.push_back(candidate);
        }

        if (selected.empty()) {
            state_machine(false);
            return false;
        }

        state_machine(true);

        // -------------------------------------------------------------------------
        // Observation-only d2.
        //
        // 这里故意不使用 candidate.cost，因为 cost 里含 prior。
        // last_armor_id 是历史锚点，不能由 prior 自证。
        // -------------------------------------------------------------------------
        const auto obs_d2_for =
            [&](const MatchCandidate<TargetInfo>& candidate) noexcept -> double {
            const auto& meas = measurements.measurements[candidate.obs_index];

            const VecZ z_raw  = measurement_to_z_(meas);
            const VecZ z_pred = target.model().h(target.x(), candidate.armor_id);
            const VecZ nu     = innovation_(z_pred, z_raw);

            return mahalanobis_(nu, candidate.R_update);
        };

        const auto& primary = selected.front();

        const double primary_obs_d2 = obs_d2_for(primary);

        double second_obs_d2 = std::numeric_limits<double>::infinity();
        for (const auto& candidate : candidates) {
            if (candidate.obs_index != primary.obs_index) {
                continue;
            }

            if (candidate.armor_id == primary.armor_id) {
                continue;
            }

            const double d2 = obs_d2_for(candidate);
            if (std::isfinite(d2)) {
                second_obs_d2 = std::min(second_obs_d2, d2);
            }
        }

        const double obs_margin = second_obs_d2 - primary_obs_d2;

        // -------------------------------------------------------------------------
        // Commit gate.
        //
        // selected 可以由 prior + obs 决定；
        // 但 last_armor_id 只能由 observation-only consistency 决定。
        //
        // 这样 prior 方向错了时，不会立刻污染 last_armor_id。
        // -------------------------------------------------------------------------
        constexpr double kMaxCommitObsD2     = 6.0;
        constexpr double kMinCommitObsMargin = 2.0;

        const bool commit_last_id = std::isfinite(primary_obs_d2)
                                 && primary_obs_d2 < kMaxCommitObsD2 && std::isfinite(obs_margin)
                                 && obs_margin > kMinCommitObsMargin;

        if (commit_last_id) {
            last_armor_id = primary.armor_id;
        }

        const auto& primary_measurement = measurements.measurements[primary.obs_index];
        last_image_center_distance_px_  = primary_measurement.distance_to_image_center;
        last_observation_timestamp_ns_  = primary_measurement.timestamp_ns;

        const bool observed_nonzero_armor = std::any_of(
            selected.begin(), selected.end(),
            [](const MatchCandidate<TargetInfo>& candidate) { return candidate.armor_id != 0; });

        if (observed_nonzero_armor) {
            target_jumped_ = true;
        }

        for (const auto& candidate : selected) {
            target.update(candidate.z_update, candidate.armor_id, candidate.R_update);
            const bool another_pair =
                enable_robot_pair_measure && (candidate.armor_id == 1 || candidate.armor_id == 3);
            set_measurement_(candidate.z_update, another_pair);
        }
        return true;
    }

    static double wrap_index_diff_(double d, int n) noexcept {
        if (n <= 0 || !std::isfinite(d)) {
            return 0.0;
        }
        const double period = static_cast<double>(n);
        double wrapped      = std::remainder(d, period); // (-period/2, period/2]
        if (!std::isfinite(wrapped)) {
            return 0.0;
        }
        // Keep historical tie behavior: map -n/2 to +n/2.
        if (wrapped <= -0.5 * period) {
            wrapped += period;
        }
        return wrapped;
    }

    static std::vector<double> build_mode_prior_(
        int armors_num, std::optional<int> last_id, double phase_shift, double phase_std) {
        std::vector<double> prior(
            static_cast<size_t>(armors_num), 1.0 / static_cast<double>(armors_num));
        if (!last_id.has_value() || armors_num <= 1) {
            return prior;
        }

        const double center      = static_cast<double>(*last_id) + phase_shift;
        const double sigma       = std::clamp(0.28 + 0.95 * phase_std, 0.25, 2.20);
        const double inv_var     = 1.0 / (sigma * sigma);
        const double uniform_mix = std::clamp(0.02 + 0.30 * phase_std, 0.02, 0.35);
        const int max_index_step = std::clamp(
            static_cast<int>(std::ceil(std::abs(phase_shift) + 3.0 * phase_std + 0.25)), 1,
            armors_num / 2 + 1);

        double sum = 0.0;
        for (int j = 0; j < armors_num; ++j) {
            const double diff = wrap_index_diff_(static_cast<double>(j) - center, armors_num);
            if (std::abs(diff) > static_cast<double>(max_index_step)) {
                prior[static_cast<size_t>(j)] = 0.0;
                continue;
            }
            const double g = std::exp(-0.5 * diff * diff * inv_var);
            prior[static_cast<size_t>(j)] =
                (1.0 - uniform_mix) * g + uniform_mix / static_cast<double>(armors_num);
            sum += prior[static_cast<size_t>(j)];
        }

        if (sum <= 0.0) {
            return std::vector<double>(static_cast<size_t>(armors_num), 1.0 / armors_num);
        }
        for (double& p : prior) {
            p /= sum;
        }
        return prior;
    }

    static RobotEkfMotionModel::VecZ measurement_to_z_(const ArmorMeasurement& meas) noexcept {
        const auto ypd = xyz2ypd(meas.transform.translation());
        RobotEkfMotionModel::VecZ z;
        z[ARMOR_YAW]       = ypd[0];
        z[ARMOR_PITCH]     = ypd[1];
        z[ARMOR_DISTANCE]  = std::log(std::max(ypd[2], 1e-9));
        z[ARMOR_YAW_ARMOR] = meas.yaw();
        return z;
    }

    template <typename TargetInfo>
    static typename TargetInfo::MatZZ pnp_augmented_measurement_R_(
        const typename TargetInfo::MatZZ& model_R, const Eigen::Matrix4d& pnp_cov_ypdr) noexcept {
        using MatZZ = typename TargetInfo::MatZZ;

        const MatZZ base_R = make_spd_R_(model_R);
        if (!pnp_cov_ypdr.allFinite()) {
            return base_R;
        }

        MatZZ pnp_R = 0.5 * (pnp_cov_ypdr + pnp_cov_ypdr.transpose());
        if (!pnp_R.allFinite()) {
            return base_R;
        }

        Eigen::SelfAdjointEigenSolver<MatZZ> es(pnp_R);
        if (es.info() == Eigen::Success && pnp_R.allFinite()) {
            const auto evals = es.eigenvalues().cwiseMax(0.0);
            pnp_R = es.eigenvectors() * evals.asDiagonal() * es.eigenvectors().transpose();
            pnp_R = 0.5 * (pnp_R + pnp_R.transpose());
        }

        const MatZZ combined_R = base_R + pnp_R;
        return make_spd_R_(combined_R);
    }

    template <typename MatZZ>
    static MatZZ make_spd_R_(const MatZZ& input) noexcept {
        MatZZ R = 0.5 * (input + input.transpose());
        for (int i = 0; i < MatZZ::RowsAtCompileTime; ++i) {
            if (!std::isfinite(R(i, i)) || R(i, i) < 1e-8) {
                R(i, i) = 1e-8;
            }
        }

        Eigen::LDLT<MatZZ> ldlt(R);
        if (R.allFinite() && ldlt.info() == Eigen::Success && ldlt.isPositive()) {
            return R;
        }

        MatZZ fallback = MatZZ::Zero();
        for (int i = 0; i < MatZZ::RowsAtCompileTime; ++i) {
            const double var = input(i, i);
            fallback(i, i)   = std::isfinite(var) ? std::max(1e-8, var) : 1e6;
        }
        return fallback;
    }

    template <typename VecZ>
    static VecZ innovation_(const VecZ& z_pred, const VecZ& z_raw) noexcept {
        VecZ nu;
        nu[ARMOR_YAW]       = shortest_rad(z_pred[ARMOR_YAW], z_raw[ARMOR_YAW]);
        nu[ARMOR_PITCH]     = shortest_rad(z_pred[ARMOR_PITCH], z_raw[ARMOR_PITCH]);
        nu[ARMOR_DISTANCE]  = z_raw[ARMOR_DISTANCE] - z_pred[ARMOR_DISTANCE];
        nu[ARMOR_YAW_ARMOR] = shortest_rad(z_pred[ARMOR_YAW_ARMOR], z_raw[ARMOR_YAW_ARMOR]);
        return nu;
    }

    template <typename VecZ>
    static VecZ wrap_measurement_(const VecZ& z_pred, const VecZ& z_raw) noexcept {
        VecZ z;
        z[ARMOR_YAW]       = unwrap_rad(z_pred[ARMOR_YAW], z_raw[ARMOR_YAW]);
        z[ARMOR_PITCH]     = unwrap_rad(z_pred[ARMOR_PITCH], z_raw[ARMOR_PITCH]);
        z[ARMOR_DISTANCE]  = z_raw[ARMOR_DISTANCE];
        z[ARMOR_YAW_ARMOR] = unwrap_rad(z_pred[ARMOR_YAW_ARMOR], z_raw[ARMOR_YAW_ARMOR]);
        return z;
    }

    /// Full Mahalanobis distance with LDLT decomposition for non-diagonal R.
    template <typename VecZ, typename MatZZ>
    static double mahalanobis_(const VecZ& nu, const MatZZ& R) noexcept {
        Eigen::LDLT<MatZZ> ldlt(R);
        if (ldlt.info() != Eigen::Success) {
            return 1e9;
        }
        return static_cast<double>(nu.transpose() * ldlt.solve(nu));
    }

    void state_machine(bool found) noexcept {
        switch (status_) {
        case TrackerStatus::Idle:
            if (found) {
                detecting_count_++;
                if (detecting_count_ > 0) {
                    status_ = TrackerStatus::Detecting;
                }
            } else {
                reset();
            }
            break;

        case TrackerStatus::Detecting:
            if (found) {
                detecting_count_++;
                if (detecting_count_ > static_cast<int>(tracking_threshold_)) {
                    status_          = TrackerStatus::Tracking;
                    detecting_count_ = 0;
                }
            } else {
                reset();
            }
            break;

        case TrackerStatus::Tracking:
            if (!found) {
                status_    = TrackerStatus::TempLost;
                lost_time_ = last_dt_;
            }
            break;

        case TrackerStatus::TempLost:
            if (found) {
                lost_time_ = 0.0;
                status_    = TrackerStatus::Tracking;
            } else {
                lost_time_ += last_dt_;
                if (lost_time_ >= lost_threshold_) {
                    reset();
                }
            }
            break;
        }
    }

    template <typename VecZ>
    void set_measurement_(const VecZ& z, bool is_another_pair) noexcept {
        if (is_another_pair) {
            measurement_[4] = z[ARMOR_YAW];
            measurement_[5] = z[ARMOR_PITCH];
            measurement_[6] = z[ARMOR_DISTANCE];
            measurement_[7] = z[ARMOR_YAW_ARMOR];
        } else {
            measurement_[0] = z[ARMOR_YAW];
            measurement_[1] = z[ARMOR_PITCH];
            measurement_[2] = z[ARMOR_DISTANCE];
            measurement_[3] = z[ARMOR_YAW_ARMOR];
        }
    }

private:
    TrackerConfig config_{};
    TrackerStatus status_{TrackerStatus::Idle};
    ArmorName target_name_{ArmorName::Invalid};
    ArmorColor target_color_{ArmorColor::Neutral};

    std::optional<EkfTargetInfo<RobotEkfMotionModel>> robot_target_{};
    std::optional<EkfTargetInfo<OutpostEkfMotionModel>> outpost_target_{};

    double lost_threshold_{0.0};
    uint32_t tracking_threshold_{0};
    double matcher_gate_{10.0};

    int detecting_count_{0};
    double last_dt_{0.0};
    double lost_time_{0.0};

    std::array<double, RobotEkfMotionModel::NZ * 2> measurement_{};
    double last_image_center_distance_px_{std::numeric_limits<double>::infinity()};
    uint64_t last_observation_timestamp_ns_{0};
    std::optional<int> robot_last_armor_id_{};
    std::optional<int> outpost_last_armor_id_{};
    bool target_jumped_{false};
};

} // namespace fcs::L3
