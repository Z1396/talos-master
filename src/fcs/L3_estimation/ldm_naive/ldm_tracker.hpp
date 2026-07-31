/**
 * @file ldm_tracker.hpp
 * @brief LDM跟踪器核心实现 - Invariant EKF封装与状态机管理
 *
 * ## 文件功能概述
 * 本文件实现了LDM（Landing Device Marker）跟踪器的核心逻辑：
 * - LdmInEkfTracker：Invariant EKF封装类，管理群上滤波
 * - LdmTracker：高层跟踪器，包含状态机、生命周期管理、输出接口
 *
 * ## 核心算法原理
 *
 * ### 1. Invariant Extended Kalman Filter (InEKF)
 * 传统EKF在欧氏空间中进行线性化，而InEKF在李群上进行滤波：
 * - 状态：SE2(3)群元素X = (R, v_body, p)
 * - 扰动：李代数ξ ∈ se2(3)（9维向量）
 * - 预测：X̂₊ = X̂ * exp(ξ_pred)
 * - 更新：通过群作用保持结构性质
 *
 * ### 2. C₈对称性处理
 * 每次更新前执行nearest_lift，将观测旋转从商空间SO(3)/C₈提升回SO(3)：
 * - 选择最接近预测姿态的C₈代表
 * - 将分支置信度传递给观测模型用于自适应噪声调节
 *
 * ### 3. 状态机设计
 * LdmTracker维护4状态有限状态机：
 * - Idle：未初始化，等待首次观测
 * - Detecting：连续检测，确认目标存在
 * - Tracking：稳定跟踪，输出有效状态
 * - TempLost：暂时丢失，等待重新捕获
 *
 * ## 关键数据结构
 * - LdmInEkfTracker：封装InEKF，提供预测/更新接口
 * - LdmTracker：高层管理器，包含状态机、时间管理、输出接口
 *
 * ## 潜在风险提示
 * - nearest_lift假设帧间运动小，高速翻滚场景可能失效
 * - 状态机阈值需要根据场景调优（tracking_threshold、lost_threshold）
 * - 预测时间窗口（kPredictionHorizon）影响下游规划器性能
 *
 * ## 优化建议
 * - 可考虑自适应预测窗口（根据运动速度动态调整）
 * - 可添加协方差监控，检测滤波器发散
 * - 可实现多假设跟踪（维护多个SE2(3)分支）
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include "L3_estimation/tracker/invariant_extended_kalman_filter.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "ldm_kinematic_model.hpp"
#include "ldm_naive_config.hpp"
#include "types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <spdlog/spdlog.h>
#include <utility>

namespace fcs::L3::ldm {

/**
 * @brief LDM Invariant EKF跟踪器 - 群上滤波封装
 *
 * 封装Invariant Extended Kalman Filter，提供：
 * - 群上预测：SE2(3)右扰动传播
 * - 群上更新：位姿测量更新（含C₈提升）
 * - 状态访问：SE2(3)群元素和协方差矩阵
 *
 * ## 设计模式
 * - RAII：通过active_标志管理生命周期
 * - 封装：隐藏InEKF实现细节，提供类型安全接口
 * - 策略注入：通过函数对象传递模型参数
 *
 * ## 使用流程
 * 1. 构造：LdmInEkfTracker()
 * 2. 初始化：initialize(params, x0, P0)
 * 3. 预测：predict(dt)
 * 4. 更新：update(measurement)
 * 5. 访问状态：nominal(), P()
 */
class LdmInEkfTracker {
public:
    using Model           = LdmKinematic;           ///< 运动学模型类型
    using Nominal         = Model::Nominal;         ///< SE2(3)群类型
    using CovXi           = Model::CovXi;           ///< 状态协方差矩阵类型
    using PoseMeasurement = Model::PoseMeasurement; ///< 位姿测量类型
    using Params          = Model::Params;          ///< 模型参数类型

    /**
     * @brief 默认构造函数
     *
     * 创建未初始化的跟踪器（active_=false）。
     * 必须在首次使用前调用initialize()。
     */
    LdmInEkfTracker() = default;

    /**
     * @brief 初始化跟踪器
     *
     * 设置模型参数、初始状态和初始协方差。
     *
     * @param params 模型参数（噪声、运动学参数）
     * @param x0 初始SE2(3)状态（旋转、速度、位置）
     * @param P0 初始协方差矩阵（9×9）
     *
     * @note 构造InEKF对象，并设置active_=true
     */
    void initialize(const Params& params, const Nominal& x0, const CovXi& P0) {
        model_.params = params;

        // 构造预测函数和Q更新函数（策略模式）
        PredictFunc f = [this](const Nominal& x, double dt) { return model_.f(x, dt); };
        UpdateQFunc q = [this](double dt) { return model_.Q(dt); };

        ekf_ = InEKF(std::move(f), std::move(q), P0);
        ekf_.setState(x0);
        active_ = true;
    }

    /**
     * @brief 预测步骤
     *
     * 基于匀速模型预测状态：
     *   X̂₊ = X̂ * exp(ξ_pred)，其中 ξ_pred = (0, 0, v_body·dt)
     *
     * @param dt 时间步长（秒）
     *
     * @note 调用InEKF::predict()，在SE2(3)群上进行右扰动
     */
    void predict(double dt) {
        if (!active_) {
            return;
        }
        ekf_.predict(std::max(0.0, dt));
    }

    /**
     * @brief 更新步骤 - 含C₈最近提升
     *
     * 执行位姿测量更新：
     * 1. 最近提升：从SO(3)/C₈提升到SO(3)
     * 2. 计算残差和雅可比
     * 3. 执行卡尔曼更新
     *
     * @param z 位姿测量（旋转、位置、分支置信度）
     *
     * @note 最近提升保证ROT_X/Z残差在PnP面跳变时保持稳定
     */
    void update(const PoseMeasurement& z) {
        if (!active_) {
            return;
        }

        // Nearest-lift: pick the C₈ symmetry representative closest to the
        // predicted rotation.  This keeps the ROT_X/Z innovation clean even
        // when PnP face assignment jumps by n·45°.
        // 最近提升：选择最接近预测姿态的C₈代表
        const Eigen::Matrix3d R_pred = ekf_.X().R();
        const auto lift              = Model::nearest_lift(z.R_world_body, R_pred);

        // 构造提升后的测量（包含分支置信度）
        PoseMeasurement z_lifted   = z;
        z_lifted.R_world_body      = lift.R_canon;
        z_lifted.branch_confidence = lift.branch_confidence;

        // 执行Invariant EKF更新
        ekf_.update(
            Model::pose_innovation(ekf_.X(), z_lifted), // 残差向量（6维）
            Model::pose_update_H(ekf_.X()),             // 观测雅可比（6×9）
            model_.R(z_lifted));                        // 观测噪声协方差（6×6）
    }

    /**
     * @brief 检查跟踪器是否激活
     * @return true if 已初始化且活跃
     */
    [[nodiscard]] bool active() const noexcept { return active_; }

    /**
     * @brief 获取当前SE2(3)状态
     * @return 常引用，指向SE2(3)群元素（旋转、速度、位置）
     */
    [[nodiscard]] const Nominal& nominal() const noexcept { return ekf_.X(); }

    /**
     * @brief 获取当前协方差矩阵
     * @return 常引用，指向状态协方差矩阵（9×9）
     */
    [[nodiscard]] const CovXi& P() const noexcept { return ekf_.P(); }

private:
    using PredictFunc = std::function<Nominal(const Nominal&, double)>; ///< 预测函数类型
    using UpdateQFunc = std::function<CovXi(double)>;                   ///< Q更新函数类型
    using InEKF       = fcs::L3::InvariantExtendedKalmanFilter<Nominal, Model::NZ>; ///< InEKF类型

    bool active_{false}; ///< 激活标志（是否已初始化）
    Model model_{};      ///< 运动学模型（含参数）
    InEKF ekf_{};        ///< Invariant EKF实例
};

// 编译期断言：验证状态和观测维度
static_assert(LdmKinematic::NX == 9);               ///< 状态维度应为9
static_assert(LdmKinematic::NZ == POSE_UPDATE_MAX); ///< 观测维度应为6

/**
 * @brief LDM高层跟踪器 - 状态机管理与输出接口
 *
 * 管理LDM跟踪的完整生命周期：
 * - 状态机：Idle → Detecting → Tracking ⇄ TempLost
 * - 时间管理：预测时间步长计算、观测时间戳记录
 * - 输出接口：提供LdmState和预测位置
 *
 * ## 状态机设计
 * ```
 * Idle → Detecting → Tracking ⇄ TempLost
 *   ↑                   ↓           ↓
 *   └───────────────────┴───────────┘
 * ```
 *
 * - Idle：未初始化，等待首次观测
 * - Detecting：连续检测，计数达阈值后进入Tracking
 * - Tracking：稳定跟踪，输出有效状态
 * - TempLost：暂时丢失，超时后重置，否则恢复Tracking
 *
 * ## 预测机制
 * - 短期预测：向L4规划层提供未来位置（默认0.5s）
 * - SE2(3)群运算：通过李代数右扰动进行状态预测
 * - 实时更新：每次update()后重新计算预测位置
 *
 * ## 使用流程
 * 1. 构造：LdmTracker(config)
 * 2. 更新：update(timestamp_ns, measurement)
 * 3. 输出：get_output() → LdmState
 * 4. 重置：reset()
 */
class LdmTracker {
public:
    using Model           = LdmKinematic;           ///< 运动学模型类型
    using Nominal         = Model::Nominal;         ///< SE2(3)群类型
    using CovXi           = Model::CovXi;           ///< 状态协方差矩阵类型
    using PoseMeasurement = Model::PoseMeasurement; ///< 位姿测量类型

    /**
     * @brief 默认构造函数
     *
     * 创建未初始化的跟踪器，需要后续设置配置。
     */
    LdmTracker() = default;

    /**
     * @brief 构造函数（带配置）
     * @param config 跟踪配置（模型参数、阈值等）
     */
    explicit LdmTracker(const NaiveLdmConfig& config)
        : config_(config) {}

    /**
     * @brief 更新跟踪器
     *
     * 执行完整的预测-更新循环：
     * 1. 状态Idle：首次观测时初始化
     * 2. 预测：传播到当前时间戳
     * 3. 更新：处理观测（如果存在）
     * 4. 状态机：更新跟踪状态
     *
     * @param timestamp_ns 当前时间戳（纳秒）
     * @param measurement 位姿测量（可选，可能缺失）
     *
     * @note 缺失测量时执行纯预测步骤
     */
    void update(uint64_t timestamp_ns, const std::optional<PoseMeasurement>& measurement) {
        // Idle状态：等待首次观测进行初始化
        if (status_ == TrackerStatus::Idle) {
            if (measurement.has_value()) {
                initialize_(*measurement, timestamp_ns);
                state_machine_(true);
            }
            return;
        }

        // 预测到当前时间戳
        predict_to_(timestamp_ns);

        // 更新（如果观测存在）
        if (measurement.has_value()) {
            target_.update(*measurement);
            last_observation_timestamp_ns_ = timestamp_ns;
            state_machine_(true);
        } else {
            state_machine_(false);
        }
    }

    /**
     * @brief 重置跟踪器
     *
     * 清空所有状态，回到Idle：
     * - 重置EKF跟踪器
     * - 清空时间戳
     * - 重置状态机和计数器
     */
    void reset() noexcept {
        target_                        = LdmInEkfTracker{};
        status_                        = TrackerStatus::Idle;
        detecting_count_               = 0;
        last_timestamp_ns_             = 0;
        last_observation_timestamp_ns_ = 0;
        last_dt_                       = 0.0;
        lost_time_                     = 0.0;
    }

    /**
     * @brief 获取当前状态
     * @return 跟踪状态（Idle/Detecting/Tracking/TempLost）
     */
    [[nodiscard]] TrackerStatus status() const noexcept { return status_; }

    /**
     * @brief 检查是否正在跟踪
     * @return true if 状态为Tracking或TempLost
     */
    [[nodiscard]] bool is_tracking() const noexcept {
        return status_ == TrackerStatus::Tracking || status_ == TrackerStatus::TempLost;
    }

    /**
     * @brief 获取输出状态
     *
     * 构造LdmState输出结构：
     * - SE2(3)状态：旋转、速度、位置
     * - 预测位置：未来时刻的位置（供L4规划使用）
     * - 时间戳：当前和最后观测时间
     * - 跟踪标志：accurate标记
     *
     * @return LdmState（成功）或错误信息（失败）
     *
     * @note 失败情况：tracker未初始化或处于Idle状态
     */
    [[nodiscard]] std::expected<LdmState, std::string> get_output() const noexcept {
        if (!target_.active()) {
            return std::unexpected(std::string("tracker not initialized: target inactive"));
        }
        if (status_ == TrackerStatus::Idle) {
            return std::unexpected(std::string("tracker is idle: status=Idle"));
        }

        LdmState output;
        output.timestamp_ns                  = last_timestamp_ns_;
        output.last_observation_timestamp_ns = last_observation_timestamp_ns_;
        output.status                        = status_;
        output.accurate                      = status_ == TrackerStatus::Tracking;
        output.X                             = target_.nominal();

        // Pre-compute predicted position for L4 aimer.
        // The LDM model uses constant velocity (world-frame) motion.
        // We predict 0.5s ahead by default; L4 will re-predict with its own delay.
        // 预计算未来位置（供L4瞄准器使用）
        // 默认预测0.5s后，L4会根据自己的延迟重新预测
        constexpr double kPredictionHorizon = 0.5;
        output.predicted_position_odom      = predict_position(kPredictionHorizon);
        output.predicted_future_ns =
            last_timestamp_ns_ + static_cast<uint64_t>(kPredictionHorizon * 1.0e9);
        return output;
    }

    /**
     * @brief 预测未来时刻的世界系位置
     *
     * 使用SE2(3)群代数进行预测：
     *   xi = (dθ=0, dv=0, dp=v_body·dt)
     *   X_pred = X * exp(xi)
     *
     * @param dt 预测时间窗口（秒）
     * @return 预测的世界系位置向量
     *
     * @note 活跃状态时返回当前位置，非活跃时返回零向量
     */
    [[nodiscard]] Eigen::Vector3d predict_position(double dt) const noexcept {
        if (!target_.active() || dt <= 0.0) {
            return target_.active() ? target_.nominal().p() : Eigen::Vector3d::Zero();
        }
        const Nominal X_pred = Model::predict_state(target_.nominal(), dt);
        return X_pred.p();
    }

private:
    /**
     * @brief 初始化EKF跟踪器
     *
     * 使用首次观测构造初始状态和协方差：
     * - 初始状态：观测位姿 + 零速度
     * - 初始协方差：对角矩阵（参数化方差）
     *
     * @param measurement 首次位姿观测
     * @param timestamp_ns 时间戳
     */
    void initialize_(const PoseMeasurement& measurement, uint64_t timestamp_ns) {
        // 构造SE2(3)初始状态：旋转 + [零速度, 位置]
        Nominal::IsometriesType t{};
        t[0] = Eigen::Vector3d::Zero();  // 零速度（body系）
        t[1] = measurement.p_world_body; // 位置（world系）

        const Nominal x0(measurement.R_world_body, t);

        // 构造初始协方差矩阵（对角）
        CovXi P0 = CovXi::Zero();
        // 旋转初始方差
        P0.template block<3, 3>(0, 0).setIdentity();
        P0.template block<3, 3>(0, 0) *= config_.initial_sigma_rot * config_.initial_sigma_rot;
        // 速度初始方差
        P0.template block<3, 3>(3, 3).setIdentity();
        P0.template block<3, 3>(3, 3) *=
            config_.initial_sigma_velocity_body * config_.initial_sigma_velocity_body;
        // 位置初始方差
        P0.template block<3, 3>(6, 6).setIdentity();
        P0.template block<3, 3>(6, 6) *=
            config_.initial_sigma_position * config_.initial_sigma_position;

        target_.initialize(config_.model, x0, P0);

        last_timestamp_ns_             = timestamp_ns;
        last_observation_timestamp_ns_ = timestamp_ns;
        last_dt_                       = 0.0;
        lost_time_                     = 0.0;
    }

    /**
     * @brief 预测到指定时间戳
     *
     * 计算时间步长并执行预测：
     *   dt = (timestamp_ns - last_timestamp_ns) / 1e9
     *   X̂₊ = X̂ * exp(ξ_pred)
     *
     * @param timestamp_ns 目标时间戳
     */
    void predict_to_(uint64_t timestamp_ns) {
        last_dt_ = 0.0;
        if (timestamp_ns > last_timestamp_ns_ && last_timestamp_ns_ != 0) {
            last_dt_ = static_cast<double>(timestamp_ns - last_timestamp_ns_) * 1e-9;
        }

        if (timestamp_ns > last_timestamp_ns_) {
            last_timestamp_ns_ = timestamp_ns;
        }

        target_.predict(last_dt_);
    }

    /**
     * @brief 状态机转移
     *
     * 根据观测存在与否更新状态：
     * - Idle → Detecting（首次观测）
     * - Detecting → Tracking（连续检测达阈值）
     * - Tracking ⇄ TempLost（观测丢失/恢复）
     * - TempLost → Idle（超时）
     *
     * @param found 观测是否存在
     */
    void state_machine_(bool found) noexcept {
        switch (status_) {
        case TrackerStatus::Idle:
            if (found) {
                ++detecting_count_;
                status_ = TrackerStatus::Detecting;
            } else {
                reset();
            }
            break;

        case TrackerStatus::Detecting:
            if (found) {
                ++detecting_count_;
                if (detecting_count_ > static_cast<int>(config_.tracking_threshold)) {
                    detecting_count_ = 0;
                    status_          = TrackerStatus::Tracking;
                }
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
                if (lost_time_ >= config_.lost_threshold) {
                    reset();
                }
            }
            break;
        }
    }

    NaiveLdmConfig config_{};                   ///< 配置参数
    TrackerStatus status_{TrackerStatus::Idle}; ///< 跟踪状态
    LdmInEkfTracker target_{};                  ///< InEKF跟踪器

    int detecting_count_{0};                    ///< 检测计数（用于Detecting→Tracking）
    uint64_t last_timestamp_ns_{0};             ///< 最后更新时间戳
    uint64_t last_observation_timestamp_ns_{0}; ///< 最后观测时间戳
    double last_dt_{0.0};                       ///< 最后时间步长
    double lost_time_{0.0};                     ///< 累计丢失时间
};

} // namespace fcs::L3::ldm
