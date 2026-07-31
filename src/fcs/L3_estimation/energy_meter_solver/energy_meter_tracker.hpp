/**
 * @file energy_meter_tracker.hpp
 * @brief 能量机关扩展卡尔曼滤波（EKF）跟踪器定义
 *
 * 本文件定义了能量机关的EKF跟踪器模板，支持大小符两种运动模型。
 * 跟踪器负责维护能量机关的状态估计，包括位置、角度、旋转方向等。
 *
 * 核心算法原理：
 * 1. 扩展卡尔曼滤波（EKF）：
 *    - 预测步骤：根据运动模型预测下一时刻状态
 *    - 更新步骤：根据观测值修正状态估计
 *    - 使用Ceres自动微分计算雅可比矩阵
 * 2. 状态机管理：
 *    - IDLE：空闲状态，未检测到目标
 *    - DETECTING：检测状态，连续检测到目标但未确认跟踪
 *    - TRACKING：跟踪状态，已确认跟踪目标
 *    - TEMP_LOST：暂时丢失状态，短暂丢失目标
 * 3. 叶片锁定机制：
 *    - 跟踪特定叶片，避免叶片跳变
 *    - 当连续丢失多帧时解锁，重新搜索
 * 4. 旋转方向检测：
 *    - 通过观测角度变化投票决定旋转方向
 *    - 需要连续多帧（20帧）确认方向
 *
 * 关键数据结构：
 * - Tracker<ModelT>: 模板化的EKF跟踪器（支持BigRuneModel和SmallRuneModel）
 * - State: 状态机枚举（IDLE/DETECTING/TRACKING/TEMP_LOST）
 * - Params: 跟踪器参数（丢失阈值、跟踪阈值、匹配门限等）
 *
 * 使用流程：
 * 1. 设置参数：set_params()
 * 2. 初始化跟踪：first_meet_u()
 * 3. 每帧更新：step() 或 predict() + update()
 * 4. 获取状态：get_state()
 *
 * 优化建议：
 * - 调整过程噪声参数以平衡预测和观测的信任度
 * - 调整匹配门限以平衡误匹配和漏匹配
 * - 优化叶片锁定和方向检测的帧数阈值
 */

#pragma once

#include "L3_estimation/energy_meter_solver/motion_model.hpp"

#include "L3_estimation/tracker/extended_kalman_filter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace energy_meter {

/**
 * @struct Tracker
 * @brief 模板化的能量机关EKF跟踪器
 *
 * 支持大符和小符两种运动模型，通过模板参数指定。
 * 使用扩展卡尔曼滤波维护能量机关的状态估计。
 *
 * @tparam ModelT 运动模型类型（BigRuneModel或SmallRuneModel）
 */
template <typename ModelT>
struct Tracker {
    using Model = ModelT;  ///< 运动模型类型
    using JetX  = ceres::Jet<double, Model::NX>;  ///< Ceres自动微分类型

    /// 预测函数类型（用于EKF）
    using PredictFunc = std::function<void(const JetX*, JetX*)>;
    /// 观测函数类型（用于EKF）
    using MeasureFunc = std::function<void(const JetX*, JetX*)>;
    /// EKF类型定义
    using EKF         = ExtendedKalmanFilter<Model::NX, Model::NZ, PredictFunc, MeasureFunc>;

    /**
     * @struct Params
     * @brief 跟踪器参数
     *
     * 包含状态机阈值、匹配门限、叶片解锁帧数等参数。
     */
    struct Params {
        int lost_thres{10};  ///< 丢失阈值：连续丢失此帧数后重置为IDLE
        int tracking_thres{3};  ///< 跟踪阈值：连续检测到此帧数后进入TRACKING状态
        double matcher_gate{25.0};  ///< 匹配门限：归一化新息平方阈值，超过此值拒绝匹配
        int blade_unlock_frames{10};  ///< 叶片解锁帧数：连续丢失此帧数后解锁叶片锁定
        typename Model::Params model_params{};  ///< 运动模型参数
    } params;

    /**
     * @enum State
     * @brief 跟踪器状态机枚举
     *
     * 状态转换图：
     * IDLE -> DETECTING -> TRACKING -> TEMP_LOST -> TRACKING (if found)
     *                                      |
     *                                      v
     *                                     IDLE (if lost too long)
     */
    enum State : uint8_t {
        IDLE,       ///< 空闲状态：未检测到目标
        DETECTING,  ///< 检测状态：连续检测到目标但未确认跟踪
        TRACKING,   ///< 跟踪状态：已确认跟踪目标
        TEMP_LOST,  ///< 暂时丢失状态：短暂丢失目标
    } state{IDLE};

    /// 默认构造函数
    Tracker() = default;

    /**
     * @brief 参数构造函数
     * @param p 跟踪器参数
     */
    explicit Tracker(const Params& p) { set_params(p); }

    /**
     * @brief 设置跟踪器参数
     * @param p 跟踪器参数
     */
    void set_params(const Params& p) {
        params        = p;
        model_.params = params.model_params;
    }

    // ── EKF访问接口 ──

    /**
     * @brief 查询EKF是否已初始化
     * @return 若EKF已初始化返回true
     */
    bool has_ekf() const { return ekf_.has_value(); }

    /**
     * @brief 重置EKF
     */
    void reset_ekf() { ekf_.reset(); }

    /**
     * @brief 获取当前状态估计
     * @return 当前状态向量（若EKF未初始化返回零向量）
     */
    auto get_state() const -> const typename Model::VecX& {
        static typename Model::VecX zero = Model::VecX::Zero();
        return ekf_ ? ekf_->X() : zero;
    }

    /**
     * @brief 获取能量机关半径
     * @return 能量机关半径（米）
     */
    auto getRadius() const -> double { return model_.params.radius; }

    /**
     * @brief 设置能量机关半径
     * @param radius 能量机关半径（米）
     */
    void setRadius(double radius) { model_.params.radius = radius; }

    /**
     * @brief 获取当前锁定的叶片ID
     * @return 叶片ID（0-4）
     */
    auto getCurrentBladeId() const -> int { return armor_id_; }

    /**
     * @brief 获取旋转方向
     * @return 旋转方向（+1为逆时针，-1为顺时针）
     */
    auto getDirection() const -> int { return dir_; }

    /**
     * @brief 重置叶片锁定状态
     */
    void reset_blade_lock() {
        blade_locked_      = false;
        blade_lost_frames_ = 0;
    }

    // ── 生命周期管理 ──

    /**
     * @brief 初始化跟踪器（首次遇到目标）
     *
     * 根据首次观测初始化EKF状态向量和协方差矩阵。
     *
     * @param r_center 能量机关中心位置（相机坐标系）
     * @param target_pos 目标装甲板位置（相机坐标系）
     * @param target_quat 目标装甲板姿态四元数
     * @return 初始化成功返回true
     */
    bool first_meet_u(
        const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& target_quat);

    /**
     * @brief 预测步骤（根据运动模型预测状态）
     * @param dt 时间增量（秒）
     */
    void predict(double dt);

    /**
     * @brief 更新步骤（根据观测修正状态）
     *
     * 执行观测匹配和EKF更新：
     * 1. 匹配观测值与预测值（选择最匹配的装甲板）
     * 2. 执行EKF更新步骤
     * 3. 更新状态机
     *
     * @param rcenter_position 能量机关中心位置
     * @param target_positions 目标装甲板位置列表
     * @param target_quats 目标装甲板姿态四元数列表
     * @return 更新成功返回true，失败返回false
     */
    bool update(
        const Eigen::Vector3d& rcenter_position,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats);

    /**
     * @brief 单步更新（预测+更新）
     *
     * 组合predict()和update()为单步操作。
     *
     * @param dt 时间增量（秒）
     * @param rcenter_position 能量机关中心位置
     * @param target_positions 目标装甲板位置列表
     * @param target_quats 目标装甲板姿态四元数列表
     */
    void step(
        double dt, const Eigen::Vector3d& rcenter_position,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats);

private:
    /**
     * @brief 状态机更新
     * @param found 是否找到目标
     */
    void state_machine(bool found);

    /**
     * @brief 初始化EKF
     * @param x0 初始状态向量
     * @param P0 初始协方差矩阵
     */
    void init_ekf(const typename Model::VecX& x0, const typename Model::MatXX& P0);

    Model model_{};  ///< 运动模型实例
    std::optional<EKF> ekf_;  ///< EKF实例（使用optional延迟构造）

    // ── 旋转方向检测状态 ──
    int dir_{1};  ///< 旋转方向（+1为逆时针，-1为顺时针）
    double prev_observed_roll_{0.0};  ///< 上一帧观测的基准滚转角（用于方向检测）
    int dir_votes_pos_{0};  ///< 逆时针方向票数
    int dir_votes_neg_{0};  ///< 顺时针方向票数
    int dir_detect_count_{0};  ///< 方向检测帧计数
    bool dir_locked_{false};  ///< 方向是否已锁定
    static constexpr int DIR_DETECT_FRAMES = 20;  ///< 方向检测所需帧数

    // ── 状态机计数器 ──
    int detecting_count_{0};  ///< 检测状态计数器
    int lost_count_{0};  ///< 丢失状态计数器

    // ── 叶片锁定状态 ──
    bool blade_locked_{false};  ///< 是否锁定特定叶片
    int blade_lost_frames_{0};  ///< 叶片连续丢失帧数

    // ── 时间和叶片ID ──
    double dt_{0.0};  ///< 时间增量
    int armor_id_{0};  ///< 当前锁定的装甲板ID（0-4）
    std::optional<typename Model::VecX> last_state_{};  ///< 上一轮跟踪的状态（用于状态恢复）
};

// ============================================================================
// 跟踪器变体类型定义 — 用于energy_meter_systems在投票器决策后选择
// ============================================================================

using SmallTracker = Tracker<SmallRuneModel>;  ///< 小符跟踪器类型
using BigTracker   = Tracker<BigRuneModel>;    ///< 大符跟踪器类型
using AnyTracker   = std::variant<std::monostate, SmallTracker, BigTracker>;  ///< 跟踪器变体类型

} // namespace energy_meter
