/**
 * @file energy_meter_tracker.cpp
 * @brief 能量机关扩展卡尔曼滤波（EKF）跟踪器实现
 *
 * 本文件实现了能量机关EKF跟踪器的核心逻辑，包括：
 * - EKF初始化与更新
 * - 观测匹配与叶片锁定
 * - 旋转方向检测
 * - 状态机管理
 *
 * 核心算法流程：
 * 1. 初始化（first_meet_u）：根据首次观测初始化EKF状态和协方差
 * 2. 预测（predict）：根据运动模型预测下一时刻状态
 * 3. 更新（update）：
 *    a. 观测匹配：从多个装甲板中选择最匹配的（归一化新息最小）
 *    b. 叶片锁定：锁定特定叶片，避免叶片跳变
 *    c. 角度解包裹：处理角度跳变（如从π到-π）
 *    d. EKF更新：修正状态估计
 *    e. 参数限制：将物理参数限制在合理范围内
 *    f. 方向检测：通过投票确定旋转方向
 * 4. 状态机：管理跟踪器的生命周期状态
 *
 * 关键技术点：
 * - 使用归一化新息（normalized innovation）作为匹配距离
 * - 角度解包裹避免角度跳变
 * - 参数限制确保物理合理性
 * - 投票机制检测旋转方向
 *
 * 潜在风险：
 * - 角度归一化在边界值时可能出现精度问题
 * - 参数边界硬编码，需根据规则调整
 * - 方向检测需要连续多帧，可能延迟较大
 *
 * 优化建议：
 * - 使用自适应过程噪声提高跟踪精度
 * - 优化匹配门限以平衡误匹配和漏匹配
 * - 考虑使用多假设跟踪处理叶片切换
 */

#include "L3_estimation/energy_meter_solver/energy_meter_tracker.hpp"

#include "L3_estimation/energy_meter_solver/types.hpp"
#include "euler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace energy_meter {

// ============================================================================
// 模板实现 — 支持BigRuneModel和SmallRuneModel两种模型
// ============================================================================

/**
 * @brief 初始化扩展卡尔曼滤波器
 *
 * 构造EKF的状态转移函数、观测函数、过程噪声协方差和测量噪声协方差，
 * 并设置初始状态向量和协方差矩阵。
 *
 * @param x0 初始状态向量
 * @param P0 初始协方差矩阵
 *
 * @note 该函数使用lambda表达式封装模型函数，支持Ceres自动微分
 */
template <typename ModelT>
void Tracker<ModelT>::init_ekf(const typename Model::VecX& x0, const typename Model::MatXX& P0) {
    // 状态转移函数：根据运动模型预测下一时刻状态
    auto f = [this](const JetX* x, JetX* xp) {
        Model::template predict_state<JetX>(x, JetX(dt_), dir_, xp);
    };

    // 观测函数：根据状态和装甲板ID计算预测观测值
    auto h = [this](const JetX* x, JetX* z) {
        Model::template measure_state<JetX>(x, armor_id_, z);
    };

    // 过程噪声协方差函数：根据时间增量计算Q矩阵
    auto q = [this]() -> typename Model::MatXX { return model_.Q(dt_); };

    // 测量噪声协方差函数：根据观测值计算R矩阵
    auto r = [this](const typename Model::VecZ& z) -> typename Model::MatZZ { return model_.R(z); };

    // 构造EKF实例并设置初始状态
    ekf_.emplace(f, h, q, r, P0);
    ekf_->setState(x0);
}

/**
 * @brief EKF更新步骤（根据观测修正状态）
 *
 * 执行观测匹配、叶片锁定、角度解包裹、EKF更新和方向检测。
 *
 * 算法流程：
 * 1. 观测匹配：从多个装甲板观测中选择最匹配的（归一化新息最小）
 *    - 若已锁定叶片，只匹配该叶片
 *    - 若未锁定叶片，遍历所有叶片寻找最佳匹配
 * 2. 叶片锁定管理：
 *    - 若锁定叶片连续丢失多帧，解锁并重新搜索
 *    - 若匹配成功，重置丢失计数
 * 3. 角度解包裹：处理角度跳变（如从π到-π）
 * 4. EKF更新：根据观测值修正状态估计
 * 5. 参数限制：将物理参数限制在合理范围内（仅大符）
 * 6. 方向检测：通过投票确定旋转方向
 *
 * @param rcenter_position 能量机关中心位置
 * @param target_positions 目标装甲板位置列表
 * @param target_quats 目标装甲板姿态四元数列表
 * @return 更新成功返回true，失败返回false
 *
 * @note 归一化新息用于匹配距离，避免量纲问题
 * @note 角度解包裹确保角度连续性
 */
template <typename ModelT>
bool Tracker<ModelT>::update(
    const Eigen::Vector3d& rcenter_position, const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats) {

    // 若EKF未初始化或观测为空，更新状态机并返回失败
    if (!ekf_ || target_positions.empty() || target_quats.empty()) {
        state_machine(false);
        return false;
    }

    using VecZ = typename Model::VecZ;

    const auto& pos          = rcenter_position;
    const auto& x_pre        = ekf_->X(); // 预测状态
    constexpr int armors_num = ARMORS_NUM;

    // 最佳匹配结果
    VecZ best_meas   = VecZ::Zero();
    double best_cost = std::numeric_limits<double>::max(); // 归一化新息平方
    int best_id      = -1;                                 // 最佳装甲板ID

    /**
     * @brief 观测匹配lambda函数
     *
     * 遍历指定范围的装甲板ID，计算归一化新息，选择最小者。
     *
     * @param meas 观测向量
     * @param id_start 起始装甲板ID（包含）
     * @param id_end 结束装甲板ID（不包含）
     */
    auto match_observation = [&](const VecZ& meas, int id_start, int id_end) {
        for (int id = id_start; id < id_end; ++id) {
            // 计算预测观测值
            VecZ z_pred = model_.h(x_pre, id);

            // 计算新息（观测残差）
            VecZ nu = meas - z_pred;

            // 归一化滚转角新息（处理角度周期性）
            nu[ARMOR_ROLL] = normalize_rad(nu[ARMOR_ROLL]);

            // 获取测量噪声协方差对角元素
            const auto R_diag = model_.R_diag(z_pred);

            // 计算归一化新息平方：d² = νᵀ R⁻¹ ν
            const double d2 = (nu.array().square() / R_diag.array()).sum();

            // 检查匹配有效性：
            // 1. 归一化新息有限（非NaN/Inf）
            // 2. 归一化新息小于门限（避免误匹配）
            // 3. 归一化新息小于当前最佳（选择最小者）
            // 4. 滚转角新息小于最大允许值（避免跨叶片匹配）
            constexpr double kMaxRollResidual = M_PI / static_cast<double>(ARMORS_NUM);
            if (std::isfinite(d2) && d2 < params.matcher_gate && d2 < best_cost
                && std::abs(nu[ARMOR_ROLL]) < kMaxRollResidual) {
                best_cost = d2;
                best_id   = id;
                best_meas = meas;
            }
        }
    };

    // 遍历所有观测，进行匹配
    for (const auto& target_quat : target_quats) {
        // 从四元数提取欧拉角
        const auto euler              = math_fuxk::rpy(target_quat);
        const auto [roll, pitch, yaw] = euler.rpy();

        // 构造观测向量
        VecZ meas;
        meas[CENTER_X]   = pos.x();
        meas[CENTER_Y]   = pos.y();
        meas[CENTER_Z]   = pos.z();
        meas[RUNE_YAW]   = yaw;
        meas[ARMOR_ROLL] = roll;

        // 根据叶片锁定状态决定匹配范围
        if (blade_locked_) {
            // 若已锁定叶片，只匹配该叶片
            match_observation(meas, armor_id_, armor_id_ + 1);
        } else {
            // 若未锁定叶片，遍历所有叶片
            match_observation(meas, 0, armors_num);
        }
    }

    // 叶片锁定管理：若锁定叶片连续丢失，解锁并重新搜索
    if (blade_locked_ && best_id < 0) {
        blade_lost_frames_++;
        if (blade_lost_frames_ >= params.blade_unlock_frames) {
            // 达到解锁阈值，解锁并重新搜索所有叶片
            blade_locked_      = false;
            blade_lost_frames_ = 0;
            SPDLOG_INFO(
                "energy_meter: blade lock released after {} frames", params.blade_unlock_frames);

            // 重新搜索所有叶片
            best_cost = std::numeric_limits<double>::max();
            best_id   = -1;
            for (const auto& target_quat : target_quats) {
                const auto euler              = math_fuxk::rpy(target_quat);
                const auto [roll, pitch, yaw] = euler.rpy();

                VecZ meas;
                meas[CENTER_X]   = pos.x();
                meas[CENTER_Y]   = pos.y();
                meas[CENTER_Z]   = pos.z();
                meas[RUNE_YAW]   = yaw;
                meas[ARMOR_ROLL] = roll;

                match_observation(meas, 0, armors_num);
            }
        }
    } else if (blade_locked_ && best_id >= 0) {
        // 匹配成功，重置丢失计数
        blade_lost_frames_ = 0;
    }

    // 若未找到有效匹配，更新状态机并返回失败
    if (best_id < 0) {
        state_machine(false);
        return false;
    }

    // 更新状态机（找到目标）
    state_machine(true);

    // 若进入跟踪状态且未锁定叶片，锁定当前叶片
    if (state == TRACKING && !blade_locked_) {
        blade_locked_      = true;
        blade_lost_frames_ = 0;
        SPDLOG_INFO("energy_meter: blade locked to id={}", best_id);
    }

    // 角度解包裹：处理角度跳变（如从π到-π）
    // 计算叶片角度偏移：blade_offset = 2π/5 * id
    const double blade_offset =
        static_cast<double>(best_id) * 2.0 * M_PI / static_cast<double>(ARMORS_NUM);

    // 计算基准滚转角（去除叶片偏移）
    // 使用unwrap_rad确保角度连续性
    const double observed_base_roll =
        unwrap_rad(x_pre[Model::State::THETA], best_meas[ARMOR_ROLL] - blade_offset);

    // 构造最终观测向量（包含解包裹后的角度）
    VecZ z;
    z[CENTER_X]   = pos.x();
    z[CENTER_Y]   = pos.y();
    z[CENTER_Z]   = pos.z();
    z[RUNE_YAW]   = best_meas[RUNE_YAW];
    z[ARMOR_ROLL] = observed_base_roll + blade_offset;

    // 执行EKF更新
    armor_id_ = best_id;
    ekf_->update(z);

    // 参数限制：将物理参数限制在合理范围内（仅大符需要）
    // 确保振幅a和角频率ω在规则规定的范围内
    if constexpr (std::is_same_v<ModelT, BigRuneModel>) {
        auto x      = ekf_->X();
        x(Model::A) = std::clamp(x(Model::A), BigRuneModel::A_LOWER, BigRuneModel::A_UPPER);
        x(Model::W) = std::clamp(x(Model::W), BigRuneModel::W_LOWER, BigRuneModel::W_UPPER);
        ekf_->setState(x);
    }

    // 旋转方向检测：通过投票确定旋转方向
    // 分析基准滚转角的变化方向，累计投票
    if (!dir_locked_) {
        if (dir_detect_count_ > 0) {
            // 计算滚转角变化量
            const double droll = normalize_rad(observed_base_roll - prev_observed_roll_);

            // 最小变化阈值：避免噪声干扰
            constexpr double kMinDelta = 0.005; // 约0.3度

            // 根据变化方向投票
            if (droll > kMinDelta) {
                dir_votes_pos_++; // 逆时针旋转
            } else if (droll < -kMinDelta) {
                dir_votes_neg_++; // 顺时针旋转
            }

            // 若票数不同，更新当前方向
            if (dir_votes_pos_ != dir_votes_neg_) {
                dir_ = (dir_votes_pos_ > dir_votes_neg_) ? 1 : -1;
            }
        }
        prev_observed_roll_ = observed_base_roll;
        dir_detect_count_++;

        // 达到检测帧数，锁定方向
        if (dir_detect_count_ >= DIR_DETECT_FRAMES) {
            dir_        = (dir_votes_pos_ >= dir_votes_neg_) ? 1 : -1;
            dir_locked_ = true;
            SPDLOG_INFO(
                "energy_meter: direction locked: dir={}, votes +{}/-{}", dir_, dir_votes_pos_,
                dir_votes_neg_);
        }
    }

    return true;
}

/**
 * @brief EKF预测步骤
 *
 * 根据运动模型预测下一时刻状态。
 * 若时间增量dt为负，则设为0（不预测）。
 *
 * @param dt 时间增量（秒）
 */
template <typename ModelT>
void Tracker<ModelT>::predict(double dt) {
    if (!ekf_) {
        return;
    }
    dt_ = std::max(0.0, dt); // 确保dt非负
    ekf_->predict();
}

/**
 * @brief 单步更新（预测+更新）
 *
 * 组合predict()和update()为单步操作。
 * 这是跟踪器的主要接口，每帧调用一次。
 *
 * @param dt 时间增量（秒）
 * @param rcenter_position 能量机关中心位置
 * @param target_positions 目标装甲板位置列表
 * @param target_quats 目标装甲板姿态四元数列表
 */
template <typename ModelT>
void Tracker<ModelT>::step(
    double dt, const Eigen::Vector3d& rcenter_position,
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats) {
    predict(dt);
    update(rcenter_position, target_positions, target_quats);
}

// ============================================================================
// 大符初始化特化（8维状态，包含TAU、A、W）
// ============================================================================

/**
 * @brief 大符跟踪器初始化（BigRuneModel特化）
 *
 * 根据首次观测初始化8维状态向量和8x8协方差矩阵。
 * 状态向量：[XC, YC, ZC, YAW, THETA, TAU, A, W]
 *
 * 初始化策略：
 * 1. 位置和角度：直接从观测值初始化
 * 2. 累积时间TAU：若有上一轮状态，继承；否则设为0
 * 3. 振幅A和角频率W：
 *    - 若有上一轮状态，继承
 *    - 否则使用中值初始化（A=0.9125, W=1.942）
 * 4. 协方差矩阵：
 *    - 位置和角度：中等不确定性（0.1）
 *    - 振幅A：较高不确定性（0.08）
 *    - 角频率W：中等不确定性（0.05）
 *    - 累积时间TAU：较高不确定性（4.0）
 *
 * @param r_center 能量机关中心位置（相机坐标系）
 * @param target_pos 目标装甲板位置（相机坐标系）
 * @param target_quat 目标装甲板姿态四元数
 * @return 初始化成功返回true
 */
template <>
bool Tracker<BigRuneModel>::first_meet_u(
    const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_quat) {

    // 从四元数提取欧拉角
    const auto euler              = math_fuxk::rpy(target_quat);
    const auto [roll, pitch, yaw] = euler.rpy();

    // 计算能量机关半径（装甲板到中心的距离）
    const double observed_radius = (target_pos - r_center).norm();
    model_.params.radius         = observed_radius;

    // 初始化8维状态向量
    BigRuneModel::VecX x0   = BigRuneModel::VecX::Zero();
    x0(BigRuneModel::XC)    = r_center.x();
    x0(BigRuneModel::YC)    = r_center.y();
    x0(BigRuneModel::ZC)    = r_center.z();
    x0(BigRuneModel::YAW)   = yaw;
    x0(BigRuneModel::THETA) = roll;

    // 恢复上一轮状态（若存在）
    // 这允许在能量机关重新激活后恢复参数估计
    if (last_state_) {
        x0(BigRuneModel::TAU) = (*last_state_)(BigRuneModel::TAU);
        x0(BigRuneModel::A)   = (*last_state_)(BigRuneModel::A);
        x0(BigRuneModel::W)   = (*last_state_)(BigRuneModel::W);
        last_state_.reset();
    } else {
        // 使用中值初始化参数
        x0(BigRuneModel::TAU) = 0.0;
        x0(BigRuneModel::A)   = 0.9125; // 区间[0.780, 1.045]的中值
        x0(BigRuneModel::W)   = 1.942;  // 区间[1.884, 2.000]的中值
    }

    // 初始化8x8协方差矩阵
    // 对角阵，各状态变量具有不同的不确定性
    BigRuneModel::MatXX P0                   = BigRuneModel::MatXX::Identity() * 0.1;
    P0(BigRuneModel::A, BigRuneModel::A)     = 0.08; // 振幅不确定性较高
    P0(BigRuneModel::W, BigRuneModel::W)     = 0.05; // 角频率不确定性中等
    P0(BigRuneModel::TAU, BigRuneModel::TAU) = 4.0;  // 累积时间不确定性较高

    // 初始化EKF
    init_ekf(x0, P0);

    // 初始化方向检测状态
    dir_                = 1; // 默认逆时针
    prev_observed_roll_ = roll;
    dir_detect_count_   = 0;
    dir_votes_pos_      = 0;
    dir_votes_neg_      = 0;
    dir_locked_         = false;

    // 重置叶片锁定状态
    reset_blade_lock();

    // 更新状态机
    state_machine(true);
    return true;
}

// ============================================================================
// 小符初始化特化（5维状态，恒定速度）
// ============================================================================

/**
 * @brief 小符跟踪器初始化（SmallRuneModel特化）
 *
 * 根据首次观测初始化5维状态向量和5x5协方差矩阵。
 * 状态向量：[XC, YC, ZC, YAW, THETA]
 *
 * 初始化策略：
 * 1. 位置和角度：直接从观测值初始化
 * 2. 协方差矩阵：
 *    - 位置：中等不确定性（0.1）
 *    - 滚转角：较高不确定性（1.0），因为恒定速度模型假设较强
 *
 * @param r_center 能量机关中心位置（相机坐标系）
 * @param target_pos 目标装甲板位置（相机坐标系）
 * @param target_quat 目标装甲板姿态四元数
 * @return 初始化成功返回true
 */
template <>
bool Tracker<SmallRuneModel>::first_meet_u(
    const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_quat) {

    // 从四元数提取欧拉角
    const auto euler              = math_fuxk::rpy(target_quat);
    const auto [roll, pitch, yaw] = euler.rpy();

    // 计算能量机关半径（装甲板到中心的距离）
    const double observed_radius = (target_pos - r_center).norm();
    model_.params.radius         = observed_radius;

    // 初始化5维状态向量
    SmallRuneModel::VecX x0   = SmallRuneModel::VecX::Zero();
    x0(SmallRuneModel::XC)    = r_center.x();
    x0(SmallRuneModel::YC)    = r_center.y();
    x0(SmallRuneModel::ZC)    = r_center.z();
    x0(SmallRuneModel::YAW)   = yaw;
    x0(SmallRuneModel::THETA) = roll;

    // 初始化5x5协方差矩阵
    // 对角阵，位置不确定性中等，滚转角不确定性较高
    SmallRuneModel::MatXX P0                         = SmallRuneModel::MatXX::Identity() * 0.1;
    P0(SmallRuneModel::THETA, SmallRuneModel::THETA) = 1.0; // 滚转角不确定性较高

    // 初始化EKF
    init_ekf(x0, P0);

    // 初始化方向检测状态
    dir_                = 1; // 默认逆时针
    prev_observed_roll_ = roll;
    dir_detect_count_   = 0;
    dir_votes_pos_      = 0;
    dir_votes_neg_      = 0;
    dir_locked_         = false;

    // 重置叶片锁定状态
    reset_blade_lock();

    // 更新状态机
    state_machine(true);
    return true;
}

/**
 * @brief 状态机更新
 *
 * 管理跟踪器的生命周期状态，实现状态转换逻辑。
 *
 * 状态转换图：
 * - IDLE -> DETECTING：首次检测到目标
 * - DETECTING -> TRACKING：连续检测超过阈值帧数
 * - DETECTING -> IDLE：检测失败，清空状态
 * - TRACKING -> TEMP_LOST：丢失目标
 * - TEMP_LOST -> TRACKING：重新找到目标
 * - TEMP_LOST -> IDLE：连续丢失超过阈值帧数，清空状态
 *
 * 状态保存与恢复：
 * - 当从IDLE或TEMP_LOST退出时，保存当前EKF状态到last_state_
 * - 当重新初始化时，恢复last_state_中的参数（仅大符）
 *
 * @param found 是否找到目标
 */
template <typename ModelT>
void Tracker<ModelT>::state_machine(bool found) {
    switch (state) {
    case IDLE:
        if (found) {
            // 空闲状态下检测到目标，进入检测状态
            ++detecting_count_;
            state = DETECTING;
        } else {
            // 未检测到目标，保存状态并清空EKF
            if (ekf_)
                last_state_.emplace(ekf_->X());
            ekf_.reset();
            detecting_count_ = 0;
        }
        break;

    case DETECTING:
        if (found) {
            // 检测状态下继续检测
            ++detecting_count_;
            if (detecting_count_ > params.tracking_thres) {
                // 连续检测超过阈值，进入跟踪状态
                state            = TRACKING;
                detecting_count_ = 0;
            }
        } else {
            // 检测失败，保存状态并返回空闲状态
            detecting_count_ = 0;
            if (ekf_)
                last_state_.emplace(ekf_->X());
            ekf_.reset();
            state = IDLE;
        }
        break;

    case TRACKING:
        if (!found) {
            // 跟踪状态下丢失目标，进入暂时丢失状态
            state       = TEMP_LOST;
            lost_count_ = 1;
        }
        break;

    case TEMP_LOST:
        if (found) {
            // 暂时丢失状态下重新找到目标，返回跟踪状态
            lost_count_ = 0;
            state       = TRACKING;
        } else {
            // 继续丢失
            ++lost_count_;
            if (lost_count_ > params.lost_thres) {
                // 连续丢失超过阈值，清空状态并返回空闲状态
                lost_count_      = 0;
                detecting_count_ = 0;
                if (ekf_)
                    last_state_.emplace(ekf_->X());
                ekf_.reset();
                reset_blade_lock();
                state = IDLE;
            }
        }
        break;
    }
}

// ============================================================================
// 显式实例化模板（支持BigRuneModel和SmallRuneModel）
// ============================================================================

/// 大符跟踪器模板实例化
template struct Tracker<BigRuneModel>;

/// 小符跟踪器模板实例化
template struct Tracker<SmallRuneModel>;

} // namespace energy_meter
