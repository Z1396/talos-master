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
#include <ceres/jet.h>   // ceres Jet：用于自动微分，求解EKF雅可比矩阵

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
    using Model             = ModelT;                            ///< 运动模型类型别名
    static constexpr int NX = Model::NX;                         ///< EKF状态向量维度（编译期常量）
    static constexpr int NZ = Model::NZ;                         ///< 观测测量向量维度（编译期常量）

    using VecX   = typename Model::VecX;                         ///< 状态向量 Eigen 列向量类型
    using VecZ   = typename Model::VecZ;                         ///< 观测测量向量 Eigen 列向量类型
    using MatXX  = typename Model::MatXX;                        ///< 状态协方差矩阵 NX×NX
    using MatZZ  = typename Model::MatZZ;                        ///< 测量噪声协方差 NZ×NZ
    using Params = typename Model::Params;                       ///< 运动模型自定义参数结构体

    using JetX        = ceres::Jet<double, NX>;                  ///< Ceres Jet自动微分类型，带NX个微分变量，用来算雅可比，不用手写雅可比矩阵
    using PredictFunc = std::function<void(const JetX*, JetX*)>; ///< 预测函数签名：输入状态Jet，输出预测后状态Jet，给EKF内部做自动微分
    using MeasureFunc = std::function<void(const JetX*, JetX*)>; ///< 观测函数签名：输入状态Jet，输出观测预测Jet
    using EKF         = ExtendedKalmanFilter<NX, NZ, PredictFunc, MeasureFunc>; ///< 实例化扩展卡尔曼滤波器类型

    // 默认构造函数，成员全部走类内初始化，此时EKF是空，active=false，需要调用initialize才可用
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
     * - 状态向量中使用log(r0)和log(r1)（避免负值约束，半径r不能小于0，log值域负无穷~正无穷，滤波数值更稳定）
     * - 过程噪声需要缩放：Q(log_r) /= r²（保持线性空间不确定性）
     * - 原因：d(log r)/dt = (1/r) * dr/dt，随机变量变换，协方差传播
     *
     * @param params 模型参数
     * @param x0 初始状态向量
     * @param P0 初始协方差矩阵
     */
    void initialize(const Params& params, const VecX& x0, const MatXX& P0) noexcept {
        model_.params = params;   // 保存运动模型配置参数
        dt_           = 0.0;      // 时间差初始化为0
        armor_id_     = 0;        // 装甲板编号初始0

        // 创建EKF预测函数f(x)：状态转移方程，使用ceres::Jet做自动微分
        // lambda捕获this，把Jet类型转发给模型predict_state模板函数
        auto f = [this](const JetX* x, JetX* xp) {
            Model::template predict_state<JetX>(x, JetX(dt_), xp);
        };

        // 创建观测函数h(x)：根据当前状态预测观测值
        auto h = [this](const JetX* x, JetX* z) {
            Model::template measure_state<JetX>(x, armor_id_, z);
        };

        // 过程噪声协方差Q的生成回调函数
        auto q = [this]() -> MatXX {
            auto Q = model_.Q(dt_); // 调用运动模型获取基础过程噪声矩阵
            // if constexpr：编译期分支，只有模型开启对数半径才编译下面代码，运行时无开销
            if constexpr (Model::kHasLogRadii) {
                // log(r) 变量的协方差变换：Var(log r) = Var(r) / r²
                const double r0 = std::exp(x_[LOG_R0]); // 从log域还原真实半径r0
                const double r1 = std::exp(x_[LOG_R1]); // 还原真实半径r1
                Q(LOG_R0, LOG_R0) /= (r0 * r0); // 缩放对应状态的过程噪声方差
                Q(LOG_R1, LOG_R1) /= (r1 * r1);
            }
            return Q;
        };

        // 测量噪声协方差R回调；支持外部传入R覆盖模型自带R
        auto r = [this](const VecZ& z) -> MatZZ {
            if (update_R_override_.has_value()) {
                return *update_R_override_; // 使用外部覆盖的R（比如PNP输出的协方差）
            }
            return model_.R(z);             // 使用运动模型计算测量噪声R
        };

        // 构造EKF对象：传入状态转移、观测、Q回调、R回调、初始协方差P0
        ekf_ = EKF(f, h, q, r, P0);
        ekf_.setState(x0); // 设置滤波器初始状态
        x_      = x0;      // 缓存一份当前状态副本
        active_ = true;    // 标记该目标滤波器激活，可以执行预测更新
    }

    /**
     * @brief 预测步（时间更新）
     *
     * @param dt 时间间隔（秒）
     *
     * @note 调用EKF::predict()，内部使用ceres Jet自动微分计算状态转移雅可比矩阵，不用手写Jf
     */
    void predict(double dt) noexcept {
        if (!active_) { // 滤波器未激活直接返回
            return;
        }
        dt_ = std::max(0.0, dt); // 时间差不能为负数，防止时间回退
        x_  = ekf_.predict();    // 执行EKF预测，返回预测后状态，缓存到成员x_
    }

    /**
     * @brief 更新步（测量更新，使用模型默认R）
     *
     * @param z 测量向量
     * @param armor_id 装甲板ID（用于选择观测哪个装甲板，机器人多个装甲共用一套状态）
     *
     * @note 更新后自动解相关log_r0和log_r1（避免虚假相关性）
     * 虚假相关性：数值误差导致两个物理独立的半径状态协方差矩阵出现非0互相关项，滤波发散
     */
    void update(const VecZ& z, int armor_id) noexcept {
        if (!active_) {
            return;
        }
        armor_id_ = Model::clamp_armor_id(armor_id); // 限制装甲id合法范围，防止越界
        x_        = ekf_.update(z);                  // EKF观测更新，返回更新后状态
        update_R_override_.reset();                 // 清除外部R覆盖标记

        // 编译期判断：如果模型使用对数半径，执行解相关操作，把P矩阵LOG_R0与LOG_R1之间互相关项强制置0
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
     * @note 用于PNP协方差传递等场景；PNP求解会输出位姿协方差，直接喂给滤波器当做观测噪声
     */
    void update(const VecZ& z, int armor_id, const MatZZ& R) noexcept {
        if (!active_) {
            return;
        }
        update_R_override_ = R;       // std::optional存储外部传入R，r回调会读取这个值
        armor_id_          = Model::clamp_armor_id(armor_id);
        x_                 = ekf_.update(z);
        update_R_override_.reset();   // 使用完立刻清空，下一次update恢复模型自带R

        if constexpr (Model::kHasLogRadii) {
            ekf_.decorrelate(LOG_R0, LOG_R1);
        }
    }

    /// 查询滤波器是否激活
    [[nodiscard]] bool active() const noexcept { return active_; }

    /// 获取当前滤波器状态向量 const只读
    [[nodiscard]] const VecX& x() const noexcept { return x_; }

    /// 获取状态协方差矩阵P
    [[nodiscard]] const MatXX& P() const noexcept { return ekf_.P(); }

    using MatXZ = Eigen::Matrix<double, NX, NZ>;
    /// 获取卡尔曼增益矩阵K
    [[nodiscard]] const MatXZ& K_gain() const noexcept { return ekf_.K_gain(); }
    /// 获取本次使用的过程噪声Q矩阵
    [[nodiscard]] const MatXX& Q_mat() const noexcept { return ekf_.Q_mat(); }
    /// 获取本次使用测量噪声R矩阵
    [[nodiscard]] const MatZZ& R_mat() const noexcept { return ekf_.R_mat(); }

    /**
     * @brief 获取滤波器完整收敛状态，对外输出给上层逻辑、日志、调试
     * @return FilterConvergenceState 收敛状态结构体，包含收敛标记、归一化新息平方、协方差最大值、连续收敛/发散计数
     */
    [[nodiscard]] FilterConvergenceState convergence() const noexcept {
        FilterConvergenceState state;
        // 把EKF内部收敛枚举，映射到业务层对外的枚举
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
        state.normalized_innovation_squared = ekf_.normalized_innovation_squared(); // NIS归一化新息平方，卡方检验判断观测是否异常
        state.max_covariance_diag           = ekf_.max_covariance_diag();           // P矩阵对角线最大元素，衡量状态不确定性大小
        state.consecutive_converged_updates = ekf_.consecutive_converged_updates();  // 连续多少次更新收敛
        state.consecutive_diverged_updates  = ekf_.consecutive_diverged_updates();   // 连续多少次更新发散
        return state;
    }

    /**
     * @brief 不安全直接覆盖滤波器状态；外部强行重置状态，不修改协方差P
     * @note unsafe：业务要自己保证x合法，不会做边界校验
     */
    void set_state_unsafe(VecX x) noexcept { ekf_.setState(x); }

    /// 获取const运动模型
    [[nodiscard]] const Model& model() const noexcept { return model_; }

    /// 获取可修改运动模型引用，运行时修改模型参数
    [[nodiscard]] Model& model() noexcept { return model_; }

private:
    bool active_{false};                    ///< 滤波器是否激活，false不可predict/update
    double dt_{0.0};                        ///< 当前预测使用的时间间隔dt
    int armor_id_{0};                       ///< 当前观测使用的装甲板编号
    Model model_{};                         ///< 运动模型实例，存噪声、模型参数
    EKF ekf_{};                             ///< 扩展卡尔曼滤波器实例，惰性初始化，initialize后有效
    VecX x_{VecX::Zero()};                  ///< 缓存当前滤波器状态副本，方便上层读取
    std::optional<MatZZ> update_R_override_{}; ///< 可选外部R覆盖，std::optional有值代表启用外部R
};

class TrackerNew {
public:
    // 默认构造，全部成员使用类内默认初始化
    TrackerNew() noexcept = default;

    /**
     * @brief 带配置的构造函数
     * @param config 跟踪器配置：丢失阈值、门限、模型噪声参数等
     */
    explicit TrackerNew(const TrackerConfig& config) noexcept
        : config_(config) {}

    /**
     * @brief 跟踪器预测阶段，每帧时间更新
     * @param dt 距离上一帧时间间隔，单位秒
     * @note 同时驱动机器人EKF、前哨站EKF做时间预测；处理临时丢失超时重置逻辑
     */
    void predict(double dt) noexcept {
        // dt不能负数，防止时间回退导致滤波崩坏
        last_dt_ = std::max(0.0, dt);

        // 如果状态是临时丢失，累计丢失时间超过阈值，直接reset清空跟踪器
        if (status_ == TrackerStatus::TempLost && (lost_time_ + last_dt_) >= lost_threshold_) {
            reset();
            return;
        }

        // 机器人目标滤波器存在，执行EKF预测步
        if (robot_target_.has_value()) {
            robot_target_->predict(last_dt_);
        }

        // 前哨站目标滤波器存在，执行EKF预测步
        if (outpost_target_.has_value()) {
            outpost_target_->predict(last_dt_);
        }
    }

    /**
     * @brief 跟踪器更新入口：使用新的装甲观测批量更新EKF
     * @param measurements 一帧图像全部装甲检测结果批量
     * @return true 更新成功；false 更新失败/滤波器不存在
     */
    [[nodiscard]] bool update(const ArmorMeasurementBatch& measurements) noexcept {
        // 清空缓存观测数组
        measurement_.fill(0.0);

        // 当前跟踪目标是前哨站
        if (target_name_ == ArmorName::Outpost) {
            // 前哨站滤波器实例不存在直接返回失败
            if (!outpost_target_.has_value()) {
                return false;
            }
            // 调用内部匹配更新函数，关联观测与滤波器，执行EKF update
            auto d = update_target_(
                *outpost_target_, measurements, OutpostEkfMotionModel::ARMORS_NUM,
                outpost_last_armor_id_, false);
            // 更新之后滤波器被销毁，返回false
            if (!outpost_target_.has_value()) {
                return false;
            }
            return d;
        }

        // 走到这里代表跟踪普通机器人，机器人滤波器不存在直接返回
        if (!robot_target_.has_value()) {
            return false;
        }
        // 更新机器人目标EKF；true代表是机器人目标
        return update_target_(
            *robot_target_, measurements, RobotEkfMotionModel::ARMORS_NUM, robot_last_armor_id_,
            true);
    }

    /**
     * @brief 首次见到目标：初始化EKF，第一次匹配到目标的时候调用
     * @param measurements 当前帧全部装甲检测结果
     * @return 识别到的目标类型(std::nullopt代表没有可用装甲)
     * @details
     * 逻辑：
     * 1. 在所有检测装甲里面，选距离图像中心最近的装甲作为初始化源
     * 2. 根据是机器人 / 前哨站，分别构造不同运动模型的初始状态x0、初始协方差P0
     * 3. emplace构造EkfTargetInfo实例，调用initialize完成EKF初始化
     * 4. 赋值跟踪器参数：lost_threshold、matcher_gate跟随目标类型切换
     * 5. 状态机切换到跟踪状态
     */
    [[nodiscard]] std::optional<ArmorName>
        first_meet(const ArmorMeasurementBatch& measurements) noexcept {
        // 本帧没有检测到任何装甲
        if (measurements.empty()) {
            state_machine(false);
            return std::nullopt;
        }

        // 挑选距离图像中心最近的装甲，作为初始化来源
        int min_idx        = -1;
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(measurements.measurements.size()); ++i) {
            const float dist = measurements.measurements[i].distance_to_image_center;
            if (dist < min_distance) {
                min_distance = dist;
                min_idx      = i;
            }
        }

        // 没有找到合法装甲
        if (min_idx < 0) {
            state_machine(false);
            return std::nullopt;
        }

        // 取出选中的装甲观测
        const auto& target     = measurements.measurements[static_cast<size_t>(min_idx)];
        const auto& pos        = target.transform.translation(); // 装甲三维世界坐标
        const double armor_yaw = target.yaw();                   // 装甲yaw角度

        // 填充跟踪器全局信息
        target_name_                   = target.name;
        target_color_                  = target.color;
        last_image_center_distance_px_ = target.distance_to_image_center;
        last_observation_timestamp_ns_ = target.timestamp_ns;
        const bool is_outpost          = (target_name_ == ArmorName::Outpost);

        // 初始化前清空旧滤波器、旧装甲ID
        robot_target_.reset();
        outpost_target_.reset();
        robot_last_armor_id_.reset();
        outpost_last_armor_id_.reset();
        target_jumped_ = false;

        // ========= 分支1：初始化前哨站EKF =========
        if (is_outpost) {
            /**
             * 前哨站模型：装甲固定在半径OUTPOST_RADIUS圆周上旋转
             * pos是装甲三维点；反推前哨站中心x0,y0
             * center = armor_pos − R * [cos(yaw), sin(yaw)]
             */
            const double x0 = pos.x() + OutpostEkfMotionModel::OUTPOST_RADIUS * std::cos(armor_yaw);
            const double y0 = pos.y() + OutpostEkfMotionModel::OUTPOST_RADIUS * std::sin(armor_yaw);
            const double z0 = pos.z();

            // 构造前哨站初始状态向量 xp0
            OutpostEkfMotionModel::VecX xp0;
            // xc,yc(中心坐标),yaw旋转角度，vyaw角速度；z0 z1 z2三层高度
            xp0 << x0, y0, armor_yaw, 0.0, z0 - 0.1, z0, z0 + 0.1;
            // 初始协方差矩阵P0，单位矩阵打底，再修改对角线方差
            OutpostEkfMotionModel::MatXX P0 = OutpostEkfMotionModel::MatXX::Identity();

            P0(O_XC, O_XC) = 0.1;    // 中心X初始不确定性
            P0(O_YC, O_YC) = 0.1;    // 中心Y初始不确定性

            P0(O_YAW, O_YAW)   = 0.4; // 角度初始方差
            P0(O_VYAW, O_VYAW) = 100;// 角速度给很大初始不确定性，刚开始不知道转多快

            P0(O_Z0, O_Z0) = 1.5;    // 三层z坐标不确定性大
            P0(O_Z1, O_Z1) = 1.5;
            P0(O_Z2, O_Z2) = 1.5;

            // 构造前哨滤波器，调用initialize启动EKF
            outpost_target_.emplace();
            outpost_target_->initialize(config_.outpost.model, xp0, P0);
            outpost_last_armor_id_ = 0;

            // 切换跟踪器配置参数为前哨站参数
            lost_threshold_     = config_.outpost.lost_threshold;
            tracking_threshold_ = config_.outpost.tracking_threshold;
            matcher_gate_       = config_.outpost.matcher_gate;
        } else {
            // ========= 分支2：初始化普通机器人EKF =========
            // 从配置读取半径、高度先验，做数值钳位，防止非法参数
            const double r0_prior = std::clamp(config_.robot_inekf.radius0, 0.10, 0.50);
            const double r1_prior = std::clamp(config_.robot_inekf.radius1, 0.10, 0.50);
            const double h_prior  = std::clamp(config_.robot_inekf.height, -0.20, 0.20);
            const double r_init   = 0.5 * (r0_prior + r1_prior);

            // 装甲点反推机器人底盘中心 x0,y0
            const double x0       = pos.x() + r_init * std::cos(armor_yaw);
            const double y0       = pos.y() + r_init * std::sin(armor_yaw);
            const double z0       = pos.z() - 0.5 * h_prior;

            // 机器人EKF状态向量：x, vx, y, vy, z, vz, yaw, vyaw, log(r0), log(r1), h
            RobotEkfMotionModel::VecX xp0;
            auto v0 = 0.0;
            xp0 << x0, v0, y0, v0, z0, v0, armor_yaw, v0, std::log(r0_prior), std::log(r1_prior),
                h_prior;

            RobotEkfMotionModel::MatXX P0 = RobotEkfMotionModel::MatXX::Identity();

            P0(XC, XC) = 1;     // 位置初始不确定性
            P0(YC, YC) = 1;

            P0(VX, VX) = 64;    // 速度初始给很大方差，一开始不知道速度
            P0(VY, VY) = 64;
            P0(VZ, VZ) = 64;

            P0(YAW, YAW)     = 0.4;
            P0(V_YAW, V_YAW) = 100;

            // log域半径初始协方差，Var(log r) = var(r)/r²
            P0(LOG_R0, LOG_R0) = 1e-5 / (r0_prior * r0_prior);
            P0(LOG_R1, LOG_R1) = 1e-5 / (r1_prior * r1_prior);
            P0(H, H)           = 1;

            // 构造机器人滤波器实例，初始化EKF
            robot_target_.emplace();
            robot_target_->initialize(config_.robot.model, xp0, P0);
            robot_last_armor_id_.reset();

            // 切换跟踪器参数为机器人配置
            lost_threshold_     = config_.robot.lost_threshold;
            tracking_threshold_ = config_.robot.tracking_threshold;
            matcher_gate_       = config_.robot.matcher_gate;
        }

        // 状态机切换为跟踪状态
        state_machine(true);
        return target_name_;
    }

    /**
     * @brief 获取跟踪器对外输出结果，供瞄准模块使用
     * @return TrackerOutput 完整输出结构体，包含状态、协方差、卡尔曼增益、收敛信息
     */
    [[nodiscard]] TrackerOutput get_output() const noexcept {
        TrackerOutput output;
        output.status                        = status_;
        output.target_name                   = target_name_;
        output.target_color                  = target_color_;
        output.target_jumped                 = target_jumped_;
        output.last_image_center_distance_px = last_image_center_distance_px_;
        output.last_observation_timestamp_ns = last_observation_timestamp_ns_;

        // 当前跟踪机器人目标
        if (robot_target_.has_value()) {
            const auto& x = robot_target_->x();
            RobotTargetState state;
            state.position       = {x[XC], x[YC], x[Z0]};
            state.velocity       = {x[VX], x[VY], x[VZ]};
            state.yaw            = x[YAW];
            state.v_yaw          = x[V_YAW];
            state.radius0        = std::exp(x[LOG_R0]); // log域还原真实半径
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
            // 当前跟踪前哨站目标
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

    /**
     * @brief 完全重置跟踪器，清空所有滤波器、状态、计时；回到空闲Idle状态
     */
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

    /// 获取跟踪器状态 Idle / Tracking / TempLost
    [[nodiscard]] TrackerStatus status() const noexcept { return status_; }

    /// 获取当前跟踪目标类型
    [[nodiscard]] ArmorName target_name() const noexcept { return target_name_; }

    /// 获取缓存的观测数组，调试可视化用
    [[nodiscard]] const std::array<double, RobotEkfMotionModel::NZ * 2>&
        measurement() const noexcept {
        return measurement_;
    }

private:
    /**
     * @brief 观测‑装甲ID匹配候选结构体
     * @tparam TargetInfo EkfTargetInfo<> 模板类型（机器人/前哨站）
     * obs_index：检测结果数组下标；armor_id：机体上装甲逻辑编号
     * cost：总代价(马氏距离 + 先验代价)；z_update：送入EKF更新的观测向量
     * R_update：本次更新使用的测量噪声协方差（融合PnP协方差）
     */
    template <typename TargetInfo>
    struct MatchCandidate {
        size_t obs_index{0};
        int armor_id{0};
        double cost{0.0};
        typename TargetInfo::VecZ z_update{};
        typename TargetInfo::MatZZ R_update{};
    };

    /**
     * @brief 跟踪器核心：单目标数据关联 + EKF测量更新
     * @tparam TargetInfo EkfTargetInfo<xxxMotionModel>
     * @param target EKF滤波器实例引用
     * @param measurements 本帧全部装甲检测批量
     * @param armors_num 机体装甲总数量(机器人4块，前哨站3块)
     * @param last_armor_id 上帧匹配成功的装甲ID，std::optional可空
     * @param enable_robot_pair_measure 是否启用机器人成对装甲观测填充调试缓存
     * @return true 成功选出观测并执行EKF更新；false 无合法候选匹配失败
     *
     * 核心思想：
     * 1. 筛选和当前跟踪目标类型一致的观测
     * 2. prior_cost_for_id：基于上帧装甲ID+滤波器角速度，给出本帧各个装甲ID的先验概率代价
     *    - 前哨站：三模式混合先验(‑vyaw,0,+vyaw)；机器人：由vyaw与协方差P生成环形高斯先验
     * 3. 遍历【观测 × 装甲ID】所有组合，计算马氏距离，门限过滤，生成候选MatchCandidate
     * 4. 按总代价cost排序，贪心不重复分配：一个观测只能分配给一个装甲ID，一个装甲ID只能分配一个观测
     * 5. 区分两套代价：
     *    - cost = 马氏d2 + prior代价：用于候选排序选候选
     *    - obs_d2：仅观测马氏距离，**不包含prior**，用来确认是否更新last_armor_id
     *      → 防止prior预测方向错，直接篡改历史锚点last_armor_id
     * 6. 满足commit门限才更新last_armor_id；遍历选中候选逐个调用target.update()更新EKF
     */
    template <typename TargetInfo>
    [[nodiscard]] bool update_target_(
        TargetInfo& target, const ArmorMeasurementBatch& measurements, int armors_num,
        std::optional<int>& last_armor_id, bool enable_robot_pair_measure) noexcept {
        using VecZ  = typename TargetInfo::VecZ;
        using MatZZ = typename TargetInfo::MatZZ;

        // 筛选出类型等于当前跟踪目标的观测索引，过滤掉敌方/其他类型装甲
        std::vector<size_t> obs_indices;
        obs_indices.reserve(measurements.measurements.size());
        for (size_t i = 0; i < measurements.measurements.size(); ++i) {
            if (measurements.measurements[i].name == target_name_) {
                obs_indices.push_back(i);
            }
        }

        // 没有匹配类型的观测，状态机标记无观测，返回false
        if (obs_indices.empty()) {
            state_machine(false);
            return false;
        }

        // 装甲ID环形步长：一圈2π分成armors_num份，每个装甲间隔角度
        const double angle_step = 2.0 * std::numbers::pi / static_cast<double>(armors_num);

        //-------------------------------------------------------------------------
        // Prior cost 先验代价计算
        // cost = observation_d2 + prior_cost
        // prior用来辅助候选排序；但是更新last_armor_id只使用纯观测d2，不受prior污染
        //-------------------------------------------------------------------------
        /**
         * @brief 根据装甲id计算先验代价 -log(p)
         * @param id 待评估装甲逻辑ID
         * @return 代价，越大代表该ID先验概率越低
         */
        const auto prior_cost_for_id = [&](int id) noexcept -> double {
            // 没有上帧装甲ID，无先验，代价0
            if (!last_armor_id.has_value()) {
                return 0.0;
            }

            constexpr double kMinProb = 1e-6; // 概率下限，避免log(0)无穷大

            //---------------------------------------------------------------------
            // 前哨站特殊先验：转速只有 -2.51 / 0 / +2.51 rad/s 三档混合模型
            //---------------------------------------------------------------------
            if (target_name_ == ArmorName::Outpost) {
                constexpr double kOutpostVyaw         = 2.51327412; // ±固定角速度
                constexpr double kOutpostModePhaseStd = 0.45;         // 相位高斯标准差
                constexpr double kOutpostPriorWeight  = 0.6;         // prior整体权重

                // 三种转速模式
                const std::array<double, 3> mode_vyaws = {
                    -kOutpostVyaw,
                    0.0,
                    +kOutpostVyaw,
                };

                double best_prior_cost = std::numeric_limits<double>::infinity();

                // 遍历三种转速模式，取最小代价（最可能模式）
                for (const double mode_vyaw : mode_vyaws) {
                    // 根据转速推算经过dt之后装甲ID相位偏移多少格
                    const double mode_phase_shift = mode_vyaw * last_dt_ / angle_step;

                    // 构造环形高斯先验分布
                    const auto mode_prior = build_mode_prior_(
                        armors_num, last_armor_id, mode_phase_shift, kOutpostModePhaseStd);

                    // 取出id对应的概率，钳位防止0
                    const double p = std::clamp(
                        id < static_cast<int>(mode_prior.size())
                            ? mode_prior[static_cast<size_t>(id)]
                            : 1.0,
                        kMinProb, 1.0);

                    // 代价 -2*log(p)，高斯负对数似然形式
                    best_prior_cost = std::min(best_prior_cost, -2.0 * std::log(p));
                }

                if (!std::isfinite(best_prior_cost)) {
                    return 0.0;
                }

                return kOutpostPriorWeight * best_prior_cost;
            }

            //---------------------------------------------------------------------
            // 普通机器人：由滤波器输出的vyaw角速度 + P协方差生成相位先验
            //---------------------------------------------------------------------
            // 相位偏移：vyaw * dt 换算为“装甲ID格数”
            const double phase_shift =
                target.x()[TargetInfo::Model::kVyawIndex] * last_dt_ / angle_step;

            const auto& P = target.P();
            // vyaw角速度方差，不能小于0
            const double vyaw_var = std::max(
                0.0, static_cast<double>(
                         P(TargetInfo::Model::kVyawIndex, TargetInfo::Model::kVyawIndex)));

            // 从vyaw协方差换算相位ID空间的标准差
            const double phase_std_from_p =
                std::sqrt(vyaw_var) * std::abs(last_dt_) / std::max(angle_step, 1e-9);

            // 基础相位噪声，随dt变大
            const double base_phase_std = std::clamp(std::abs(last_dt_) * 0.5, 0.05, 1.0);

            // 合并两项噪声，hypot等价sqrt(a²+b²)
            const double phase_std =
                std::clamp(std::hypot(base_phase_std, phase_std_from_p), 0.08, 1.5);

            // prior权重：vyaw不确定性越大，prior权重越低，越相信观测
            const double prior_weight =
                std::clamp(1.0 / (1.0 + phase_std_from_p * phase_std_from_p), 0.0, 1.0);

            // 生成环形混合高斯先验数组，每个装甲id对应概率
            const auto prior = build_mode_prior_(armors_num, last_armor_id, phase_shift, phase_std);

            const double p = std::clamp(
                id < static_cast<int>(prior.size()) ? prior[static_cast<size_t>(id)] : 1.0,
                kMinProb, 1.0);

            return prior_weight * (-2.0 * std::log(p));
        };

        // ========== 遍历全部观测 × 全部装甲ID，生成候选 ==========
        std::vector<MatchCandidate<TargetInfo>> candidates;
        candidates.reserve(obs_indices.size() * static_cast<size_t>(armors_num));

        for (const size_t obs_index : obs_indices) {
            const auto& meas = measurements.measurements[obs_index];

            // 将装甲检测结果转为EKF观测空间z_raw
            const VecZ z_raw    = measurement_to_z_(meas);
            // 获取模型自带测量噪声R
            const MatZZ R_model = target.model().R(z_raw);
            MatZZ R_update      = R_model;

            // 前哨站：融合PnP输出的位姿协方差叠加到R；PnP求解越不准，R越大，滤波越不信任该观测
            if (target_name_ == ArmorName::Outpost) {
                R_update = pnp_augmented_measurement_R_<TargetInfo>(R_model, meas.pnp_cov_ypdr);
            }

            // 遍历机体所有装甲逻辑ID，尝试把当前观测匹配到该ID
            for (int id = 0; id < armors_num; ++id) {
                // h(x,id): 根据滤波器状态预测该装甲id对应的观测z_pred
                const VecZ z_pred = target.model().h(target.x(), id);
                // 预测观测无效(装甲背面不可见)，跳过该ID
                if (!armor_measurement_visible_from_origin(z_pred)) {
                    continue;
                }

                // 计算新息nu（角度做最短角度环绕）
                const VecZ nu   = innovation_(z_pred, z_raw);
                // 马氏距离d2
                const double d2 = mahalanobis_(nu, R_model);
                // 无穷大或者超过门限matcher_gate_，直接丢弃该候选
                if (!std::isfinite(d2) || d2 >= matcher_gate_) {
                    continue;
                }

                // 总代价 = 观测马氏距离 + ID先验代价
                const double cost = d2 + prior_cost_for_id(id);

                MatchCandidate<TargetInfo> candidate;
                candidate.obs_index = obs_index;
                candidate.armor_id  = id;
                candidate.cost      = cost;
                // 角度解环绕，生成真正送入EKF更新的观测向量
                candidate.z_update  = wrap_measurement_(z_pred, z_raw);
                candidate.R_update  = R_update;
                candidates.push_back(candidate);
            }
        }

        // 没有任何合法候选，状态机标记无观测，返回false
        if (candidates.empty()) {
            state_machine(false);
            return false;
        }

        // 候选按总代价从小到大排序
        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return a.cost < b.cost;
        });

        // ========== 贪心分配：一个观测只能分配给一个装甲ID；一个装甲ID只能分配一个观测 ==========
        std::vector<bool> used_obs(measurements.measurements.size(), false);
        std::vector<bool> used_id(static_cast<size_t>(armors_num), false);
        std::vector<MatchCandidate<TargetInfo>> selected;
        selected.reserve(static_cast<size_t>(armors_num));

        for (const auto& candidate : candidates) {
            // 观测已经被占用 / 装甲ID已经被占用，跳过
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

        // 本帧选出有效观测，状态机标记有观测
        state_machine(true);

        //-------------------------------------------------------------------------
        // obs_d2_for：只算纯观测马氏距离，**不加入prior代价**
        // last_armor_id更新只靠真实观测，不被先验预测绑架
        //-------------------------------------------------------------------------
        const auto obs_d2_for =
            [&](const MatchCandidate<TargetInfo>& candidate) noexcept -> double {
            const auto& meas = measurements.measurements[candidate.obs_index];

            const VecZ z_raw  = measurement_to_z_(meas);
            const VecZ z_pred = target.model().h(target.x(), candidate.armor_id);
            const VecZ nu     = innovation_(z_pred, z_raw);

            return mahalanobis_(nu, candidate.R_update);
        };

        // 代价最小的主候选
        const auto& primary = selected.front();
        const double primary_obs_d2 = obs_d2_for(primary);

        // 同一个观测，其他装甲ID的最小观测d2，用来计算margin区分度
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

        // margin：最佳ID和次佳ID的马氏距离差值；margin越大代表ID区分越确信
        const double obs_margin = second_obs_d2 - primary_obs_d2;

        //-------------------------------------------------------------------------
        // Commit gate：是否更新last_armor_id（历史锚点装甲ID）
        // 条件：主候选d2足够小；margin足够大，本观测可以明确区分装甲ID
        //-------------------------------------------------------------------------
        constexpr double kMaxCommitObsD2     = 6.0;
        constexpr double kMinCommitObsMargin = 2.0;

        const bool commit_last_id = std::isfinite(primary_obs_d2)
                                 && primary_obs_d2 < kMaxCommitObsD2 && std::isfinite(obs_margin)
                                 && obs_margin > kMinCommitObsMargin;

        if (commit_last_id) {
            last_armor_id = primary.armor_id;
        }

        // 保存主观测信息，对外输出调试
        const auto& primary_measurement = measurements.measurements[primary.obs_index];
        last_image_center_distance_px_  = primary_measurement.distance_to_image_center;
        last_observation_timestamp_ns_  = primary_measurement.timestamp_ns;

        // 是否匹配到非0号装甲；标记target_jumped_装甲跳变，上层瞄准做平滑抑制
        const bool observed_nonzero_armor = std::any_of(
            selected.begin(), selected.end(),
            [](const MatchCandidate<TargetInfo>& candidate) { return candidate.armor_id != 0; });

        if (observed_nonzero_armor) {
            target_jumped_ = true;
        }

        // 遍历所有选中候选，逐个送入EKF update；同时填充调试measurement缓存
        for (const auto& candidate : selected) {
            // 调用EkfTargetInfo::update，传入外部R_update
            target.update(candidate.z_update, candidate.armor_id, candidate.R_update);
            const bool another_pair =
                enable_robot_pair_measure && (candidate.armor_id == 1 || candidate.armor_id == 3);
            set_measurement_(candidate.z_update, another_pair);
        }
        return true;
    }

    /**
     * @brief 环形ID差值环绕，把差值d映射到 [-n/2 , n/2]，用于装甲ID环形高斯
     * @param d j‑center 原始差值
     * @param n 装甲总数量
     * @return wrapped 环绕后的差值
     */
    static double wrap_index_diff_(double d, int n) noexcept {
        if (n <= 0 || !std::isfinite(d)) {
            return 0.0;
        }
        const double period = static_cast<double>(n);
        // remainder做模运算，输出(-period/2, period/2]
        double wrapped      = std::remainder(d, period);
        if (!std::isfinite(wrapped)) {
            return 0.0;
        }
        // 边界处理：‑n/2映射为+n/2，保持历史行为
        if (wrapped <= -0.5 * period) {
            wrapped += period;
        }
        return wrapped;
    }

    /**
     * @brief build_mode_prior_ 构造装甲ID环形混合高斯先验概率分布
     * @param armors_num 装甲总块数
     * @param last_id 上帧装甲ID
     * @param phase_shift 经过dt之后ID相位偏移（格数）
     * @param phase_std 高斯标准差
     * @return vector<double> size=armors_num，每个ID对应的概率，归一化总和=1
     *
     * 模型：(1‑uniform_mix)*Gaussian + uniform_mix * uniform
     * 一部分权重给高斯，一部分权重给均匀分布防止完全锁死ID；phase_std越大，越偏向均匀。
     */
    static std::vector<double> build_mode_prior_(
        int armors_num, std::optional<int> last_id, double phase_shift, double phase_std) {
        // 默认全部等概率均匀分布
        std::vector<double> prior(
            static_cast<size_t>(armors_num), 1.0 / static_cast<double>(armors_num));
        if (!last_id.has_value() || armors_num <= 1) {
            return prior;
        }

        // 先验中心ID = last_id + phase_shift
        const double center      = static_cast<double>(*last_id) + phase_shift;
        const double sigma       = std::clamp(0.28 + 0.95 * phase_std, 0.25, 2.20);
        const double inv_var     = 1.0 / (sigma * sigma);
        // 混合均匀分布权重，std越大，均匀占比越高，prior越弱
        const double uniform_mix = std::clamp(0.02 + 0.30 * phase_std, 0.02, 0.35);
        // 只计算中心附近max_index_step个ID，远处直接置0，加速
        const int max_index_step = std::clamp(
            static_cast<int>(std::ceil(std::abs(phase_shift) + 3.0 * phase_std + 0.25)), 1,
            armors_num / 2 + 1);

        double sum = 0.0;
        for (int j = 0; j < armors_num; ++j) {
            // 环形环绕求ID之间差值
            const double diff = wrap_index_diff_(static_cast<double>(j) - center, armors_num);
            // 超出截断距离直接置0概率
            if (std::abs(diff) > static_cast<double>(max_index_step)) {
                prior[static_cast<size_t>(j)] = 0.0;
                continue;
            }
            // 高斯核
            const double g = std::exp(-0.5 * diff * diff * inv_var);
            // 高斯 + 均匀混合
            prior[static_cast<size_t>(j)] =
                (1.0 - uniform_mix) * g + uniform_mix / static_cast<double>(armors_num);
            sum += prior[static_cast<size_t>(j)];
        }

        // 概率总和<=0，回退全部均匀
        if (sum <= 0.0) {
            return std::vector<double>(static_cast<size_t>(armors_num), 1.0 / armors_num);
        }
        // 归一化，总和等于1
        for (double& p : prior) {
            p /= sum;
        }
        return prior;
    }

    /**
     * @brief measurement_to_z_：装甲检测结果结构体转为EKF观测向量VecZ
     * @param meas 单块装甲检测结果
     * @return VecZ：yaw,pitch,log(distance),armor_yaw
     * 距离取log，和EKF状态空间保持一致
     */
    static RobotEkfMotionModel::VecZ measurement_to_z_(const ArmorMeasurement& meas) noexcept {
        const auto ypd = xyz2ypd(meas.transform.translation());
        RobotEkfMotionModel::VecZ z;
        z[ARMOR_YAW]       = ypd[0];
        z[ARMOR_PITCH]     = ypd[1];
        z[ARMOR_DISTANCE]  = std::log(std::max(ypd[2], 1e-9)); // log距离，防止log(0)
        z[ARMOR_YAW_ARMOR] = meas.yaw();
        return z;
    }

    /**
     * @brief pnp_augmented_measurement_R_：把PnP求解输出的协方差叠加到测量噪声R
     * @param model_R 模型原生R矩阵
     * @param pnp_cov_ypdr PnP输出位姿协方差矩阵
     * @return 融合之后SPD正定的R矩阵
     * 逻辑：model_R + pnp_R；强制保证矩阵对称正定SPD；PnP误差大，R变大，滤波降低观测信任度
     */
    template <typename TargetInfo>
    static typename TargetInfo::MatZZ pnp_augmented_measurement_R_(
        const typename TargetInfo::MatZZ& model_R, const Eigen::Matrix4d& pnp_cov_ypdr) noexcept {
        using MatZZ = typename TargetInfo::MatZZ;

        // 强制转为对称正定矩阵
        const MatZZ base_R = make_spd_R_(model_R);
        // PnP协方差含NaN/inf，直接使用model_R
        if (!pnp_cov_ypdr.allFinite()) {
            return base_R;
        }

        // 强制对称 (A+A^T)/2
        MatZZ pnp_R = 0.5 * (pnp_cov_ypdr + pnp_cov_ypdr.transpose());
        if (!pnp_R.allFinite()) {
            return base_R;
        }

        // 特征值分解，把负特征值钳位到0，保证SPD正定
        Eigen::SelfAdjointEigenSolver<MatZZ> es(pnp_R);
        if (es.info() == Eigen::Success && pnp_R.allFinite()) {
            const auto evals = es.eigenvalues().cwiseMax(0.0);
            pnp_R = es.eigenvectors() * evals.asDiagonal() * es.eigenvectors().transpose();
            pnp_R = 0.5 * (pnp_R + pnp_R.transpose());
        }

        // 叠加模型噪声与PnP噪声
        const MatZZ combined_R = base_R + pnp_R;
        return make_spd_R_(combined_R);
    }

    /**
     * @brief make_spd_R_ 强制把输入协方差矩阵修正为对称半正定SPD矩阵
     * @tparam MatZZ 协方差矩阵类型
     * @param input 原始矩阵，可能不对称、负方差、NaN
     * @return 一定SPD正定，对角线最小1e‑8，防止EKF数值崩溃
     */
    template <typename MatZZ>
    static MatZZ make_spd_R_(const MatZZ& input) noexcept {
        // 强制对称
        MatZZ R = 0.5 * (input + input.transpose());
        // 对角线方差不能小于1e‑8，不能NaN
        for (int i = 0; i < MatZZ::RowsAtCompileTime; ++i) {
            if (!std::isfinite(R(i, i)) || R(i, i) < 1e-8) {
                R(i, i) = 1e-8;
            }
        }

        // LDLT分解检测是否正定
        Eigen::LDLT<MatZZ> ldlt(R);
        if (R.allFinite() && ldlt.info() == Eigen::Success && ldlt.isPositive()) {
            return R;
        }

        // 分解失败回退：只保留对角线，丢弃非对角项
        MatZZ fallback = MatZZ::Zero();
        for (int i = 0; i < MatZZ::RowsAtCompileTime; ++i) {
            const double var = input(i, i);
            fallback(i, i)   = std::isfinite(var) ? std::max(1e-8, var) : 1e6;
        }
        return fallback;
    }

    /**
     * @brief innovation_ 计算EKF新息nu，角度做最短弧度差
     * @tparam VecZ 观测向量
     * @param z_pred EKF预测观测
     * @param z_raw 原始检测观测
     * @return nu 新息向量，角度用‑π~π最短差值；距离直接减法
     */
    template <typename VecZ>
    static VecZ innovation_(const VecZ& z_pred, const VecZ& z_raw) noexcept {
        VecZ nu;
        nu[ARMOR_YAW]       = shortest_rad(z_pred[ARMOR_YAW], z_raw[ARMOR_YAW]);
        nu[ARMOR_PITCH]     = shortest_rad(z_pred[ARMOR_PITCH], z_raw[ARMOR_PITCH]);
        nu[ARMOR_DISTANCE]  = z_raw[ARMOR_DISTANCE] - z_pred[ARMOR_DISTANCE];
        nu[ARMOR_YAW_ARMOR] = shortest_rad(z_pred[ARMOR_YAW_ARMOR], z_raw[ARMOR_YAW_ARMOR]);
        return nu;
    }

    /**
     * @brief wrap_measurement_ 观测角度解环绕；根据预测值把原始观测角度unwrap，避免±π跳变
     * @tparam VecZ
     * @param z_pred EKF预测观测
     * @param z_raw 原始观测
     * @return 解环绕后的观测，直接送入EKF update
     */
    template <typename VecZ>
    static VecZ wrap_measurement_(const VecZ& z_pred, const VecZ& z_raw) noexcept {
        VecZ z;
        z[ARMOR_YAW]       = unwrap_rad(z_pred[ARMOR_YAW], z_raw[ARMOR_YAW]);
        z[ARMOR_PITCH]     = unwrap_rad(z_pred[ARMOR_PITCH], z_raw[ARMOR_PITCH]);
        z[ARMOR_DISTANCE]  = z_raw[ARMOR_DISTANCE];
        z[ARMOR_YAW_ARMOR] = unwrap_rad(z_pred[ARMOR_YAW_ARMOR], z_raw[ARMOR_YAW_ARMOR]);
        return z;
    }

    /**
     * @brief mahalanobis_ 计算马氏距离 nu^T * R^{-1} * nu
     * 使用LDLT分解，支持非对角R；分解失败返回很大代价1e9
     */
    template <typename VecZ, typename MatZZ>
    static double mahalanobis_(const VecZ& nu, const MatZZ& R) noexcept {
        Eigen::LDLT<MatZZ> ldlt(R);
        if (ldlt.info() != Eigen::Success) {
            return 1e9;
        }
        return static_cast<double>(nu.transpose() * ldlt.solve(nu));
    }

    /**
     * @brief state_machine 跟踪器状态机流转
     * 状态：Idle → Detecting → Tracking → TempLost
     * @param found true本帧有有效观测；false无观测
     *
     * Idle空闲：检测到目标进入Detecting；无观测reset
     * Detecting确认中：连续检测计数>tracking_threshold，进入Tracking；丢失直接reset
     * Tracking正常跟踪：丢失进入TempLost临时丢失，开始计时lost_time_
     * TempLost临时丢失：重新看到切回Tracking；累计lost_time_>=阈值，彻底reset回到Idle
     */
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
                // 连续检测到足够帧数，进入正式跟踪
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
                // 临时丢失超时，彻底重置跟踪器
                if (lost_time_ >= lost_threshold_) {
                    reset();
                }
            }
            break;
        }
    }

    /**
     * @brief set_measurement_ 把观测存入measurement_调试缓存，供Foxglove可视化
     * @param z 观测向量
     * @param is_another_pair true填入后半段数组，用于机器人成对装甲同时观测
     */
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
    // 跟踪器全部配置参数，从yaml配置文件加载而来
    TrackerConfig config_{};

    // 跟踪器状态机状态：Idle/Detecting/Tracking/TempLost
    TrackerStatus status_{TrackerStatus::Idle};

    // 当前跟踪目标类型：Robot / Outpost / Invalid无效
    ArmorName target_name_{ArmorName::Invalid};

    // 目标装甲颜色：红/蓝/中性
    ArmorColor target_color_{ArmorColor::Neutral};

    /**
    * @brief 机器人EKF滤波器实例，std::optional延迟初始化
    * 只有跟踪机器人的时候才会emplace构造；不跟踪则无值
    * EkfTargetInfo封装状态x、协方差P、运动模型h()/R()
    */
    std::optional<EkfTargetInfo<RobotEkfMotionModel>> robot_target_{};

    /**
    * @brief 前哨站EKF滤波器实例
    * 跟踪前哨站才实例化；和机器人是两套独立运动模型
    */
    std::optional<EkfTargetInfo<OutpostEkfMotionModel>> outpost_target_{};

    // TempLost临时丢失超时阈值(秒)，超过直接reset跟踪器
    double lost_threshold_{0.0};

    // 进入Tracking需要连续检测到目标的帧数，防误触发
    uint32_t tracking_threshold_{0};

    // 马氏距离门限，大于该值直接丢弃该匹配候选
    double matcher_gate_{10.0};

    // Detecting阶段计数：连续检测到目标的帧数
    int detecting_count_{0};

    // 上一帧时间间隔dt，单位秒，用于预测相位偏移、运动外推
    double last_dt_{0.0};

    // TempLost状态下，累计丢失时间(秒)
    double lost_time_{0.0};

    /**
    * @brief 调试观测缓存，给Foxglove可视化用
    * RobotEkfMotionModel::NZ 单组观测维度；*2 支持同时存两组成对装甲观测
    */
    std::array<double, RobotEkfMotionModel::NZ * 2> measurement_{};

    // 主匹配装甲距离图像中心像素距离，用于调试、选目标
    double last_image_center_distance_px_{std::numeric_limits<double>::infinity()};

    // 本帧观测对应的硬件时间戳，纳秒，时序对齐
    uint64_t last_observation_timestamp_ns_{0};

    // 机器人上一帧成功commit的装甲ID，std::optional空代表无历史锚点
    std::optional<int> robot_last_armor_id_{};

    // 前哨站上一帧commit装甲ID
    std::optional<int> outpost_last_armor_id_{};

    /**
    * @brief 装甲跳变标记
    * true代表本帧匹配到非0号有效装甲，上层瞄准系统可以做平滑防抖
    * 防止装甲ID切换导致枪口剧烈抖动
    */
    bool target_jumped_{false};

};

} // namespace fcs::L3
