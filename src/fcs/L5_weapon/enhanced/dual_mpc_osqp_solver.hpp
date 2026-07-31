/**
 * @file dual_mpc_osqp_solver.hpp
 * @brief 轨迹平滑二次规划(QP)问题的黄金基准求解器
 *
 * 本文件实现了一个独立依赖的离线基准求解器,用于验证DualSmallMpcSolver的正确性。
 * 核心思想是求解一个带箱式约束的二次规划问题,优化云台轨迹的加速度序列。
 *
 * 算法原理:
 * 1. 将轨迹优化问题建模为QP问题:minimize 位置/速度误差 + 控制量
 * 2. 通过动力学约束消元,将状态变量转化为输入变量的函数
 * 3. 使用投影梯度法+ FISTA加速求解
 * 4. 通过KKT残差验证解的最优性
 *
 * 数学形式:
 *     minimize_u  0.5 * sum_k q_pos * (pos_k - ref_pos_k)^2
 *               + 0.5 * sum_k q_vel * (vel_k - ref_vel_k)^2
 *               + 0.5 * sum_k r     * u_k^2
 *     subject to  -max_acc <= u_k <= max_acc
 *
 * 重要限制:
 * - 本求解器仅用于离线基准测试,不适用于实时控制
 * - 状态约束(enable_state_bound)仅为API兼容性保留,实际不强制执行
 * - rho参数仅为API兼容性保留,实际不使用(rho是ADMM参数,不是QP目标函数的一部分)
 *
 * @note 性能特性:使用double精度和动态内存分配,不适合嵌入式平台
 * @warning 不要在生产代码中使用此求解器,请使用DualSmallMpcSolver
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <limits>
#include <string>

namespace fcs::L5 {

/**
 * @brief 无依赖的轨迹平滑QP黄金基准求解器
 *
 * 本类与DualSmallMpcSolver接口完全一致,但内部实现使用精确的QP求解方法。
 * 主要用途:
 * 1. 作为基准验证DualSmallMpcSolver的数值正确性
 * 2. 离线测试和算法开发
 * 3. 性能对比分析
 *
 * 求解方法:
 * - 使用动力学精确消元(condensation),将状态变量完全由输入变量表示
 * - 使用投影梯度下降法求解箱式约束QP
 * - 采用FISTA加速策略提升收敛速度
 * - 通过KKT条件验证解的最优性
 *
 * 动力学模型(默认):
 *     pos[k+1] = pos[k] + dt * vel[k]
 *     vel[k+1] = vel[k] + dt * acc[k]
 * 即二阶积分器模型,位置和速度的关系由时间步长dt决定
 */
class DualMpcOsqpSolver {
public:
    /**
     * @brief 单轴MPC配置参数
     *
     * 定义单个轴(yaw或pitch)的MPC优化参数。
     * 包括权重矩阵、约束边界等关键参数。
     */
    struct AxisConfig {
        /// 位置误差权重Q_pos(默认:9e6)
        /// 值越大,轨迹跟踪精度越高,但控制量可能增大
        float q_pos{9e6f};

        /// 速度误差权重Q_vel(默认:0.0)
        /// 通常设为0,因为位置控制隐含了速度控制
        float q_vel{0.0f};

        /// 控制量权重R(默认:1.0)
        /// 用于平滑控制输入,防止加速度突变
        float r{1.0f};

        /// 最大加速度约束(默认:50.0 rad/s²)
        /// 限制云台的角加速度,保护机械结构
        float max_acc{50.0f};

        /// 状态下界(默认:-inf)
        /// 用于限制pitch角度范围,防止机械碰撞
        float state_min{-std::numeric_limits<float>::infinity()};

        /// 状态上界(默认:+inf)
        float state_max{std::numeric_limits<float>::infinity()};

        /// 是否启用状态约束(默认:false)
        /// 注意:本求解器不强制执行状态约束,仅为API兼容性保留
        bool enable_state_bound{false};
    };

    /// 轴数量:固定为2(yaw和pitch)
    static constexpr int kAxisCount = 2;

    /// yaw轴索引
    static constexpr int kYawAxis = 0;

    /// pitch轴索引
    static constexpr int kPitchAxis = 1;

    /// 最大时间步数(固定容量数组大小)
    /// 限制轨迹预测长度,防止内存分配
    static constexpr int kMaxHorizon = 101;

    /// 最大输入步数(比状态步数少1)
    static constexpr int kMaxInputHorizon = kMaxHorizon - 1;

    /// 默认构造函数
    DualMpcOsqpSolver() = default;

    /// 禁止拷贝(包含大量Eigen矩阵,拷贝成本高)
    DualMpcOsqpSolver(const DualMpcOsqpSolver&)            = delete;
    DualMpcOsqpSolver& operator=(const DualMpcOsqpSolver&) = delete;

    /// 允许移动(资源转移成本低)
    DualMpcOsqpSolver(DualMpcOsqpSolver&&) noexcept            = default;
    DualMpcOsqpSolver& operator=(DualMpcOsqpSolver&&) noexcept = default;

    /// 默认析构函数
    ~DualMpcOsqpSolver() = default;

    /**
     * @brief 工厂方法:创建并初始化求解器
     *
     * 采用静态工厂模式而非构造函数,原因:
     * 1. 初始化可能失败(参数校验),需返回错误信息
     * 2. 构造函数不能返回std::expected
     * 3. 符合RAII原则:对象构造完成后一定可用
     *
     * @param dt 时间步长(秒),必须>0且有限
     * @param horizon 预测步数,范围[2, kMaxHorizon]
     * @param rho ADMM参数(仅为API兼容性保留,实际不使用)
     * @param yaw yaw轴配置参数
     * @param pitch pitch轴配置参数
     *
     * @return 成功返回初始化后的求解器,失败返回错误信息
     *
     * @note 所有参数在初始化时校验,运行时不再检查
     * @warning rho参数在本求解器中无作用,仅为保持接口一致
     */
    [[nodiscard]] static std::expected<DualMpcOsqpSolver, std::string>
        create(float dt, int horizon, float rho, const AxisConfig& yaw, const AxisConfig& pitch) {
        // 参数校验:时间步长必须为正有限值
        if (dt <= 0.0f || !std::isfinite(dt)) {
            return std::unexpected("dt must be positive and finite");
        }

        // 参数校验:预测步数至少为2(当前状态+至少1步预测)
        if (horizon < 2) {
            return std::unexpected("horizon must be >= 2");
        }

        // 参数校验:预测步数不能超过固定容量上限
        if (horizon > kMaxHorizon) {
            return std::unexpected(
                "horizon exceeds fixed-capacity limit of " + std::to_string(kMaxHorizon));
        }

        // 参数校验:rho必须为正有限值(虽然不使用,但需保持API一致性)
        if (rho <= 0.0f || !std::isfinite(rho)) {
            return std::unexpected("rho must be positive and finite");
        }

        // Lambda函数:校验单轴配置参数
        auto validate_axis = [](const AxisConfig& cfg,
                                const char* name) -> std::expected<void, std::string> {
            // Q权重必须非负(半正定要求)
            if (cfg.q_pos < 0.0f || cfg.q_vel < 0.0f) {
                return std::unexpected(std::string{name} + ": Q weights must be nonnegative");
            }

            // R权重必须非负且有限(正定要求)
            if (cfg.r < 0.0f || !std::isfinite(cfg.r)) {
                return std::unexpected(
                    std::string{name} + ": R weight must be nonnegative and finite");
            }

            // 最大加速度必须非负且有限(约束有效性要求)
            if (cfg.max_acc < 0.0f || !std::isfinite(cfg.max_acc)) {
                return std::unexpected(
                    std::string{name} + ": max_acc must be nonnegative and finite");
            }
            return {};
        };

        // 校验yaw轴配置
        if (auto ok = validate_axis(yaw, "yaw"); !ok) {
            return std::unexpected(ok.error());
        }

        // 校验pitch轴配置
        if (auto ok = validate_axis(pitch, "pitch"); !ok) {
            return std::unexpected(ok.error());
        }

        // 构造求解器对象
        DualMpcOsqpSolver s;
        s.N_                = horizon;
        s.last_state_index_ = horizon - 1;
        s.last_input_index_ = horizon - 2;
        s.rho_              = rho; // API兼容性保留,实际不使用

        // 初始化动力学矩阵(二阶积分器模型)
        // 状态转移矩阵A:
        //   pos[k+1] = pos[k] + dt * vel[k]  => a00=1, a01=dt
        //   vel[k+1] = vel[k]                => a10=0, a11=1
        s.a00_ = 1.0;
        s.a01_ = static_cast<double>(dt);
        s.a10_ = 0.0;
        s.a11_ = 1.0;

        // 输入矩阵B:
        //   pos[k+1] += 0 * acc[k]  => b0=0(位置不直接受加速度影响)
        //   vel[k+1] += dt * acc[k] => b1=dt
        s.b0_ = 0.0;
        s.b1_ = static_cast<double>(dt);

        // 外力项(默认为0,无外部扰动)
        s.f_pos_ = 0.0;
        s.f_vel_ = 0.0;

        // 初始化两个轴的配置
        s.init_axis(kYawAxis, yaw);
        s.init_axis(kPitchAxis, pitch);

        // 重置工作空间
        s.reset_workspace();
        return s;
    }

    /**
     * @brief 设置求解器收敛参数
     *
     * @param abs_tol 绝对容差,KKT残差低于此值认为收敛
     * @param max_iter 最大迭代次数,防止无限循环
     */
    void set_settings(float abs_tol, int max_iter) noexcept {
        abs_tol_  = static_cast<double>(abs_tol);
        max_iter_ = max_iter;
    }

    /**
     * @brief 设置单轴初始状态
     *
     * @param axis 轴索引(kYawAxis或kPitchAxis)
     * @param pos 初始位置(弧度)
     * @param vel 初始速度(弧度/秒)
     */
    void set_x0(int axis, double pos, double vel) noexcept {
        x0_pos_(axis) = pos;
        x0_vel_(axis) = vel;
    }

    /**
     * @brief 同时设置yaw和pitch轴初始状态(便捷接口)
     *
     * @param yaw_pos yaw初始位置(弧度)
     * @param yaw_vel yaw初始速度(弧度/秒)
     * @param pitch_pos pitch初始位置(弧度)
     * @param pitch_vel pitch初始速度(弧度/秒)
     */
    void set_x0(double yaw_pos, double yaw_vel, double pitch_pos, double pitch_vel) noexcept {
        set_x0(kYawAxis, yaw_pos, yaw_vel);
        set_x0(kPitchAxis, pitch_pos, pitch_vel);
    }

    /**
     * @brief 设置单轴单个时间步的参考轨迹
     *
     * @param axis 轴索引
     * @param k 时间步索引[0, horizon-1]
     * @param pos 参考位置(弧度)
     * @param vel 参考速度(弧度/秒)
     */
    void set_ref_col(int axis, int k, float pos, float vel) noexcept {
        ref_pos_(axis, k) = static_cast<double>(pos);
        ref_vel_(axis, k) = static_cast<double>(vel);
    }

    /**
     * @brief 同时设置yaw和pitch轴单个时间步的参考轨迹(便捷接口)
     *
     * @param k 时间步索引
     * @param yaw_pos yaw参考位置
     * @param yaw_vel yaw参考速度
     * @param pitch_pos pitch参考位置
     * @param pitch_vel pitch参考速度
     */
    void set_ref_col(
        int k, float yaw_pos, float yaw_vel, float pitch_pos, float pitch_vel) noexcept {
        set_ref_col(kYawAxis, k, yaw_pos, yaw_vel);
        set_ref_col(kPitchAxis, k, pitch_pos, pitch_vel);
    }

    /**
     * @brief 批量设置完整参考轨迹
     *
     * 通过Eigen矩阵一次性设置所有时间步的参考轨迹,避免循环调用开销。
     *
     * @param yaw_ref yaw参考轨迹矩阵(2×N),第一行为位置,第二行为速度
     * @param pitch_ref pitch参考轨迹矩阵(2×N)
     */
    void set_reference(
        const Eigen::Ref<const Eigen::MatrixXf>& yaw_ref,
        const Eigen::Ref<const Eigen::MatrixXf>& pitch_ref) noexcept {
        for (int k = 0; k < N_; ++k) {
            ref_pos_(kYawAxis, k)   = static_cast<double>(yaw_ref(0, k));
            ref_vel_(kYawAxis, k)   = static_cast<double>(yaw_ref(1, k));
            ref_pos_(kPitchAxis, k) = static_cast<double>(pitch_ref(0, k));
            ref_vel_(kPitchAxis, k) = static_cast<double>(pitch_ref(1, k));
        }
    }

    /**
     * @brief 执行MPC求解
     *
     * 核心算法流程:
     * 1. 对yaw和pitch轴独立求解
     * 2. 每个轴执行投影梯度下降
     * 3. 通过KKT条件验证收敛性
     *
     * @return true表示求解成功且收敛,false表示失败或未收敛
     */
    bool solve() noexcept {
        converged_ = false;
        converged_axis_.fill(false);
        iters_ = 0;

        // 独立求解yaw和pitch轴
        const bool yaw_ok   = solve_axis(kYawAxis);
        const bool pitch_ok = solve_axis(kPitchAxis);

        // 记录每个轴的收敛状态
        converged_axis_[kYawAxis]   = yaw_ok;
        converged_axis_[kPitchAxis] = pitch_ok;

        // 整体收敛条件:两个轴都收敛
        converged_ = yaw_ok && pitch_ok;

        // 记录最大迭代次数(用于性能分析)
        iters_ = std::max(axis_iters_[kYawAxis], axis_iters_[kPitchAxis]);
        return converged_;
    }

    /**
     * @brief 获取求解后的状态值
     *
     * @param axis 轴索引
     * @param dim 维度索引(0=位置,1=速度)
     * @param k 时间步索引
     *
     * @return 状态值(位置或速度)
     */
    [[nodiscard]] float state(int axis, int dim, int k) const noexcept {
        return static_cast<float>(dim == 0 ? sol_pos_(axis, k) : sol_vel_(axis, k));
    }

    /**
     * @brief 获取求解后的控制输入(加速度)
     *
     * @param axis 轴索引
     * @param k 时间步索引[0, horizon-2]
     *
     * @return 加速度值(弧度/秒²)
     */
    [[nodiscard]] float input(int axis, int k) const noexcept {
        return static_cast<float>(sol_acc_(axis, k));
    }

    /// 检查求解器是否整体收敛(两个轴都收敛)
    [[nodiscard]] bool converged() const noexcept { return converged_; }

    /// 检查指定轴是否收敛
    [[nodiscard]] bool converged(int axis) const noexcept { return converged_axis_[axis]; }

    /// 获取总迭代次数
    [[nodiscard]] int iterations() const noexcept { return iters_; }

    /// 获取预测步数
    [[nodiscard]] int horizon() const noexcept { return N_; }

    /**
     * @brief 获取目标函数值(用于调试和性能分析)
     *
     * 目标函数 = 位置误差项 + 速度误差项 + 控制量项
     *
     * @param axis 轴索引
     *
     * @return 目标函数值
     */
    [[nodiscard]] double objective(int axis) const noexcept { return objective_(axis); }

    /**
     * @brief 获取动力学约束违反程度(应为0或极小)
     *
     * 理论上解应该严格满足动力学方程,此值用于验证求解正确性。
     *
     * @param axis 轴索引
     *
     * @return 最大动力学违反量
     */
    [[nodiscard]] double dynamics_violation(int axis) const noexcept {
        return dyn_violation_(axis);
    }

    /**
     * @brief 获取输入约束违反程度(应为0或极小)
     *
     * 验证解是否满足加速度约束[-max_acc, max_acc]。
     *
     * @param axis 轴索引
     *
     * @return 最大输入约束违反量
     */
    [[nodiscard]] double input_bound_violation(int axis) const noexcept {
        return input_violation_(axis);
    }

    /**
     * @brief 获取KKT条件违反程度(收敛性指标)
     *
     * KKT条件是优化问题最优解的必要条件。
     * 此值越小,解越接近最优解。
     *
     * @param axis 轴索引
     *
     * @return 最大KKT残差
     */
    [[nodiscard]] double kkt_violation(int axis) const noexcept { return kkt_violation_(axis); }

    /**
     * @brief 获取状态约束违反程度
     *
     * @note 本求解器不强制执行状态约束,始终返回0
     * @param axis 轴索引(未使用)
     * @return 0.0
     */
    [[nodiscard]] double state_bound_violation(int) const noexcept { return 0.0; }

    /**
     * @brief 检查是否忽略了状态约束
     *
     * @param axis 轴索引
     *
     * @return true表示配置了状态约束但被忽略(本求解器特性)
     */
    [[nodiscard]] bool state_bounds_ignored(int axis) const noexcept {
        return enable_state_bound_[axis];
    }

private:
    // ==========================================================================
    // 类型别名定义(使用固定大小Eigen数组避免动态内存分配)
    // ==========================================================================

    /// 双轴向量类型(2×1数组)
    using LaneD = Eigen::Array<double, kAxisCount, 1>;

    /// 状态矩阵类型(2×kMaxHorizon数组)
    using StateMatD = Eigen::Array<double, kAxisCount, kMaxHorizon>;

    /// 输入矩阵类型(2×kMaxInputHorizon数组)
    using InputMatD = Eigen::Array<double, kAxisCount, kMaxInputHorizon>;

    /**
     * @brief 获取输入变量数量
     *
     * 对于N步预测,有N-1个控制输入(加速度)
     *
     * @return 输入变量数量
     */
    [[nodiscard]] int num_inputs() const noexcept { return N_ - 1; }

    /**
     * @brief 标量投影到区间[lo, hi]
     *
     * 箱式约束的核心操作:将值限制在可行域内
     *
     * @param v 待投影值
     * @param lo 下界
     * @param hi 上界
     *
     * @return 投影后的值
     */
    static double clamp_scalar(double v, double lo, double hi) noexcept {
        return std::min(std::max(v, lo), hi);
    }

    /**
     * @brief 向量投影到区间[lo, hi](逐元素)
     *
     * 使用Eigen数组运算批量处理,比循环更高效
     *
     * @param v 待投影向量
     * @param lo 下界(应用到所有元素)
     * @param hi 上界(应用到所有元素)
     *
     * @return 投影后的向量
     */
    static Eigen::VectorXd clamp_vec(const Eigen::VectorXd& v, double lo, double hi) noexcept {
        return v.array().min(hi).max(lo).matrix();
    }

    // ==========================================================================
    // 动力学矩阵参数(二阶积分器模型)
    // ==========================================================================

    /// 状态转移矩阵A的元素(位置方程)
    double a00_{1.0}; ///< pos[k+1] = a00*pos[k] + a01*vel[k]
    double a01_{0.0}; ///< 通常a01=dt,表示速度对位置的影响
    double a10_{0.0}; ///< vel[k+1] = a10*pos[k] + a11*vel[k]
    double a11_{1.0}; ///< 通常a11=1,表示速度的惯性

    /// 输入矩阵B的元素(控制方程)
    double b0_{0.0}; ///< pos[k+1] += b0*acc[k](通常为0)
    double b1_{0.0}; ///< vel[k+1] += b1*acc[k](通常为dt)

    /// 外力/扰动项(默认为0)
    double f_pos_{0.0}; ///< 位置扰动
    double f_vel_{0.0}; ///< 速度扰动

    /// ADMM参数rho(仅API兼容性保留,实际不使用)
    double rho_{1.0};

    // ==========================================================================
    // MPC权重和约束参数
    // ==========================================================================

    LaneD q_pos_{LaneD::Zero()}; ///< 位置误差权重(每轴独立)
    LaneD q_vel_{LaneD::Zero()}; ///< 速度误差权重(每轴独立)
    LaneD r_val_{LaneD::Zero()}; ///< 控制量权重(每轴独立)
    LaneD u_min_{LaneD::Zero()}; ///< 加速度下界(每轴独立)
    LaneD u_max_{LaneD::Zero()}; ///< 加速度上界(每轴独立)

    /// 状态约束启用标志(仅为API兼容性,实际不强制执行)
    std::array<bool, kAxisCount> enable_state_bound_{};

    // ==========================================================================
    // 问题规模参数
    // ==========================================================================

    int N_{0};                ///< 预测步数
    int last_state_index_{0}; ///< 最后一个状态索引(N-1)
    int last_input_index_{0}; ///< 最后一个输入索引(N-2)

    // ==========================================================================
    // 输入数据(初始状态和参考轨迹)
    // ==========================================================================

    LaneD x0_pos_{LaneD::Zero()};          ///< 初始位置(两轴)
    LaneD x0_vel_{LaneD::Zero()};          ///< 初始速度(两轴)
    StateMatD ref_pos_{StateMatD::Zero()}; ///< 参考位置轨迹(两轴×N步)
    StateMatD ref_vel_{StateMatD::Zero()}; ///< 参考速度轨迹(两轴×N步)

    // ==========================================================================
    // 输出数据(求解结果)
    // ==========================================================================

    StateMatD sol_pos_{StateMatD::Zero()}; ///< 优化后位置轨迹
    StateMatD sol_vel_{StateMatD::Zero()}; ///< 优化后速度轨迹
    InputMatD sol_acc_{InputMatD::Zero()}; ///< 优化后加速度序列

    // ==========================================================================
    // 诊断数据(验证和调试用)
    // ==========================================================================

    LaneD objective_{LaneD::Zero()};       ///< 目标函数值
    LaneD dyn_violation_{LaneD::Zero()};   ///< 动力学违反量
    LaneD input_violation_{LaneD::Zero()}; ///< 输入约束违反量
    LaneD kkt_violation_{LaneD::Zero()};   ///< KKT条件违反量

    // ==========================================================================
    // 求解器参数
    // ==========================================================================

    double abs_tol_{1e-9}; ///< 绝对收敛容差
    int max_iter_{200000}; ///< 最大迭代次数(基准测试允许较多迭代)

    // ==========================================================================
    // 求解器状态
    // ==========================================================================

    bool converged_{false};                         ///< 整体收敛标志
    std::array<bool, kAxisCount> converged_axis_{}; ///< 每轴收敛标志
    std::array<int, kAxisCount> axis_iters_{};      ///< 每轴迭代次数
    int iters_{0};                                  ///< 总迭代次数(取最大值)

    /**
     * @brief 初始化单轴配置参数
     *
     * 将配置结构体转换为内部参数格式
     *
     * @param axis 轴索引
     * @param cfg 配置参数
     */
    void init_axis(int axis, const AxisConfig& cfg) noexcept {
        q_pos_(axis)              = static_cast<double>(cfg.q_pos);
        q_vel_(axis)              = static_cast<double>(cfg.q_vel);
        r_val_(axis)              = static_cast<double>(cfg.r);
        u_min_(axis)              = -static_cast<double>(cfg.max_acc);
        u_max_(axis)              = static_cast<double>(cfg.max_acc);
        enable_state_bound_[axis] = cfg.enable_state_bound;
    }

    /**
     * @brief 重置工作空间
     *
     * 清零所有数据矩阵和状态标志,为新一轮求解做准备
     */
    void reset_workspace() noexcept {
        x0_pos_.setZero();
        x0_vel_.setZero();
        ref_pos_.setZero();
        ref_vel_.setZero();
        sol_pos_.setZero();
        sol_vel_.setZero();
        sol_acc_.setZero();
        objective_.setZero();
        dyn_violation_.setZero();
        input_violation_.setZero();
        kkt_violation_.setZero();
        converged_axis_.fill(false);
        axis_iters_.fill(0);
        converged_ = false;
        iters_     = 0;
    }

    /**
     * @brief 构建消元后的QP问题
     *
     * 核心算法步骤:
     * 1. 从初始状态前向传播,计算无控制输入的基础轨迹
     * 2. 构建灵敏度矩阵Gp和Gv,表示输入对状态的影响
     * 3. 组装Hessian矩阵H和线性项h
     *
     * 数学推导:
     * - 状态方程: x[k] = base[k] + G[k]*u[0:k-1]
     * - 目标函数: minimize 0.5*(x-x_ref)^T Q (x-x_ref) + 0.5*u^T R u
     * - 展开后: H = G^T Q G + R,  h = G^T Q (base - x_ref)
     *
     * @param axis 轴索引
     * @param H [out] Hessian矩阵(M×M)
     * @param h [out] 线性项向量(M×1)
     * @param base_pos [out] 基础位置轨迹(无控制)
     * @param base_vel [out] 基础速度轨迹(无控制)
     * @param Gp [out] 位置灵敏度矩阵(N×M)
     * @param Gv [out] 速度灵敏度矩阵(N×M)
     */
    void build_condensed_problem(
        int axis, Eigen::MatrixXd& H, Eigen::VectorXd& h, Eigen::VectorXd& base_pos,
        Eigen::VectorXd& base_vel, Eigen::MatrixXd& Gp, Eigen::MatrixXd& Gv) const {
        const int M = num_inputs();
        base_pos    = Eigen::VectorXd::Zero(N_);
        base_vel    = Eigen::VectorXd::Zero(N_);
        Gp          = Eigen::MatrixXd::Zero(N_, M);
        Gv          = Eigen::MatrixXd::Zero(N_, M);

        base_pos(0) = x0_pos_(axis);
        base_vel(0) = x0_vel_(axis);

        for (int k = 0; k < M; ++k) {
            base_pos(k + 1) = a00_ * base_pos(k) + a01_ * base_vel(k) + f_pos_;
            base_vel(k + 1) = a10_ * base_pos(k) + a11_ * base_vel(k) + f_vel_;

            Gp.row(k + 1) = a00_ * Gp.row(k) + a01_ * Gv.row(k);
            Gv.row(k + 1) = a10_ * Gp.row(k) + a11_ * Gv.row(k);
            if (b0_ != 0.0) {
                Gp(k + 1, k) += b0_;
            }
            Gv(k + 1, k) += b1_;
        }

        Eigen::VectorXd rpos(N_);
        Eigen::VectorXd rvel(N_);
        for (int k = 0; k < N_; ++k) {
            rpos(k) = ref_pos_(axis, k);
            rvel(k) = ref_vel_(axis, k);
        }

        H = Eigen::MatrixXd::Zero(M, M);
        h = Eigen::VectorXd::Zero(M);
        if (q_pos_(axis) != 0.0) {
            H.noalias() += q_pos_(axis) * (Gp.transpose() * Gp);
            h.noalias() += q_pos_(axis) * (Gp.transpose() * (base_pos - rpos));
        }
        if (q_vel_(axis) != 0.0) {
            H.noalias() += q_vel_(axis) * (Gv.transpose() * Gv);
            h.noalias() += q_vel_(axis) * (Gv.transpose() * (base_vel - rvel));
        }
        if (r_val_(axis) != 0.0) {
            H.diagonal().array() += r_val_(axis);
        }
    }

    static double projected_gradient_kkt_residual(
        const Eigen::VectorXd& u, const Eigen::VectorXd& grad, double lo, double hi) noexcept {
        double max_res             = 0.0;
        constexpr double kBoundTol = 1e-10;
        for (int i = 0; i < u.size(); ++i) {
            double res = 0.0;
            if (u(i) <= lo + kBoundTol) {
                // lower-bound active: stationarity requires grad >= 0.
                res = std::max(0.0, -grad(i));
            } else if (u(i) >= hi - kBoundTol) {
                // upper-bound active: stationarity requires grad <= 0.
                res = std::max(0.0, grad(i));
            } else {
                res = std::abs(grad(i));
            }
            max_res = std::max(max_res, res);
        }
        return max_res;
    }

    [[nodiscard]] bool solve_box_qp_projected_gradient(
        const Eigen::MatrixXd& H, const Eigen::VectorXd& h, double lo, double hi,
        Eigen::VectorXd& u, int& iterations, double& kkt) const noexcept {
        const int M = static_cast<int>(h.size());
        if (M == 0) {
            u.resize(0);
            iterations = 0;
            kkt        = 0.0;
            return true;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H);
        if (eig.info() != Eigen::Success) {
            return false;
        }
        const double lmax = std::max(1e-18, eig.eigenvalues().maxCoeff());
        const double step = 1.0 / lmax;

        // Start from clipped unconstrained Newton step if possible, otherwise zero.
        Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
        if (ldlt.info() == Eigen::Success) {
            u = clamp_vec(ldlt.solve(-h), lo, hi);
        } else {
            u = Eigen::VectorXd::Zero(M);
            u = clamp_vec(u, lo, hi);
        }
        if (!u.allFinite()) {
            u = Eigen::VectorXd::Zero(M);
            u = clamp_vec(u, lo, hi);
        }

        Eigen::VectorXd y = u;
        double t          = 1.0;
        double last_obj   = std::numeric_limits<double>::infinity();

        for (iterations = 1; iterations <= max_iter_; ++iterations) {
            const Eigen::VectorXd grad_y = H * y + h;
            Eigen::VectorXd u_next       = clamp_vec(y - step * grad_y, lo, hi);

            const Eigen::VectorXd grad_next = H * u_next + h;
            kkt              = projected_gradient_kkt_residual(u_next, grad_next, lo, hi);
            const double obj = 0.5 * u_next.dot(H * u_next) + h.dot(u_next);

            if (kkt <= abs_tol_) {
                u = u_next;
                return true;
            }

            // Monotone FISTA restart. If acceleration overshoots, fall back to plain PG.
            if (obj > last_obj) {
                y = u;
                t = 1.0;
                continue;
            }

            const double t_next = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * t * t));
            y                   = u_next + ((t - 1.0) / t_next) * (u_next - u);
            y                   = clamp_vec(y, lo, hi);
            u                   = u_next;
            t                   = t_next;
            last_obj            = obj;
        }

        const Eigen::VectorXd grad = H * u + h;
        kkt                        = projected_gradient_kkt_residual(u, grad, lo, hi);
        return kkt <= std::max(abs_tol_, 1e-7);
    }

    [[nodiscard]] bool solve_axis(int axis) noexcept {
        Eigen::MatrixXd H;
        Eigen::VectorXd h;
        Eigen::VectorXd base_pos;
        Eigen::VectorXd base_vel;
        Eigen::MatrixXd Gp;
        Eigen::MatrixXd Gv;
        build_condensed_problem(axis, H, h, base_pos, base_vel, Gp, Gv);

        Eigen::VectorXd u;
        int iterations = 0;
        double kkt     = std::numeric_limits<double>::infinity();
        const bool ok =
            solve_box_qp_projected_gradient(H, h, u_min_(axis), u_max_(axis), u, iterations, kkt);

        axis_iters_[axis]    = iterations;
        kkt_violation_(axis) = kkt;
        if (!ok || u.size() != num_inputs() || !u.allFinite()) {
            return false;
        }

        const Eigen::VectorXd pos = base_pos + Gp * u;
        const Eigen::VectorXd vel = base_vel + Gv * u;
        for (int k = 0; k < N_; ++k) {
            sol_pos_(axis, k) = pos(k);
            sol_vel_(axis, k) = vel(k);
        }
        for (int k = 0; k < num_inputs(); ++k) {
            sol_acc_(axis, k) = clamp_scalar(u(k), u_min_(axis), u_max_(axis));
        }

        compute_diagnostics(axis);
        return true;
    }

    void compute_diagnostics(int axis) noexcept {
        double obj = 0.0;
        for (int k = 0; k < N_; ++k) {
            const double ep = sol_pos_(axis, k) - ref_pos_(axis, k);
            const double ev = sol_vel_(axis, k) - ref_vel_(axis, k);
            obj += 0.5 * q_pos_(axis) * ep * ep;
            obj += 0.5 * q_vel_(axis) * ev * ev;
        }
        for (int k = 0; k < num_inputs(); ++k) {
            const double u = sol_acc_(axis, k);
            obj += 0.5 * r_val_(axis) * u * u;
        }
        objective_(axis) = obj;

        double dyn_max = 0.0;
        for (int k = 0; k < num_inputs(); ++k) {
            const double pred_pos = a00_ * sol_pos_(axis, k) + a01_ * sol_vel_(axis, k)
                                  + b0_ * sol_acc_(axis, k) + f_pos_;
            const double pred_vel = a10_ * sol_pos_(axis, k) + a11_ * sol_vel_(axis, k)
                                  + b1_ * sol_acc_(axis, k) + f_vel_;
            dyn_max = std::max(dyn_max, std::abs(sol_pos_(axis, k + 1) - pred_pos));
            dyn_max = std::max(dyn_max, std::abs(sol_vel_(axis, k + 1) - pred_vel));
        }
        dyn_violation_(axis) = dyn_max;

        double input_vio = 0.0;
        for (int k = 0; k < num_inputs(); ++k) {
            input_vio = std::max(input_vio, u_min_(axis) - sol_acc_(axis, k));
            input_vio = std::max(input_vio, sol_acc_(axis, k) - u_max_(axis));
        }
        input_violation_(axis) = std::max(0.0, input_vio);
    }
};

// Optional benchmark-side alias. Do not enable this in production next to DualSmallMpcSolver.
// using DualSmallMpcSolver = DualMpcOsqpSolver;

} // namespace fcs::L5
