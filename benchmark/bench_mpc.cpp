// 双轴MPC严格基准测试：两套求解器对比
// 对比对象：2组独立TinyMPC（双精度动态分配） vs 一体化DualSmallMpcSolver（单精度定点批量双轴）
//
// 正确性校验目标：仅对比产线实际使用的6个中心输出量（中间预测步）
//     yaw         偏航角位置
//     pitch       俯仰角位置
//     yaw_vel     偏航角速度
//     pitch_vel   俯仰角速度
//     yaw_accel   偏航加速度控制输入
//     pitch_accel 俯仰加速度控制输入
//
// 编译指令：cmake --build build --target bench_mpc
// 运行指令：./build/bin/bench_mpc

// 自研双轴一体化MPC求解器头文件（float定点批量双轴）
#include "dual_small_mpc_solver.hpp"
// TinyMPC 单轴求解器C API（double动态内存单轴）
#include "tiny_api.hpp"

// Eigen 线性代数矩阵库，状态/参考轨迹存储
#include <Eigen/Core>

// C++标准库,
#include <algorithm> // std::sort, std::max, std::clamp, std::min
#include <array>     // 固定数组
#include <cassert>   // 运行时断言，校验求解器创建成功
#include <chrono>    // 高精度计时基准测试
#include <cmath>     // 数学计算 sin/cos/pi/abs/sqrt
#include <cstdlib>   // 基础C标准库
#include <iomanip>   // 控制台格式化输出 精度/对齐/科学计数法
#include <iostream>  // 标准输入输出
#include <memory>    // std::unique_ptr 自动内存管理
#include <numbers>   // C++20 标准数学常量 π
#include <random>    // 随机噪声生成器（噪声参考轨迹）
#include <string>    // 字符串
#include <utility>   // std::move 所有权转移
#include <vector>    // 动态数组，存储耗时采样、误差统计

// 项目命名空间：飞控L5机型
using namespace fcs::L5;

/**
 * @brief TinySolver 自定义删除器，释放TinyMPC内部多层动态内存
 * TinyMPC C API 裸指针包含多层堆分配：work、settings、cache、solution、solver本体
 * 不能直接delete，需要逐层递归释放防止内存泄漏
 */
struct TinySolverDeleter {
    void operator()(TinySolver* s) const noexcept {
        if (!s)
            return;
        // 泛型删除闭包，统一释放任意裸指针
        auto d = [](auto* p) { delete p; };
        d(s->work);
        d(s->settings);
        d(s->cache);
        d(s->solution);
        d(s);
    }
};
// TinyMPC求解器自动释放智能指针封装
using TinySolverPtr = std::unique_ptr<TinySolver, TinySolverDeleter>;

/**
 * @brief 编译器内存屏障，阻止编译器跨区间优化计时区间代码
 * 防止编译器把计时区间内的求解计算提前/延后，导致计时失真
 * [[maybe_unused]] 未使用不触发编译警告
 */
[[maybe_unused]] static inline void compiler_barrier() noexcept { asm volatile("" ::: "memory"); }

/**
 * @brief 强制编译器不优化掉目标变量，防止基准测试采样被优化消除
 * 将变量地址传入汇编，编译器无法判定变量无副作用删除
 * @param val 任意类型待保护变量
 */
template <typename T>
[[maybe_unused]] static inline void do_not_optimize(const T& val) noexcept {
    asm volatile("" : : "r"(&val) : "memory");
}

// 高精度稳定时钟，不受系统时钟调整影响
using Clock = std::chrono::steady_clock;
// 时钟时间点类型
using TimePoint = std::chrono::time_point<Clock>;

/**
 * @brief 计算两个时间点差值，返回微秒浮点数
 * @param a 起始时间点
 * @param b 结束时间点
 * @return 浮点微秒耗时
 */
[[nodiscard]] static inline double elapsed_us(TimePoint a, TimePoint b) noexcept {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

// ===================== 全局MPC超参数常量 =====================
// 前向预测时域长度
static constexpr int kHorizonAhead = 50;
// 后向历史时域长度（中心输出取该位置）
static constexpr int kHorizonBack = 50;
// 完整预测总时域：历史+前向+当前步
static constexpr int kHorizon = kHorizonAhead + kHorizonBack + 1;
// 离散控制步长 10ms
static constexpr double kDt = 0.01;
// ADMM 惩罚系数 rho
static constexpr double kRho = 1.0;
// 原始/对偶残差收敛阈值
static constexpr double kAbsTol = 1e-3;
// 基准预热迭代次数，消除冷启动缓存/分支预测干扰
static constexpr int kWarmup = 200;
// 基准正式采样迭代次数，统计分布分位数
static constexpr int kTrials = 1000;

/**
 * @brief 单轴MPC权重与硬件约束参数结构体
 */
struct AxisParams {
    double q_pos;   // 位置代价权重
    double q_vel;   // 速度代价权重
    double r;       // 控制输入加速度代价权重
    double max_acc; // 最大允许加速度控制输入上限
};

// 偏航轴配置参数
static constexpr AxisParams kYaw = {9e6, 0.0, 1.0, 50.0};
// 俯仰轴配置参数（带位置硬约束）
static constexpr AxisParams kPitch = {9e6, 0.0, 1.0, 100.0};

// 参考轨迹矩阵：2行N列 [位置; 速度]
using RefTrajectory = Eigen::MatrixXd;

/**
 * @brief 生成正弦参考轨迹（平滑周期运动）
 * @param N 时域长度 kHorizon
 * @param freq 正弦频率 Hz
 * @param amp 振幅 弧度
 * @return 2行时域参考矩阵
 */
[[nodiscard]] static RefTrajectory ref_sinusoid(int N, double freq, double amp) {
    Eigen::MatrixXd ref(2, N);
    const double omega = 2.0 * std::numbers::pi_v<double> * freq;
    for (int i = 0; i < N; ++i) {
        // 时间轴以历史后沿为0点对齐
        const double t = static_cast<double>(i - kHorizonBack) * kDt;
        ref(0, i)      = amp * std::sin(omega * t);
        ref(1, i)      = amp * omega * std::cos(omega * t);
    }
    return ref;
}

/**
 * @brief 线性扫频chirp轨迹（频率从0.5~3Hz线性上升）
 * 位置与速度严格同源微分，无轨迹内部不一致误差
 * @param N 时域长度
 * @param amp 振幅
 * @return 2行参考矩阵
 */
[[nodiscard]] static RefTrajectory ref_chirp(int N, double amp) {
    Eigen::MatrixXd ref(2, N);
    const double f0 = 0.5;
    const double f1 = 3.0;
    const double T  = static_cast<double>(std::max(N - 1, 1)) * kDt;

    for (int i = 0; i < N; ++i) {
        const double tau = static_cast<double>(i) * kDt;
        // 当前瞬时频率
        const double freq = f0 + (f1 - f0) * tau / T;
        // 线性调频积分相位
        const double phase =
            2.0 * std::numbers::pi_v<double> * (f0 * tau + 0.5 * (f1 - f0) / T * tau * tau);

        ref(0, i) = amp * std::sin(phase);
        // 速度 = d(pos)/dt 解析导数，保证轨迹自洽
        ref(1, i) = amp * std::cos(phase) * 2.0 * std::numbers::pi_v<double> * freq;
    }

    return ref;
}

/**
 * @brief 带高斯噪声正弦参考轨迹（模拟真实传感器噪声）
 * @param N 时域长度
 * @param seed 随机种子固定可复现
 * @param amp 基础正弦振幅
 * @return 带噪声2行参考矩阵
 */
[[nodiscard]] static RefTrajectory ref_noisy_sinusoid(int N, unsigned seed, double amp) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, 0.01f);
    auto ref = ref_sinusoid(N, 1.0, amp);
    for (int i = 0; i < N; ++i) {
        ref(0, i) += static_cast<double>(noise(rng));
        // 速度噪声放大10倍，模拟微分放大噪声
        ref(1, i) += static_cast<double>(noise(rng)) * 10.0;
    }
    return ref;
}

/**
 * @brief 零参考轨迹（定点悬停）
 * @param N 时域长度
 * @return 全零矩阵
 */
[[nodiscard]] static RefTrajectory ref_zero(int N) { return Eigen::MatrixXd::Zero(2, N); }

/**
 * @brief 阶跃参考轨迹（恒定位置、恒定速度）
 * @param N 时域长度
 * @param pos 目标恒定位置
 * @param vel 目标恒定速度 默认0
 * @return 2行常数矩阵
 */
[[nodiscard]] static RefTrajectory ref_step(int N, double pos, double vel = 0.0) {
    Eigen::MatrixXd ref(2, N);
    ref.row(0).setConstant(pos);
    ref.row(1).setConstant(vel);
    return ref;
}

/**
 * @brief 斜坡线性轨迹（恒定速度持续运动）
 * @param N 时域长度
 * @param start 初始位置
 * @param slope 速度斜率 rad/s
 * @return 线性增长位置、恒定速度
 */
[[nodiscard]] static RefTrajectory ref_ramp(int N, double start, double slope) {
    Eigen::MatrixXd ref(2, N);
    for (int i = 0; i < N; ++i) {
        const double t = static_cast<double>(i - kHorizonBack) * kDt;
        ref(0, i)      = start + slope * t;
        ref(1, i)      = slope;
    }
    return ref;
}

// ===================== TinyMPC 双轴求解器封装 =====================
/**
 * @brief 双独立TinyMPC求解器容器，偏航/俯仰各单轴实例
 */
struct TinyPairSetup {
    TinySolverPtr yaw;
    TinySolverPtr pitch;

    // 构造时自动创建两个单轴求解器
    TinyPairSetup() {
        yaw   = make(false);
        pitch = make(true);
    }

private:
    /**
     * @brief 静态创建单轴TinyMPC求解器实例
     * @param is_pitch true=俯仰轴（带位置硬约束） false=偏航轴（无位置约束）
     * @return 自动释放智能指针TinySolverPtr
     */
    [[nodiscard]] static TinySolverPtr make(bool is_pitch) {
        const AxisParams& p = is_pitch ? kPitch : kYaw;
        // 离散状态矩阵 A=[1 dt; 0 1] 积分模型
        const Eigen::Matrix2d A = (Eigen::Matrix2d{} << 1.0, kDt, 0.0, 1.0).finished();
        // 输入矩阵 B=[0; dt] 加速度输入积分速度
        const Eigen::Matrix<double, 2, 1> B =
            (Eigen::Matrix<double, 2, 1>{} << 0.0, kDt).finished();
        // 常数扰动项 0
        const Eigen::Vector2d f = Eigen::Vector2d::Zero();
        // 状态代价对角矩阵 [q_pos, q_vel]
        const Eigen::Matrix2d Q = Eigen::Vector2d{p.q_pos, p.q_vel}.asDiagonal();
        // 输入代价标量矩阵 R=r
        const Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Constant(p.r);

        TinySolver* raw = nullptr;
        // 调用C API分配求解器内存、初始化模型
        tiny_setup(&raw, A, B, f, Q, R, kRho, 2, 1, kHorizon, 0);
        TinySolverPtr solver(raw);

        auto* s = solver.get();
        // 收敛阈值与全局常量对齐
        s->settings->abs_pri_tol = kAbsTol;
        s->settings->abs_dua_tol = kAbsTol;
        // 开启迭代终止判断
        s->settings->check_termination = 1;
        // 开启输入加速度上下限约束
        s->settings->en_input_bound = 1;
        // 俯仰轴开启位置硬约束，偏航关闭
        s->settings->en_state_bound = is_pitch ? 1 : 0;

        // 默认状态上下限极大值（无约束）
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);

        // 俯仰轴位置硬约束 ±40°=±0.698rad
        if (is_pitch) {
            x_min.row(0).setConstant(-0.698);
            x_max.row(0).setConstant(0.698);
        }

        // 设置状态约束、输入加速度约束
        tiny_set_bound_constraints(
            s, x_min, x_max, Eigen::MatrixXd::Constant(1, kHorizon - 1, -p.max_acc),
            Eigen::MatrixXd::Constant(1, kHorizon - 1, p.max_acc));

        return solver;
    }
};

// ===================== 一体化双轴浮点MPC求解器封装 =====================
struct DualSetup {
    DualSmallMpcSolver solver;

    // 构造时创建批量双轴浮点求解器
    DualSetup() {
        DualSmallMpcSolver::AxisConfig yaw_cfg;
        yaw_cfg.q_pos   = static_cast<float>(kYaw.q_pos);
        yaw_cfg.q_vel   = static_cast<float>(kYaw.q_vel);
        yaw_cfg.r       = static_cast<float>(kYaw.r);
        yaw_cfg.max_acc = static_cast<float>(kYaw.max_acc);

        DualSmallMpcSolver::AxisConfig pitch_cfg;
        pitch_cfg.q_pos              = static_cast<float>(kPitch.q_pos);
        pitch_cfg.q_vel              = static_cast<float>(kPitch.q_vel);
        pitch_cfg.r                  = static_cast<float>(kPitch.r);
        pitch_cfg.max_acc            = static_cast<float>(kPitch.max_acc);
        pitch_cfg.enable_state_bound = true;
        pitch_cfg.state_min          = -0.698f;
        pitch_cfg.state_max          = 0.698f;

        // 静态工厂创建双轴求解器实例
        auto result = DualSmallMpcSolver::create(
            static_cast<float>(kDt), kHorizon, static_cast<float>(kRho), yaw_cfg, pitch_cfg);
        // 断言求解器创建成功，失败直接终止程序
        assert(result.has_value());

        // 转移所有权到成员变量
        solver = std::move(*result);
        // 设置收敛阈值、最大迭代次数默认1000
        solver.set_settings(static_cast<float>(kAbsTol), 1000);
    }
};

/**
 * @brief 基准统计输出结构体，存储单次基准全部采样分布指标
 */
struct Stats {
    double min;          // 最小耗时微秒
    double p50;          // 50分位数（中位数）
    double p95;          // 95分位数（长尾性能指标）
    double max;          // 最大耗时微秒
    double mean;         // 算术平均耗时
    double stddev;       // 标准差，波动程度
    int converged_count; // 迭代收敛成功次数
};

/**
 * @brief 从耗时采样数组计算全套统计分位数、均值、标准差
 * @param samples 所有迭代耗时微秒数组
 * @param converged 收敛成功次数
 * @return 填充完整Stats结构体
 */
[[nodiscard]] static Stats compute_stats(std::vector<double> samples, int converged) {
    assert(!samples.empty());

    // 排序用于分位数提取
    std::sort(samples.begin(), samples.end());

    double sum = 0.0;
    for (double s : samples)
        sum += s;

    const double mean = sum / static_cast<double>(samples.size());

    double sq_sum = 0.0;
    for (double s : samples) {
        const double d = s - mean;
        sq_sum += d * d;
    }

    // 95分位数下标
    const size_t p95_index = std::min(
        samples.size() - 1, static_cast<size_t>(static_cast<double>(samples.size()) * 0.95));

    return {
        samples.front(),
        samples[samples.size() / 2],
        samples[p95_index],
        samples.back(),
        mean,
        std::sqrt(sq_sum / static_cast<double>(samples.size())),
        converged,
    };
}

/**
 * @brief 控制台格式化打印一套基准统计
 * @param label 求解器名称标签
 * @param s 统计结构体
 * @param total 总迭代次数
 */
static void print_stats(const char* label, const Stats& s, int total) {
    std::cout << "  " << std::left << std::setw(24) << label << " ";
    std::cout << std::fixed << std::setprecision(2) << "p50=" << std::setw(8) << s.p50 << "us"
              << " mean=" << std::setw(8) << s.mean << "us"
              << " p95=" << std::setw(8) << s.p95 << "us"
              << " min=" << std::setw(8) << s.min << "us"
              << " max=" << std::setw(8) << s.max << "us"
              << " conv=" << s.converged_count << "/" << total << "\n";
}

/**
 * @brief 单次基准测试返回值：统计+本次最大迭代步数
 */
struct TimingResult {
    Stats stats;
    int iterations;
};

/**
 * @brief 仅执行solve求解运算基准（排除初始化、参考赋值开销）
 * @param setup Tiny双轴求解器实例
 * @param yaw_ref 偏航参考轨迹
 * @param pitch_ref 俯仰参考轨迹
 * @param max_iter 单次求解最大迭代上限
 * @param trials 采样迭代次数 kTrials
 * @return 耗时分布统计+本次最大迭代步数
 */
[[nodiscard]] static TimingResult bench_tiny_pair_solve(
    TinyPairSetup& setup, const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref,
    int max_iter, int trials) {
    auto* yaw                 = setup.yaw.get();
    auto* pitch               = setup.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;

    // 预热迭代，消除缓存/分支预测冷启动影响
    for (int w = 0; w < kWarmup; ++w) {
        tiny_set_x0(yaw, yaw_ref.col(0));
        tiny_set_x0(pitch, pitch_ref.col(0));
        yaw->work->Xref   = yaw_ref;
        pitch->work->Xref = pitch_ref;
        tiny_solve(yaw);
        tiny_solve(pitch);
    }

    std::vector<double> times;
    times.reserve(trials);

    int converged      = 0;
    int max_iterations = 0;

    // 正式采样计时循环
    for (int i = 0; i < trials; ++i) {
        tiny_set_x0(yaw, yaw_ref.col(0));
        tiny_set_x0(pitch, pitch_ref.col(0));
        yaw->work->Xref   = yaw_ref;
        pitch->work->Xref = pitch_ref;

        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        // 核心求解运算
        const int yaw_status   = tiny_solve(yaw);
        const int pitch_status = tiny_solve(pitch);

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        // 阻止编译器优化删除求解返回值
        do_not_optimize(yaw_status);
        do_not_optimize(pitch_status);

        // 双轴全部收敛才算单次收敛成功
        if (yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
            && pitch->solution->solved == 1) {
            ++converged;
        }

        // 记录本次循环最大迭代步数
        max_iterations =
            std::max(max_iterations, std::max(yaw->solution->iter, pitch->solution->iter));
        times.push_back(elapsed_us(t0, t1));
    }

    return {compute_stats(std::move(times), converged), max_iterations};
}

/**
 * @brief DualSmallMpcSolver 仅solve求解基准（纯运算耗时）
 */
[[nodiscard]] static TimingResult bench_dual_solve(
    DualSetup& setup, const Eigen::MatrixXd& yaw_ref_d, const Eigen::MatrixXd& pitch_ref_d,
    int max_iter, int trials) {
    setup.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    // 双精度参考轨迹转float单精度
    const Eigen::MatrixXf yaw_ref   = yaw_ref_d.cast<float>();
    const Eigen::MatrixXf pitch_ref = pitch_ref_d.cast<float>();

    // 预热
    for (int w = 0; w < kWarmup; ++w) {
        setup.solver.set_x0(yaw_ref_d(0, 0), yaw_ref_d(1, 0), pitch_ref_d(0, 0), pitch_ref_d(1, 0));
        setup.solver.set_reference(yaw_ref, pitch_ref);
        setup.solver.solve();
    }

    std::vector<double> times;
    times.reserve(trials);

    int converged      = 0;
    int max_iterations = 0;

    for (int i = 0; i < trials; ++i) {
        setup.solver.set_x0(yaw_ref_d(0, 0), yaw_ref_d(1, 0), pitch_ref_d(0, 0), pitch_ref_d(1, 0));
        setup.solver.set_reference(yaw_ref, pitch_ref);

        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        const bool ok = setup.solver.solve();

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        do_not_optimize(ok);

        if (ok)
            ++converged;

        max_iterations = std::max(max_iterations, setup.solver.iterations());
        times.push_back(elapsed_us(t0, t1));
    }

    return {compute_stats(std::move(times), converged), max_iterations};
}

/**
 * @brief TinyMPC完整流水线基准（包含初始化、参考赋值+solve全流程）
 * 模拟真实飞控完整调用链路开销
 */
[[nodiscard]] static Stats bench_tiny_pair_pipeline(
    TinyPairSetup& setup, const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref,
    int max_iter, int trials) {
    auto* yaw                 = setup.yaw.get();
    auto* pitch               = setup.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;

    std::vector<double> times;
    times.reserve(trials);

    int converged = 0;

    for (int i = 0; i < trials; ++i) {
        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        // 完整业务流水线：初始状态赋值+参考轨迹写入+求解
        tiny_set_x0(yaw, yaw_ref.col(0));
        tiny_set_x0(pitch, pitch_ref.col(0));
        yaw->work->Xref        = yaw_ref;
        pitch->work->Xref      = pitch_ref;
        const int yaw_status   = tiny_solve(yaw);
        const int pitch_status = tiny_solve(pitch);

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        if (yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
            && pitch->solution->solved == 1) {
            ++converged;
        }

        times.push_back(elapsed_us(t0, t1));
    }

    return compute_stats(std::move(times), converged);
}

/**
 * @brief Dual一体化求解器完整流水线基准（初始化+参考赋值+solve全链路）
 */
[[nodiscard]] static Stats bench_dual_pipeline(
    DualSetup& setup, const Eigen::MatrixXd& yaw_ref_d, const Eigen::MatrixXd& pitch_ref_d,
    int max_iter, int trials) {
    setup.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    const Eigen::MatrixXf yaw_ref   = yaw_ref_d.cast<float>();
    const Eigen::MatrixXf pitch_ref = pitch_ref_d.cast<float>();

    std::vector<double> times;
    times.reserve(trials);

    int converged = 0;

    for (int i = 0; i < trials; ++i) {
        compiler_barrier();
        const auto t0 = Clock::now();
        compiler_barrier();

        setup.solver.set_x0(yaw_ref_d(0, 0), yaw_ref_d(1, 0), pitch_ref_d(0, 0), pitch_ref_d(1, 0));
        setup.solver.set_reference(yaw_ref, pitch_ref);
        const bool ok = setup.solver.solve();

        compiler_barrier();
        const auto t1 = Clock::now();
        compiler_barrier();

        if (ok)
            ++converged;

        times.push_back(elapsed_us(t0, t1));
    }

    return compute_stats(std::move(times), converged);
}

// ================================================================================================
// 核心正确性校验模块：中心输出误差对比（产线真实使用的6个输出量，唯一通过标准）
// ================================================================================================

/**
 * @brief 产线实际使用6个中心输出量结构体
 */
struct CenterOutput {
    double yaw;
    double pitch;
    double yaw_vel;
    double pitch_vel;
    double yaw_accel;
    double pitch_accel;
};

/**
 * @brief 两套求解器输出对比完整报告
 */
struct CenterOutputReport {
    CenterOutput tiny;     // TinyMPC输出
    CenterOutput dual;     // Dual一体化求解器输出
    CenterOutput abs_err;  // 各分量绝对误差
    CenterOutput norm_err; // 归一化工程误差（按执行量程缩放）

    double max_abs_err;    // 全局最大绝对误差
    double max_norm_err;   // 全局最大归一化误差

    int center_index;      // 中心时域步索引 kHorizonBack
    int control_index;     // 控制输入时域步索引 center_index-1

    bool tiny_converged;   // Tiny双轴是否全部收敛
    bool dual_converged;   // Dual求解器是否收敛
    bool pass;             // 整体正确性校验是否通过
};

/**
 * @brief 取输出结构体六个分量中最大数值
 */
[[nodiscard]] static double max_center_output_component(const CenterOutput& v) {
    return std::max({
        v.yaw,
        v.pitch,
        v.yaw_vel,
        v.pitch_vel,
        v.yaw_accel,
        v.pitch_accel,
    });
}

/**
 * @brief 计算两套输出各分量绝对差值
 */
[[nodiscard]] static CenterOutput abs_diff(const CenterOutput& a, const CenterOutput& b) {
    return {
        std::abs(a.yaw - b.yaw),
        std::abs(a.pitch - b.pitch),
        std::abs(a.yaw_vel - b.yaw_vel),
        std::abs(a.pitch_vel - b.pitch_vel),
        std::abs(a.yaw_accel - b.yaw_accel),
        std::abs(a.pitch_accel - b.pitch_accel),
    };
}

/**
 * @brief 按工程执行量程归一化误差，消除量纲差异统一评判标准
 * 缩放系数为真实飞控执行机构物理量程
 */
[[nodiscard]] static CenterOutput normalized_error(const CenterOutput& e) {
    constexpr double kYawScale        = 1.0;
    constexpr double kPitchScale      = 1.0;
    constexpr double kYawVelScale     = 10.0;
    constexpr double kPitchVelScale   = 10.0;
    constexpr double kYawAccelScale   = kYaw.max_acc;
    constexpr double kPitchAccelScale = kPitch.max_acc;

    return {
        e.yaw / kYawScale,
        e.pitch / kPitchScale,
        e.yaw_vel / kYawVelScale,
        e.pitch_vel / kPitchVelScale,
        e.yaw_accel / kYawAccelScale,
        e.pitch_accel / kPitchAccelScale,
    };
}

/**
 * @brief 同一参考轨迹下，运行两套求解器，对比中心输出生成正确性报告
 * @param yaw_ref 偏航参考轨迹
 * @param pitch_ref 俯仰参考轨迹
 * @param max_iter 求解最大迭代步数
 * @return 完整误差对比报告
 */
[[nodiscard]] static CenterOutputReport compare_center_output(
    const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref, int max_iter) {
    TinyPairSetup tiny;
    DualSetup dual;

    auto* yaw                 = tiny.yaw.get();
    auto* pitch               = tiny.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;
    dual.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    // TinyMPC求解
    tiny_set_x0(yaw, yaw_ref.col(0));
    tiny_set_x0(pitch, pitch_ref.col(0));
    yaw->work->Xref        = yaw_ref;
    pitch->work->Xref      = pitch_ref;
    const int yaw_status   = tiny_solve(yaw);
    const int pitch_status = tiny_solve(pitch);

    // Dual一体化求解
    dual.solver.set_x0(yaw_ref(0, 0), yaw_ref(1, 0), pitch_ref(0, 0), pitch_ref(1, 0));
    dual.solver.set_reference(yaw_ref.cast<float>(), pitch_ref.cast<float>());
    const bool dual_ok = dual.solver.solve();

    // 产线固定索引规则：中心步=历史时域后端，控制输入步=中心前一步
    const int effective_horizon = kHorizon;
    const int center_index      = std::clamp(kHorizonBack, 0, effective_horizon - 1);
    const int control_index     = std::clamp(center_index, 0, effective_horizon - 2);

    // 提取TinyMPC中心6个输出
    const CenterOutput tiny_out{
        yaw->solution->x(0, center_index),  pitch->solution->x(0, center_index),
        yaw->solution->x(1, center_index),  pitch->solution->x(1, center_index),
        yaw->solution->u(0, control_index), pitch->solution->u(0, control_index),
    };

    // 提取Dual一体化求解器6个中心输出
    const CenterOutput dual_out{
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 0, center_index)),
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 0, center_index)),
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 1, center_index)),
        static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 1, center_index)),
        static_cast<double>(dual.solver.input(DualSmallMpcSolver::kYawAxis, control_index)),
        static_cast<double>(dual.solver.input(DualSmallMpcSolver::kPitchAxis, control_index)),
    };

    const CenterOutput err      = abs_diff(tiny_out, dual_out);
    const CenterOutput norm_err = normalized_error(err);

    const double max_abs  = max_center_output_component(err);
    const double max_norm = max_center_output_component(norm_err);

    // Tiny双轴全部收敛判定
    const bool tiny_ok = yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
                      && pitch->solution->solved == 1;

    // 工程验收误差阈值
    constexpr double kPosTol   = 1e-3;
    constexpr double kVelTol   = 1e-2;
    constexpr double kAccelTol = 1e-1;
    constexpr double kNormTol  = 1e-3;

    // 全部误差满足阈值、两套均收敛才算通过
    const bool pass = tiny_ok && dual_ok && err.yaw <= kPosTol && err.pitch <= kPosTol
                   && err.yaw_vel <= kVelTol && err.pitch_vel <= kVelTol
                   && err.yaw_accel <= kAccelTol && err.pitch_accel <= kAccelTol
                   && max_norm <= kNormTol;

    return {
        tiny_out,     dual_out,      err,     norm_err, max_abs, max_norm,
        center_index, control_index, tiny_ok, dual_ok,  pass,
    };
}

/**
 * @brief 打印多场景误差汇总表头
 */
static void print_center_output_summary_header() {
    std::cout << "  " << std::left << std::setw(16) << "Scenario" << std::setw(12) << "yaw"
              << std::setw(12) << "pitch" << std::setw(12) << "yaw_vel" << std::setw(12)
              << "pit_vel" << std::setw(12) << "yaw_acc" << std::setw(12) << "pit_acc"
              << std::setw(12) << "max_norm" << std::setw(8) << "Pass"
              << "\n";
    std::cout << "  " << std::string(108, '-') << "\n";
}

/**
 * @brief 打印单场景误差汇总行（科学计数法显示绝对误差）
 */
static void print_center_output_summary_row(const char* name, const CenterOutputReport& r) {
    std::cout << "  " << std::left << std::setw(16) << name << std::scientific
              << std::setprecision(2) << std::setw(12) << r.abs_err.yaw << std::setw(12)
              << r.abs_err.pitch << std::setw(12) << r.abs_err.yaw_vel << std::setw(12)
              << r.abs_err.pitch_vel << std::setw(12) << r.abs_err.yaw_accel << std::setw(12)
              << r.abs_err.pitch_accel << std::setw(12) << r.max_norm_err << std::setw(8)
              << (r.pass ? "Y" : "N") << "\n";
}

/**
 * @brief 打印单场景完整详细误差报告，逐分量展示两套输出、绝对误差、归一化误差
 */
static void print_center_output_report(const char* label, const CenterOutputReport& r) {
    std::cout << "  " << label << "\n";
    std::cout << "  center_index=" << r.center_index << " control_index=" << r.control_index
              << " tiny_ok=" << (r.tiny_converged ? "Y" : "N")
              << " dual_ok=" << (r.dual_converged ? "Y" : "N") << " pass=" << (r.pass ? "Y" : "N")
              << "\n";

    auto print_row = [](const char* name, double tiny, double dual, double err, double norm) {
        std::cout << "    " << std::left << std::setw(12) << name << " tiny=" << std::scientific
                  << std::setprecision(6) << std::setw(14) << tiny << " dual=" << std::setw(14)
                  << dual << " abs_err=" << std::setw(14) << err << " norm_err=" << norm << "\n";
    };

    print_row("yaw", r.tiny.yaw, r.dual.yaw, r.abs_err.yaw, r.norm_err.yaw);
    print_row("pitch", r.tiny.pitch, r.dual.pitch, r.abs_err.pitch, r.norm_err.pitch);
    print_row("yaw_vel", r.tiny.yaw_vel, r.dual.yaw_vel, r.abs_err.yaw_vel, r.norm_err.yaw_vel);
    print_row(
        "pitch_vel", r.tiny.pitch_vel, r.dual.pitch_vel, r.abs_err.pitch_vel, r.norm_err.pitch_vel);
    print_row(
        "yaw_accel", r.tiny.yaw_accel, r.dual.yaw_accel, r.abs_err.yaw_accel, r.norm_err.yaw_accel);
    print_row(
        "pitch_accel", r.tiny.pitch_accel, r.dual.pitch_accel, r.abs_err.pitch_accel,
        r.norm_err.pitch_accel);

    std::cout << "    max_abs_err=" << std::scientific << std::setprecision(6) << r.max_abs_err
              << " max_norm_err=" << r.max_norm_err << "\n";
}

// ================================================================================================
// 全时域完整轨迹调试误差（仅调试使用，不作为正式正确性标准）
// ================================================================================================

/**
 * @brief 完整预测时域全状态/输入误差报告结构体，仅用于调试对比
 */
struct FullHorizonDebugReport {
    double max_abs_state_err; // 全时域最大状态绝对误差
    double rmse_state;        // 状态均方根误差
    double max_abs_input_err; // 全时域最大控制输入绝对误差
    bool tiny_converged;
    bool dual_converged;
};

/**
 * @brief 遍历全部时域步，计算整套轨迹完整误差（调试用）
 */
[[nodiscard]] static FullHorizonDebugReport compare_full_horizon_debug(
    const Eigen::MatrixXd& yaw_ref, const Eigen::MatrixXd& pitch_ref, int max_iter) {
    TinyPairSetup tiny;
    DualSetup dual;

    auto* yaw                 = tiny.yaw.get();
    auto* pitch               = tiny.pitch.get();
    yaw->settings->max_iter   = max_iter;
    pitch->settings->max_iter = max_iter;
    dual.solver.set_settings(static_cast<float>(kAbsTol), max_iter);

    tiny_set_x0(yaw, yaw_ref.col(0));
    tiny_set_x0(pitch, pitch_ref.col(0));
    yaw->work->Xref        = yaw_ref;
    pitch->work->Xref      = pitch_ref;
    const int yaw_status   = tiny_solve(yaw);
    const int pitch_status = tiny_solve(pitch);

    dual.solver.set_x0(yaw_ref(0, 0), yaw_ref(1, 0), pitch_ref(0, 0), pitch_ref(1, 0));
    dual.solver.set_reference(yaw_ref.cast<float>(), pitch_ref.cast<float>());
    const bool dual_ok = dual.solver.solve();

    double max_state = 0.0;
    double sum_sq    = 0.0;
    double max_input = 0.0;

    // 遍历所有时域步，计算状态误差
    for (int k = 0; k < kHorizon; ++k) {
        const double yaw_pos_err = std::abs(
            yaw->solution->x(0, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 0, k)));
        const double yaw_vel_err = std::abs(
            yaw->solution->x(1, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kYawAxis, 1, k)));
        const double pitch_pos_err = std::abs(
            pitch->solution->x(0, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 0, k)));
        const double pitch_vel_err = std::abs(
            pitch->solution->x(1, k)
            - static_cast<double>(dual.solver.state(DualSmallMpcSolver::kPitchAxis, 1, k)));

        max_state = std::max({max_state, yaw_pos_err, yaw_vel_err, pitch_pos_err, pitch_vel_err});
        sum_sq += yaw_pos_err * yaw_pos_err + yaw_vel_err * yaw_vel_err;
        sum_sq += pitch_pos_err * pitch_pos_err + pitch_vel_err * pitch_vel_err;
    }

    // 遍历控制输入时域（N-1个输入）
    for (int k = 0; k < kHorizon - 1; ++k) {
        const double yaw_input_err = std::abs(
            yaw->solution->u(0, k)
            - static_cast<double>(dual.solver.input(DualSmallMpcSolver::kYawAxis, k)));
        const double pitch_input_err = std::abs(
            pitch->solution->u(0, k)
            - static_cast<double>(dual.solver.input(DualSmallMpcSolver::kPitchAxis, k)));

        max_input = std::max({max_input, yaw_input_err, pitch_input_err});
    }

    return {
        max_state,
        std::sqrt(sum_sq / static_cast<double>(4 * kHorizon)),
        max_input,
        yaw_status == 0 && pitch_status == 0 && yaw->solution->solved == 1
            && pitch->solution->solved == 1,
        dual_ok,
    };
}

/**
 * @brief 打印分段标题分割线
 */
static void print_section(const char* title) {
    std::cout << "\n" << std::string(78, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(78, '=') << "\n";
}

// 程序入口主函数
int main() {
    // 无缓冲立即输出，基准打印实时刷新
    std::cout << std::unitbuf;

    // 预生成全部测试参考轨迹
    const auto yaw_ref_sin     = ref_sinusoid(kHorizon, 1.0, 0.3);
    const auto pitch_ref_sin   = ref_sinusoid(kHorizon, 0.8, 0.2);
    const auto yaw_ref_chirp   = ref_chirp(kHorizon, 0.3);
    const auto pitch_ref_noisy = ref_noisy_sinusoid(kHorizon, 42, 0.2);
    const auto yaw_ref_zero    = ref_zero(kHorizon);
    const auto pitch_ref_zero  = ref_zero(kHorizon);
    const auto yaw_ref_step    = ref_step(kHorizon, 0.3);
    const auto pitch_ref_step  = ref_step(kHorizon, 0.6);
    const auto yaw_ref_ramp    = ref_ramp(kHorizon, 0.0, 0.8);
    const auto pitch_ref_bound = ref_step(kHorizon, 0.68);

    print_section("Dual-Axis MPC Benchmark");
    std::cout << "  Horizon: " << kHorizon << " (ahead=" << kHorizonAhead
              << " back=" << kHorizonBack << ")\n";
    std::cout << "  dt=" << kDt << " rho=" << kRho << " tol=" << kAbsTol << "\n";
    std::cout << "  Trials=" << kTrials << " Warmup=" << kWarmup << "\n";

    // 第一部分：仅求解运算耗时对比
    {
        print_section("Solve-Only Timing");

        TinyPairSetup tiny;
        DualSetup dual;

        const auto tiny_t = bench_tiny_pair_solve(tiny, yaw_ref_sin, pitch_ref_sin, 10, kTrials);
        const auto dual_t = bench_dual_solve(dual, yaw_ref_sin, pitch_ref_sin, 10, kTrials);

        print_stats("2x TinyMPC", tiny_t.stats, kTrials);
        print_stats("DualSmallMpcSolver", dual_t.stats, kTrials);

        std::cout << "  tiny_max_iter_seen=" << tiny_t.iterations
                  << " dual_max_iter_seen=" << dual_t.iterations << "\n";

        std::cout << "  speedup=" << std::fixed << std::setprecision(2)
                  << tiny_t.stats.p50 / std::max(dual_t.stats.p50, 0.001) << "x\n";
    }

    // 第二部分：完整业务流水线全链路耗时对比
    {
        print_section("Full Pipeline Timing");

        TinyPairSetup tiny;
        DualSetup dual;

        const auto tiny_t = bench_tiny_pair_pipeline(tiny, yaw_ref_sin, pitch_ref_sin, 10, kTrials);
        const auto dual_t = bench_dual_pipeline(dual, yaw_ref_sin, pitch_ref_sin, 10, kTrials);

        print_stats("2x TinyMPC", tiny_t, kTrials);
        print_stats("DualSmallMpcSolver", dual_t, kTrials);

        std::cout << "  speedup=" << std::fixed << std::setprecision(2)
                  << tiny_t.p50 / std::max(dual_t.p50, 0.001) << "x\n";
    }

    // 多场景测试用例定义
    struct Scenario {
        const char* name;
        const Eigen::MatrixXd& yaw_ref;
        const Eigen::MatrixXd& pitch_ref;
    };

    const Scenario scenarios[] = {
        {    "sin/sin",   yaw_ref_sin,   pitch_ref_sin},
        {"chirp/noisy", yaw_ref_chirp, pitch_ref_noisy},
        {  "zero/zero",  yaw_ref_zero,  pitch_ref_zero},
        { "step/bound",  yaw_ref_step, pitch_ref_bound},
        {  "ramp/step",  yaw_ref_ramp,  pitch_ref_step},
    };

    // 第三部分：多参考场景中位数耗时遍历对比
    {
        print_section("Scenario Timing Sweep");

        std::cout << "  " << std::left << std::setw(16) << "Scenario" << std::setw(14) << "Tiny p50"
                  << std::setw(14) << "Dual p50" << std::setw(10) << "Speedup"
                  << "\n";
        std::cout << "  " << std::string(54, '-') << "\n";

        for (const auto& scenario : scenarios) {
            TinyPairSetup tiny;
            DualSetup dual;

            const auto tiny_t =
                bench_tiny_pair_solve(tiny, scenario.yaw_ref, scenario.pitch_ref, 10, kTrials);
            const auto dual_t =
                bench_dual_solve(dual, scenario.yaw_ref, scenario.pitch_ref, 10, kTrials);

            std::cout << "  " << std::left << std::setw(16) << scenario.name << std::fixed
                      << std::setprecision(2) << std::setw(14) << tiny_t.stats.p50 << std::setw(14)
                      << dual_t.stats.p50 << std::setw(10)
                      << tiny_t.stats.p50 / std::max(dual_t.stats.p50, 0.001) << "\n";
        }
    }

    // 第四部分：核心正确性校验——中心输出误差对比（产线验收标准）
    {
        print_section("Center Output Consistency");

        bool all_pass = true;

        print_center_output_summary_header();

        for (const auto& scenario : scenarios) {
            const auto report = compare_center_output(scenario.yaw_ref, scenario.pitch_ref, 100);

            print_center_output_summary_row(scenario.name, report);
            all_pass = all_pass && report.pass;
        }

        std::cout << "  overall_center_output_consistency=" << (all_pass ? "PASS" : "FAIL") << "\n";

        std::cout << "\n";
        const auto detailed = compare_center_output(yaw_ref_sin, pitch_ref_noisy, 100);
        print_center_output_report("detailed sin/noisy", detailed);
    }

    // 第五部分：全时域完整轨迹调试误差（仅开发调试）
    {
        print_section("Full Horizon Debug Accuracy");

        const auto debug = compare_full_horizon_debug(yaw_ref_sin, pitch_ref_noisy, 100);

        std::cout << "  tiny_ok=" << (debug.tiny_converged ? "Y" : "N")
                  << " dual_ok=" << (debug.dual_converged ? "Y" : "N") << "\n";
        std::cout << "  max_state_err=" << std::scientific << std::setprecision(4)
                  << debug.max_abs_state_err << " rmse_state=" << debug.rmse_state
                  << " max_input_err=" << debug.max_abs_input_err << "\n";

        std::cout << "  note: full horizon errors are debug-only. "
                  << "The pass/fail correctness metric is Center Output Consistency.\n";
    }

    return 0;
}