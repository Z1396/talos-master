// ===========================================================================
// demo.cpp - 旋转机器人目标跟踪仿真（使用真实项目 EKF + 运动模型）
//
// 本演示的每一行逻辑都对照真实项目源码：
//   1. EKF 本体     = 真实 extended_kalman_filter.hpp（逐字复制，零改动）
//   2. 运动模型     = 真实 new_motion_model.hpp（逐字复制，仅改 include）
//   3. EKF 初始化    = 照搬 new_tracker.hpp first_meet() L484-521（x0/P0 构造）
//   4. f/h/Q/R 回调  = 照搬 new_tracker.hpp EkfTargetInfo::initialize() L143-186
//   5. 预测更新流程  = 照搬 EkfTargetInfo::predict()/update() L195-247
//
// 仿真场景（对应真实跟踪对象）：
//   敌方机器人底盘中心 (xc,yc) 匀速移动，同时车体以 v_yaw 匀速旋转，
//   4 块装甲板分布在半径 r0/r1 的圆周上。相机在原点，
//   每帧观测 0 号装甲板，得到球坐标观测 [yaw, pitch, log_d, armor_yaw]。
//
// 阅读建议：看完本文件再去看 new_tracker.hpp，会发现流程完全一样。
// ===========================================================================

// ---------- 头文件包含 ----------
#include "armor/ekf.hpp"          // 引入真实的 EKF 类模板（带自动微分、收敛监控、Joseph更新）
#include "armor/motion_model.hpp" // 引入真实的运动模型（机器人11维状态模型）

#include <Eigen/Dense>            // Eigen 线性代数库（矩阵运算）
#include <ceres/jet.h>            // Ceres 的 Jet 类型（自动微分核心）

#include <cmath>                  // 数学函数（sin, cos, log, exp 等）
#include <iomanip>                // 格式化输出（std::setw, std::setprecision）
#include <iostream>               // 标准输入输出（std::cout）
#include <numbers>                // 数学常量（std::numbers::pi）
#include <random>                 // 随机数生成（模拟传感器噪声）

namespace {  // 匿名命名空间：这里面的内容只在这个文件内可见，防止污染其他文件

// ---------- 引入运动模型中的状态/观测索引常量 ----------
// 这些是枚举或者 constexpr 变量，告诉我们状态向量的第几位代表什么物理意义
using fcs::L3::ARMOR_DISTANCE;    // 观测索引：距离（对数空间）
using fcs::L3::ARMOR_PITCH;       // 观测索引：俯仰角
using fcs::L3::ARMOR_YAW;         // 观测索引：水平方位角
using fcs::L3::ARMOR_YAW_ARMOR;   // 观测索引：装甲自身朝向角
using fcs::L3::H;                 // 状态索引：高低装甲高度差
using fcs::L3::LOG_R0;            // 状态索引：平地装甲半径的对数
using fcs::L3::LOG_R1;            // 状态索引：高地装甲半径的对数
using fcs::L3::RobotEkfMotionModel; // 运动模型类型
using fcs::L3::V_YAW;             // 状态索引：机体偏航角速度
using fcs::L3::VX;                // 状态索引：X方向速度
using fcs::L3::VY;                // 状态索引：Y方向速度
using fcs::L3::VZ;                // 状态索引：Z方向速度
using fcs::L3::XC;                // 状态索引：底盘中心X坐标
using fcs::L3::YAW;               // 状态索引：机体偏航角
using fcs::L3::YC;                // 状态索引：底盘中心Y坐标
using fcs::L3::Z0;                // 状态索引：低装甲Z坐标

// ---------- 定义编译期常量 ----------
constexpr int NX = RobotEkfMotionModel::NX;            // 状态维度：11（位置、速度、角度等）
constexpr int NZ = RobotEkfMotionModel::NZ;            // 观测维度：4（yaw, pitch, log_d, armor_yaw）

// ---------- 定义类型别名 ----------
using VecX  = RobotEkfMotionModel::VecX;  // 11维状态向量
using VecZ  = RobotEkfMotionModel::VecZ;  // 4维观测向量
using MatXX = RobotEkfMotionModel::MatXX; // 11x11 状态协方差矩阵
using MatZZ = RobotEkfMotionModel::MatZZ; // 4x4 观测协方差矩阵
using JetX  = ceres::Jet<double, NX>;     // 自动微分包装类型：包含状态值 + 导数
using EKF   = ExtendedKalmanFilter<
      NX, NZ, std::function<void(const JetX*, JetX*)>, // 预测函数类型签名
      std::function<void(const JetX*, JetX*)>>;        // 测量函数类型签名

/// 把 EKF 内部的收敛状态枚举转成字符串，方便打印到控制台
const char* convergence_name(EKF::ConvergenceStatus s) noexcept {
    switch (s) {
    case EKF::ConvergenceStatus::Unknown:    return "Unknown";    // 未知
    case EKF::ConvergenceStatus::Converging: return "Converging"; // 收敛中
    case EKF::ConvergenceStatus::Converged:  return "Converged";  // 已收敛
    case EKF::ConvergenceStatus::Diverging:  return "Diverging";  // 发散
    }
    return "?"; // 理论上不会走到这里
}

} // namespace

// ---------- 主函数 ----------
int main() {
    std::cout << "=== 旋转机器人 EKF 跟踪仿真（真实项目模型照搬版）===\n\n";

    // ------------------------------------------------------------------
    // 0. 真值初始化：上帝视角，真实机器人是怎么动的（用于和估计值做对比）
    // ------------------------------------------------------------------
    const double true_r0   = 0.20;   // 0、2号装甲的旋转半径（真值 0.2米）
    const double true_r1   = 0.20;   // 1、3号高地装甲的旋转半径（真值 0.2米）
    const double true_h    = 0.10;   // 高地和平地装甲的高度差（真值 0.1米）
    const double true_vyaw = 2.0;    // 车体旋转角速度（真值 2 rad/s，很快）

    VecX x_true;  // 11维的真值状态向量
    // 手动填充：底盘中心 (2, -1)，X/Y速度 (0.3, 0.1)，Z高度 0.3
    // 初始朝向 0.5 rad，角速度 2 rad/s，半径取对数，高度差 0.1
    x_true << 2.0, 0.3,              // xc=2m, vx=0.3m/s（向右移动）
        -1.0, 0.1,                   // yc=-1m, vy=0.1m/s
        0.30, 0.0,                   // z0=0.3m, vz=0
        0.5, true_vyaw, std::log(true_r0), std::log(true_r1), true_h;

    const RobotEkfMotionModel model; // 创建真实运动模型实例（包含默认噪声参数）

    // ------------------------------------------------------------------
    // 1. 构造 EKF 的 f/h/Q/R 回调 —— 把数学公式传给滤波器
    // ------------------------------------------------------------------
    double dt_    = 0.0; // 当前帧的时间间隔（真实代码中由外部传入）
    int armor_id_ = 0;   // 当前观测的是哪块装甲（0号装甲）

    // 预测函数 f：将 Jet 数组传给模型的静态预测函数
    // 使用 Jet 类型是为了让模型计算时自动得出雅可比矩阵
    auto f = [&dt_](const JetX* x, JetX* xp) {
        RobotEkfMotionModel::predict_state<JetX>(x, JetX(dt_), xp);
    };

    // 观测函数 h：根据状态和装甲ID，推算相机应该看到的观测值
    // 同样使用 Jet 类型自动获得雅可比矩阵 H
    auto h = [&armor_id_](const JetX* x, JetX* z) {
        RobotEkfMotionModel::measure_state<JetX>(x, armor_id_, z);
    };

    // 过程噪声 Q 回调：告诉滤波器“我的预测有多不靠谱”
    // 特例：因为状态里用了对数半径，所以噪声方差也要除以半径平方
    auto q = [&dt_, &model]() -> MatXX {
        auto Q              = model.Q(dt_);        // 调用模型的 Q 生成函数
        const double est_r0 = 0.25;                 // 估算的半径（近似值）
        const double est_r1 = 0.25;
        Q(LOG_R0, LOG_R0) /= (est_r0 * est_r0);     // 数学恒等式：Var(log r) = Var(r) / r²
        Q(LOG_R1, LOG_R1) /= (est_r1 * est_r1);
        return Q;
    };

    // 测量噪声 R 回调：告诉滤波器“相机看东西有多不准”
    // 注意：R 会随着距离和观测角度的变化而动态变化
    auto r = [&model](const VecZ& z) -> MatZZ { return model.R(z); };

    // ------------------------------------------------------------------
    // 2. 首次见到目标：初始化 x0/P0
    // ------------------------------------------------------------------
    // 用真值在模型中的投影，模拟真实首帧检测到的观测值（通常是相机 PnP 算出来的）
    const VecZ armor0_true = model.h(x_true, 0);   // 计算真值下0号装甲的观测
    const double armor_yaw = armor0_true[ARMOR_YAW_ARMOR]; // 提取装甲自身朝向角

    // 只用装甲位置反推底盘中心的初始位置
    const double r_prior  = 0.5 * (true_r0 + true_r1);    // 先验半径（用两个半径平均值）
    const double h_prior  = true_h;                       // 先验高度差
    const double init_yaw = armor_yaw + std::numbers::pi; // 装甲朝向 + π = 指向底盘中心的反方向

    // 初始状态 x0：位置能推算，但速度、角速度全部置为 0（假设未知）
    VecX x0;
    x0 << 0.0, 0.0, 0.0, 0.0, 0.30, 0.0, init_yaw, 0.0, std::log(r_prior), std::log(r_prior),
        h_prior;
    
    // 核心几何反解：将相机看到的装甲球坐标转回世界坐标系下的底盘中心位置
    {
        const double dist  = std::exp(armor0_true[ARMOR_DISTANCE]); // 反对数得到真实距离
        const double ayaw  = armor0_true[ARMOR_YAW];                // 水平角
        const double pitch = armor0_true[ARMOR_PITCH];              // 俯仰角
        const double az    = -dist * std::sin(pitch);               // 计算 Z 坐标
        const double aho   = dist * std::cos(pitch);                // 计算水平距离
        const double ax    = aho * std::cos(ayaw);                  // 计算 X 坐标
        const double ay    = aho * std::sin(ayaw);                  // 计算 Y 坐标
        x0[XC]             = ax + r_prior * std::cos(armor_yaw);    // 中心X = 装甲X + 半径*余弦
        x0[YC]             = ay + r_prior * std::sin(armor_yaw);    // 中心Y = 装甲Y + 半径*正弦
        x0[Z0]             = az - 0.5 * h_prior;                    // 低装甲Z = 装甲Z - 一半高度差
    }

    // 初始协方差 P0：告诉 EKF 一开始我的估计有多不确定
    MatXX P0           = MatXX::Identity();  // 先初始化为单位矩阵
    P0(XC, XC)         = 1;     // 位置方差 1（还算确定）
    P0(YC, YC)         = 1;
    P0(VX, VX)         = 64;    // 速度方差 64（完全不知道速度，所以给很大的不确定性）
    P0(VY, VY)         = 64;
    P0(VZ, VZ)         = 64;
    P0(YAW, YAW)       = 0.4;   // 角度方差 0.4（有一定把握）
    P0(V_YAW, V_YAW)   = 100;   // 角速度方差 100（初始以为角速度是0，其实很大）
    P0(LOG_R0, LOG_R0) = 1e-5 / (r_prior * r_prior); // 对数半径方差
    P0(LOG_R1, LOG_R1) = 1e-5 / (r_prior * r_prior);
    P0(H, H)           = 1;     // 高度差方差 1

    // 实例化并初始化 EKF 滤波器
    EKF ekf(f, h, q, r, P0);
    ekf.setState(x0);

    // ------------------------------------------------------------------
    // 3. 帧循环：模拟 200 帧的摄像头输入
    // ------------------------------------------------------------------
    constexpr double kDt   = 0.01; // 时间步长：100 FPS（每帧10毫秒）
    constexpr int kFrames  = 200;  // 总帧数
    constexpr int kArmorId = 0;    // 每次只看 0 号装甲

    // 模拟传感器噪声：给观测值加上高斯白噪声（模拟相机测量误差）
    std::mt19937 rng(42); // 固定种子（保证每次运行结果一致，方便调试）
    std::normal_distribution<double> n_yaw(0.0, 0.063);   // 水平角噪声
    std::normal_distribution<double> n_pitch(0.0, 0.063); // 俯仰角噪声
    std::normal_distribution<double> n_logd(0.0, 0.05);   // 对数距离噪声
    std::normal_distribution<double> n_ayaw(0.0, 0.30);   // 装甲朝向噪声

    // 打印表头
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "帧  收敛状态   | 真实xc 估计xc | 真实vyaw 估计vyaw | 真实r0 估计r0 |  NIS\n";
    std::cout << "------------------------------------------------------------------------\n";

    for (int frame = 1; frame <= kFrames; ++frame) {
        // ---- 上帝视角：把真值往前推一帧（让机器人跑起来）----
        x_true = model.f(x_true, kDt);

        // ---- 模拟相机：从真实状态生成观测值，并加上噪声 ----
        VecZ z_meas = model.h(x_true, kArmorId);
        z_meas[ARMOR_YAW] += n_yaw(rng);      // 加噪声
        z_meas[ARMOR_PITCH] += n_pitch(rng);  // 加噪声
        z_meas[ARMOR_DISTANCE] += n_logd(rng);// 加噪声
        z_meas[ARMOR_YAW_ARMOR] += n_ayaw(rng);// 加噪声

        // ---- 实时滤波：预测步 ----
        dt_ = kDt;          // 设置当前时间间隔
        ekf.predict();      // 先根据运动学模型往前推一步

        // ---- 实时滤波：更新步 ----
        armor_id_ = kArmorId;    // 设置观测的是哪块装甲
        (void)ekf.update(z_meas); // 融合测量值，修正状态
        ekf.decorrelate(LOG_R0, LOG_R1); // 人为取消 r0 和 r1 的数学关联（物理上它们独立）

        // ---- 每20帧打印一次结果 ----
        if (frame % 20 == 0 || frame == kFrames) {
            const VecX& x = ekf.X(); // 获取当前估计状态
            std::cout << std::setw(3) << frame << "  " << std::left << std::setw(10)
                      << convergence_name(ekf.convergence_status()) << std::right << "| "
                      << std::setw(6) << x_true[XC] << " " << std::setw(6) << x[XC] << " | "
                      << std::setw(8) << x_true[V_YAW] << " " << std::setw(8) << x[V_YAW] << " | "
                      << std::setw(6) << true_r0 << " " << std::setw(6) << std::exp(x[LOG_R0])
                      << " | " << std::setw(7) << ekf.normalized_innovation_squared() << "\n";
        }
    }

    // ------------------------------------------------------------------
    // 4. 结果分析
    // ------------------------------------------------------------------
    const VecX& x           = ekf.X(); // 获取最终估计状态
    const double center_err = std::hypot(x_true[XC] - x[XC], x_true[YC] - x[YC]); // 中心位置平面误差
    const double vyaw_err   = std::abs(x_true[V_YAW] - x[V_YAW]); // 角速度绝对误差

    std::cout << "\n=== 结果分析 ===\n";
    std::cout << "中心位置误差: " << center_err << " m\n";
    std::cout << "角速度误差:   " << vyaw_err << " rad/s\n";
    std::cout << "估计半径 r0:  " << std::exp(x[LOG_R0]) << " m (真值 " << true_r0 << ")\n";
    std::cout << "估计高度差 h: " << x[H] << " m (真值 " << true_h << ")\n";
    std::cout << "最终收敛状态: " << convergence_name(ekf.convergence_status()) << "\n";
    std::cout << "\n说明：\n";
    std::cout
        << "  - EKF 从零速度初值出发，靠观测自动收敛出 vx/vy/vyaw —— 这就是卡尔曼增益的作用\n";
    std::cout << "  - 半径/高度是缓变量，靠位置观测的几何关系缓慢修正\n";
    std::cout << "  - NIS 收敛到 2*NZ=8 附近说明噪声标定与实际匹配（卡方 95% 置信）\n";
    std::cout << "  - r1 未被观测（只看 0 号装甲），其估计保持先验值 —— 可观测性由观测几何决定\n";
    return 0;
}