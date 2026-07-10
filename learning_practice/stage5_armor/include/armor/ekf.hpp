// ===========================================================================
// ekf.hpp - 扩展卡尔曼滤波器（装甲板跟踪）
//
// 状态向量 x = [x, y, z, vx, vy, vz, yaw, v_yaw]^T  (8 维)
//   - 位置 (x, y, z) 米
//   - 速度 (vx, vy, vz) 米/秒
//   - 偏航角 yaw 弧度
//   - 偏航角速度 v_yaw 弧度/秒
//
// 观测向量 z = [x, y, z, yaw]^T  (4 维)
//   - 直接观测位置和偏航角（来自 PnP）
//
// 状态转移函数 f(x)：
//   x' = x + vx*dt
//   y' = y + vy*dt
//   z' = z + vz*dt
//   vx' = vx
//   vy' = vy
//   vz' = vz
//   yaw' = yaw + v_yaw*dt
//   v_yaw' = v_yaw
//
// 雅可比 F = df/dx（恒定，因为 f 是线性函数）：
//   [1 0 0 dt 0 0  0   0 ]
//   [0 1 0 0  dt 0  0   0 ]
//   [0 0 1 0  0  dt 0   0 ]
//   [0 0 0 1  0  0  0   0 ]
//   [0 0 0 0  1  0  0   0 ]
//   [0 0 0 0  0  1  0   0 ]
//   [0 0 0 0  0  0  1   dt]
//   [0 0 0 0  0  0  0   1 ]
//
// 观测函数 h(x)：
//   z = [x, y, z, yaw]^T
//
// 雅可比 H = dh/dx（4x8 常数矩阵）：
//   [1 0 0 0 0 0 0 0]
//   [0 1 0 0 0 0 0 0]
//   [0 0 1 0 0 0 0 0]
//   [0 0 0 0 0 0 1 0]
// ===========================================================================
#pragma once

#include "armor/matrix.hpp"

#include <cmath>

namespace armor {

constexpr std::size_t kStateDim = 8;    // 状态维度
constexpr std::size_t kMeasureDim = 4;  // 观测维度

using StateVector = Vector<kStateDim>;
using MeasureVector = Vector<kMeasureDim>;
using StateCov = Matrix<kStateDim, kStateDim>;
using ProcessNoiseCov = Matrix<kStateDim, kStateDim>;
using MeasureNoiseCov = Matrix<kMeasureDim, kMeasureDim>;

// 观测数据：来自检测器
struct Observation {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    bool valid = false;  // 是否有效（检测到目标）
};

// ===========================================================================
// EKF 跟踪器
// ===========================================================================
class EkfTracker {
public:
    EkfTracker() {
        // 初始化状态协方差 P（对角线，表示初始不确定性）
        P_ = StateCov::zeros();
        for (std::size_t i = 0; i < kStateDim; ++i) {
            // 位置不确定性 1 米，速度不确定性 10 m/s，
            // yaw 不确定性 1 rad，v_yaw 不确定性 10 rad/s
            double init_var = (i < 3) ? 1.0 : (i < 6) ? 10.0 : (i == 6) ? 1.0 : 10.0;
            P_(i, i) = init_var;
        }

        // 过程噪声协方差 Q（白噪声驱动模型）
        // 较大值 → 更相信观测；较小值 → 更相信预测
        Q_ = StateCov::zeros();
        for (std::size_t i = 0; i < 3; ++i) {
            Q_(i, i) = 0.01;  // 位置噪声 0.1 m
        }
        for (std::size_t i = 3; i < 6; ++i) {
            Q_(i, i) = 0.1;   // 速度噪声 0.3 m/s
        }
        Q_(6, 6) = 0.01;     // yaw 噪声 0.1 rad
        Q_(7, 7) = 0.1;      // v_yaw 噪声 0.3 rad/s

        // 观测噪声协方差 R（来自检测器的不确定性）
        R_ = MeasureNoiseCov::zeros();
        R_(0, 0) = 0.05;  // x 噪声
        R_(1, 1) = 0.05;  // y 噪声
        R_(2, 2) = 0.05;  // z 噪声
        R_(3, 3) = 0.02;  // yaw 噪声
    }

    // 初始化状态（首次观测）
    void initialize(const Observation& obs) {
        x_ = StateVector::zeros();
        x_(0, 0) = obs.x;
        x_(1, 0) = obs.y;
        x_(2, 0) = obs.z;
        x_(6, 0) = obs.yaw;
        // 速度初始为 0
    }

    // 预测步骤：根据状态转移模型外推
    // 输入：dt 时间间隔（秒）
    void predict(double dt) {
        // 1. 状态转移：x' = f(x)
        //    位置 += 速度 * dt
        x_(0, 0) += x_(3, 0) * dt;
        x_(1, 0) += x_(4, 0) * dt;
        x_(2, 0) += x_(5, 0) * dt;
        //    yaw += v_yaw * dt
        x_(6, 0) += x_(7, 0) * dt;
        //    速度、v_yaw 保持不变

        // 2. 计算雅可比 F（线性模型，F 是常数矩阵）
        StateMatrix F = StateMatrix::identity();
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;
        F(6, 7) = dt;

        // 3. 协方差预测：P' = F * P * F^T + Q
        P_ = F * P_ * F.transpose() + Q_;
    }

    // 更新步骤：融合新观测
    void update(const Observation& obs) {
        if (!obs.valid) {
            return;  // 无有效观测，跳过更新
        }

        // 1. 构造观测向量 z
        MeasureVector z;
        z(0, 0) = obs.x;
        z(1, 0) = obs.y;
        z(2, 0) = obs.z;
        z(3, 0) = obs.yaw;

        // 2. 观测函数 h(x)：直接取状态的位置和 yaw
        MeasureVector h_x = MeasureVector::zeros();
        h_x(0, 0) = x_(0, 0);
        h_x(1, 0) = x_(1, 0);
        h_x(2, 0) = x_(2, 0);
        h_x(3, 0) = x_(6, 0);

        // 3. 雅可比 H（观测矩阵，4x8：从 8 维状态取 4 维观测）
        Matrix<kMeasureDim, kStateDim> H = Matrix<kMeasureDim, kStateDim>::zeros();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;
        H(3, 6) = 1.0;

        // 4. 新息（残差）y = z - h(x)
        MeasureVector y = z - h_x;

        // 5. 处理 yaw 角度回绕（-π 到 π）
        //    如果差值超过 π，调整到 ±π 范围
        while (y(3, 0) > M_PI) y(3, 0) -= 2.0 * M_PI;
        while (y(3, 0) < -M_PI) y(3, 0) += 2.0 * M_PI;

        // 6. 新息协方差 S = H * P * H^T + R
        MeasureMatrix S = H * P_ * H.transpose() + R_;

        // 7. 卡尔曼增益 K = P * H^T * S^(-1)
        MeasureMatrix S_inv = S.inverse();
        Matrix<kStateDim, kMeasureDim> K = P_ * H.transpose() * S_inv;

        // 8. 状态更新：x' = x + K * y
        x_ = x_ + K * y;

        // 9. 协方差更新：P' = (I - K * H) * P
        StateMatrix I = StateMatrix::identity();
        P_ = (I - K * H) * P_;
    }

    // 获取当前状态估计
    [[nodiscard]] const StateVector& state() const noexcept { return x_; }

    // 获取位置预测
    [[nodiscard]] Point3D position() const noexcept {
        return {x_(0, 0), x_(1, 0), x_(2, 0)};
    }

    // 获取速度估计
    [[nodiscard]] Point3D velocity() const noexcept {
        return {x_(3, 0), x_(4, 0), x_(5, 0)};
    }

    // 获取 yaw 估计
    [[nodiscard]] double yaw() const noexcept { return x_(6, 0); }

    // 获取 yaw 角速度
    [[nodiscard]] double yaw_rate() const noexcept { return x_(7, 0); }

private:
    using StateMatrix = Matrix<kStateDim, kStateDim>;
    using MeasureMatrix = Matrix<kMeasureDim, kMeasureDim>;

    StateVector x_ = StateVector::zeros();     // 状态估计
    StateCov P_;                                // 状态协方差
    ProcessNoiseCov Q_;                         // 过程噪声协方差
    MeasureNoiseCov R_;                         // 观测噪声协方差
};

}  // namespace armor
