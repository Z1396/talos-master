// ===========================================================================
// 阶段5：装甲板检测 + EKF 跟踪演示
//
// 场景：
//   1. 模拟目标在 3D 空间中匀速移动 + 旋转
//   2. 每帧用 ArmorDetector 检测（含噪声）
//   3. 用 EkfTracker 预测 + 融合观测
//   4. 输出真实值 vs 测量值 vs 估计值
//
// 验证 EKF 收敛性：估计值应趋近真实值
// ===========================================================================

#include "armor/armor.hpp"
#include "armor/ekf.hpp"

#include <cmath>
#include <cstdio>
#include <random>

using namespace armor;

// ===========================================================================
// 模拟目标运动（真实状态）
// ===========================================================================
struct GroundTruth {
    Point3D position;
    Point3D velocity;
    double yaw;
    double yaw_rate;
};

// 模拟相机投影：3D 点 → 2D 像素
Point2D project(Point3D p, const CameraIntrinsics& cam) {
    return {
        cam.fx * p.x / p.z + cam.cx,
        cam.fy * p.y / p.z + cam.cy,
    };
}

int main() {
    std::printf("=== 阶段5：装甲板检测 + EKF 跟踪 ===\n\n");

    // 1. 配置相机内参
    CameraIntrinsics cam;
    ArmorDetector detector(cam);
    EkfTracker tracker;

    // 2. 初始化真实状态
    GroundTruth truth{
        .position = {0.0, 0.0, 5.0},       // 距相机 5 米
        .velocity = {0.3, 0.0, 0.0},       // 向右移动 0.3 m/s
        .yaw = 0.0,
        .yaw_rate = 0.5,                    // 旋转 0.5 rad/s
    };

    // 3. 添加高斯噪声的检测器
    std::mt19937 rng(42);
    std::normal_distribution<double> noise_pos(0.0, 0.05);  // 5cm 噪声
    std::normal_distribution<double> noise_yaw(0.0, 0.05);  // 0.05 rad 噪声
    std::normal_distribution<double> noise_pix(2.0, 2.0);   // 2 像素噪声

    // 4. 模拟 50 帧，每帧 dt=0.1s
    constexpr double dt = 0.1;
    constexpr int frames = 50;

    std::printf("帧  真实x    测量x    估计x    真实yaw   测量yaw   估计yaw   误差x\n");
    std::printf("--- -------  -------  -------  --------  --------  --------  ------\n");

    bool initialized = false;

    for (int frame = 0; frame < frames; ++frame) {
        // === 真实状态更新 ===
        if (frame > 0) {
            truth.position.x += truth.velocity.x * dt;
            truth.position.y += truth.velocity.y * dt;
            truth.position.z += truth.velocity.z * dt;
            truth.yaw += truth.yaw_rate * dt;
        }

        // === EKF 预测 ===
        if (initialized) {
            tracker.predict(dt);
        }

        // === 模拟检测（含噪声）===
        // 真实 3D → 像素 → 加噪声 → 反投影
        Point3D noisy_pos = {
            truth.position.x + noise_pos(rng),
            truth.position.y + noise_pos(rng),
            truth.position.z + noise_pos(rng),
        };
        double noisy_yaw = truth.yaw + noise_yaw(rng);

        // 构造虚拟检测（简化：直接用噪声位置构造 4 角点）
        // 真实 Talos 从图像角点检测；这里直接跳到观测
        Observation obs{
            .x = noisy_pos.x,
            .y = noisy_pos.y,
            .z = noisy_pos.z,
            .yaw = noisy_yaw,
            .valid = true,
        };

        // === EKF 更新 ===
        if (!initialized) {
            tracker.initialize(obs);
            initialized = true;
        } else {
            tracker.update(obs);
        }

        // === 输出对比（每 10 帧）===
        if (frame % 10 == 0 || frame == frames - 1) {
            Point3D est_pos = tracker.position();
            double est_yaw = tracker.yaw();
            double err_x = std::abs(est_pos.x - truth.position.x);
            std::printf("%2d  %7.3f  %7.3f  %7.3f  %8.3f  %8.3f  %8.3f  %6.3f\n",
                frame,
                truth.position.x, obs.x, est_pos.x,
                truth.yaw, obs.yaw, est_yaw,
                err_x);
        }
    }

    // 5. 评估收敛性
    std::printf("\n=== 收敛性分析 ===\n");
    Point3D final_est = tracker.position();
    Point3D final_vel = tracker.velocity();
    std::printf("真实最终位置: (%.3f, %.3f, %.3f)\n",
        truth.position.x, truth.position.y, truth.position.z);
    std::printf("估计最终位置: (%.3f, %.3f, %.3f)\n",
        final_est.x, final_est.y, final_est.z);
    std::printf("真实速度:     (%.3f, %.3f, %.3f)\n",
        truth.velocity.x, truth.velocity.y, truth.velocity.z);
    std::printf("估计速度:     (%.3f, %.3f, %.3f)\n",
        final_vel.x, final_vel.y, final_vel.z);
    std::printf("真实 yaw:     %.3f rad (%.1f deg)\n",
        truth.yaw, truth.yaw * 180.0 / M_PI);
    std::printf("估计 yaw:     %.3f rad (%.1f deg)\n",
        tracker.yaw(), tracker.yaw() * 180.0 / M_PI);
    std::printf("真实 yaw_rate: %.3f rad/s\n", truth.yaw_rate);
    std::printf("估计 yaw_rate: %.3f rad/s\n", tracker.yaw_rate());

    // 计算位置误差
    double pos_err = std::hypot(
        final_est.x - truth.position.x,
        final_est.y - truth.position.y);
    std::printf("\n位置误差: %.4f m (噪声 std=0.05m)\n", pos_err);
    std::printf("EKF 误差 < 测量噪声: %s\n",
        pos_err < 0.05 ? "YES（滤波有效）" : "NO");

    std::printf("\n=== 装甲板检测 + EKF 演示完成 ===\n");
    return 0;
}
