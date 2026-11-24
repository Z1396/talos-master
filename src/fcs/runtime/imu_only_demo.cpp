/**
 * @file imu_only_demo.cpp
 * @brief ImuOnly 调试模式的正弦云台指令源实现
 */

#include "runtime/imu_only_demo.hpp"

#include "L5_weapon/fire_control.hpp"
#include "core/channel_topics.hpp"
#include "core/time.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <spdlog/spdlog.h>

namespace fcs::runtime {

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;

/// 波形模式
enum class DemoMode { Spin, Lateral, LateralSpin };

[[nodiscard]] DemoMode parse_mode(const std::string& mode) noexcept {
    if (mode == "spin") {
        return DemoMode::Spin;
    }
    if (mode == "lateral_spin") {
        return DemoMode::LateralSpin;
    }
    return DemoMode::Lateral; // 默认左右平移
}

/// 三角波：u∈[0,1) 时输出 -1 → +1 → -1
[[nodiscard]] double triangle_wave(double u) noexcept {
    return 1.0 - 4.0 * std::abs(u - 0.5);
}

} // namespace

void register_imu_only_demo_system(
    talos::Scheduler& scheduler, const hardware::ImuOnlyDemoConfig& cfg) {
    // 记录启动时刻，用作波形相位起点；shared_ptr 让 lambda 捕获后生命周期跟随系统
    auto start_ns = std::make_shared<uint64_t>(fcs::clock::now_ns());

    const DemoMode mode = parse_mode(cfg.mode);

    // 按模式取各自独立的参数
    double lateral_amp_deg    = 0.0;
    double lateral_period     = 1.0;
    double spin_amp_deg       = 0.0;
    double spin_period        = 1.0;
    const bool use_lateral    = mode != DemoMode::Spin;
    const bool use_spin       = mode != DemoMode::Lateral;
    if (mode == DemoMode::Spin) {
        spin_amp_deg = cfg.spin.amplitude_deg;
        spin_period  = cfg.spin.period_s;
    } else if (mode == DemoMode::Lateral) {
        lateral_amp_deg = cfg.lateral.amplitude_deg;
        lateral_period  = cfg.lateral.period_s;
    } else { // LateralSpin
        lateral_amp_deg = cfg.lateral_spin.lateral_amplitude_deg;
        lateral_period  = cfg.lateral_spin.lateral_period_s;
        spin_amp_deg    = cfg.lateral_spin.spin_amplitude_deg;
        spin_period     = cfg.lateral_spin.spin_period_s;
    }

    // 预计算波形参数，避免每帧重复换算
    const double lateral_amp = lateral_amp_deg * kDegToRad;
    const double spin_amp    = spin_amp_deg * kDegToRad;
    const double pitch_amp   = cfg.pitch_amplitude_deg * kDegToRad;
    const double pitch_omega = 2.0 * std::numbers::pi * cfg.pitch_frequency_hz;
    lateral_period           = std::max(0.1, lateral_period);
    spin_period              = std::max(0.05, spin_period);

    SPDLOG_INFO(
        "imu-only demo: mode={} lateral {:.1f}deg/{:.2f}s spin {:.2f}deg/{:.2f}s "
        "pitch {:.1f}deg/{:.2f}Hz",
        cfg.mode, lateral_amp_deg, lateral_period, spin_amp_deg, spin_period,
        cfg.pitch_amplitude_deg, cfg.pitch_frequency_hz);

    // 200Hz 生成指令，L1 的 weapon_output 以 250Hz 读取最新值下发
    scheduler.add_system<talos::fixed_rate<200>>(
        "imu_only_demo",
        [start_ns, use_lateral, use_spin, lateral_amp, lateral_period, spin_amp, spin_period,
         pitch_amp, pitch_omega, distance = cfg.distance](
            talos::spmc_mut<L5::WeaponCommand, WeaponCommandChannelTopic> cmd_out) {
            const uint64_t now_ns = fcs::clock::now_ns();
            const double t        = static_cast<double>(now_ns - *start_ns) * 1e-9;

            L5::WeaponCommand cmd;
            cmd.timestamp_ns      = now_ns;
            cmd.plan_timestamp_ns = now_ns;

            double yaw     = 0.0;
            double yaw_vel = 0.0;

            // 左右平移：三角波，速度恒定为 ±4A/T
            if (use_lateral) {
                const double u  = t / lateral_period;
                const double uf = u - std::floor(u);
                yaw += lateral_amp * triangle_wave(uf);
                yaw_vel += lateral_amp * ((uf < 0.5 ? 4.0 : -4.0) / lateral_period);
            }
            // 小陀螺：锯齿波（缓变 + 回跳），速度恒定为 -2A/T
            if (use_spin) {
                const double u  = t / spin_period;
                const double uf = u - std::floor(u);
                yaw += spin_amp * (1.0 - 2.0 * uf);
                yaw_vel += spin_amp * (-2.0 / spin_period);
            }

            cmd.yaw       = yaw;
            cmd.yaw_vel   = yaw_vel;
            cmd.pitch     = pitch_amp * std::sin(pitch_omega * t);
            cmd.pitch_vel = pitch_amp * pitch_omega * std::cos(pitch_omega * t);
            cmd.vel_valid = true;

            // 调试响应度：只动云台，不开火
            cmd.fire = false;

            // 调试字段填充，便于 Foxglove 对照
            cmd.distance      = distance;
            cmd.plan_yaw      = cmd.yaw;
            cmd.plan_pitch    = cmd.pitch;
            cmd.plan_distance = distance;
            cmd.ref_yaw       = cmd.yaw;
            cmd.ref_pitch     = cmd.pitch;

            cmd_out.write(cmd);
        });
}

} // namespace fcs::runtime
