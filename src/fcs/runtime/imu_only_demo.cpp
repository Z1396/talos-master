/**
 * @file imu_only_demo.cpp
 * @brief ImuOnly 调试模式的正弦云台指令源实现
 *
 * @details
 * 本文件是 ImuOnly 后端（backend = "ImuOnly"）下的“假目标”指令源：
 * 不接相机、不做识别/跟踪/预瞄，直接按配置生成 yaw/pitch 波形写入
 * WeaponCommand 通道，由 L1 的 weapon_output 系统按 250Hz 下发给 MCU，
 * 用于单独调试云台伺服的响应速度、方向和限位。
 *
 * 数据流：
 *   ImuOnlyDemoSystem（200Hz）
 *     → WeaponCommandChannelTopic（SPMC 三缓冲通道）
 *     → weapon_output（250Hz）
 *     → USB: SendVisionData / 串口: SendSimpleVisionData
 *     → MCU 云台
 *
 * 安全约束：
 * 1. fire 恒为 false，只动云台不出弹；
 * 2. yaw/pitch 输出始终钳制在配置的 ±limit 内；
 * 3. 启动时以 MCU 回传的当前姿态为摆动中心，避免从 0° 突然跳变；
 * 4. IMU 数据超过 200ms 未更新时自动停止摆动；
 * 5. 可选等待下位机能力帧 following=1 才启动（arm_by_mcu）。
 *
 * 波形模式：
 * - sine         : yaw、pitch 各自独立正弦，可设相位差（默认模式）
 * - spin         : 原地小陀螺，锯齿波
 * - lateral      : 左右平移，三角波
 * - lateral_spin : 三角波 + 锯齿波叠加，边走边转
 */

#include "runtime/imu_only_demo.hpp"

#include "L5_weapon/fire_control.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <spdlog/spdlog.h>

namespace fcs::runtime {

namespace {

/// 角度转弧度常数：π / 180
constexpr double kDegToRad = std::numbers::pi / 180.0;

/// IMU 数据超过该时长未更新即视为失效，停止摆动（200ms）
constexpr uint64_t kImuTimeoutNs = 200'000'000;

/**
 * @brief 波形模式枚举
 *
 * sine         : yaw/pitch 独立正弦，用于调试伺服响应
 * Spin         : 原地小陀螺（锯齿波）
 * Lateral      : 左右平移（三角波）
 * LateralSpin  : 三角波 + 锯齿波叠加
 */
enum class DemoMode { Sine, Spin, Lateral, LateralSpin };

/**
 * @brief 解析配置里的 mode 字符串
 * @param mode TOML 中的 mode 字段
 * @return 对应枚举；无法识别时退回 Sine
 *
 * 注意：字符串比较不区分前后缀，必须完全匹配；
 * 例如 "lateral_spin" 不能简写成 "lateral"。
 */
[[nodiscard]] DemoMode parse_mode(const std::string& mode) noexcept {
    if (mode == "spin") {
        return DemoMode::Spin;
    }
    if (mode == "lateral") {
        return DemoMode::Lateral;
    }
    if (mode == "lateral_spin") {
        return DemoMode::LateralSpin;
    }
    return DemoMode::Sine; // 默认 yaw/pitch 正弦
}

/**
 * @brief 三角波
 * @param u 归一化相位，取值 [0, 1)
 * @return u=0 时 -1，u=0.5 时 +1，u→1 时回到 -1
 *
 * 用于左右平移模式：云台以恒定角速度在两端之间往返。
 * 速度是分段常数 ±4A/T，在端点处符号突变。
 */
[[nodiscard]] double triangle_wave(double u) noexcept {
    return 1.0 - 4.0 * std::abs(u - 0.5);
}

/**
 * @brief 适配后的波形参数
 * center    : 实际摆动中心（弧度）
 * amplitude : 实际摆动幅度（弧度，非负）
 */
struct FittedWave {
    double center{0.0};
    double amplitude{0.0};
};

/**
 * @brief 把幅度收缩到限位内，并把中心钳制到 center±amp 不越界的位置
 * @param center 期望中心（弧度）
 * @param amplitude 期望幅度（弧度，内部取绝对值）
 * @param limit 单边限位（弧度）
 * @return 适配后的中心与幅度
 *
 * 例：limit=30°，amp=20°，center=25°
 *   → amp 保持 20°，center 被钳到 10°，输出范围 [-10°, +30°]，不会越界
 * 例：amp 本身大于 limit
 *   → amp 收缩到 limit，center 强制为 0
 *
 * 这是“即使配置写大了也不会顶死限位”的第一道保护；
 * 最终每帧输出还会再 clamp 一次，作为第二道保护。
 */
[[nodiscard]] FittedWave fit_wave(
    const double center, const double amplitude, const double limit) noexcept {
    const double lim = std::max(0.0, limit);
    const double amp = std::clamp(std::abs(amplitude), 0.0, lim);
    // 幅度等于限位时只能把中心放在 0，否则 center±amp 必有一侧越界
    const double c = (lim > amp) ? std::clamp(center, -lim + amp, lim - amp) : 0.0;
    return FittedWave{.center = c, .amplitude = amp};
}

} // namespace

/**
 * @brief 注册 ImuOnly 正弦云台指令系统
 * @param scheduler 调度器
 * @param cfg ImuOnly 调试配置（模式、幅度、频率、限位、是否由 MCU 使能）
 *
 * 注册后系统以 200Hz 运行：
 * - 未使能：把当前实测姿态写入指令通道，云台原地保持；
 * - 使能上升沿：锁存当前姿态为摆动中心，开始输出波形；
 * - 使能下降沿：停止波形，回到“保持当前姿态”；
 * - IMU 失效：自动失能并停止写入。
 */
void register_imu_only_demo_system(
    talos::Scheduler& scheduler, const hardware::ImuOnlyDemoConfig& cfg) {
    // 解析波形模式；未知字符串统一按 sine 处理
    const DemoMode mode = parse_mode(cfg.mode);

    // ===================== 1. 按模式取各自独立的参数 =====================
    // lateral / spin 只在对应模式下生效；sine 模式两者都为 0
    double lateral_amp_deg = 0.0;
    double lateral_period  = 1.0;
    double spin_amp_deg    = 0.0;
    double spin_period     = 1.0;
    // Sine 模式两个分量都不使用；LateralSpin 两个都用
    const bool use_lateral = mode != DemoMode::Spin;
    const bool use_spin    = mode != DemoMode::Lateral;
    if (mode == DemoMode::Spin) {
        spin_amp_deg = cfg.spin.amplitude_deg;   // 原地小陀螺幅度
        spin_period  = cfg.spin.period_s;        // 原地小陀螺周期
    } else if (mode == DemoMode::Lateral) {
        lateral_amp_deg = cfg.lateral.amplitude_deg; // 左右平移幅度
        lateral_period  = cfg.lateral.period_s;      // 左右平移周期
    } else if (mode == DemoMode::LateralSpin) {
        lateral_amp_deg = cfg.lateral_spin.lateral_amplitude_deg; // 左右平移幅度
        lateral_period  = cfg.lateral_spin.lateral_period_s;      // 左右平移周期
        spin_amp_deg    = cfg.lateral_spin.spin_amplitude_deg;    // 原地小陀螺幅度
        spin_period     = cfg.lateral_spin.spin_period_s;         // 原地小陀螺周期
    }

    // ===================== 2. 预计算波形参数 =====================
    // 角度统一转弧度，周期做下限保护，避免除零或频率过高
    const double lateral_amp = lateral_amp_deg * kDegToRad; // 左右平移幅度
    lateral_period           = std::max(0.1, lateral_period);
    const double spin_amp    = spin_amp_deg * kDegToRad; // 原地小陀螺幅度
    spin_period              = std::max(0.1, spin_period); // 原地小陀螺周期

    // sine 模式：yaw/pitch 各自独立正弦
    const double sine_yaw_amp    = cfg.yaw_amplitude_deg * kDegToRad;
    const double sine_yaw_omega  = 2.0 * std::numbers::pi * std::max(0.0, cfg.yaw_frequency_hz);
    const double sine_pitch_amp  = cfg.pitch_amplitude_deg * kDegToRad;
    const double sine_pitch_omega = 2.0 * std::numbers::pi * std::max(0.0, cfg.pitch_frequency_hz);
    // 相位差：0° = 同时上下左右摆；90° = 画圆/椭圆
    const double sine_phase      = cfg.phase_deg * kDegToRad;

    // ===================== 3. 安全限位 =====================
    // yaw / pitch 的单边限位，负值按 0 处理
    const double yaw_limit   = std::max(0.0, cfg.yaw_limit_deg) * kDegToRad;
    const double pitch_limit = std::max(0.0, cfg.pitch_limit_deg) * kDegToRad;

    // 该模式下 yaw 的最大幅度，用于启动时做“幅度收缩 + 中心钳制”
    // LateralSpin 是三角波与锯齿波叠加，所以幅度相加
    const double yaw_wave_amp =
        (mode == DemoMode::Sine) ? sine_yaw_amp : (lateral_amp * (use_lateral ? 1.0 : 0.0)
                                                   + spin_amp * (use_spin ? 1.0 : 0.0));
    // pitch 在所有模式下都只叠加一个正弦
    const double pitch_wave_amp =
        (mode == DemoMode::Sine) ? sine_pitch_amp : cfg.pitch_amplitude_deg * kDegToRad;

    // true：必须收到下位机能力帧且 following=1 才启动
    // false：收到 IMU 数据后立即启动，便于无 MCU 联调时单独测试
    const bool arm_by_mcu = cfg.arm_by_mcu;

    // 启动日志：把实际生效的参数打出来，方便确认 TOML 是否解析正确
    SPDLOG_INFO(
        "imu-only demo: mode={} yaw {:.1f}deg/{:.2f}Hz pitch {:.1f}deg/{:.2f}Hz "
        "phase {:.0f}deg limits yaw±{:.0f} pitch±{:.0f} arm_by_mcu={}",
        cfg.mode, cfg.yaw_amplitude_deg, cfg.yaw_frequency_hz, cfg.pitch_amplitude_deg,
        cfg.pitch_frequency_hz, cfg.phase_deg, cfg.yaw_limit_deg, cfg.pitch_limit_deg,
        arm_by_mcu);

    // ===================== 4. 摆动运行状态 =====================
    // 放在 shared_ptr 里，让 lambda 捕获后生命周期跟随系统；
    // 这些字段只在 imu_only_demo 系统内读写，无需加锁。
    struct ArmState {
        bool armed{false};        // 当前是否正在摆动
        double center_yaw{0.0};   // 锁存的 yaw 摆动中心（弧度）
        double center_pitch{0.0}; // 锁存的 pitch 摆动中心（弧度，指令空间）
        double amp_yaw{0.0};      // 适配限位后的 yaw 幅度（弧度）
        double amp_pitch{0.0};    // 适配限位后的 pitch 幅度（弧度）
        uint64_t start_ns{0};     // 本次开始摆动的时刻，用于计算相位
    };
    auto arm = std::make_shared<ArmState>();

    // ===================== 5. 注册 200Hz 系统 =====================
    // 200Hz 生成指令，L1 的 weapon_output 以 250Hz 读取最新值下发。
    // 生成频率略低于下发频率，保证 weapon_output 总能读到“最新的一帧”。
    scheduler.add_system<talos::fixed_rate<200>>(
        "imu_only_demo",
        [arm, mode, use_lateral, use_spin, lateral_amp, lateral_period, spin_amp, spin_period,
         sine_yaw_omega, sine_pitch_omega, sine_phase, yaw_limit, pitch_limit, yaw_wave_amp,
         pitch_wave_amp, arm_by_mcu, distance = cfg.distance](
            // 指令写通道：weapon_output 从这里取最新指令
            talos::spmc_mut<L5::WeaponCommand, WeaponCommandChannelTopic> cmd_out,
            // 下位机能力开关：following=1 表示操作手要求开始摆动（ImuOnly 下复用）
            core::following following,
            // 下位机回传的 IMU 姿态：提供当前 yaw/pitch，作为摆动中心
            core::imu_state imu) {
            const uint64_t now_ns = fcs::clock::now_ns();

            // ---------- 5.1 判断下位机数据是否可用 ----------
            // IMU 帧用于获取当前姿态；能力帧只需确认“曾经收到过”，
            // 避免固件能力帧发送频率较低时被误判为失效。
            // 链路断开时 IMU 会同时失效，由下面的 imu_fresh 负责兜底。
            const bool imu_fresh = imu->timestamp_ns != 0
                                   && now_ns - imu->timestamp_ns < kImuTimeoutNs;
            // 下位机能力帧是否可用
            const bool cap_seen  = following->last_update_ns() != 0;
            // 是否需要使能：下位机能力帧且 following=1 才使能
            const bool want_arm  = arm_by_mcu ? (cap_seen && following->load()) : true;

            // 没有 IMU 数据时不盲目下发 0 度，避免云台突然跳到中位。
            // 如果正在摆动但数据断了，立即失能，等待数据恢复后重新锁存中心。
            if (!imu_fresh) {
                if (arm->armed) {
                    arm->armed = false;
                    SPDLOG_WARN("imu-only demo: imu data stale, disarm");
                }
                return;
            }

            // ---------- 5.2 使能/失能状态切换 ----------
            // 上升沿：以下位机回传的当前姿态作为摆动中心
            if (want_arm && !arm->armed) {
                const double raw_center_yaw = imu->yaw;
                // 下发链路会对 pitch 取反（output_interface 里 target_pitch = -cmd.pitch），
                // 这里换算到指令空间，保证“保持当前姿态”时不会跳变。
                const double raw_center_pitch = -imu->pitch;

                // 幅度收缩 + 中心钳制，保证 center±amp 不越界
                const FittedWave yaw_fit =
                    fit_wave(raw_center_yaw, yaw_wave_amp, yaw_limit);
                const FittedWave pitch_fit =
                    fit_wave(raw_center_pitch, pitch_wave_amp, pitch_limit);

                arm->center_yaw   = yaw_fit.center;// 锁存的 yaw 摆动中心（弧度）
                arm->center_pitch = pitch_fit.center;// 锁存的 pitch 摆动中心（弧度，指令空间）
                arm->amp_yaw      = yaw_fit.amplitude;// 适配限位后的 yaw 幅度（弧度）
                arm->amp_pitch    = pitch_fit.amplitude;// 适配限位后的 pitch 幅度（弧度）
                arm->start_ns     = now_ns; // 相位从这一刻开始计算
                arm->armed        = true;       // 使能摆动
                SPDLOG_INFO(
                    "imu-only demo armed: imu yaw={:.1f}deg pitch={:.1f}deg -> "
                    "center yaw={:.1f}deg pitch={:.1f}deg amp yaw={:.1f}deg pitch={:.1f}deg",
                    imu->yaw / kDegToRad, imu->pitch / kDegToRad, arm->center_yaw / kDegToRad,
                    arm->center_pitch / kDegToRad, arm->amp_yaw / kDegToRad,
                    arm->amp_pitch / kDegToRad);
            } else if (!want_arm && arm->armed) {
                // 下降沿：操作手关闭 following，或 arm_by_mcu=false 时不会走到这里
                arm->armed = false;
                SPDLOG_INFO("imu-only demo disarmed");
            }

            // ---------- 5.3 生成 WeaponCommand ----------
            L5::WeaponCommand cmd;
            cmd.timestamp_ns      = now_ns; // 当前时间戳
            cmd.plan_timestamp_ns = now_ns; // 计划时间戳

            if (!arm->armed) {
                // 未使能：下发当前实测姿态，等价于“原地保持”，不会跳变。
                // 这样 weapon_output 始终有最新指令，不会重复发送上一次的正弦值。
                cmd.yaw      = imu->yaw;
                cmd.pitch    = -imu->pitch;
                cmd.yaw_vel  = 0.0;
                cmd.pitch_vel = 0.0;
            } else {
                // 已使能：按模式生成波形，t 为本次摆动已持续的秒数
                const double t = static_cast<double>(now_ns - arm->start_ns) * 1e-9;

                double yaw       = arm->center_yaw;// // 当前 yaw 角度（弧度）
                double yaw_vel   = 0.0;// // 当前 yaw 角速度（弧度/秒）
                double pitch     = arm->center_pitch;// // 当前 pitch 角度（弧度，指令空间）
                double pitch_vel = 0.0;// // 当前 pitch 角速度（弧度/秒）

                if (mode == DemoMode::Sine) {
                    // yaw = 中心 + A·sin(ωt)
                    // 角速度前馈 = A·ω·cos(ωt)，供下位机做速度前馈
                    yaw     = arm->center_yaw + arm->amp_yaw * std::sin(sine_yaw_omega * t);
                    yaw_vel = arm->amp_yaw * sine_yaw_omega * std::cos(sine_yaw_omega * t);

                    // pitch 多一个相位差，phase=90° 时形成椭圆轨迹
                    pitch     = arm->center_pitch
                            + arm->amp_pitch * std::sin(sine_pitch_omega * t + sine_phase);
                    pitch_vel = arm->amp_pitch * sine_pitch_omega
                                * std::cos(sine_pitch_omega * t + sine_phase);
                } else {
                    // 左右平移：三角波，速度恒定为 ±4A/T
                    if (use_lateral) {
                        const double u  = t / lateral_period;
                        const double uf = u - std::floor(u); // 归一化相位 [0,1)
                        yaw += lateral_amp * triangle_wave(uf);
                        // 前半周期向右、后半周期向左，速度符号相反
                        yaw_vel += lateral_amp * ((uf < 0.5 ? 4.0 : -4.0) / lateral_period);
                    }
                    // 小陀螺：锯齿波（缓变 + 回跳），速度恒定为 -2A/T
                    if (use_spin) {
                        const double u  = t / spin_period;
                        const double uf = u - std::floor(u);
                        yaw += spin_amp * (1.0 - 2.0 * uf);
                        yaw_vel += spin_amp * (-2.0 / spin_period);
                    }
                    // 非 sine 模式的 pitch 仍然是一个可选正弦
                    pitch     = arm->center_pitch
                            + arm->amp_pitch * std::sin(sine_pitch_omega * t);
                    pitch_vel = arm->amp_pitch * sine_pitch_omega
                                * std::cos(sine_pitch_omega * t);
                }

                // 第二道保护：最终输出再钳制一次，确保绝不越过机械限位
                cmd.yaw       = std::clamp(yaw, -yaw_limit, yaw_limit);
                cmd.pitch     = std::clamp(pitch, -pitch_limit, pitch_limit);
                cmd.yaw_vel   = yaw_vel;
                cmd.pitch_vel = pitch_vel;
            }

            // 角速度前馈有效：下位机可以据此做速度前馈；旧固件忽略即可
            cmd.vel_valid = true;

            // 调试响应度：只动云台，不开火
            cmd.fire = false;

            // 调试字段填充，便于 Foxglove 对照指令与实测姿态
            cmd.distance      = distance;
            cmd.plan_yaw      = cmd.yaw;
            cmd.plan_pitch    = cmd.pitch;
            cmd.plan_distance = distance;
            cmd.ref_yaw       = cmd.yaw;
            cmd.ref_pitch     = cmd.pitch;

            // 写入三缓冲通道；weapon_output 下一次 tick 会读到最新值
            cmd_out.write(cmd);
        });
}

} // namespace fcs::runtime
