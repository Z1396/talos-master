// ============================================================================
// stage13：视觉程序（消费者端，模拟 Talos 视觉程序角色）
//
// 复刻主项目 src/fcs 中视觉端使用 ShmClient 的典型方式：
//   1. connect() 连接 + wait_for_producer() 等仿真上线（防连到残留旧内存）
//   2. 轮询 recv_image()：零拷贝 cv::Mat 直接读共享内存像素，
//      统计帧率 + 用 seq 跳变检测丢帧（latest-wins 下属正常现象）
//   3. recv_pose(POSE_GIMBAL) 读取云台位姿，做简单 P 控制（假自瞄：
//      朝图像序号正弦方向摆动），send_gimbal_cmd 下发指令形成闭环
//   4. 心跳超时检测：生产者消失（Ctrl+C）时打印告警并退出
//
// 运行：./vision_consumer（先启动 ./sim_producer）
// ============================================================================

#include "shm_client.hpp"
#include "shm_layout.hpp"

// C++ 标准库
#include <atomic>   // 原子退出标志
#include <chrono>   // 计时 / sleep_until 周期节拍
#include <cmath>    // std::sin/std::atan2/std::abs
#include <csignal>  // SIGINT 优雅退出
#include <cstdint>
#include <cstdio>   // fflush
#include <cstdlib>
#include <optional>
#include <thread>

// fmt：格式化输出
#include <fmt/core.h>

using namespace std::chrono_literals;

// Ctrl+C 退出标志
static volatile std::sig_atomic_t g_stop = 0;
static void handle_signal(int) { g_stop = 1; }

/// 从单位四元数 (w,x,y,z) 提取偏航角 yaw（rad），用于 P 控制反馈
static double quaternion_to_yaw(
    const double qw, const double qx, const double qy, const double qz) {
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    fmt::print("==== stage13 视觉程序（消费者，模拟 Talos 端）====\n");
    std::fflush(stdout);

    // ---------------- 连接共享内存 + 等待生产者上线 ----------------
    auto client = ipc::ShmClient::connect();
    if (!client) {
        fmt::print(stderr, "[FAIL] 连接失败: {}（先启动 ./sim_producer）\n", client.error());
        return 1;
    }
    fmt::print("[ OK ] 已连接共享内存（magic={:#x} version={} 图像 {}x{}）\n",
               client->header().magic, client->header().version,
               client->header().image_width, client->header().image_height);

    if (!client->wait_for_producer(std::chrono::seconds(10))) {
        fmt::print(stderr, "[FAIL] 等待生产者心跳超时（10s 无心跳）\n");
        return 1;
    }
    fmt::print("[ OK ] 生产者已上线（心跳活跃），开始订阅...\n\n");
    std::fflush(stdout);

    // ---------------- 统计量 ----------------
    uint64_t frames = 0;        // 收到的图像帧数
    uint64_t lost_frames = 0;   // seq 跳变推算的丢帧数（latest-wins 正常现象）
    uint64_t poses = 0;         // 位姿更新次数
    uint64_t cmds_sent = 0;     // 下发指令数
    std::optional<uint64_t> last_seq;      // 上一帧 seq（跳变检测）
    double   cur_yaw_rad = 0.0;            // P 控制反馈：当前云台 yaw
    double   last_cmd_yaw = 0.0;           // 最近下发内容（打印用）
    double   last_cmd_pitch = 0.0;
    bool     last_cmd_fire = false;

    // 假自瞄目标：朝图像序号方向正弦摆动 ±20°（模拟检测到目标方位）
    constexpr double kTargetAmpDeg = 20.0;
    constexpr double kPGain = 0.8; // P 控制增益

    const auto t0 = std::chrono::steady_clock::now();
    auto       next_tick = t0;       // 60fps 轮询节拍
    auto       next_stat = t0 + 1s;  // 秒级统计时刻

    while (!g_stop) {
        const auto now = std::chrono::steady_clock::now();

        // --- 心跳超时检测：生产者退出 → 告警退出 ---
        if (!client->is_producer_alive()) {
            fmt::print("\n[WARN] 生产者心跳超时（>1s），仿真程序已退出，本程序退出\n");
            break;
        }

        // --- 订阅图像：零拷贝读取（无新帧时 recv 返回 nullopt）---
        if (const auto frame = client->recv_image()) {
            if (last_seq.has_value() && frame->seq > *last_seq + 1) {
                // latest-wins：生产者比消费者快时旧帧被覆盖，属正常语义
                lost_frames += frame->seq - *last_seq - 1;
            }
            last_seq = frame->seq;
            ++frames;

            // --- 假自瞄：P 控制朝"目标"（seq 正弦方位）摆动 ---
            const double target_rad
                = std::sin(static_cast<double>(frame->seq) * 0.05) * kTargetAmpDeg * M_PI / 180.0;
            const double err_rad = target_rad - cur_yaw_rad; // 误差
            const double cmd_rad = cur_yaw_rad + kPGain * err_rad; // P 控制
            const float cmd_yaw_deg = static_cast<float>(cmd_rad * 180.0 / M_PI);
            const float cmd_pitch_deg
                = static_cast<float>(std::sin(static_cast<double>(frame->seq) * 0.03) * 5.0);
            const bool fire = std::abs(err_rad) < 2.0 * M_PI / 180.0; // 误差<2°才建议开火

            client->send_gimbal_cmd(cmd_yaw_deg, cmd_pitch_deg, 3.5f, fire);
            last_cmd_yaw = cmd_yaw_deg;
            last_cmd_pitch = cmd_pitch_deg;
            last_cmd_fire = fire;
            ++cmds_sent;
        }

        // --- 订阅云台位姿：更新 P 控制反馈 ---
        if (const auto pose = client->recv_pose(ipc::POSE_GIMBAL)) {
            cur_yaw_rad = quaternion_to_yaw(pose->qw, pose->qx, pose->qy, pose->qz);
            ++poses;
        }

        // --- 秒级统计：帧率 / 丢帧 / 指令 ---
        if (now >= next_stat) {
            next_stat += 1s;
            fmt::print("[VIS ] 帧 {:>5} | 位姿 {:>5} | 指令 {:>5} | 丢帧 {:>3} | "
                       "下发 yaw={:+7.2f}° pitch={:+6.2f}° fire={}\n",
                       frames, poses, cmds_sent, lost_frames,
                       last_cmd_yaw, last_cmd_pitch, last_cmd_fire ? "YES" : "no");
            std::fflush(stdout);
        }

        // --- 60fps 轮询节拍 ---
        next_tick += 16ms;
        std::this_thread::sleep_until(next_tick);
    }

    // ---------------- 汇总退出 ----------------
    const double dur = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - t0).count();
    fmt::print("\n[VIS ] 运行 {:.1f}s：收帧 {}（{:.1f} fps，丢帧 {} 占 {:.1f}%）"
               " | 位姿 {} | 下发指令 {}\n",
               dur, frames, frames / dur, lost_frames,
               frames + lost_frames > 0
                   ? 100.0 * lost_frames / (frames + lost_frames) : 0.0,
               poses, cmds_sent);
    return 0;
}
