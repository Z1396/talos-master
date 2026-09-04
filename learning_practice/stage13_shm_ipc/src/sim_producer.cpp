// ============================================================================
// stage13：仿真程序（生产者端，模拟 Daedalus / Rust 仿真器角色）
//
// 真实项目中这一端由 Rust 实现（memmap2 文件映射，字节布局由
// shm_layout.hpp 的 static_assert 保证两端一致）。本程序用 C++ 调用
// ShmClient::create() 复刻同样行为，演示：
//   1. 建立两块共享内存（元数据 + 图像池），写入魔数/版本/心跳
//   2. ~30fps 发布合成图像（渐变 + 移动方块 + 序号条带图案）
//   3. 发布云台/相机位姿（正弦轨迹模拟机器人运动）
//   4. 通过 GimbalOps 读回视觉端下发的云台指令，形成闭环演示
//
// 运行：./sim_producer（Ctrl+C 退出，析构自动删除 /tmp/talos_ipc_* 文件）
// 配合：./vision_consumer（另开终端）
// ============================================================================

#include "shm_client.hpp"
#include "shm_layout.hpp"
#include "shm_region.hpp"
#include "shm_triple_buffer.hpp"

// C++ 标准库
#include <atomic>  // 原子退出标志（信号处理与主循环共享）
#include <chrono>  // steady_clock 计时 / 纳秒时间戳 / sleep_until 周期节拍
#include <cmath>   // std::sin 正弦轨迹
#include <csignal> // SIGINT/SIGTERM 优雅退出
#include <cstdint>
#include <cstdio>  // fflush 保证 stdout 输出顺序
#include <cstdlib>
#include <thread>

// fmt：格式化输出
#include <fmt/core.h>

// OpenCV：合成图像矩阵
#include <opencv2/core.hpp>

using namespace std::chrono_literals;

// Ctrl+C 退出标志：信号处理函数只做置位，无锁安全
static volatile std::sig_atomic_t g_stop = 0;
static void handle_signal(int) { g_stop = 1; }

/// 系统时钟纳秒时间戳（与 ShmClient::update_heartbeat 内部实现一致）
static uint64_t now_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

// ============================================================================
// 合成图像：行渐变背景 + 顶部 64bit 序号条带 + 移动方块
// 只用 OpenCV core（无 imgproc 依赖），像素图案编码帧序号
// ============================================================================
static void render_frame(cv::Mat& img, const uint64_t seq, const double t) {
    // 行渐变背景：颜色随行号变化，肉眼可辨识"图像在动"
    for (int r = 0; r < img.rows; ++r) {
        img.row(r) = cv::Scalar(
            (r + static_cast<int>(t * 60)) & 0xFF,     // B
            (r * 2 + static_cast<int>(t * 30)) & 0xFF, // G
            (r * 3) & 0xFF);                           // R
    }
    // 顶部序号条带：64 bit 二进制图案，宽 1440 / 64 bit ≈ 22 列/bit
    constexpr int kBitWidth = 22;
    for (int bit = 0; bit < 64; ++bit) {
        // 白=1 黑=0，从最高位画到最低位
        const bool on = (seq >> (63 - bit)) & 1ULL;
        img(cv::Rect(bit * kBitWidth, 0, kBitWidth, 64)).setTo(on ? 255 : 0);
    }
    // 移动方块：水平往返运动，模拟画面中的运动目标
    const int x =
        static_cast<int>((std::sin(t * 0.8) * 0.5 + 0.5) * (img.cols - 200));   // t → [0, cols-200]
    img(cv::Rect(x, img.rows / 2 - 75, 150, 150)).setTo(cv::Scalar(0, 0, 255)); // 红方块
}

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    fmt::print("==== stage13 仿真程序（生产者，模拟 Daedalus/Rust 端）====\n");
    // {:.1f} 定点小数输出；写成 {:.1} 会被 fmt 按默认浮点格式转成科学计数法（1e+01）
    fmt::print(
        "共享内存: {} + {}（各 {:.1f} MB）\n", ipc::shm_path(ipc::SHM_NAME_META).string(),
        ipc::shm_path(ipc::SHM_NAME_IMAGE_POOL).string(), ipc::IMAGE_POOL_SIZE / 1024.0 / 1024.0);
    std::fflush(stdout);

    // ---------------- 建立共享内存（真实项目由 Rust memmap2 完成）----------------
    auto client = ipc::ShmClient::create();
    if (!client) {
        fmt::print(stderr, "[FAIL] 创建共享内存失败: {}\n", client.error());
        return 1;
    }
    fmt::print(
        "[ OK ] 共享内存已建立（magic={:#x} version={} 图像 {}x{}）\n", ipc::SHM_MAGIC,
        ipc::SHM_VERSION, ipc::IMAGE_WIDTH, ipc::IMAGE_HEIGHT);
    std::fflush(stdout);

    // ---------------- 第二映射：读视觉端回发的云台指令 ----------------
    // ShmClient 未暴露 recv_gimbal_cmd（业务上生产者是 Rust），这里用
    // ShmRegion::open 再映射一次元数据区，以 GimbalOps 消费指令通道
    auto cmd_region = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
    if (!cmd_region) {
        fmt::print(stderr, "[FAIL] 打开指令通道失败: {}\n", cmd_region.error());
        return 1;
    }
    ipc::GimbalOps cmd_ops(&cmd_region->as<ipc::ShmMetaRegion>()->gimbal_cmd);

    // ---------------- 30fps 主循环 ----------------
    cv::Mat img(ipc::IMAGE_HEIGHT, ipc::IMAGE_WIDTH, CV_8UC3); // 全尺寸复用缓冲
    uint64_t seq           = 0;                                // 帧序号
    uint64_t cmds_received = 0;                                // 收到的指令计数
    double last_cmd_yaw    = 0.0;                              // 最近一次指令内容（节流打印用）
    double last_cmd_pitch  = 0.0;
    bool last_cmd_fire     = false;
    const auto t0          = std::chrono::steady_clock::now();
    auto next_tick         = t0;                               // 循环节拍锚点（不漂移）
    auto next_stat         = t0 + 1s;                          // 秒级统计打印时刻

    fmt::print("开始发布图像/位姿 @30fps，等待视觉程序连接...（Ctrl+C 退出）\n\n");
    std::fflush(stdout);

    while (!g_stop) {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - t0).count();

        // --- 合成并发布当前帧 ---
        render_frame(img, seq, t);
        client->publish_image(img, seq, now_ns());

        // --- 发布云台位姿：yaw 正弦摆动，模拟机器人运动 ---
        const double yaw = std::sin(t * 0.5) * 0.5; // rad
        const ipc::ShmClient::Pose gimbal_pose{
            // 绕 z 轴 yaw 旋转的单位四元数 (w, x, y, z)
            .x            = 0.0,
            .y            = 0.0,
            .z            = 0.0,
            .qw           = std::cos(yaw / 2),
            .qx           = 0.0,
            .qy           = 0.0,
            .qz           = std::sin(yaw / 2),
            .frame_seq    = seq,
            .timestamp_ns = now_ns(),
        };
        client->publish_pose(ipc::POSE_GIMBAL, gimbal_pose);
        // 相机位姿：固定在云台前方 0.1m
        const ipc::ShmClient::Pose cam_pose{
            .x            = 0.1,
            .y            = 0.0,
            .z            = 0.0,
            .qw           = 1.0,
            .qx           = 0.0,
            .qy           = 0.0,
            .qz           = 0.0,
            .frame_seq    = seq,
            .timestamp_ns = now_ns(),
        };
        client->publish_pose(ipc::POSE_CAMERA, cam_pose);

        // --- 心跳保活：视觉端据此判断仿真存活 ---
        client->update_heartbeat();

        // --- 读回视觉端指令（latest-wins：只取最新一条）---
        if (const auto cmd = cmd_ops.borrow()) {
            ++cmds_received;
            last_cmd_yaw   = (*cmd)->yaw_deg;
            last_cmd_pitch = (*cmd)->pitch_deg;
            last_cmd_fire  = (*cmd)->fire_advice != 0;
        }

        // --- 秒级统计：帧序号 + 最新指令内容 ---
        if (now >= next_stat) {
            next_stat += 1s;
            fmt::print(
                "[SIM ] 已发布 {} 帧 | 收到指令 {} 条 | 最新: yaw={:+7.2f}° "
                "pitch={:+7.2f}° fire={}\n",
                seq + 1, cmds_received, last_cmd_yaw, last_cmd_pitch, last_cmd_fire ? "YES" : "no");
            std::fflush(stdout);
        }

        // --- 30fps 固定节拍：sleep_until 补偿循环耗时 ---
        next_tick += 33ms;
        std::this_thread::sleep_until(next_tick);
        ++seq;
    }

    // ---------------- 退出：析构链自动清理 ----------------
    fmt::print("\n[SIM ] 退出，共发布 {} 帧、收到 {} 条指令\n", seq, cmds_received);
    fmt::print("[SIM ] 共享内存文件将由 owner 析构自动删除\n");
    return 0;
}
