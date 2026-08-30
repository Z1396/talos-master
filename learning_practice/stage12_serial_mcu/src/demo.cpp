// ============================================================================
// stage12：真实下位机串口通信实测程序（STM32 云台主控）
//
// 与 PTY 单元测试（crates/hardware/at_gimbal/tests/serial_protocol_test.cpp）
// 的区别：本程序部署在机器人上，对【真实硬件】做验收测试。
//
// 使用的主项目真实 API（与 src/fcs/L1_sensor/output_interface.cpp 同源）：
//   McuDeviceHandle::create_serial()   创建串口设备句柄（termios 8N1 原始模式）
//   Stm32Parser::latest_imu            解析器静态输出槽（IMU 姿态）
//   Stm32Parser::latest_capabilities    解析器静态输出槽（能力开关）
//   handle->handle_events()             事件轮询：读串口 + 拆帧 + 解析
//   handle->send_sync()                 同步下发指令帧（带超时）
//
// 实测项：
//   A. 连接层 —— 串口路径 / 权限 / 波特率真实生效（termios 真配置）
//   B. 上行   —— IMU 帧率、时间戳单调性、帧完整性（SoF/EoF 魔数）、丢帧
//   C. 能力帧 —— following / power_rune / quanta 开关变化打印
//   D. 下行   —— SendSimpleVisionData 指令帧下发成功率（可选，默认关闭）
//   E. 稳定性 —— 断线自动重连、长时间运行统计汇总
//
// 用法：
//   ./serial_mcu_demo --list                       # 列出本机候选串口
//   ./serial_mcu_demo                              # 默认 /dev/ttyS4@115200 只收不发
//   ./serial_mcu_demo -d /dev/ttyUSB0 -b 460800    # 指定设备与波特率
//   ./serial_mcu_demo -t 60                        # 跑 60 秒自动退出（0=Ctrl+C 退出）
//   ./serial_mcu_demo -s 100                       # 100Hz 下发正弦扫摆指令
//                                                  # ！！！云台会 ±30° 真实转动！！！
//
// 权限提示：串口默认属于 dialout 组：
//   sudo usermod -aG dialout $USER   # 然后重新登录生效
// ============================================================================

#include "talos_gimbal/mcu_device.hpp"
#include "talos_gimbal/packet.hpp"

// C++ 标准库
#include <algorithm>   // std::min/std::max 时间戳间隔统计
#include <chrono>      // steady_clock 高精度计时 / sleep_until 周期节拍
#include <cmath>       // std::sin 正弦扫摆轨迹
#include <csignal>     // SIGINT 信号处理（Ctrl+C 优雅退出）
#include <cstdint>
#include <cstdio>      // fflush 保证 stdout/stderr 交替输出顺序
#include <cstdlib>
#include <filesystem>  // 扫描 /dev 列出候选串口设备
#include <string>
#include <thread>

// fmt：格式化输出（与主项目一致，packet.hpp 已依赖）
#include <fmt/core.h>

namespace tg = talos_gimbal;
using namespace std::chrono_literals;

// ============================================================================
// 全局退出标志：信号处理函数与主循环之间通信
// volatile sig_atomic_t 是信号安全异步通信的唯一标准写法
// ============================================================================
static volatile std::sig_atomic_t g_stop = 0;

static void handle_signal(int) { g_stop = 1; }

// ============================================================================
// 命令行参数
// ============================================================================
struct Options {
    std::string device     = "/dev/ttyS4"; // 主项目 SerialImpl::connect 的默认路径
    int         baud       = 115200;      // 主项目默认波特率
    double      duration_s = 0.0;         // 运行时长，0 = 一直跑到 Ctrl+C
    double      send_hz    = 0.0;         // 下发频率，0 = 只收不发（安全默认）
    bool        list_only  = false;       // 仅列出候选串口后退出
};

static void print_usage(const char* prog) {
    fmt::print(
        "用法: {} [选项]\n"
        "  -d, --device <path>   串口设备路径（默认 /dev/ttyS4）\n"
        "  -b, --baud <rate>     波特率（默认 115200；支持 9600~921600）\n"
        "  -t, --duration <sec>  运行秒数，0 = 直到 Ctrl+C（默认 0）\n"
        "  -s, --send-hz <hz>    指令下发频率（默认 0 = 关闭；！开启后云台会真实转动！）\n"
        "  --list                列出本机候选串口设备后退出\n"
        "  -h, --help            显示本帮助\n",
        prog);
}

/// 解析命令行（系统边界：参数来自用户输入，逐项校验）
static Options parse_args(int argc, char* argv[]) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        // 取值辅助：缺失取值直接报错退出
        auto next_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                fmt::print(stderr, "错误: 参数 {} 缺少取值\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        try {
            if (arg == "-d" || arg == "--device") {
                opt.device = next_value("-d");
            } else if (arg == "-b" || arg == "--baud") {
                opt.baud = std::stoi(next_value("-b"));
            } else if (arg == "-t" || arg == "--duration") {
                opt.duration_s = std::stod(next_value("-t"));
            } else if (arg == "-s" || arg == "--send-hz") {
                opt.send_hz = std::stod(next_value("-s"));
            } else if (arg == "--list") {
                opt.list_only = true;
            } else if (arg == "-h" || arg == "--help") {
                print_usage(argv[0]);
                std::exit(0);
            } else {
                fmt::print(stderr, "错误: 未知参数 {}\n", arg);
                print_usage(argv[0]);
                std::exit(2);
            }
        } catch (const std::exception& e) {
            fmt::print(stderr, "错误: 参数 {} 取值非法（{}）\n", arg, e.what());
            std::exit(2);
        }
    }
    // 波特率白名单：与 serial.hpp baud_to_speed 的 switch 分支一致，
    // 白名单外的值会被静默回退为 115200 —— 上真机前必须提醒
    switch (opt.baud) {
    case 9600: case 19200: case 38400: case 57600:
    case 115200: case 230400: case 460800: case 921600:
        break;
    default:
        fmt::print(stderr,
                    "警告: 波特率 {} 不在 serial.hpp 支持列表，将被回退为 115200！\n"
                    "      支持: 9600/19200/38400/57600/115200/230400/460800/921600\n",
                    opt.baud);
        break;
    }
    if (opt.duration_s < 0.0 || opt.send_hz < 0.0) {
        fmt::print(stderr, "错误: 时长/频率不允许为负数\n");
        std::exit(2);
    }
    return opt;
}

/// 扫描 /dev 下常见串口设备名前缀并打印（帮现场快速定位设备路径）
static void list_serial_devices() {
    fmt::print("本机候选串口设备（/dev/ttyS* / ttyUSB* / ttyACM* / ttyAMA*）:\n");
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator("/dev", ec)) {
        const auto name = entry.path().filename().string();
        if (name.starts_with("ttyS") || name.starts_with("ttyUSB") || name.starts_with("ttyACM")
            || name.starts_with("ttyAMA")) {
            fmt::print("  {}\n", entry.path().string());
        }
    }
    fmt::print("权限提示: 无权打开时执行  sudo usermod -aG dialout $USER  并重新登录\n");
}

// ============================================================================
// 统计量：整场实测的验收数据
// ============================================================================
struct RxStats {
    uint64_t frames       = 0; // 收到的 IMU 帧总数
    uint64_t caps_frames  = 0; // 收到的能力帧总数
    uint64_t anomalies    = 0; // 时间戳异常（回退/重复/大跳变）次数
    uint64_t tx_ok        = 0; // 下发成功次数
    uint64_t tx_fail      = 0; // 下发失败次数
    uint64_t reconnects   = 0; // 成功重连次数

    // MCU 时间戳间隔（ms）——用于验证下位机发送频率的稳定性
    int64_t dt_min_ms = INT64_MAX;
    int64_t dt_max_ms = 0;
    double  dt_sum_ms = 0.0;
};

// ============================================================================
// 主流程
// ============================================================================
int main(int argc, char* argv[]) {
    const Options opt = parse_args(argc, argv);

    if (opt.list_only) {
        list_serial_devices();
        return 0;
    }

    // Ctrl+C / kill 优雅退出（不写共享资源，仅置标志）
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    fmt::print("==== stage12 串口下位机实测 ====\n");
    fmt::print("设备: {} @ {}\n", opt.device, opt.baud);
    std::fflush(stdout); // 保证后续 spdlog(stderr) 日志顺序稳定

    // ---------------- 第一步：连接真实下位机 ----------------
    auto connected = tg::McuDeviceHandle::create_serial(opt.device, opt.baud);
    if (!connected) {
        // 失败现场排查提示：路径错 / 无权限 / 设备未上电，三类最常见
        fmt::print(stderr, "\n[FAIL] 连接失败: {}\n", connected.error());
        fmt::print(stderr, "排查步骤:\n"
                            "  1. ./serial_mcu_demo --list        确认设备路径存在\n"
                            "  2. ls -l {}               检查属主与权限\n"
                            "  3. sudo usermod -aG dialout $USER   加入 dialout 组后重新登录\n"
                            "  4. 确认下位机已上电、串口线已接对（注意别接到调试口）\n",
                    opt.device);
        return 1;
    }
    auto handle = std::move(*connected);
    fmt::print("[ OK ] 已连接（variant 分发: serial={}）\n", handle.is_serial());

    // ---------------- 第二步：绑定解析器输出槽 ----------------
    // Stm32Parser 通过静态指针输出解析结果（USB/串口共用同一套解析），
    // 测试程序把指针绑到本地变量，退出时恢复，不污染全局状态
    tg::ReceiveImuData          imu{};
    tg::ReceiveCapabilitiesData caps{};
    tg::Stm32Parser::latest_imu          = &imu;
    tg::Stm32Parser::latest_capabilities = &caps;

    // 下发模式安全横幅：云台真实运动会伤人/撞杆，必须显式醒目
    if (opt.send_hz > 0.0) {
        fmt::print("\n"
                   "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                   "!! 下发模式已开启: 云台将以 ±30° 正弦扫摆（周期 6s）       !!\n"
                   "!! 确认云台周围无人员、无障碍物后再继续！                  !!\n"
                   "!! fire_advice 恒为 false（本测试不触发开火）             !!\n"
                   "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    } else {
        fmt::print("模式: 只收不发（安全默认；-s <hz> 开启下发测试）\n");
    }

    // ---------------- 第三步：200Hz 主循环（与 fcs 主循环同频） ----------------
    RxStats         stats;
    const auto      t_start = std::chrono::steady_clock::now();
    auto            next_tick = t_start;             // 循环节拍锚点
    auto            next_stat  = t_start + 1s;        // 秒级统计打印时刻
    auto            next_reconnect = t_start;         // 断线重连尝试时刻
    double          send_phase = 0.0;                  // 下发相位积分（周期控制）
    auto            last_rx_time = t_start;           // 最近一次收到新帧的时刻
    uint32_t        prev_stamp = 0;                   // 上一帧 MCU 时间戳
    uint32_t        prev_caps_stamp = 0;               // 上一帧能力帧时间戳
    bool            have_frame = false;               // 是否已收到首帧
    bool            have_caps = false;

    while (!g_stop) {
        const auto now = std::chrono::steady_clock::now();
        // -t 到时自动退出（0 = 永不）
        if (opt.duration_s > 0.0) {
            const double elapsed = std::chrono::duration<double>(now - t_start).count();
            if (elapsed >= opt.duration_s) {
                break;
            }
        }

        // --- 事件轮询：read + 拆帧 + Stm32Parser 解析（项目真实收包路径）---
        handle.handle_events();

        // --- 上行帧验收 A：新帧检测 + 完整性 + 时间戳单调性 ---
        // sof 魔数守卫：imu 零初始化时 sof=0，排除"未收到真实帧"的启动伪影
        if (imu.header.sof == tg::HeaderFrame::SoF()
            && (!have_frame || imu.time_stamp != prev_stamp)) {
            // 帧魔数完整性（解析层理论上已过滤坏帧，此处防御性复核）
            if (imu.header.sof != tg::HeaderFrame::SoF() || imu.eof != tg::HeaderFrame::EoF()) {
                fmt::print("[WARN] 帧魔数异常: sof={:#04x} eof={:#04x}（解析层疑似被绕过）\n",
                           imu.header.sof, imu.eof);
                ++stats.anomalies;
            }
            if (have_frame) {
                // MCU 时间戳为毫秒计数：正常应严格递增且间隔稳定。
                // dt<=0 → MCU 复位或重复帧；dt 过大 → 丢帧/串口阻塞
                const int64_t dt = static_cast<int64_t>(imu.time_stamp)
                                 - static_cast<int64_t>(prev_stamp);
                if (dt <= 0 || dt > 1000) {
                    fmt::print("[WARN] 时间戳异常: dt={} ms（stamp {} -> {}）\n",
                               dt, prev_stamp, imu.time_stamp);
                    ++stats.anomalies;
                }
                stats.dt_sum_ms += static_cast<double>(dt);
                stats.dt_min_ms   = std::min(stats.dt_min_ms, dt);
                stats.dt_max_ms   = std::max(stats.dt_max_ms, dt);
            }
            have_frame = true;
            prev_stamp = imu.time_stamp;
            ++stats.frames;
            last_rx_time = now;
            // 实时单行刷新打印（packet.hpp 自带的调试打印）
            tg::print_imu(imu);
        }

        // --- 上行帧验收 B：能力帧变化（只在变化时打印一行）---
        // sof 魔数校验：caps 零初始化时 sof=0，排除"未收到真实能力帧"的启动伪影
        if (caps.header.sof == tg::HeaderFrame::SoF()
            && (!have_caps || caps.time_stamp != prev_caps_stamp)) {
            have_caps = true;
            prev_caps_stamp = caps.time_stamp;
            ++stats.caps_frames;
            fmt::print("\n[CAPS] stamp={} following={} power_rune={} quanta={}\n",
                       caps.time_stamp, caps.data.following, caps.data.power_rune,
                       caps.data.quanta);
        }

        // --- 下行链路验收（可选）---
        // 照抄 src/fcs/L1_sensor/output_interface.cpp 串口分支的真实写法：
        //   SendSimpleVisionData，len=sizeof(data)，id=0x04，角度制
        if (opt.send_hz > 0.0 && handle.is_connected()) {
            send_phase += opt.send_hz * 0.005; // 每个循环 5ms 的相位增量
            // 正弦扫摆：±30°，周期 6s（相位/周期=send_phase/120 圈）
            constexpr double kTwoPi   = 6.283185307179586;
            const float      target_deg = 30.0f * static_cast<float>(
                                      std::sin(kTwoPi * send_phase / 120.0));
            const tg::SendSimpleVisionData packet{
                .header = {
                    .sof = tg::HeaderFrame::SoF(),
                    .len = sizeof(tg::SendSimpleVisionData::data),
                    .id  = 0x04, // 简易视觉指令帧 ID（真实业务同款）
                },
                .data = {
                    .target_yaw = target_deg,
                },
                .eof = tg::HeaderFrame::EoF(),
            };
            if (handle.send_sync(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet))) {
                ++stats.tx_ok;
            } else {
                ++stats.tx_fail;
            }
        }

        // --- 秒级统计（破坏上面的单行刷新，验收数据优先）---
        if (now >= next_stat) {
            next_stat += 1s;
            const double age_ms
                = std::chrono::duration<double, std::milli>(now - last_rx_time).count();
            fmt::print(
                "\n[STAT] rx={:>6} 帧 | 帧龄={:6.1f}ms | tx ok/fail={}/{} | 重连={} | 异常={}\n",
                stats.frames, age_ms, stats.tx_ok, stats.tx_fail, stats.reconnects,
                stats.anomalies);
            // 看门狗：1 秒没有任何新帧 → 上行链路大概率有问题
            if (age_ms > 1000.0) {
                fmt::print("[WARN] 超过 1 秒未收到 IMU 帧 —— 检查: 下位机供电/发送使能/"
                           "波特率/串口线 TX-RX 是否接反\n");
            }
        }

        // --- 断线自动重连（每秒尝试一次，SerialImpl 读错误会自动 disconnect）---
        if (!handle.is_connected() && now >= next_reconnect) {
            next_reconnect = now + 1s;
            fmt::print("\n[WARN] 串口已断开，尝试重连...\n");
            auto re = tg::McuDeviceHandle::create_serial(opt.device, opt.baud);
            if (re) {
                handle = std::move(*re);
                ++stats.reconnects;
                fmt::print("[ OK ] 重连成功（第 {} 次）\n", stats.reconnects);
            }
        }

        // --- 200Hz 固定节拍：sleep_until 补偿循环耗时，节拍不漂移 ---
        next_tick += 5ms;
        std::this_thread::sleep_until(next_tick);
    }

    // ---------------- 第四步：恢复全局解析槽 + 输出验收报告 ----------------
    tg::Stm32Parser::latest_imu          = nullptr;
    tg::Stm32Parser::latest_capabilities = nullptr;

    const double duration_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    const double avg_fps = duration_s > 0.0
                             ? static_cast<double>(stats.frames) / duration_s : 0.0;
    const double avg_dt  = stats.frames > 1
                             ? stats.dt_sum_ms / static_cast<double>(stats.frames - 1) : 0.0;

    fmt::print("\n==================== 串口下位机验收报告 ====================\n");
    fmt::print("运行时长          : {:8.1f} s\n", duration_s);
    fmt::print("设备              : {} @ {}（重连 {} 次）\n",
               opt.device, opt.baud, stats.reconnects);
    fmt::print("上行 IMU 帧       : {:8} 帧，平均 {:7.1f} fps\n", stats.frames, avg_fps);
    fmt::print("MCU 时间戳间隔    : avg {:6.2f} ms", avg_dt);
    if (stats.frames > 1) {
        fmt::print(" / min {} ms / max {} ms\n", stats.dt_min_ms, stats.dt_max_ms);
    } else {
        fmt::print("\n");
    }
    fmt::print("能力帧            : {:8} 帧\n", stats.caps_frames);
    if (opt.send_hz > 0.0) {
        fmt::print("下行指令帧        : 成功 {} / 失败 {}\n", stats.tx_ok, stats.tx_fail);
    } else {
        fmt::print("下行指令帧        : 未开启（-s <hz> 开启下发测试）\n");
    }
    fmt::print("时间戳/魔数异常   : {:8} 次\n", stats.anomalies);

    // 验收结论：三档判定
    const bool tx_pass = opt.send_hz <= 0.0 || stats.tx_fail == 0;
    if (stats.frames == 0) {
        fmt::print("结论              : FAIL —— 未收到任何 IMU 帧，上行链路不通\n");
        return 1;
    }
    if (stats.anomalies == 0 && tx_pass && stats.reconnects == 0) {
        fmt::print("结论              : PASS —— 收发稳定，无异常记录\n");
    } else if (stats.anomalies <= 3 && tx_pass) {
        fmt::print("结论              : PASS（含 {} 处异常/重连，建议排查）\n",
                   stats.anomalies + stats.reconnects);
    } else {
        fmt::print("结论              : WARN —— 异常较多（{} 次），建议排查波特率/接地/干扰\n",
                   stats.anomalies + stats.reconnects);
    }
    fmt::print("============================================================\n");
    return 0;
}
