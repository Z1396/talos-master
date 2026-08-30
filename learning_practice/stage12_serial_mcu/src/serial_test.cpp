// ============================================================================
// serial_test.cpp - 串口测试核心逻辑
// 负责：连接下位机、接收解析数据、下发指令、统计验收
// ============================================================================

#include "serial_test.hpp"      // 本模块头文件：声明 run_serial_test 函数
#include "stats.hpp"            // 统计数据结构 RxStats
#include "utils.hpp"            // 工具函数：print_report 打印验收报告
#include "talos_gimbal/mcu_device.hpp"  // 统一设备句柄 McuDeviceHandle
#include "talos_gimbal/packet.hpp"      // 协议包定义：IMU帧、能力帧、指令帧

#include <chrono>   // steady_clock 高精度计时 / duration 时间计算
#include <cmath>    // std::sin 正弦函数（生成扫摆轨迹）
#include <csignal>  // std::signal 注册信号处理函数（Ctrl+C 优雅退出）
#include <cstdio>   // std::fflush 强制刷新输出缓冲区
#include <thread>   // std::this_thread::sleep_until 精确延时
#include <fmt/core.h>  // fmt::print 格式化输出

// ============================================================================
// 命名空间别名 & 字面量
// ============================================================================

namespace tg = talos_gimbal;              // talos_gimbal 命名空间简写
using namespace std::chrono_literals;     // 1s、5ms 等时间字面量

// ============================================================================
// 全局退出标志 & 信号处理
// ============================================================================

/**
 * @brief 全局退出标志
 * volatile: 告诉编译器不要优化这个变量（每次从内存读取）
 * sig_atomic_t: 信号安全的原子类型，读写操作是原子的
 * 这是信号处理函数与主循环之间通信的唯一安全方式
 */
static volatile std::sig_atomic_t g_stop = 0;

/**
 * @brief 信号处理函数
 * 当用户按 Ctrl+C (SIGINT) 或 kill (SIGTERM) 时被调用
 * 把 g_stop 置为 1，主循环检测到后退出
 */
static void handle_signal(int) { g_stop = 1; }

// ============================================================================
// 构建下发的指令包
// ============================================================================

/**
 * @brief 根据相位生成正弦扫摆指令包
 * @param phase 当前相位（累计周期数）
 * @return SendSimpleVisionData 指令包
 * 
 * 云台会按照正弦曲线 ±30° 摆动，周期 6 秒
 * 相位达到 120 时完成一个完整周期（2π）
 */
static tg::SendSimpleVisionData build_packet(double phase) {
    // 2π 常量，用于正弦计算
    constexpr double kTwoPi = 6.283185307179586;
    
    /**
     * 计算目标角度：±30° 正弦扫摆
     * phase / 120.0: phase 累计到 120 时，角度为 sin(2π)，完成一个周期
     * 周期 = 120 / send_hz，当 send_hz=20 时周期=6s
     */
    const float target_deg = 30.0f * static_cast<float>(
        std::sin(kTwoPi * phase / 120.0));
    
    /**
     * 构造指令包（C++20 指定初始化器语法）
     * header: 帧头 { SOF魔数, 数据长度, 指令ID }
     * data:   数据负载 { 目标偏航角 }
     * eof:    帧尾 EOF魔数
     */
    return {
        .header = { 
            .sof = tg::HeaderFrame::SoF(),          // 0xA5 帧头魔数
            .len = sizeof(tg::SendSimpleVisionData::data),  // 数据长度
            .id  = 0x04                             // 指令ID：简易视觉指令
        },
        .data = { 
            .target_yaw = target_deg                // 目标偏航角（度）
        },
        .eof = tg::HeaderFrame::EoF()               // 0x5A 帧尾魔数
    };
}

// ============================================================================
// 核心测试函数
// ============================================================================

/**
 * @brief 运行串口下位机实测
 * @param opt 命令行解析后的选项
 * @return 0 成功, 1 失败
 * 
 * 流程：
 * 1. 注册信号处理函数（Ctrl+C 优雅退出）
 * 2. 连接串口设备
 * 3. 绑定解析器输出槽（接收 IMU/能力帧数据）
 * 4. 进入 200Hz 主循环：
 *    a. 调用 handle_events() 收包+解析
 *    b. 检测新 IMU 帧 -> 统计时间戳间隔、打印数据
 *    c. 检测新能力帧 -> 打印开关状态
 *    d. 可选：下发正弦扫摆指令
 *    e. 每秒打印一次统计信息
 *    f. 断线自动重连
 * 5. 退出后打印验收报告
 */
int run_serial_test(const Options& opt) {
    // ------------------------------------------------------------------------
    // 第一步：注册信号处理函数
    // ------------------------------------------------------------------------
    
    /**
     * SIGINT: 终端中断信号（Ctrl+C）
     * SIGTERM: 终止信号（kill 命令默认发送）
     * 注册后按 Ctrl+C 不会立即退出，而是优雅地结束主循环
     */
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // ------------------------------------------------------------------------
    // 第二步：打印启动信息
    // ------------------------------------------------------------------------
    
    // 打印测试标题
    fmt::print("==== stage12 串口下位机实测 ====\n");
    // 打印设备路径和波特率
    fmt::print("设备: {} @ {}\n", opt.device, opt.baud);
    // 强制刷新 stdout 缓冲区，确保信息立即显示
    // 防止与后续 spdlog（输出到 stderr）日志顺序混乱
    std::fflush(stdout);

    // ------------------------------------------------------------------------
    // 第三步：连接真实下位机
    // ------------------------------------------------------------------------
    
    /**
     * McuDeviceHandle::create_serial()
     * 创建串口设备句柄，内部执行：
     *   - open() 打开串口设备
     *   - tcgetattr() 读取当前配置
     *   - cfmakeraw() 设置为原始模式（8N1）
     *   - tcsetattr() 应用配置
     *   - tcflush() 清空缓冲区
     * 返回 std::expected<McuDeviceHandle, std::string>
     *   - 成功：包含句柄
     *   - 失败：包含错误信息字符串
     */
    auto connected = tg::McuDeviceHandle::create_serial(opt.device, opt.baud);
    
    /**
     * if (!connected) 检查 std::expected 是否包含值
     * connected.error() 获取错误信息
     */
    if (!connected) {
        // 打印失败信息到 stderr（fmt::print(stderr, ...)）
        fmt::print(stderr, "\n[FAIL] 连接失败: {}\n", connected.error());
        // 打印排查建议
        fmt::print(stderr, "排查: 确认设备存在、权限正确、下位机已上电\n");
        return 1;  // 返回 1 表示失败
    }
    
    /**
     * std::move(*connected) 移动句柄（不能复制，只能移动）
     * 转移设备资源的所有权
     */
    auto handle = std::move(*connected);
    fmt::print("[ OK ] 已连接\n");

    // ------------------------------------------------------------------------
    // 第四步：绑定解析器输出槽
    // ------------------------------------------------------------------------
    
    /**
     * ReceiveImuData: IMU 数据帧结构（姿态角、角速度、时间戳）
     * ReceiveCapabilitiesData: 能力帧结构（following、power_rune、quanta）
     * 
     * Stm32Parser 是协议解析器，解析完成后把数据写到静态指针指向的位置
     * USB 和串口共用同一套解析器
     * 
     * 测试程序把指针指向本地变量，退出时恢复为 nullptr
     * 不污染全局状态，避免影响主程序
     */
    tg::ReceiveImuData imu{};                    // IMU 数据接收缓冲区
    tg::ReceiveCapabilitiesData caps{};          // 能力帧数据接收缓冲区
    
    // 把解析器的输出指针指向本地变量
    tg::Stm32Parser::latest_imu = &imu;          // IMU 数据输出目标
    tg::Stm32Parser::latest_capabilities = &caps; // 能力帧输出目标

    // ------------------------------------------------------------------------
    // 第五步：下发模式安全警告
    // ------------------------------------------------------------------------
    
    /**
     * 如果 send_hz > 0，表示会下发指令让云台转动
     * 必须打印醒目的警告横幅，提醒操作人员确认安全
     */
    if (opt.send_hz > 0.0) {
        fmt::print("\n!! 警告: 下发模式已开启，云台会真实转动 !!\n\n");
    }

    // ------------------------------------------------------------------------
    // 第六步：初始化统计变量
    // ------------------------------------------------------------------------
    
    RxStats stats;  // 统计数据全部初始化为 0
    
    /**
     * t_start: 测试开始时刻（steady_clock 单调时钟，不受系统时间调整影响）
     * steady_clock 适合测量时间间隔，system_clock 适合获取日历时间
     */
    const auto t_start = std::chrono::steady_clock::now();
    
    /**
     * next_tick: 下一个 5ms 节拍时刻
     * 用于 sleep_until 实现 200Hz 固定频率循环
     */
    auto next_tick = t_start;
    
    /**
     * next_stat: 下一个统计打印时刻
     * 每秒打印一次，初始为开始后 1 秒
     */
    auto next_stat = t_start + 1s;
    
    /**
     * next_reconnect: 下一个重连尝试时刻
     * 断开后每秒尝试一次重连
     */
    auto next_reconnect = t_start;
    
    double send_phase = 0.0;          // 下发相位累计（用于正弦扫摆）
    auto last_rx_time = t_start;       // 最后一次收到 IMU 帧的时刻
    
    uint32_t prev_stamp = 0;           // 上一帧 IMU 时间戳
    uint32_t prev_caps_stamp = 0;      // 上一帧能力帧时间戳
    
    bool have_frame = false;           // 是否已经收到过第一帧 IMU
    bool have_caps = false;            // 是否已经收到过第一帧能力帧

    // ------------------------------------------------------------------------
    // 第七步：200Hz 主循环
    // ------------------------------------------------------------------------
    
    /**
     * while (!g_stop): 
     *   g_stop 在信号处理函数中被置为 1
     *   按 Ctrl+C 或 kill 后退出循环
     */
    while (!g_stop) {
        // 获取当前时间
        const auto now = std::chrono::steady_clock::now();

        // --- 7a. 检查是否到达运行时长 ---
        /**
         * 如果用户指定了 -t 参数（duration_s > 0）
         * 计算已运行时间，超过则退出循环
         */
        if (opt.duration_s > 0.0) {
            // 计算从开始到现在的秒数（浮点数）
            double elapsed = std::chrono::duration<double>(now - t_start).count();
            if (elapsed >= opt.duration_s) {
                break;  // 到达设定时长，退出主循环
            }
        }

        // --- 7b. 事件轮询：收包 + 拆帧 + 解析 ---
        /**
         * handle_events() 是核心收包函数：
         *   1. 从串口读取原始字节流（read()）
         *   2. 在内部缓冲区中寻找 SOF 魔数（0xA5）
         *   3. 根据帧头中的 len 字段提取完整帧
         *   4. 校验 EOF 魔数（0x5A）
         *   5. 调用 Stm32Parser 解析帧内容
         *   6. 解析结果写入 Stm32Parser::latest_imu / latest_capabilities
         * 
         * 这个函数是非阻塞的：读到多少处理多少，读不到立即返回
         */
        handle.handle_events();

        // --- 7c. 检测新的 IMU 帧 ---
        /**
         * 条件1: imu.header.sof == HeaderFrame::SoF()
         *   检查帧头魔数是否为 0xA5，排除未初始化的零值
         * 
         * 条件2: (!have_frame || imu.time_stamp != prev_stamp)
         *   首次收到帧（!have_frame）或者时间戳发生了变化
         *   排除重复处理同一帧数据
         * 
         * 注意：Stm32Parser 的静态指针指向 imu，每收到新帧就会覆盖
         * 所以必须立即处理，否则下一帧会覆盖当前数据
         */
        if (imu.header.sof == tg::HeaderFrame::SoF() &&
            (!have_frame || imu.time_stamp != prev_stamp)) {
            
            /**
             * 如果已经收到过帧（have_frame == true）
             * 计算时间戳间隔，检测异常
             */
            if (have_frame) {
                // 当前时间戳 - 上一帧时间戳 = 帧间隔（毫秒）
                int64_t dt = static_cast<int64_t>(imu.time_stamp) - 
                             static_cast<int64_t>(prev_stamp);
                
                /**
                 * 异常检测：
                 * - dt <= 0: 时间戳回退或重复（MCU 复位或帧重复）
                 * - dt > 1000: 帧间隔超过 1 秒（丢帧或串口阻塞）
                 */
                if (dt <= 0 || dt > 1000) {
                    fmt::print("[WARN] 时间戳异常: dt={} ms\n", dt);
                    ++stats.anomalies;  // 异常计数 +1
                }
                
                // 统计间隔：累加用于计算平均值
                stats.dt_sum_ms += static_cast<double>(dt);
                // 更新最小间隔
                stats.dt_min_ms = std::min(stats.dt_min_ms, dt);
                // 更新最大间隔
                stats.dt_max_ms = std::max(stats.dt_max_ms, dt);
            }
            
            // 标记已收到帧
            have_frame = true;
            // 保存当前时间戳作为上一帧时间戳
            prev_stamp = imu.time_stamp;
            // 帧计数 +1
            ++stats.frames;
            // 更新最后收到帧的时刻（用于看门狗检测）
            last_rx_time = now;
            
            /**
             * print_imu() 打印 IMU 数据：
             * 包含：roll/pitch/yaw、角速度、时间戳
             * 这是 packet.hpp 提供的调试函数
             */
            tg::print_imu(imu);
        }

        // --- 7d. 检测新的能力帧 ---
        /**
         * 能力帧包含：following、power_rune、quanta 三个开关
         * 与 IMU 帧逻辑相同，但只在状态变化时打印（节约输出）
         */
        if (caps.header.sof == tg::HeaderFrame::SoF() &&
            (!have_caps || caps.time_stamp != prev_caps_stamp)) {
            
            have_caps = true;
            prev_caps_stamp = caps.time_stamp;
            ++stats.caps_frames;
            
            /**
             * 打印能力帧状态：
             * - following: 跟随模式开关
             * - power_rune: 能量机关开关
             * - quanta: 量子开关
             */
            fmt::print("\n[CAPS] stamp={} following={} power_rune={} quanta={}\n",
                       caps.time_stamp, caps.data.following, 
                       caps.data.power_rune, caps.data.quanta);
        }

        // --- 7e. 下发指令（可选）---
        /**
         * 条件1: opt.send_hz > 0.0  用户开启了下发模式
         * 条件2: handle.is_connected()  设备当前处于连接状态
         */
        if (opt.send_hz > 0.0 && handle.is_connected()) {
            /**
             * 相位累计：每个循环 5ms
             * phase += send_hz * 0.005
             * 当 send_hz=20Hz 时，相位每秒增加 20，6 秒增加 120（一个周期）
             */
            send_phase += opt.send_hz * 0.005;
            
            // 根据当前相位生成指令包
            auto packet = build_packet(send_phase);
            
            /**
             * send_sync() 同步发送指令：
             *   将 packet 转换为字节流发送
             *   阻塞等待发送完成
             *   超时时间默认 500ms
             * 返回 true 表示发送成功，false 表示失败
             */
            if (handle.send_sync(reinterpret_cast<const uint8_t*>(&packet), 
                                 sizeof(packet))) {
                ++stats.tx_ok;     // 发送成功 +1
            } else {
                ++stats.tx_fail;   // 发送失败 +1
            }
        }

        // --- 7f. 秒级统计信息 ---
        /**
         * 每秒打印一次统计信息
         * now >= next_stat 判断是否到达打印时刻
         */
        if (now >= next_stat) {
            // 下次打印时刻 = 当前 + 1秒
            next_stat += 1s;
            
            /**
             * age_ms: 帧龄 = 从最后一次收到 IMU 帧到现在的时间（毫秒）
             * 用于检测上行链路是否工作正常
             */
            double age_ms = std::chrono::duration<double, std::milli>(
                now - last_rx_time).count();
            
            /**
             * 打印统计信息：
             * - rx: 收到的总帧数
             * - 帧龄: 最后一次收到帧到现在的时间
             * - tx: 发送成功/失败次数
             * - 重连: 自动重连次数
             * - 异常: 时间戳异常次数
             */
            fmt::print("\n[STAT] rx={:>6} 帧 | 帧龄={:6.1f}ms | tx={}/{} | 重连={} | 异常={}\n",
                       stats.frames, age_ms, stats.tx_ok, stats.tx_fail, 
                       stats.reconnects, stats.anomalies);
            
            /**
             * 看门狗检测：
             * 如果超过 1 秒没有收到任何 IMU 帧，说明上行链路有问题
             */
            if (age_ms > 1000.0) {
                fmt::print("[WARN] 超过1秒未收到IMU帧\n");
            }
        }

        // --- 7g. 断线自动重连 ---
        /**
         * 条件1: !handle.is_connected()  设备已断开
         * 条件2: now >= next_reconnect   到达重连尝试时刻
         * 每秒尝试一次，避免过于频繁
         */
        if (!handle.is_connected() && now >= next_reconnect) {
            // 下次重连尝试时刻 = 当前 + 1秒
            next_reconnect = now + 1s;
            
            fmt::print("\n[WARN] 串口断开，尝试重连...\n");
            
            // 重新调用 create_serial 创建新连接
            auto re = tg::McuDeviceHandle::create_serial(opt.device, opt.baud);
            
            if (re) {
                // 重连成功：移动新句柄替换旧句柄
                handle = std::move(*re);
                ++stats.reconnects;  // 重连次数 +1
                fmt::print("[ OK ] 重连成功\n");
            }
            // 重连失败：什么也不做，下次循环再尝试
        }

        // --- 7h. 200Hz 固定节拍 ---
        /**
         * 每次循环结束，等待到下一个 5ms 节拍点
         * 实现精确的 200Hz 循环频率（1000ms / 200 = 5ms）
         * 
         * sleep_until() 优于 sleep_for()：
         *   不累积误差，循环耗时不会导致节拍漂移
         */
        next_tick += 5ms;   // 下一个节拍点 = 当前节拍点 + 5ms
        
        /**
         * sleep_until(绝对时间点) 阻塞到指定时刻
         * 如果已经过了该时刻（循环耗时 > 5ms），立即返回
         * 不会累积延迟
         */
        std::this_thread::sleep_until(next_tick);
    }  // while (!g_stop) 主循环结束

    // ------------------------------------------------------------------------
    // 第八步：清理并输出验收报告
    // ------------------------------------------------------------------------
    
    /**
     * 恢复解析器的静态指针为 nullptr
     * 防止程序退出后，主程序误用已销毁的指针
     * 不污染全局状态
     */
    tg::Stm32Parser::latest_imu = nullptr;
    tg::Stm32Parser::latest_capabilities = nullptr;

    /**
     * duration_s: 总运行时长（秒）
     * now() - t_start 计算时间间隔
     */
    double duration_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    
    /**
     * avg_fps: 平均帧率
     * 总帧数 / 运行时长（秒）
     */
    double avg_fps = duration_s > 0.0 ? stats.frames / duration_s : 0.0;
    
    /**
     * avg_dt: 平均帧间隔（毫秒）
     * 所有间隔之和 / 间隔数量（帧数 - 1）
     */
    double avg_dt = stats.frames > 1 ? stats.dt_sum_ms / (stats.frames - 1) : 0.0;

    /**
     * print_report() 打印完整的验收报告
     * 包含：运行时长、设备信息、帧统计、间隔统计、结论
     */
    print_report(opt, stats, duration_s, avg_fps, avg_dt);
    
    /**
     * 返回值：
     * - 0: 成功（至少收到了 1 帧 IMU 数据）
     * - 1: 失败（0 帧，上行链路不通）
     */
    return stats.frames == 0 ? 1 : 0;
}