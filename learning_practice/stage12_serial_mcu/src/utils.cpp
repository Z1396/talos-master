// ============================================================================
// utils.cpp - 工具函数实现
// 负责：列出串口设备、打印验收报告
// ============================================================================

#include "utils.hpp"           // 本模块头文件：声明 list_serial_devices、print_report
#include <filesystem>          // std::filesystem::directory_iterator 遍历 /dev 目录
#include <fmt/core.h>          // fmt::print 格式化输出

// ============================================================================
// 辅助函数：判断字符串是否以指定前缀开头
// ============================================================================

/**
 * @brief 判断字符串是否以指定前缀开头
 * @param str   要检查的字符串
 * @param prefix 前缀字符串
 * @return true 如果 str 以 prefix 开头，否则 false
 * 
 * 实现原理：str.rfind(prefix, 0) == 0
 *   rfind() 从指定位置开始逆向查找子串
 *   参数 0 表示从字符串开头开始查找
 *   如果 prefix 在位置 0 找到，说明 str 以 prefix 开头
 * 
 * 为什么不用 std::string::starts_with()？
 *   因为 GCC 11 不支持 C++20 的 starts_with()
 *   用 rfind 是兼容 C++17 的写法
 * 
 * 示例：
 *   starts_with("/dev/ttyS0", "ttyS")  -> true
 *   starts_with("/dev/ttyUSB0", "ttyS") -> false
 */
static bool starts_with(const std::string& str, const std::string& prefix) {
    // rfind(prefix, 0) 从位置 0 开始查找 prefix
    // 返回 0 表示 prefix 在位置 0 匹配（即在字符串开头）
    return str.rfind(prefix, 0) == 0;
}

// ============================================================================
// 列出本机所有候选串口设备
// ============================================================================

/**
 * @brief 扫描 /dev 目录，列出所有串口设备
 * 
 * 扫描的串口设备类型：
 *   ttyS*   - 主板原生硬件串口 UART（如 /dev/ttyS0、/dev/ttyS4）
 *   ttyUSB* - USB 转串口芯片（CH340、CP2102、FTDI）
 *   ttyACM* - CDC-ACM 虚拟串口（STM32 USB 虚拟串口、Arduino）
 *   ttyAMA* - 树莓派 / ARM 板载串口
 * 
 * 权限提示：串口设备默认属于 dialout 组
 *   需要加入 dialout 组才能读写串口
 */
void list_serial_devices() {
    // 打印标题
    fmt::print("本机候选串口设备（/dev/ttyS* / ttyUSB* / ttyACM* / ttyAMA*）:\n");
    
    /**
     * std::error_code ec：错误码对象
     * 如果 directory_iterator 遍历出错，错误信息存入 ec
     * 不会抛出异常（std::filesystem 默认可能会抛异常）
     */
    std::error_code ec;
    
    /**
     * std::filesystem::directory_iterator("/dev", ec)
     *   遍历 /dev 目录下的所有条目
     *   返回迭代器，每次迭代一个目录项 entry
     *   entry 的类型是 std::filesystem::directory_entry
     */
    for (const auto& entry : std::filesystem::directory_iterator("/dev", ec)) {
        /**
         * entry.path()          返回 std::filesystem::path 对象
         * .filename()           返回路径中的文件名部分（不含目录）
         * .string()             转换为 std::string
         * 
         * 例如：entry.path() = "/dev/ttyS0"
         *       filename() = "ttyS0"
         *       string() = "ttyS0"
         */
        const auto name = entry.path().filename().string();
        
        /**
         * 检查文件名是否以串口设备前缀开头
         * starts_with() 自定义函数，兼容 C++17
         */
        if (starts_with(name, "ttyS") ||     // 硬件串口
            starts_with(name, "ttyUSB") ||   // USB 转串口
            starts_with(name, "ttyACM") ||   // CDC-ACM 虚拟串口
            starts_with(name, "ttyAMA")) {   // ARM 板载串口
            
            // 打印完整设备路径
            fmt::print("  {}\n", entry.path().string());
        }
    }
    
    // 打印权限提示
    fmt::print("权限提示: 无权打开时执行 sudo usermod -aG dialout $USER 并重新登录\n");
}

// ============================================================================
// 打印验收报告
// ============================================================================

/**
 * @brief 打印完整的串口下位机验收报告
 * @param opt         命令行选项（设备路径、波特率等）
 * @param stats       统计信息（帧数、异常数、重连次数等）
 * @param duration_s  运行时长（秒）
 * @param avg_fps     平均帧率（帧/秒）
 * @param avg_dt      平均帧间隔（毫秒）
 * 
 * 报告内容：
 *   1. 运行时长
 *   2. 设备信息（路径、波特率、重连次数）
 *   3. IMU 帧统计（总帧数、平均帧率）
 *   4. MCU 时间戳间隔（平均值、最小值、最大值）
 *   5. 能力帧统计
 *   6. 下行指令帧统计（如果开启）
 *   7. 异常统计
 *   8. 验收结论（PASS / WARN / FAIL）
 */
void print_report(const Options& opt, const RxStats& stats,
                  double duration_s, double avg_fps, double avg_dt) {
    
    // ========================================================================
    // 报告标题
    // ========================================================================
    fmt::print("\n==================== 串口下位机验收报告 ====================\n");
    
    // ========================================================================
    // 运行时长
    // ========================================================================
    /**
     * {:8.1f} 格式：宽度 8 字符，保留 1 位小数
     * 例如：    "     5.0 s"
     */
    fmt::print("运行时长          : {:8.1f} s\n", duration_s);
    
    // ========================================================================
    // 设备信息
    // ========================================================================
    /**
     * opt.device:     设备路径，如 /dev/ttyS4
     * opt.baud:       波特率，如 115200
     * stats.reconnects: 自动重连次数
     */
    fmt::print("设备              : {} @ {}（重连 {} 次）\n",
               opt.device, opt.baud, stats.reconnects);
    
    // ========================================================================
    // IMU 帧统计
    // ========================================================================
    /**
     * stats.frames:   收到的 IMU 帧总数
     * avg_fps:        平均帧率 = 总帧数 / 运行秒数
     * 
     * {:8} 格式：宽度 8 字符，右对齐
     * {:7.1f} 格式：宽度 7 字符，保留 1 位小数
     */
    fmt::print("上行 IMU 帧       : {:8} 帧，平均 {:7.1f} fps\n", stats.frames, avg_fps);
    
    // ========================================================================
    // MCU 时间戳间隔统计
    // ========================================================================
    /**
     * avg_dt: 平均帧间隔（毫秒）
     * 正常值：约 5ms（200Hz 下位机发送频率）
     */
    fmt::print("MCU 时间戳间隔    : avg {:6.2f} ms", avg_dt);
    
    /**
     * 只有在收到至少 2 帧时才有 min/max 数据
     * stats.frames > 1 表示至少 2 帧
     * stats.dt_min_ms: 最小帧间隔
     * stats.dt_max_ms: 最大帧间隔
     */
    if (stats.frames > 1) {
        fmt::print(" / min {} ms / max {} ms\n", stats.dt_min_ms, stats.dt_max_ms);
    } else {
        // 只有 1 帧或 0 帧，无法计算 min/max
        fmt::print("\n");
    }
    
    // ========================================================================
    // 能力帧统计
    // ========================================================================
    /**
     * stats.caps_frames: 收到的能力帧总数
     * 能力帧包含：following、power_rune、quanta 开关状态
     */
    fmt::print("能力帧            : {:8} 帧\n", stats.caps_frames);
    
    // ========================================================================
    // 下行指令帧统计（可选）
    // ========================================================================
    /**
     * opt.send_hz > 0.0 表示用户开启了下发测试
     * stats.tx_ok:   发送成功次数
     * stats.tx_fail: 发送失败次数
     */
    if (opt.send_hz > 0.0) {
        fmt::print("下行指令帧        : 成功 {} / 失败 {}\n", stats.tx_ok, stats.tx_fail);
    } else {
        // 未开启下发测试
        fmt::print("下行指令帧        : 未开启（-s <hz> 开启下发测试）\n");
    }
    
    // ========================================================================
    // 异常统计
    // ========================================================================
    /**
     * stats.anomalies: 异常次数
     * 异常类型：
     *   - 时间戳回退（dt <= 0）
     *   - 时间戳跳变过大（dt > 1000ms）
     *   - 帧魔数错误（SOF/EOF 不匹配）
     */
    fmt::print("时间戳/魔数异常   : {:8} 次\n", stats.anomalies);
    
    // ========================================================================
    // 验收结论
    // ========================================================================
    
    /**
     * tx_pass: 下发测试是否通过
     * 条件1: opt.send_hz <= 0.0  未开启下发测试 → 自动通过
     * 条件2: stats.tx_fail == 0  开启了下发测试，且没有失败 → 通过
     */
    const bool tx_pass = opt.send_hz <= 0.0 || stats.tx_fail == 0;
    
    /**
     * 三档判定逻辑：
     * 
     * 第一档：FAIL
     *   stats.frames == 0  没有收到任何 IMU 帧
     *   → 上行链路完全不通
     * 
     * 第二档：PASS（完美）
     *   stats.anomalies == 0       无任何异常
     *   tx_pass == true            下发测试通过
     *   stats.reconnects == 0      无重连（链路稳定）
     *   → 完美通过
     * 
     * 第三档：PASS（有瑕疵）
     *   stats.anomalies <= 3       异常次数不超过 3 次
     *   tx_pass == true            下发测试通过
     *   → 通过，但建议排查问题
     * 
     * 第四档：WARN
     *   异常次数超过 3 次 或 下发测试失败
     *   → 警告，需要排查
     */
    if (stats.frames == 0) {
        // 没有任何 IMU 帧 → 完全失败
        fmt::print("结论              : FAIL —— 未收到任何 IMU 帧，上行链路不通\n");
        
    } else if (stats.anomalies == 0 && tx_pass && stats.reconnects == 0) {
        // 完美通过
        fmt::print("结论              : PASS —— 收发稳定，无异常记录\n");
        
    } else if (stats.anomalies <= 3 && tx_pass) {
        // 有少量异常，但可接受
        fmt::print("结论              : PASS（含 {} 处异常/重连，建议排查）\n",
                   stats.anomalies + stats.reconnects);
        
    } else {
        // 异常过多或下发测试失败
        fmt::print("结论              : WARN —— 异常较多（{} 次），建议排查\n",
                   stats.anomalies + stats.reconnects);
    }
    
    // ========================================================================
    // 报告结束分隔线
    // ========================================================================
    fmt::print("============================================================\n");
}