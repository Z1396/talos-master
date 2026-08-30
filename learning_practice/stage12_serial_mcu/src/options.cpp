// ============================================================================
// options.cpp - 命令行参数解析
// 负责：解析用户输入的参数、校验合法性、打印帮助信息
// ============================================================================

#include "options.hpp"       // 本模块头文件：声明 Options 结构体和解析函数
#include <cstdlib>           // std::exit() 退出程序
#include <fmt/core.h>        // fmt::print 格式化输出（替代 std::cout）

// ============================================================================
// 打印帮助信息
// ============================================================================

/**
 * @brief 打印程序使用说明
 * @param prog 程序名称（argv[0]）
 * 
 * 输出所有支持的命令行选项及其说明
 * 用户通过 -h 或 --help 触发
 * 
 * 格式说明：
 *   fmt::print() 支持 printf 风格的格式字符串
 *   {} 是占位符，会被后面的参数替换
 *   \n 表示换行
 *   prog 会填充到第一个 {} 位置
 */
void print_usage(const char* prog) {
    fmt::print(
        "用法: {} [选项]\n"
        "  -d, --device <path>   串口设备路径（默认 /dev/ttyS4）\n"
        "  -b, --baud <rate>     波特率（默认 115200）\n"
        "  -t, --duration <sec>  运行秒数，0 = 直到 Ctrl+C（默认 0）\n"
        "  -s, --send-hz <hz>    指令下发频率（默认 0 = 关闭）\n"
        "  --list                列出本机候选串口设备后退出\n"
        "  -h, --help            显示本帮助\n",
        prog);
}

// ============================================================================
// 解析命令行参数
// ============================================================================

/**
 * @brief 解析命令行参数并返回 Options 结构体
 * @param argc 参数个数（main 函数传入）
 * @param argv 参数数组（main 函数传入）
 * @return Options 结构体，包含所有解析后的选项
 * 
 * 解析流程：
 *   1. 遍历 argv[1] 到 argv[argc-1]
 *   2. 识别选项名称（如 -d、--device）
 *   3. 如果选项需要取值，读取下一个参数作为值
 *   4. 存储到 Options 结构体
 *   5. 校验合法性（波特率白名单、非负数检查）
 * 
 * 错误处理：
 *   - 选项缺少取值：打印错误并 exit(2)
 *   - 未知参数：打印错误并 exit(2)
 *   - 取值非法（如字符串转数字失败）：捕获异常并 exit(2)
 *   - 波特率不在白名单：打印警告但不退出
 *   - 时长/频率为负数：打印错误并 exit(2)
 */
Options parse_args(int argc, char* argv[]) {
    // 创建 Options 结构体，使用默认值初始化
    // 默认值在 options.hpp 中定义
    Options opt;
    
    /**
     * 从 i=1 开始遍历（跳过 argv[0]，即程序名称）
     * 例如：./serial_mcu_demo -d /dev/ttyS0 -b 460800
     *       i=1 时 arg = "-d"
     *       i=3 时 arg = "-b"
     */
    for (int i = 1; i < argc; ++i) {
        // 当前参数（std::string 方便比较）
        const std::string arg = argv[i];
        
        /**
         * next_value: Lambda 函数，用于获取选项的参数值
         * 例如：-d /dev/ttyS0，获取 "/dev/ttyS0"
         * 
         * [&] 捕获所有外部变量（按引用）
         * (const char* name) 参数是选项名称（用于错误信息）
         * 
         * 检查：如果 i+1 超出 argc，说明缺少取值
         * 例如：./serial_mcu_demo -d   (后面没有路径)
         */
        auto next_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                // 打印错误信息到 stderr
                fmt::print(stderr, "错误: 参数 {} 缺少取值\n", name);
                // 退出程序，返回码 2 表示命令行用法错误
                std::exit(2);
            }
            // 返回下一个参数（并移动 i）
            return argv[++i];
        };
        
        /**
         * try-catch 捕获 std::stoi() 和 std::stod() 的异常
         * 这些函数在转换失败时会抛出 std::invalid_argument
         */
        try {
            // ---------- 解析 -d 或 --device：设备路径 ----------
            if (arg == "-d" || arg == "--device") {
                opt.device = next_value("-d");
            }
            // ---------- 解析 -b 或 --baud：波特率 ----------
            else if (arg == "-b" || arg == "--baud") {
                // std::stoi() 将字符串转换为 int
                opt.baud = std::stoi(next_value("-b"));
            }
            // ---------- 解析 -t 或 --duration：运行时长（秒） ----------
            else if (arg == "-t" || arg == "--duration") {
                // std::stod() 将字符串转换为 double
                opt.duration_s = std::stod(next_value("-t"));
            }
            // ---------- 解析 -s 或 --send-hz：下发频率（Hz） ----------
            else if (arg == "-s" || arg == "--send-hz") {
                opt.send_hz = std::stod(next_value("-s"));
            }
            // ---------- 解析 --list：仅列出设备 ----------
            else if (arg == "--list") {
                opt.list_only = true;  // 设置为 true，主程序检测后执行 list 并退出
            }
            // ---------- 解析 -h 或 --help：显示帮助 ----------
            else if (arg == "-h" || arg == "--help") {
                print_usage(argv[0]);  // 打印帮助信息
                std::exit(0);          // 正常退出（返回码 0）
            }
            // ---------- 未知参数 ----------
            else {
                fmt::print(stderr, "错误: 未知参数 {}\n", arg);
                print_usage(argv[0]);  // 打印帮助信息
                std::exit(2);          // 退出
            }
        } catch (const std::exception& e) {
            /**
             * 捕获 std::stoi/std::stod 转换异常
             * e.what() 返回异常信息
             * 例如：用户输入 -b abc，std::stoi("abc") 抛出异常
             */
            fmt::print(stderr, "错误: 参数 {} 取值非法（{}）\n", arg, e.what());
            std::exit(2);
        }
    }
    
    // ========================================================================
    // 波特率白名单校验
    // ========================================================================
    
    /**
     * 串口支持的标准波特率列表
     * 对应 serial.hpp 中 baud_to_speed() 函数的 switch 分支
     * 
     * 注意：
     *   如果不匹配，程序会打印警告但不会退出
     *   实际串口打开时，baud_to_speed() 会回退为 115200
     *   这里提前警告让用户知道
     */
    switch (opt.baud) {
    case 9600: case 19200: case 38400: case 57600:
    case 115200: case 230400: case 460800: case 921600:
        break;  // 在白名单中，什么也不做
    default:
        // 不在白名单：打印警告，但不退出
        fmt::print(stderr, "警告: 波特率 {} 不在支持列表，将被回退为 115200\n", opt.baud);
        break;
    }
    
    // ========================================================================
    // 非负数校验
    // ========================================================================
    
    /**
     * duration_s 和 send_hz 应该是非负数
     * 如果为负数，打印错误并退出
     */
    if (opt.duration_s < 0.0 || opt.send_hz < 0.0) {
        fmt::print(stderr, "错误: 时长/频率不允许为负数\n");
        std::exit(2);
    }
    
    // 返回解析后的 Options 结构体
    return opt;
}