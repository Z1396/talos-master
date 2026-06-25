// 引入当前模块的错误头文件，声明panic_message函数原型
#include "scheduler/error.hpp"

// 引入spdlog日志库，用于打印致命错误日志
#include <spdlog/spdlog.h>

// 引入标准库进程终止函数 std::abort
#include <cstdlib>

// 嵌套命名空间：talos框架 -> scheduler调度器模块 -> detail内部实现细节（对外隐藏）
namespace talos::scheduler::detail {

/**
 * @brief 底层致命错误处理实现函数
 * @param message 拼接完成的完整崩溃报错字符串
 * @ noexcept 函数不会抛出C++异常
 * 功能：打印致命日志 + 直接终止整个程序进程，无恢复可能
 */
void panic_message(std::string message) noexcept {
    // SPDLOG_CRITICAL：spdlog最高级别日志（致命错误）
    // 会输出红色高亮日志、携带时间戳、线程号，写入控制台/日志文件/自定义sink（如FoxgloveSink）
    SPDLOG_CRITICAL("{}", message);

    // 标准库进程终止函数：
    // 1. 触发SIGABRT信号
    // 2. 程序立即退出，返回非0异常退出码
    // 3. 会生成core dump核心转储文件（系统开启dump时），方便事后调试崩溃堆栈
    // 4. 不会执行对象析构、atexit回调，属于硬终止，用于不可修复严重错误
    std::abort();
}

} // namespace talos::scheduler::detail