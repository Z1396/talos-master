// C标准可变参数头文件 va_list / va_start / va_end / va_copy
#include <cstdarg>
// C标准IO 底层打印函数 fprintf
#include <cstdio>
// C++标准异常基类捕获
#include <exception>
// 存储格式化日志字符串
#include <string>
// 动态字符缓冲区，存放vsnprintf格式化输出
#include <vector>

// spdlog 高性能日志库核心头文件，用于统一接管Axera SDK日志输出
#include <spdlog/spdlog.h>

// Axera 平台SDK 日志系统头文件，定义日志等级、回调原型、宏
#include "ax_sys_log.h"

// 编译器宏判断：GCC / Clang 编译器
#if defined(__GNUC__) || defined(__clang__)
// 动态库导出符号标记，对外暴露C接口给SDK调用
# define TALOS_AX_LOG_EXPORT __attribute__((visibility("default")))
// printf格式检查注解：编译器静态校验可变参数格式化字符串匹配
// format_index: 格式化字符串参数位置；first_arg_index: 可变参数起始位置，0代表va_list
# define TALOS_AX_PRINTF_FORMAT(format_index, first_arg_index) \
     __attribute__((format(printf, format_index, first_arg_index)))
#else
// MSVC/其他编译器不支持GCC属性，置空定义
# define TALOS_AX_LOG_EXPORT
# define TALOS_AX_PRINTF_FORMAT(format_index, first_arg_index)
#endif

namespace { // 匿名命名空间，内部工具函数仅当前编译单元可见，不对外导出

/**
 * @brief 可变参数格式化日志字符串，生成去除首尾换行的标准日志文本
 * @param format printf风格格式化字符串
 * @param args 可变参数列表 va_list
 * @return 格式化完成的std::string，自动剔除末尾\r\n换行符
 * TALOS_AX_PRINTF_FORMAT(1,0)：第1个参数是格式化串，可变参数从va_list(0)开始
 */
std::string format_ax_message(const char* format, va_list args) TALOS_AX_PRINTF_FORMAT(1, 0);

std::string format_ax_message(const char* format, va_list args) {
    // 空格式化串直接返回空字符串
    if (format == nullptr) {
        return {};
    }

    va_list size_args;
    // 复制一份va_list用于先计算所需缓冲区长度，不破坏原args
    va_copy(size_args, args);
    // vsnprintf 传空缓冲区，仅计算输出字符串总字节长度（不含终止'\0'）
    const int required = std::vsnprintf(nullptr, 0, format, size_args);
    // 释放复制的va_list资源
    va_end(size_args);

    // 格式化失败（返回负数），直接返回原始format字符串兜底
    if (required < 0) {
        return format;
    }

    // 分配缓冲区：字符串长度 + 1字节字符串结束符
    std::vector<char> buffer(static_cast<std::size_t>(required) + 1U);
    va_list write_args;
    // 再次复制原始va_list用于实际格式化写入缓冲区
    va_copy(write_args, args);
    // 执行格式化，写入预分配缓冲区
    std::vsnprintf(buffer.data(), buffer.size(), format, write_args);
    va_end(write_args);

    // 转为C++字符串
    std::string message(buffer.data(), static_cast<std::size_t>(required));
    // 循环删除字符串末尾所有换行/回车符，统一日志行格式
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.pop_back();
    }
    return message;
}

/**
 * @brief Axera SDK日志等级 映射到 spdlog 内部日志等级枚举
 * @param level AX_S32 类型SDK日志等级常量
 * @return spdlog::level::level_enum 对应spdlog等级
 * noexcept 无异常抛出
 */
spdlog::level::level_enum to_spdlog_level(const AX_S32 level) noexcept {
    switch (level) {
    // 紧急/告警/严重错误 → spdlog critical 致命级日志
    case SYS_LOG_EMERGENCY:
    case SYS_LOG_ALERT:
    case SYS_LOG_CRITICAL: return spdlog::level::critical;
    // 普通错误 → err 错误级
    case SYS_LOG_ERROR: return spdlog::level::err;
    // 警告 → warn 警告级
    case SYS_LOG_WARN: return spdlog::level::warn;
    // 通知/普通信息 → info 信息级
    case SYS_LOG_NOTICE:
    case SYS_LOG_INFO: return spdlog::level::info;
    // 调试日志 → debug 调试级
    case SYS_LOG_DEBUG: return spdlog::level::debug;
    // 未知等级默认降级info
    default: return spdlog::level::info;
    }
}

/**
 * @brief 核心日志转发函数：格式化文本 + 调用spdlog输出，兜底标准错误打印
 * @param level SDK日志等级数值
 * @param tag 日志分类标签（模块名）
 * @param id 模块实例ID，-1代表无ID
 * @param format 原始格式化串
 * @param args 可变参数列表
 * TALOS_AX_PRINTF_FORMAT(4,0)：第4参数为格式化串，参数来自va_list
 */
void log_ax_message(AX_S32 level, const char* tag, int id, const char* format, va_list args)
    TALOS_AX_PRINTF_FORMAT(4, 0);

void log_ax_message(
    const AX_S32 level, const char* tag, const int id, const char* format, va_list args) {
    std::string message;

    try {
        // 第一步：格式化可变参数字符串
        message = format_ax_message(format, args);

        // 获取全局默认spdlog日志句柄
        auto* logger = spdlog::default_logger_raw();
        // 无spdlog实例，直接打印到stderr标准错误流兜底
        if (logger == nullptr) {
            std::fprintf(
                stderr, "[AX_SYS][%s:%d] %s\n", tag != nullptr ? tag : "-", id, message.c_str());
            return;
        }

        // 转换SDK日志等级为spdlog等级
        const auto spd_level = to_spdlog_level(level);
        // 伪造源码位置，统一标记日志来源为AX_SYS SDK重定向
        const spdlog::source_loc source{"ax_sys_log", 0, "AX_SYS"};

        // 分多分支拼接日志前缀：tag存在/不存在、id有效/无效
        if (tag != nullptr && tag[0] != '\0') {
            if (id >= 0) {
                // 有标签+实例ID：[tag:id] 日志内容
                logger->log(source, spd_level, "[{}:{}] {}", tag, id, message);
            } else {
                // 仅标签无ID：[tag] 日志内容
                logger->log(source, spd_level, "[{}] {}", tag, message);
            }
        } else {
            // 无标签，直接打印日志文本
            logger->log(source, spd_level, "{}", message);
        }
    } catch (const std::exception& ex) {
        // 捕获标准C++异常，打印兜底错误日志到stderr
        std::fprintf(
            stderr, "[AX_SYS][log-redirect-error] %s; original format=%s\n", ex.what(),
            format != nullptr ? format : "<null>");
    } catch (...) {
        // 捕获所有非标准异常，防止日志转发崩溃主程序
        std::fprintf(
            stderr, "[AX_SYS][log-redirect-error] unknown exception; original format=%s\n",
            format != nullptr ? format : "<null>");
    }
}

} // 匿名内部工具命名空间结束

// ===================== 对外导出C接口，供Axera SDK注册日志回调 =====================
/**
 * @brief Axera SDK 标准日志输出回调接口（基础版，无自定义tag/id）
 * SDK内部日志统一走该回调，转发至spdlog
 * @param target 日志输出目标，SYS_LOG_TARGET_NULL代表丢弃日志
 * @param level SDK日志等级枚举
 * @param format printf格式化字符串
 * @param vlist 可变参数va_list
 */
extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* format, va_list vlist)
    TALOS_AX_PRINTF_FORMAT(3, 0);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* format, va_list vlist) {
    // 输出目标为空，直接丢弃日志不处理
    if (target == SYS_LOG_TARGET_NULL) {
        return;
    }
    // 转发至核心日志处理函数，使用系统默认日志标签，无实例ID(-1)
    log_ax_message(static_cast<AX_S32>(level), AX_MSYS_LOG_TAG, -1, format, vlist);
}

/**
 * @brief Axera SDK 扩展日志回调接口，支持自定义模块tag、实例ID
 * @param target 输出目标
 * @param level 日志等级
 * @param tag 自定义模块标签
 * @param id 硬件/模块实例编号
 * @param format 格式化串
 * @param vlist 可变参数列表
 */
extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput_Ex(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* tag, int id, AX_CHAR const* format,
    va_list vlist) TALOS_AX_PRINTF_FORMAT(5, 0);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput_Ex(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* tag, int id, AX_CHAR const* format,
    va_list vlist) {
    // 空输出目标直接丢弃
    if (target == SYS_LOG_TARGET_NULL) {
        return;
    }
    // 携带自定义tag与实例ID转发日志
    log_ax_message(static_cast<AX_S32>(level), tag, id, format, vlist);
}

/**
 * @brief 对外同步打印日志C接口，可变参数直接传入（非va_list形式）
 * @param level 日志等级
 * @param format 格式化字符串
 * @param ... 可变参数
 */
extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogPrint(AX_S32 level, AX_CHAR const* format, ...)
    TALOS_AX_PRINTF_FORMAT(2, 3);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogPrint(AX_S32 level, AX_CHAR const* format, ...) {
    va_list args;
    // 初始化可变参数列表，从format之后开始读取参数
    va_start(args, format);
    log_ax_message(level, AX_MSYS_LOG_TAG, -1, format, args);
    // 释放可变参数栈资源
    va_end(args);
}

/**
 * @brief 扩展打印接口，支持自定义tag、实例ID + 可变参数
 * @param level 日志等级
 * @param tag 模块标签
 * @param id 实例ID
 * @param format 格式化串
 * @param ... 可变参数
 */
extern "C" TALOS_AX_LOG_EXPORT AX_VOID
    AX_SYS_LogPrint_Ex(AX_S32 level, AX_CHAR const* tag, int id, AX_CHAR const* format, ...)
        TALOS_AX_PRINTF_FORMAT(4, 5);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID
    AX_SYS_LogPrint_Ex(AX_S32 level, AX_CHAR const* tag, int id, AX_CHAR const* format, ...) {
    va_list args;
    va_start(args, format);
    log_ax_message(level, tag, id, format, args);
    va_end(args);
}

// 取消编译器专属宏定义，避免污染后续编译单元
#undef TALOS_AX_PRINTF_FORMAT
#undef TALOS_AX_LOG_EXPORT