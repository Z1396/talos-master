
#pragma once

#include <iostream>
#include <memory>
#include <streambuf>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>


// 将 std::cout / std::cerr 接入 spdlog。
class spdlog_streambuf : public std::streambuf {
public:
    spdlog_streambuf(
        std::shared_ptr<spdlog::logger> logger,  // 日志记录器
        std::string_view name,                  // 日志名称
        spdlog::level::level_enum level)     // 日志级别
        : logger_(std::move(logger)),
          level_(level),
          name_(name)
    {
        // .reserve(512) 预分配 5124 字节的内存，避免频繁分配内存属于string类型的api调用
        buffer_.reserve(512);
    }

protected:
    // 收集字符，遇到换行符时提交当前日志。
    // int_type 是 std::streambuf 的字符类型，用于表示输入字符或 EOF。
    // override 关键字确保该方法是重写的，而不是默认的。强调编译器检查。
    int_type overflow(int_type ch) override
    {
        // traits_type 是 std::streambuf 的字符类型，用于处理字符。
        // eq_int_type 检查字符是否为 EOF。
        if (!traits_type::eq_int_type(ch, traits_type::eof())) {
            // to_char_type 将 int_type 转换为 char_type，用于存储在缓冲区中。
            buffer_ += traits_type::to_char_type(ch);
            // 检查字符是否为换行符。
            if (traits_type::to_char_type(ch) == '\n') {
                flush_buffer();
            }
        }
        // not_eof 返回字符的 int_type 表示，用于存储在缓冲区中。
        return traits_type::not_eof(ch);
    }

    // 响应 ostream 的 flush 请求。
    // sync 方法刷新缓冲区，确保所有日志都被写入。
    int sync() override
    {
        flush_buffer();
        return 0;
    }

private:
    // 提交日志缓冲区中的内容到 spdlog 日志记录器。
    void flush_buffer()
    {
        if (buffer_.empty()) {
            return;
        }
        // 提交日志到 spdlog 日志记录器。
        logger_->log(
            // source_loc 是 spdlog 的日志源位置，用于记录日志的来源文件和行号。
            // name_.data() 返回日志名称的指针，用于记录日志的来源文件。
            // 1 是日志的行号，用于记录日志的来源行。
            // nullptr 是日志的函数名，用于记录日志的来源函数。
            spdlog::source_loc{name_.data(), 1, nullptr},
            level_,     // 日志级别
            "{}",      // 日志格式
            buffer_    // 日志内容
        );
        // 清空日志缓冲区，准备下一次日志记录。
        buffer_.clear();
    }

    std::shared_ptr<spdlog::logger> logger_;  // 日志记录器
    spdlog::level::level_enum level_;        // 日志级别
    std::string_view name_;                  // 日志名称
    std::string buffer_;                    // 日志缓冲区
};


// 创建控制台和滚动文件 sink，并注册默认 logger。
inline std::shared_ptr<spdlog::logger> init_logger() noexcept
{
    // 单个文件最大 5 MiB，最多保留 3 个轮转文件。
    const auto file_sink =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            "logs/talos.log",
            1024 * 1024 * 5,
            3
        );

    // 控制台 sink，用于输出日志到标准输出。
    // stdout_color_sink_mt 是 spdlog 提供的控制台 sink，支持颜色输出。
    const auto console_sink =
        std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // 设置控制台 sink 的日志格式。
    // 日志格式包括时间、日志级别、日志来源文件、日志来源行、日志函数名、日志内容。
    // %H:%M:%S.%e 是时间格式，%^%l%$ 是日志级别格式，%@ 是日志来源文件格式。
    console_sink->set_pattern(
        "[%H:%M:%S.%e %^%l%$ %@] %v"
    );

    // 设置滚动文件 sink 的日志格式。
    // 日志格式包括时间、日志级别、日志来源文件、日志来源行、日志函数名、日志内容。
    // %Y-%m-%d 是日期格式，%H:%M:%S.%f 是时间格式，%^%l%$ 是日志级别格式，%@ 是日志来源文件格式。
    file_sink->set_pattern(
        "[%Y-%m-%d %H:%M:%S.%f %^%l%$ %@] %v"
    );

    // sinks 是 spdlog 的 sink 组合，用于输出日志到多个目标。
    // console_sink 是控制台 sink，用于输出日志到标准输出。
    // file_sink 是文件 sink，用于输出日志到文件。
    std::vector<spdlog::sink_ptr> sinks{
        console_sink,
        file_sink
    };

    const auto logger = std::make_shared<spdlog::logger>(
        "talos",
        sinks.begin(),
        sinks.end()
    );
    // 配置日志刷新策略：
    logger->flush_on(spdlog::level::info);

    // 注册为 spdlog 全局默认 logger。
    spdlog::set_default_logger(logger);

    return logger;
}


// 将标准输出流重定向到 spdlog。
inline void hook_cstream() noexcept
{
    // 获取默认 logger。(就是如果之前运行了 init_logger 函数，就会返回这个 logger。)
    // 如果未注册默认 logger，会创建一个默认的 logger。
    const auto logger = spdlog::default_logger();

    // 使用局部 static，保证 streambuf 生命周期长于函数调用。防止后面调用std::cout / std::cerr 时，streambuf 被销毁。
    /*这就是局部 static 的核心作用：
    变量虽然定义在函数内部，但它的生命周期不再受函数调用结束的影响。*/
    // cout / cerr 会一直持有这两个对象的地址。
    static spdlog_streambuf cout_buf(
        logger,
        "cout",
        spdlog::level::info
    );

    static spdlog_streambuf cerr_buf(
        logger,
        "cerr",
        spdlog::level::err
    );

    // 重定向 cout / cerr 到 spdlog_streambuf。
    // cout / cerr 会将所有输出重定向到 spdlog_streambuf 中。
    // .rdbuf() 方法返回 cout / cerr 的缓冲区指针，用于将 cout / cerr 的输出重定向到 spdlog_streambuf 中。
    std::cout.rdbuf(&cout_buf);
    std::cerr.rdbuf(&cerr_buf);
}
