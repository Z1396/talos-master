// 头文件保护，防止重复包含
#pragma once

// C++ 标准库
#include <iostream>         // 标准输出/错误流 std::cout / std::cerr
#include <memory>           // 智能指针 std::shared_ptr
#include <streambuf>        // 标准流缓冲区基类 std::streambuf
#include <string>           // 字符串
#include <utility>          // 移动语义 std::move
#include <vector>           // 动态数组，存放多个日志输出槽

// spdlog 日志库
#include <spdlog/sinks/rotating_file_sink.h>  // 滚动文件日志输出（按大小分割日志）
#include <spdlog/sinks/stdout_color_sinks.h>  // 彩色控制台输出
#include <spdlog/spdlog.h>                    // spdlog 核心接口

/**
 * @brief 自定义流缓冲区：将 std::cout / std::cerr 输出重定向到 spdlog
 * 继承标准流缓冲区基类 std::streambuf，拦截标准输出字符，拼接后交给 spdlog 打印
 * 作用：统一日志出口，让原生 cout/cerr 也能走日志分级、文件落盘、格式统一
 */
class spdlog_streambuf : public std::streambuf {
public:
    /**
     * @brief 构造函数
     * @param logger 绑定的 spdlog 日志器智能指针
     * @param name 日志来源标识（区分 cout / cerr）
     * @param lvl 对应 spdlog 日志级别
     */
    spdlog_streambuf(
        std::shared_ptr<spdlog::logger> logger, std::string_view name,
        const spdlog::level::level_enum lvl)
        : logger_(std::move(logger))  // 移动语义接管日志器，避免拷贝
        , level_(lvl)                 // 记录日志级别
        , name_(name)                 // 来源名称
        , buffer_()                   // 初始化行缓冲区
    {
        // 预分配 512 字节空间，适配常规单行日志长度，减少动态扩容开销
        buffer_.reserve(512);
    }

protected:
    /**
     * @brief 重写字符溢出回调
     * 标准流每输出一个字符都会进入该函数
     * @param ch 单个字符，eof 表示流结束
     * @return 原字符，维持流正常流转
     */
    int_type overflow(const int_type ch) override {
        // 不是流结束符，则追加字符到本地缓冲区
        if (ch != traits_type::eof()) {
            buffer_ += static_cast<char>(ch);
            // 遇到换行符，说明单行日志结束，立即刷写日志
            if (ch == '\n')
                flush_buffer();
        }
        return ch;
    }

    /**
     * @brief 重写同步回调
     * 调用 std::flush 时触发，强制刷写缓冲区
     * @return 固定返回 0 表示执行成功
     */
    int sync() override {
        flush_buffer();
        return 0;
    }

private:
    /**
     * @brief 刷写缓冲区核心逻辑
     * 将拼接好的单行内容交给 spdlog 输出，之后清空缓冲区
     */
    void flush_buffer() {
        if (!buffer_.empty()) {
            // 调用 spdlog 日志接口输出
            // source_loc：模拟源码位置，填入来源名称、行号1、无函数名
            logger_->log(spdlog::source_loc{name_.data(), 1, nullptr}, level_, buffer_);
            buffer_.clear(); // 清空缓冲区，准备接收下一行
        }
    }

    std::shared_ptr<spdlog::logger> logger_; // 绑定的 spdlog 日志器
    spdlog::level::level_enum level_;        // 当前流对应的日志级别
    std::string_view name_;                  // 流名称（cout / cerr），用于日志标识
    std::string buffer_;                     // 行缓冲区，临时拼接输出字符
};

/**
 * @brief 初始化全局日志器
 * @return 构建完成的 spdlog 日志器智能指针
 * @note noexcept 保证函数不会抛出异常
 * 功能：同时开启「彩色控制台输出」+「滚动文件日志」，设置统一日志格式
 */
inline std::shared_ptr<spdlog::logger> init_logger() noexcept {
    // 1. 创建【滚动文件输出槽】
    // 参数：日志文件名、单文件最大 5MB、最多保留 3 个历史日志文件
    const auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "logs/talos.log", 1024 * 1024 * 5, 3);

    // 2. 创建【彩色控制台输出槽】
    const auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // ========== 设置日志格式 ==========
    // 控制台格式：[时:分:秒.毫秒 日志级别 源码位置] 日志内容
    // %H:%M:%S.%e 时间 | %^%l%$ 彩色级别 | %@ 源码位置 | %v 日志正文
    console_sink->set_pattern("[%H:%M:%S.%e %^%l%$ %@] %v");

    // 文件日志格式：带完整年月日 + 微秒，颜色标记自动忽略
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%f %^%l%$ %@] %v");

    // 整合多个输出槽：日志同时打印到控制台 + 写入文件
    std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};

    // 3. 创建名为 "talos" 的主日志器
    const auto logger = std::make_shared<spdlog::logger>("talos", sinks.begin(), sinks.end());

    // 设置触发自动落盘的级别：info 及以上级别日志立即刷新写入文件
    logger->flush_on(spdlog::level::info);

    // 设置为 spdlog 全局默认日志器
    spdlog::set_default_logger(logger);

    return logger;
}

/**
 * @brief 钩子函数：劫持标准输出流
 * 将 std::cout、std::cerr 重定向到上面自定义的 spdlog 缓冲区
 * 此后原生 cout/cerr 等价于调用 spdlog 日志接口
 */
inline void hook_cstream() noexcept {
    // 获取全局默认日志器
    const std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();

    // 静态缓冲区：全局单例，生命周期跟随程序，避免反复创建销毁
    // cout 映射为 info 级别日志，来源标记为 "cout"
    static spdlog_streambuf cout_buf(logger, "cout", spdlog::level::info);
    // cerr 映射为 error 级别日志，来源标记为 "cerr"
    static spdlog_streambuf cerr_buf(logger, "cerr", spdlog::level::err);

    // 替换标准流的底层缓冲区，完成重定向
    std::cout.rdbuf(&cout_buf);
    std::cerr.rdbuf(&cerr_buf);
}