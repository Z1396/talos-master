#pragma once

#include <iostream>
#include <memory>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

class spdlog_streambuf : public std::streambuf {
public:
    spdlog_streambuf(
        std::shared_ptr<spdlog::logger> logger, std::string_view name,
        const spdlog::level::level_enum lvl)
        : logger_(std::move(logger))
        , level_(lvl)
        , name_(name)
        , buffer_() {
        buffer_.reserve(512); // Pre-allocate for typical log line size
    }

protected:
    int_type overflow(const int_type ch) override {
        if (ch != traits_type::eof()) {
            buffer_ += static_cast<char>(ch);
            if (ch == '\n')
                flush_buffer();
        }
        return ch;
    }

    int sync() override {
        flush_buffer();
        return 0;
    }

private:
    void flush_buffer() {
        if (!buffer_.empty()) {
            logger_->log(spdlog ::source_loc{name_.data(), 1, nullptr}, level_, buffer_);
            buffer_.clear();
        }
    }

    std::shared_ptr<spdlog::logger> logger_;
    spdlog::level::level_enum level_;
    std::string_view name_;
    std::string buffer_;
};

inline std::shared_ptr<spdlog::logger> init_logger() noexcept {
    const auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "logs/talos.log", 1024 * 1024 * 5, 3);
    const auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    console_sink->set_pattern("[%H:%M:%S.%e %^%l%$ %@] %v");
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%f %^%l%$ %@] %v");

    std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
    const auto logger = std::make_shared<spdlog::logger>("talos", sinks.begin(), sinks.end());
    logger->flush_on(spdlog::level::info);
    spdlog::set_default_logger(logger);
    return logger;
}

inline void hook_cstream() noexcept {
    const std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
    static spdlog_streambuf cout_buf(logger, "cout", spdlog::level::info);
    static spdlog_streambuf cerr_buf(logger, "cerr", spdlog::level::err);
    std::cout.rdbuf(&cout_buf);
    std::cerr.rdbuf(&cerr_buf);
}
