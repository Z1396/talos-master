#include <cstdarg>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include "ax_sys_log.h"

#if defined(__GNUC__) || defined(__clang__)
# define TALOS_AX_LOG_EXPORT __attribute__((visibility("default")))
# define TALOS_AX_PRINTF_FORMAT(format_index, first_arg_index) \
     __attribute__((format(printf, format_index, first_arg_index)))
#else
# define TALOS_AX_LOG_EXPORT
# define TALOS_AX_PRINTF_FORMAT(format_index, first_arg_index)
#endif

namespace {

std::string format_ax_message(const char* format, va_list args) TALOS_AX_PRINTF_FORMAT(1, 0);

std::string format_ax_message(const char* format, va_list args) {
    if (format == nullptr) {
        return {};
    }

    va_list size_args;
    va_copy(size_args, args);
    const int required = std::vsnprintf(nullptr, 0, format, size_args);
    va_end(size_args);

    if (required < 0) {
        return format;
    }

    std::vector<char> buffer(static_cast<std::size_t>(required) + 1U);
    va_list write_args;
    va_copy(write_args, args);
    std::vsnprintf(buffer.data(), buffer.size(), format, write_args);
    va_end(write_args);

    std::string message(buffer.data(), static_cast<std::size_t>(required));
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.pop_back();
    }
    return message;
}

spdlog::level::level_enum to_spdlog_level(const AX_S32 level) noexcept {
    switch (level) {
    case SYS_LOG_EMERGENCY:
    case SYS_LOG_ALERT:
    case SYS_LOG_CRITICAL: return spdlog::level::critical;
    case SYS_LOG_ERROR: return spdlog::level::err;
    case SYS_LOG_WARN: return spdlog::level::warn;
    case SYS_LOG_NOTICE:
    case SYS_LOG_INFO: return spdlog::level::info;
    case SYS_LOG_DEBUG: return spdlog::level::debug;
    default: return spdlog::level::info;
    }
}

void log_ax_message(AX_S32 level, const char* tag, int id, const char* format, va_list args)
    TALOS_AX_PRINTF_FORMAT(4, 0);

void log_ax_message(
    const AX_S32 level, const char* tag, const int id, const char* format, va_list args) {
    std::string message;

    try {
        message = format_ax_message(format, args);

        auto* logger = spdlog::default_logger_raw();
        if (logger == nullptr) {
            std::fprintf(
                stderr, "[AX_SYS][%s:%d] %s\n", tag != nullptr ? tag : "-", id, message.c_str());
            return;
        }

        const auto spd_level = to_spdlog_level(level);
        const spdlog::source_loc source{"ax_sys_log", 0, "AX_SYS"};

        if (tag != nullptr && tag[0] != '\0') {
            if (id >= 0) {
                logger->log(source, spd_level, "[{}:{}] {}", tag, id, message);
            } else {
                logger->log(source, spd_level, "[{}] {}", tag, message);
            }
        } else {
            logger->log(source, spd_level, "{}", message);
        }
    } catch (const std::exception& ex) {
        std::fprintf(
            stderr, "[AX_SYS][log-redirect-error] %s; original format=%s\n", ex.what(),
            format != nullptr ? format : "<null>");
    } catch (...) {
        std::fprintf(
            stderr, "[AX_SYS][log-redirect-error] unknown exception; original format=%s\n",
            format != nullptr ? format : "<null>");
    }
}

} // namespace

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* format, va_list vlist)
    TALOS_AX_PRINTF_FORMAT(3, 0);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* format, va_list vlist) {
    if (target == SYS_LOG_TARGET_NULL) {
        return;
    }

    log_ax_message(static_cast<AX_S32>(level), AX_MSYS_LOG_TAG, -1, format, vlist);
}

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput_Ex(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* tag, int id, AX_CHAR const* format,
    va_list vlist) TALOS_AX_PRINTF_FORMAT(5, 0);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogOutput_Ex(
    AX_LOG_TARGET_E target, AX_LOG_LEVEL_E level, AX_CHAR const* tag, int id, AX_CHAR const* format,
    va_list vlist) {
    if (target == SYS_LOG_TARGET_NULL) {
        return;
    }

    log_ax_message(static_cast<AX_S32>(level), tag, id, format, vlist);
}

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogPrint(AX_S32 level, AX_CHAR const* format, ...)
    TALOS_AX_PRINTF_FORMAT(2, 3);

extern "C" TALOS_AX_LOG_EXPORT AX_VOID AX_SYS_LogPrint(AX_S32 level, AX_CHAR const* format, ...) {
    va_list args;
    va_start(args, format);
    log_ax_message(level, AX_MSYS_LOG_TAG, -1, format, args);
    va_end(args);
}

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

#undef TALOS_AX_PRINTF_FORMAT
#undef TALOS_AX_LOG_EXPORT
