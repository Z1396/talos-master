#pragma once

// ==============================
// C++ 标准库
// ==============================

#include <iostream>         // std::cout / std::cerr：程序的标准输出、错误输出流
#include <memory>           // std::shared_ptr：管理 spdlog::logger 的生命周期
#include <streambuf>        // std::streambuf：std::cout / std::cerr 底层使用的缓冲区基类
#include <string>           // std::string / std::string_view
#include <utility>          // std::move：移动 std::shared_ptr，减少一次拷贝
#include <vector>           // std::vector：保存多个 sink


// ==============================
// spdlog
// ==============================

#include <spdlog/sinks/rotating_file_sink.h>
// rotating_file_sink_mt：多线程安全的“滚动文件” sink
// 日志文件达到指定大小后，会自动轮换旧文件，避免单个日志文件无限增长

#include <spdlog/sinks/stdout_color_sinks.h>
// stdout_color_sink_mt：多线程安全的彩色控制台 sink
// 用于把日志输出到终端，并给不同日志级别添加颜色

#include <spdlog/spdlog.h>
// spdlog 核心接口
// 包括 logger、level、source_loc、default_logger 等


// ============================================================================
// 一、spdlog_streambuf
//
// 这个类是整个文件里最关键的“适配层”。
//
// 它解决的问题是：
//
//     原来的代码：
//         std::cout << "hello" << std::endl;
//
//     但 Talos 希望最终统一走 spdlog：
//         cout
//          ↓
//      spdlog logger
//          ↓
//       console/file
//
// 所以这里继承 std::streambuf，自己实现一个“新的输出缓冲区”。
// 然后通过 std::cout.rdbuf(&cout_buf)，把 cout 原来的缓冲区替换掉。
//
// 最终数据流：
//
//     std::cout
//         ↓
//     spdlog_streambuf
//         ↓
//       buffer_
//         ↓
//     flush_buffer()
//         ↓
//     spdlog::logger
//         ↓
//     ┌───────────────┐
//     ↓               ↓
//   console           file
// ============================================================================
class spdlog_streambuf : public std::streambuf {
public:

    /**
     * @brief 构造一个“把 ostream 输出转给 spdlog”的 streambuf。
     *
     * @param logger
     *     实际负责记录日志的 spdlog::logger。
     *
     * @param name
     *     这条输出流的来源名称。
     *     本项目中：
     *         cout -> "cout"
     *         cerr -> "cerr"
     *
     * @param lvl
     *     这条流对应的日志级别。
     *     本项目中：
     *         cout -> info
     *         cerr -> err
     *
     * 例如：
     *
     *     cout_buf(logger, "cout", spdlog::level::info)
     *
     * 以后 cout 的输出，就会被当成 info 日志处理。
     */
    spdlog_streambuf(
        std::shared_ptr<spdlog::logger> logger,
        std::string_view name,
        const spdlog::level::level_enum lvl)

        // 接管传入的 shared_ptr。
        // streambuf 自己持有 logger，保证 streambuf 存活期间 logger 不会被释放。
        : logger_(std::move(logger))

        // 保存日志级别，例如 info / err。
        , level_(lvl)

        // 保存来源名称，例如 "cout" / "cerr"。
        , name_(name)

        // 初始化字符串缓冲区，刚开始没有任何内容。
        , buffer_()
    {
        // 提前分配 512 字节容量。
        //
        // 注意：
        // reserve() 是“预留容量”，不是把 string 长度变成 512。
        // 当前 buffer_ 仍然是空字符串，只是后续追加内容时更不容易触发重新分配。
        buffer_.reserve(512);
    }


protected:

    /**
     * @brief std::streambuf 的字符处理回调。
     *
     * 当底层 streambuf 需要处理一个字符，并进入 overflow() 时，
     * 我们就把这个字符加入自己的 buffer_。
     *
     * 注意：
     * “std::cout 每输出一个字符就一定调用 overflow()”并不严谨。
     * 实际是否进入 overflow()，还和 streambuf 自身的缓冲机制有关。
     *
     * 这里我们真正关心的是：
     *
     *     字符
     *       ↓
     *     buffer_
     *       ↓
     *     遇到换行
     *       ↓
     *     flush_buffer()
     */
    int_type overflow(const int_type ch) override
    {
        // traits_type::eof() 表示“没有有效字符 / EOF”。
        //
        // 只有真正收到字符时，才把它加入 buffer_。
        if (ch != traits_type::eof()) {

            // 将 int_type 转成 char，并追加到当前日志缓冲区。
            //
            // 比如：
            //
            //     std::cout << "abc\n";
            //
            // 最终 buffer_ 会逐渐变成：
            //
            //     "a"
            //     "ab"
            //     "abc"
            //     "abc\n"
            //
            buffer_ += static_cast<char>(ch);


            // 遇到换行符，说明这一行日志结束。
            //
            // 于是立即把当前整行交给 spdlog。
            if (ch == '\n')
                flush_buffer();
        }

        // 按 streambuf 约定返回传入的字符。
        return ch;
    }


    /**
     * @brief 同步 / 刷新当前缓冲区。
     *
     * 当上层请求刷新输出流时，例如：
     *
     *     std::cout << "hello" << std::flush;
     *
     * 最终会走到 sync()。
     *
     * 这里的策略很简单：
     *
     *     不管有没有 '\n'
     *     只要要求 flush
     *         ↓
     *     就调用 flush_buffer()
     */
    int sync() override
    {
        flush_buffer();

        // streambuf::sync() 返回 0 通常表示同步成功。
        return 0;
    }


private:

    /**
     * @brief 把当前 buffer_ 里的内容真正交给 spdlog。
     *
     * 这是：
     *
     *     std::cout / std::cerr 世界
     *
     *     ↓
     *
     *     spdlog 世界
     *
     * 的真正连接点。
     */
    void flush_buffer()
    {
        // 如果当前没有内容，就什么都不用做。
        if (!buffer_.empty()) {

            // 将整行文本交给 spdlog。
            //
            // 这里没有使用：
            //
            //     logger_->info(...)
            //
            // 而是直接调用 logger_->log(...)
            // 因为需要同时手动指定：
            //
            //     1. source_loc
            //     2. level
            //     3. message
            //
            // source_loc 用来告诉 spdlog：
            // “这条日志的源码位置信息是什么。”
            //
            // 这里传入：
            //
            //     file = "cout" / "cerr"
            //     line = 1
            //     func = nullptr
            //
            // 所以这里的源码位置实际上是“人为构造”的，
            // 并不是真正产生 std::cout 的那个 cpp 和行号。
            logger_->log(
                spdlog::source_loc{
                    name_.data(), // 文件名字段：这里借用 "cout" / "cerr" 表示来源
                    1,             // 行号固定写成 1
                    nullptr        // 函数名没有提供
                },
                level_,           // 当前流对应的日志级别：info / err
                buffer_           // 当前已经拼接好的整行日志
            );

            // 日志提交完成后清空缓冲区。
            //
            // 例如：
            //
            //     "Camera initialized\n"
            //
            // 处理结束后：
            //
            //     buffer_ = ""
            //
            // 下一行可以重新开始收集。
            buffer_.clear();
        }
    }


    // ==============================
    // 成员变量
    // ==============================

    // spdlog 日志器。
    //
    // 这个对象是“日志总管”。
    // 当前 streambuf 收到的文本最终都会交给它。
    std::shared_ptr<spdlog::logger> logger_;


    // 当前 streambuf 对应的日志级别。
    //
    // 这里最终会形成：
    //
    //     cout -> info
    //     cerr -> err
    spdlog::level::level_enum level_;


    // 当前 stream 的名字。
    //
    // 例如：
    //
    //     cout_buf -> "cout"
    //     cerr_buf -> "cerr"
    //
    // 主要用于生成 source_loc。
    std::string_view name_;


    // 当前正在拼接的一行日志。
    //
    // cout 的字符不是立即全部交给 spdlog，
    // 而是先积累到这里，直到：
    //
    //     1. 遇到 '\n'
    //     2. 或者调用 flush / sync
    //
    // 再统一提交。
    std::string buffer_;
};


// ============================================================================
// 二、init_logger()
//
// 这个函数负责“搭建整个 spdlog 日志系统”。
//
// 它主要完成 4 件事：
//
//     1. 创建文件 sink
//     2. 创建控制台 sink
//     3. 用这两个 sink 创建 talos logger
//     4. 把这个 logger 注册成 spdlog 的 default logger
//
// 最终结构：
//
//                   default_logger
//                         │
//                         ▼
//                    talos logger
//                    /          \
//                   /            \
//                  ▼              ▼
//             console sink     file sink
//
// 所以后面只要：
//
//     spdlog::info("hello");
//
// logger 就知道：
//
//     同时输出到控制台
//     +
//     写入日志文件
// ============================================================================
inline std::shared_ptr<spdlog::logger> init_logger() noexcept
{
    // ========================================================================
    // 1. 创建滚动文件 sink
    // ========================================================================

    const auto file_sink =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            "logs/talos.log",       // 日志文件路径
            1024 * 1024 * 5,       // 单个日志文件最大 5 MiB
            3                      // 最多保留 3 个历史文件
        );

    // rotating 的含义：
    //
    // 当 talos.log 达到设定大小后，spdlog 会滚动日志文件，
    // 防止一个文件无限增大。
    //
    // _mt 表示 multi-thread：
    // 这个 sink 设计为可以从多个线程安全地使用。


    // ========================================================================
    // 2. 创建控制台 sink
    // ========================================================================

    const auto console_sink =
        std::make_shared<
            spdlog::sinks::stdout_color_sink_mt
        >();

    // stdout_color_sink_mt：
    //
    //     输出到 stdout / 控制台
    //     支持不同日志级别使用颜色
    //     _mt 表示多线程版本


    // ========================================================================
    // 3. 设置 console 日志格式
    // ========================================================================

    console_sink->set_pattern(
        "[%H:%M:%S.%e %^%l%$ %@] %v"
    );

    // 各占位符含义：
    //
    // %H:%M:%S
    //     时:分:秒
    //
    // %e
    //     毫秒部分
    //
    // %^ ... %$
    //     在这里开启 / 关闭颜色
    //
    // %l
    //     日志级别，例如：
    //         INFO
    //         WARN
    //         ERROR
    //
    // %@
    //     source location，也就是：
    //         文件名:行号
    //
    // %v
    //     日志正文
    //
    // 最终类似：
    //
    //     [11:23:45.123 INFO foo.cpp:42] Camera started


    // ========================================================================
    // 4. 设置 file 日志格式
    // ========================================================================

    file_sink->set_pattern(
        "[%Y-%m-%d %H:%M:%S.%f %^%l%$ %@] %v"
    );

    // 与 console 类似，但这里加入了完整日期。
    //
    // 例如：
    //
    //     [2026-09-14 11:23:45.123456 INFO foo.cpp:42]
    //     Camera started
    //
    // %Y-%m-%d
    //     年-月-日
    //
    // %f
    //     微秒部分


    // ========================================================================
    // 5. 把多个 sink 组织成一个列表
    // ========================================================================

    std::vector<spdlog::sink_ptr> sinks{
        console_sink,
        file_sink
    };

    // 这里可以理解成：
    //
    //     sinks
    //       ├── console_sink
    //       └── file_sink
    //
    // 之后 logger 收到一条日志时，
    // 会把日志交给这些 sink。


    // ========================================================================
    // 6. 创建 talos logger
    // ========================================================================

    const auto logger =
        std::make_shared<spdlog::logger>(
            "talos",       // logger 名字
            sinks.begin(), // sink 列表起点
            sinks.end()    // sink 列表终点
        );

    // 现在系统正式变成：
    //
    //                 talos logger
    //                   /       \
    //                  /         \
    //                 ▼           ▼
    //          console sink     file sink
    //
    // 所以：
    //
    //     logger->info("hello");
    //
    // 会同时发送到：
    //
    //     控制台
    //     +
    //     logs/talos.log


    // ========================================================================
    // 7. 设置自动刷新策略
    // ========================================================================

    logger->flush_on(spdlog::level::info);

    // 表示：
    //
    //     info / warn / error / critical
    //
    // 这些日志触发时，会自动要求 logger 刷新 sink。
    //
    // 目的：
    //     重要日志更及时地落到实际输出目标，
    //     特别是程序异常退出时，可以尽量减少日志还停留在缓冲区里的情况。


    // ========================================================================
    // 8. 设置成 spdlog 全局默认 logger
    // ========================================================================

    spdlog::set_default_logger(logger);

    // 设置之后：
    //
    //     spdlog::info("hello");
    //
    // 可以直接使用这里的 "talos" logger，
    // 不需要每次都手动保存 logger 指针。
    //
    // 于是：
    //
    //     spdlog::info(...)
    //             ↓
    //      default_logger
    //             ↓
    //          talos
    //         /    \
    //        ↓      ↓
    //    console    file


    // ========================================================================
    // 9. 返回 logger
    // ========================================================================

    return logger;
}


// ============================================================================
// 三、hook_cstream()
//
// 这个函数的作用不是“创建日志系统”。
//
// 日志系统已经由 init_logger() 创建好了。
//
// 它现在做的是另一件事：
//
//     把 std::cout / std::cerr 接进刚刚建立的 spdlog 系统。
//
// 原本：
//
//     std::cout
//         ↓
//     默认 std::streambuf
//         ↓
//       终端
//
// 执行 hook_cstream() 以后：
//
//     std::cout
//         ↓
//     cout_buf
//         ↓
//     spdlog_streambuf
//         ↓
//     talos logger
//         ↓
//     console + file
//
// cerr 的路径同理。
// ============================================================================
inline void hook_cstream() noexcept
{
    // 获取当前 spdlog 的默认 logger。
    //
    // 这里拿到的就是 init_logger() 中设置的 "talos" logger。
    const std::shared_ptr<spdlog::logger> logger =
        spdlog::default_logger();


    // ========================================================================
    // 1. 创建 cout 专用 streambuf
    // ========================================================================

    static spdlog_streambuf cout_buf(
        logger,
        "cout",
        spdlog::level::info
    );

    // 这表示：
    //
    //     std::cout
    //          ↓
    //     cout_buf
    //          ↓
    //       INFO 日志
    //
    // 为什么是 static？
    //
    // 因为下面我们会让：
    //
    //     std::cout.rdbuf(&cout_buf);
    //
    // 也就是说 cout 会长期保存 cout_buf 的地址。
    //
    // 如果 cout_buf 是普通局部变量：
    //
    //     void hook_cstream()
    //     {
    //         spdlog_streambuf cout_buf(...);
    //         std::cout.rdbuf(&cout_buf);
    //     }
    //
    // 那么函数返回后 cout_buf 就被销毁了，
    // std::cout 却还在指向这个已经死亡的对象。
    //
    // 所以必须让它拥有足够长的生命周期。
    //
    // 函数局部 static 的生命周期：
    //
    //     第一次调用 hook_cstream()
    //         ↓
    //     创建 cout_buf
    //
    //     后续调用
    //         ↓
    //     不重复创建
    //
    //     程序结束
    //         ↓
    //     才销毁


    // ========================================================================
    // 2. 创建 cerr 专用 streambuf
    // ========================================================================

    static spdlog_streambuf cerr_buf(
        logger,
        "cerr",
        spdlog::level::err
    );

    // 与 cout 类似，只不过：
    //
    //     cerr
    //      ↓
    //   level::err
    //
    // 因此：
    //
    //     std::cerr << "something wrong\n";
    //
    // 最终会作为 error 日志提交给 spdlog。


    // ========================================================================
    // 3. 替换 cout 的底层 streambuf
    // ========================================================================

    std::cout.rdbuf(&cout_buf);

    // 这一行是整个文件最关键的代码之一。
    //
    // rdbuf() 可以理解为：
    //
    //     “获取 / 修改 ostream 当前使用的底层缓冲区”
    //
    // 原来：
    //
    //     std::cout
    //         ↓
    //     标准库自己的 streambuf
    //         ↓
    //       终端
    //
    // 现在：
    //
    //     std::cout
    //         ↓
    //     cout_buf
    //         ↓
    //     spdlog_streambuf
    //         ↓
    //     talos logger
    //
    // 所以其他地方完全不需要修改原来的代码。
    //
    // 原本：
    //
    //     std::cout << "Camera started\n";
    //
    // 仍然可以保持不变。


    // ========================================================================
    // 4. 替换 cerr 的底层 streambuf
    // ========================================================================

    std::cerr.rdbuf(&cerr_buf);

    // 最终：
    //
    //     std::cout -> cout_buf -> INFO -> spdlog
    //
    //     std::cerr -> cerr_buf -> ERROR -> spdlog
}

/*                 init_logger()
                      │
          ┌───────────┴───────────┐
          ↓                       ↓
     console_sink             file_sink
          \                       /
           \                     /
            └──────→ logger ←───┘
                       │
                       ↓
                default_logger
                       │
                       ↓
                 hook_cstream()
                  /          \
                 ↓            ↓
              cout_buf     cerr_buf
                 │            │
                 ↓            ↓
              streambuf    streambuf
                 │            │
                 └─────┬──────┘
                       ↓
                 flush_buffer()
                       ↓
                  logger->log()*/