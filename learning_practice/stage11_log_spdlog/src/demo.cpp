// ===========================================================================
// 阶段11：log 模块 —— spdlog 全局初始化 + cout/cerr 重定向钩子
//
// 文件对齐真实项目（crates/log/src/spdlog_hook.hpp，149 行）：
//   - init_logger()   双输出槽（彩色控制台 + 5MB×3 滚动文件 logs/talos.log），
//                     统一 pattern，flush_on(info)，设为 spdlog 全局默认日志器
//   - hook_cstream()  自定义 std::streambuf 拦截 std::cout/std::cerr，
//                     行缓冲拼满遇 '\n' 即走 spdlog（cout→info，cerr→err）
//
// 为什么不直接用 spdlog：
//   全项目一处配置格式/级别/落盘策略；第三方 cout 输出也能被收编进
//   统一日志出口（分级 + 文件留痕），而不是散落在黑窗口里。
//
// 验证策略（重定向 stdout 抓字符串在双输出槽下不可行，改用读文件断言）：
//   文件槽 logs/talos.log 是 init_logger 落盘的真实产物，日志行格式
//   完全由 hook 设置的 pattern 决定 —— 读文件做字符串/正则断言，
//   等价于验证 pattern、级别过滤、cout 劫持三条链路。
//
// 测试清单
// 测试1：init_logger 结构：名字 talos、2 个 sink、flush_on == info
// 测试2：六个级别全量输出：trace..critical 均落盘
// 测试3：pattern 格式：文件行匹配 [YYYY-MM-DD HH:MM:SS.ffffff level ] msg
// 测试4：级别过滤：logger 级别设 err 后 info 被丢弃、error 保留
// 测试5：cout/cerr 劫持：std::cout 行以 info 级、source_loc "cout:1" 落盘
// ===========================================================================

// 真实项目头文件：spdlog 初始化 + 流重定向钩子
#include "spdlog_hook.hpp"

#include <spdlog/sinks/basic_file_sink.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

// ===========================================================================
// 轻量断言：失败打印位置并累计，main 末尾以非零退出码结束
// ===========================================================================
static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "  [CHECK 失败] " #cond "  (" << __FILE__ << ":" \
                      << __LINE__ << ")\n";                               \
            ++g_failures;                                                \
        }                                                                 \
    } while (0)

// 本次运行唯一标识：日志文件是追加模式，防止旧运行的行污染
// "不应出现"类断言（比如上次运行留下的 filtered 行）
static const std::string g_run_id = [] {
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    return "run" + std::to_string(ticks);
}();

// 读取整个日志文件内容
static std::string read_log_file() {
    // 日志行写入后可能还在 FILE* 用户态缓冲里，先强制刷盘
    spdlog::default_logger()->flush();
    std::ifstream file("logs/talos.log");
    if (!file) {
        std::cerr << "  [错误] 无法打开 logs/talos.log（请从 build 目录运行 demo）\n";
        return {};
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

// ===========================================================================
// 测试1：init_logger 的结构 —— 名字、双输出槽、info 级自动落盘
// ===========================================================================
void test_init_logger_structure() {
    std::cout << "=== 测试1：init_logger 结构 ===\n";

    const auto logger = init_logger();

    CHECK(logger != nullptr);
    CHECK(logger->name() == "talos");
    CHECK(logger->sinks().size() == 2); // 控制台 + 滚动文件
    CHECK(logger->flush_level() == spdlog::level::info);
    CHECK(spdlog::default_logger() == logger); // 已设为全局默认

    std::cout << "  日志器名     : " << logger->name() << "\n";
    std::cout << "  输出槽数量   : " << logger->sinks().size() << " (console + rotating file)\n";
    std::cout << "  flush 级别   : info（info 及以上立即落盘）\n";
    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：六个级别全量输出（级别开到 trace）
// ===========================================================================
void test_all_levels() {
    std::cout << "=== 测试2：全级别输出 ===\n";

    const auto logger = spdlog::default_logger();
    logger->set_level(spdlog::level::trace); // 放行所有级别

    logger->trace("trace_line_{}", g_run_id);
    logger->debug("debug_line_{}", g_run_id);
    logger->info("info_line_{}", g_run_id);
    logger->warn("warn_line_{}", g_run_id);
    logger->error("error_line_{}", g_run_id);
    logger->critical("critical_line_{}", g_run_id);

    const auto content = read_log_file();
    // 文件行形如 "[.. .. trace ] trace_line_xxx]"：级别名 + " ] " + 消息体
    // 注意 spdlog 把 warn 级渲染为 "warning"
    for (const auto* lvl : {"trace", "debug", "info", "warn", "error", "critical"}) {
        const std::string rendered = std::strcmp(lvl, "warn") == 0 ? "warning" : lvl;
        CHECK(content.find(rendered + " ] " + lvl + "_line_" + g_run_id)
              != std::string::npos);
    }

    std::cout << "  trace/debug/info/warn/error/critical 六行均已写入 logs/talos.log\n";
    std::cout << "测试2通过\n\n";

    logger->set_level(spdlog::level::info); // 恢复常用级别
}

// ===========================================================================
// 测试3：pattern 格式验证（文件槽：%Y-%m-%d %H:%M:%S.%f %l %@] %v）
// 匹配形如：[2026-08-29 15:30:00.123456 info ] pattern_line_xxx
// （无 source_loc 的日志 %@ 为空，故 level 后有一个空格再接 ]）
// ===========================================================================
void test_pattern_format() {
    std::cout << "=== 测试3：pattern 格式验证 ===\n";

    const auto logger = spdlog::default_logger();
    logger->info("pattern_line_{}", g_run_id);
    const auto content = read_log_file();

    const std::regex re(R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6} info \] pattern_line_)"
                        + g_run_id);
    CHECK(std::regex_search(content, re));

    std::cout << "  文件行匹配: [YYYY-MM-DD HH:MM:SS.ffffff info ] pattern_line_"
              << g_run_id << "\n";
    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：级别过滤 —— logger 级别设为 err 后，info 被丢弃、error 保留
// ===========================================================================
void test_level_filter() {
    std::cout << "=== 测试4：级别过滤 ===\n";

    const auto logger = spdlog::default_logger();
    logger->set_level(spdlog::level::err); // 只放行 err 及以上

    logger->info("filtered_line_{}", g_run_id);  // 应被丢弃
    logger->error("kept_line_{}", g_run_id);     // 应保留

    const auto content = read_log_file();
    CHECK(content.find("filtered_line_" + g_run_id) == std::string::npos);
    CHECK(content.find("kept_line_" + g_run_id) != std::string::npos);

    std::cout << "  级别=err 时: info 行未落盘（被过滤），error 行保留\n";
    std::cout << "测试4通过\n\n";

    logger->set_level(spdlog::level::info); // 恢复
}

// ===========================================================================
// 测试5：hook_cstream —— std::cout/cerr 被劫持进 spdlog
// cout 行以 info 级、source_loc 显示 "cout:1"；cerr 行以 err 级、显示 "cerr:1"
// ===========================================================================
void test_hook_cstream() {
    std::cout << "=== 测试5：cout/cerr 重定向钩子 ===\n";

    // 劫持前保存原生缓冲区，测试完恢复（否则后续断言输出也变成日志）
    auto* cout_orig = std::cout.rdbuf();
    auto* cerr_orig = std::cerr.rdbuf();

    hook_cstream(); // ← 真实钩子：此后 cout/cerr 即 spdlog
    std::cout << "hooked_cout_line_" << g_run_id << "\n";
    // ⚠ std::cerr 自带 unitbuf（每次 << 后自动 flush）→ streambuf::sync()
    // 会被中途触发，链式 << 的每个片段都成为独立日志行。因此 cerr 的
    // 测试用单次插入整行字符串，保证一行日志只落一条记录。
    std::cerr << ("hooked_cerr_line_" + g_run_id + "\n");

    // 恢复原生流，后续 std::cout 正常走控制台
    std::cout.rdbuf(cout_orig);
    std::cerr.rdbuf(cerr_orig);

    const auto content = read_log_file();
    // cout → info 级，source_loc 由 streambuf 伪装成 "cout:1"
    const std::regex re_cout(R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6} info cout:1\] hooked_cout_line_)"
                             + g_run_id);
    // cerr → error 级，source_loc "cerr:1"
    const std::regex re_cerr(R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6} error cerr:1\] hooked_cerr_line_)"
                             + g_run_id);
    CHECK(std::regex_search(content, re_cout));
    CHECK(std::regex_search(content, re_cerr));

    std::cout << "  std::cout 行 → [.. info cout:1] hooked_cout_line_" << g_run_id << "\n";
    std::cout << "  std::cerr 行 → [.. error cerr:1] hooked_cerr_line_" << g_run_id << "\n";
    std::cout << "  （均已恢复原生流缓冲区，本行正常走控制台）\n";
    std::cout << "测试5通过\n\n";
}

// ===========================================================================
// 主函数：先确保 logs/ 目录存在（rotating_file_sink 构造时不会自建目录），
// 依次运行所有测试，任一断言失败返回非零
// ===========================================================================
int main() {
    std::filesystem::create_directories("logs");

    test_init_logger_structure(); // 1. init_logger 结构
    test_all_levels();            // 2. 全级别输出
    test_pattern_format();        // 3. pattern 格式
    test_level_filter();          // 4. 级别过滤
    test_hook_cstream();          // 5. cout/cerr 劫持

    if (g_failures == 0) {
        std::cout << "=== stage11 log 模块全部测试通过 ===\n";
        std::cout << "（logs/talos.log 中可见完整日志留痕，控制台为彩色版本）\n";
        return 0;
    }
    std::cerr << "=== stage11 失败断言数: " << g_failures << " ===\n";
    return 1;
}
