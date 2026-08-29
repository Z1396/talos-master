# stage11：log 模块 —— spdlog 全局初始化 + cout/cerr 重定向钩子

对应真实代码：[crates/log/src/spdlog_hook.hpp](../../crates/log/src/spdlog_hook.hpp)（149 行，1 个头文件）

## 功能分析

### init_logger() —— 全项目一处配置

- **双输出槽**：彩色控制台（`stdout_color_sink_mt`）+ 滚动文件
  （`rotating_file_sink_mt`，`logs/talos.log`，单文件 5MB × 保留 3 个）；
- **统一 pattern**：控制台 `[HH:MM:SS.mmm 级别 位置] 内容`，
  文件带完整日期 + 微秒；
- `flush_on(info)`：info 及以上立即落盘，崩溃前的关键日志不丢；
- 设为 spdlog 全局默认日志器 —— 全项目 `SPDLOG_INFO` 等宏共用这一份配置。

为什么不直接用 spdlog：统一格式/级别/落盘策略收敛到一处，
改 pattern 或加 sink 不用动业务代码。

### spdlog_streambuf + hook_cstream() —— 收编原生 cout/cerr

自定义 `std::streambuf` 拦截 `std::cout` / `std::cerr`：
逐字符拼进本地缓冲，遇 `'\n'` 才把整行交给 `logger_->log()`，
并伪装 `source_loc` 为 `cout:1` / `cerr:1` 便于区分来源。
效果：第三方库往 cout 打的输出也被分级、落盘，日志出口唯一。

> 读 demo 验证时发现的真实行为：`std::cerr` 自带 `unitbuf`
> （每次 `<<` 后自动 flush），会中途触发 `streambuf::sync()` ——
> 链式 `cerr << a << b << "\n"` 会被拆成多条独立日志行。
> 而 `std::cout` 无此标志，整行缓冲到 `'\n'` 才落一条记录。

## 运行方法

```bash
cd learning_practice/stage11_log_spdlog
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cd build && ./log_spdlog_demo   # 需在 build 目录运行（logs/ 相对路径）
```

依赖：spdlog 1.15.3（CMake FetchContent 自动拉取，等价项目主 CMake 的 `spdlog::spdlog`）。

## 预期输出

控制台（demo 自身输出为普通文本；被劫持的 cout/cerr 行会带 `[时:分:秒.毫秒 级别 来源:行号]` 前缀）：

```
=== 测试1：init_logger 结构 ===
  日志器名     : talos
  输出槽数量   : 2 (console + rotating file)
  flush 级别   : info（info 及以上立即落盘）
测试1通过
...
=== 测试5：cout/cerr 重定向钩子 ===
[15:45:01.046 info cout:1] hooked_cout_line_run8168...
[15:45:01.047 error cerr:1] hooked_cerr_line_run8168...
测试5通过

=== stage11 log 模块全部测试通过 ===
```

文件 `logs/talos.log`（追加模式）：

```
[2026-08-29 15:30:00.123456 info ] info_line_run123...
[2026-08-29 15:30:00.123456 info cout:1] hooked_cout_line_run123...
[2026-08-29 15:30:00.123456 error cerr:1] hooked_cerr_line_run123...
```

## 测试清单（src/demo.cpp）

| 测试 | 断言要点 |
|---|---|
| 1 结构 | 日志器名 `talos`、2 个 sink、`flush_on == info`、已设全局默认 |
| 2 全级别 | trace..critical 六行均落盘（级别开到 trace） |
| 3 pattern | 文件行正则匹配 `[YYYY-MM-DD HH:MM:SS.ffffff info ] msg` |
| 4 级别过滤 | 级别设 err 后 info 行不落盘、error 行保留 |
| 5 cout 劫持 | `std::cout` 行以 `info` 级 + `cout:1` 落盘；`std::cerr` → `error` + `cerr:1` |

> 验证策略说明：双输出槽下"重定向 stdout 抓字符串"不可行，改为读
> `logs/talos.log` 做字符串/正则断言 —— 文件行完全由 hook 的 pattern
> 产生，等价验证了 pattern、级别、劫持三条链路。每次运行生成唯一
> run_id，避免追加模式下旧日志污染"不应出现"类断言。
