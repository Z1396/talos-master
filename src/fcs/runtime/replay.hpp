#pragma once
/// #pragma once 头文件保护，防止重复包含引发重定义编译错误

// 标准库依赖
#include <chrono>            // 时间单位：std::chrono::milliseconds
#include <expected>          // C++23 std::expected<T,E> 错误处理范式
#include <filesystem>        // std::filesystem::path 文件路径跨平台类型
#include <optional>          // std::optional 可空类型，表达“可选参数”
#include <string>            // 标准字符串
#include <string_view>       // 只读字符串视图，避免字符串拷贝

namespace fcs::runtime {

/**
 * @brief 回放模块启动参数配置结构体
 * 用于离线录像回放功能：读取录制好的数据文件，模拟在线运行时数据流
 */
struct ReplayOptions {
    /// 回放录像文件/目录路径
    /// std::nullopt = 不开启回放模式，正常实时运行
    std::optional<std::filesystem::path> input_path{};

    /// 回放倍速：1.0 = 原始速度；0.5慢放；2.0快进
    double speed{1.0};

    /// 是否循环播放录像
    bool loop{false};

    /// 回放启动时间偏移：跳过录像前N毫秒数据再开始播放
    std::chrono::milliseconds startup_offset{0};

    /// 最大读取图像帧数上限
    /// std::nullopt = 不限制，完整播放录像
    std::optional<size_t> max_images{};
};

/**
 * @brief 生成回放程序命令行帮助文本
 * @param program 当前程序名（argv[0]）
 * @return 格式化的帮助说明字符串，用于控制台打印用法
 * [[nodiscard]] 强制调用方接收返回值，禁止丢弃结果
 */
[[nodiscard]] auto replay_usage(std::string_view program) -> std::string;

/**
 * @brief 解析命令行参数，转换为回放配置 ReplayOptions
 * @param argc 参数数量
 * @param argv 命令行参数字符数组
 * @return std::expected
 *      成功：携带填充完成的 ReplayOptions
 *      失败：unexpected 携带错误描述字符串（参数非法、路径不存在等）
 * @noexcept 函数保证不会抛出C++异常
 */
[[nodiscard]] auto parse_replay_options(int argc, char** argv) noexcept
    -> std::expected<ReplayOptions, std::string>;

/**
 * @brief 执行离线录像回放主逻辑
 * 根据传入配置加载录像文件，按时序推送回放数据至系统
 * @param options 回放配置参数（输入路径、倍速、循环、帧数限制等）
 * @return std::expected<void, std::string>
 *      成功：空void
 *      失败：返回错误信息（文件打不开、录像格式损坏等）
 * @noexcept 不会抛出异常，所有错误通过返回值传递
 */
[[nodiscard]] auto run_replay(const ReplayOptions& options) noexcept
    -> std::expected<void, std::string>;

} // namespace fcs::runtime