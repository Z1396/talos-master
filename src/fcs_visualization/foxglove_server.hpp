// 头文件保护，避免重复包含引发编译错误
#pragma once

// 项目内部头文件
#include "foxglove_types.hpp"   // Foxglove 可视化自定义类型、枚举、消息定义
#include "foxglove_export.hpp"  // 动态库导出/导入宏定义
#include "frame.hpp"            // 图像帧基础类型

// C++ 标准库 & 并发组件
#include <atomic>               // 原子变量，线程安全状态标记
#include <expected>             // C++23 结果类型，承载返回值/错误信息
#include <memory>               // 智能指针 std::unique_ptr
#include <string>               // 字符串，主题名、地址、错误描述
#include <system_info.hpp>      // 系统信息发布相关
#include <thread>               // 标准线程，消息发送工作线程

// 第三方库
#include <fmt/format.h>         // 字符串格式化
#include <spdlog/spdlog.h>      // 日志库
#include <tbb/concurrent_queue.h> // Intel TBB 线程安全并发队列，消息缓冲

// Foxglove SDK 核心头文件
#include <foxglove/error.hpp>   // Foxglove 错误码、错误信息接口
#include <foxglove/foxglove.hpp>// Foxglove 基础上下文、通用能力
#include <foxglove/mcap.hpp>    // MCAP 离线文件录制能力
#include <foxglove/server.hpp>  // WebSocket 在线服务端能力
#include <utility.hpp>          // 项目通用工具函数

// 可视化模块命名空间：封装 Foxglove 可视化服务全部逻辑
namespace fcs::visualization {

// 前置声明
class FoxgloveServer;        // Foxglove 可视化服务主类
class FoxgloveServerFactory; // 服务工厂类，统一创建服务实例

// 内部实现细节命名空间，对外隐藏底层工具函数
namespace detail {

// ============================================================================
// 通道初始化工具函数 —— 基于消息描述符自动创建通信通道
// ============================================================================

/**
 * @brief 初始化**类型化通道**（结构化消息通道，如位姿、状态、检测结果）
 * @tparam Def 消息/通道描述符模板参数
 * @param member 待初始化的通道成员变量（可选通道）
 * @param context Foxglove 全局上下文
 * @return 成功无返回，失败返回错误字符串
 */
template <typename Def>
[[nodiscard]] inline std::expected<void, std::string> init_typed_channel(
    std::optional<typename Def::channel_type>& member, const ::foxglove::Context& context) {
    // 调用SDK创建对应主题的类型化通道
    auto result = Def::channel_type::create(std::string(Def::topic), context);
    if (result.has_value()) {
        // 创建成功，转移所有权
        member = std::move(*result);
        return {};
    }
    // 创建失败，拼接主题名+错误信息返回
    return std::unexpected(fmt::format("{}: {}", Def::topic, ::foxglove::strerror(result.error())));
}

/**
 * @brief 初始化**原始字节通道**（RAW通道，多用于图像、码流等二进制数据）
 * @tparam Def 通道描述符
 * @param member 待初始化的原始通道成员
 * @param context Foxglove 全局上下文
 * @return 成功无返回，失败返回错误字符串
 */
template <typename Def>
[[nodiscard]] inline std::expected<void, std::string> init_raw_channel(
    std::optional<::foxglove::RawChannel>& member, const ::foxglove::Context& context) {
    // 创建原始数据通道：主题名、编码格式、元数据、上下文
    auto result = ::foxglove::RawChannel::create(
        std::string(Def::topic), std::string(Def::encoding), {}, context);
    if (result.has_value()) {
        member = std::move(*result);
        return {};
    }
    return std::unexpected(fmt::format("{}: {}", Def::topic, ::foxglove::strerror(result.error())));
}

/**
 * @brief 根据描述符类型，分发调用对应通道初始化函数
 * @tparam Def 通道描述符
 * @param channels 所有通道集合实例
 * @param context Foxglove 上下文
 * @return 初始化结果
 *
 * 编译期分支：
 * - 原始通道(is_raw=true) → 调用 init_raw_channel
 * - 类型化通道(is_raw=false) → 调用 init_typed_channel
 */
template <typename Def>
[[nodiscard]] inline std::expected<void, std::string>
    init_channel_for(FoxgloveChannels& channels, const ::foxglove::Context& context) {
    // 编译期判断：描述符是否定义传输方式，此处仅占位
    if constexpr (requires { Def::transport; }) {}

    if constexpr (Def::is_raw) {
        // 二进制原始通道
        return init_raw_channel<Def>(channels.*(Def::member), context);
    } else {
        // 结构化类型通道
        return init_typed_channel<Def>(channels.*(Def::member), context);
    }
}

/**
 * @brief 带传输方式过滤的通道初始化
 * @tparam Def 通道描述符
 * @param channels 通道集合
 * @param context 上下文
 * @param active_transport 当前启用的传输模式(WebSocket/MCAP)
 * @return 初始化结果
 *
 * 功能：如果通道描述符指定了专属传输方式，与当前模式不匹配则跳过创建
 */
template <typename Def>
[[nodiscard]] inline std::expected<void, std::string> init_channel_for(
    FoxgloveChannels& channels, const ::foxglove::Context& context,
    FoxgloveTransport active_transport) {
    // 编译期判断：通道绑定了指定传输方式，且与当前模式不一致 → 直接跳过
    if constexpr (requires { Def::transport; }) {
        if (Def::transport != active_transport)
            return {};
    }
    // 执行通用初始化
    return init_channel_for<Def>(channels, context);
}

/**
 * @brief 批量初始化所有通道（基于C++折叠表达式遍历元组内所有描述符）
 * @tparam Defs 多个通道描述符模板包
 * @param channels 通道集合
 * @param context 上下文
 * @param transport 当前传输模式
 * @param 承载所有描述符的元组
 * @return 全部初始化成功才返回正常，任一失败则返回错误
 */
 /*typename：说明后面是类型
...：省略号，代表 “一堆、任意多个”
Defs：这一整堆类型的统称（包名，随便取名）*/
template <typename... Defs>
[[nodiscard]] inline std::expected<void, std::string> init_all_channels(
    FoxgloveChannels& channels, const ::foxglove::Context& context, FoxgloveTransport transport,
    std::tuple<Defs...>) 
{
    std::expected<void, std::string> result;
    // 折叠表达式：依次初始化每一个通道，短路求值，失败立即终止
    bool ok =
        ((result = init_channel_for<Defs>(channels, context, transport), result.has_value())
         && ...);
    if (!ok)
        return result;
    return {};
}

// ============================================================================
// 后端输出(Sink)创建函数
// ============================================================================

/**
 * @brief 创建 WebSocket 服务端（在线实时可视化）
 * @param config Foxglove 全局配置
 * @param context Foxglove 上下文
 * @return WebSocket服务实例 / 错误信息
 */
[[nodiscard]] std::expected<::foxglove::WebSocketServer, std::string>
    create_websocket_sink(const FoxgloveConfig& config, const ::foxglove::Context& context);

/**
 * @brief 创建 MCAP 文件写入器（离线录制可视化数据）
 * @param config Foxglove 全局配置
 * @param context Foxglove 上下文
 * @return MCAP写入实例 / 错误信息
 */
[[nodiscard]] std::expected<::foxglove::McapWriter, std::string>
    create_mcap_sink(const FoxgloveConfig& config, const ::foxglove::Context& context);

} // namespace detail

// ============================================================================
// 工厂类 & 对外自由函数
// ============================================================================

/**
 * @brief Foxglove 服务工厂类
 * 统一封装服务创建逻辑，隔离内部构造细节
 */
class FoxgloveServerFactory {
public:
    /**
     * @brief 根据配置创建完整可视化服务实例
     * @param config 可视化配置
     * @return 服务智能指针 / 错误信息
     */
    [[nodiscard]] static std::expected<std::unique_ptr<FoxgloveServer>, std::string>
        create(FoxgloveConfig config);
};

// 对外快捷创建函数：使用完整配置创建服务
[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(FoxgloveConfig config);

// 对外快捷创建函数：仅指定端口+地址，使用默认配置
[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(uint16_t port, std::string host);

// ============================================================================
// FoxgloveServer 可视化服务主类
// 核心入口：管理通道、队列、后台线程、双输出(WebSocket/MCAP)
// ============================================================================
class FoxgloveServer {
    /* 私有域
    下面两个东西都放在类私有区域：
    ConstructorToken 令牌结构体
    友元声明 friend class FoxgloveServerFactory;
    外部代码（main、其他业务类）完全看不到这两个，不能使用。
    2. struct ConstructorToken 令牌结构体
    ① 为什么放 private
    外部拿不到这个类型，就没法生成 ConstructorToken{} 实例。
    如果构造函数要求第一个参数是 ConstructorToken，外部写不出合法参数，自然不能直接 FoxgloveServer s(...)。
    ② explicit ConstructorToken() = default;
    explicit：禁止隐式转换，必须显式写 ConstructorToken{}，不会随便传别的类型冒充令牌；
    = default：编译器生成默认无参构造，空结构体，没有任何成员，只起 “通行证标记” 作用；
    无任何数据，纯编译期校验，运行时零开销。*/
private:
    // 私有构造令牌：外部无法直接构造，强制通过工厂创建
    struct ConstructorToken {
        explicit ConstructorToken() = default;
    };
    /*友元规则：被 friend 标记的类，可以访问本类所有 private 成员。
    这里权限效果：
    只有 FoxgloveServerFactory 内部代码，能看见私有 ConstructorToken；
    只有工厂能创建 ConstructorToken{} 令牌实例；
    只有工厂能合法调用 FoxgloveServer 的私有构造函数；
    其余所有外部类、全局函数、业务代码一律禁止直接构造服务。*/
    friend class FoxgloveServerFactory;

public:
    /**
     * @brief 私有构造函数（仅工厂可调用）
     * @param ConstructorToken 私有令牌，限制外部实例化
     * @param context Foxglove SDK上下文
     * @param channels 所有通信通道集合
     * @param publisher 系统信息发布器
     * @param websocket_server WebSocket服务实例（可选）
     * @param mcap_writer MCAP文件写入实例（可选）
     */
    explicit FoxgloveServer(
        ConstructorToken, ::foxglove::Context context, FoxgloveChannels channels,
        std::optional<::foxglove::SystemInfoPublisher> publisher,
        std::optional<::foxglove::WebSocketServer> websocket_server,
        std::optional<::foxglove::McapWriter> mcap_writer);

    // 析构函数：停止线程、释放资源
    ~FoxgloveServer();

    // 禁用拷贝、移动：服务对象全局唯一，禁止复制转移
    FoxgloveServer(const FoxgloveServer&)            = delete;
    FoxgloveServer& operator=(const FoxgloveServer&) = delete;
    FoxgloveServer(FoxgloveServer&&)                 = delete;
    FoxgloveServer& operator=(FoxgloveServer&&)      = delete;

    /**
     * @brief 查询服务是否初始化完成
     */
    [[nodiscard]] bool is_initialized() const noexcept { return server_initialized_; }

    /**
     * @brief 入队一条可视化消息（主线程/业务线程调用，线程安全）
     * @param msg 待发送消息
     */
    void enqueue_message(FoxgloveMessage msg) noexcept { message_queue_.push(std::move(msg)); }

    /**
     * @brief 获取输出端存活状态标记
     */
    [[nodiscard]] std::shared_ptr<std::atomic<bool>> sink_alive() const noexcept {
        return sink_alive_;
    }

    /**
     * @brief 发布相机内参、畸变参数标定信息
     * @param width/height 图像分辨率
     * @param camera_matrix 相机内参矩阵(3x3)
     * @param distortion 畸变系数
     * @param timestamp_ns 时间戳(纳秒)
     */
    void publish_camera_calibration(
        uint32_t width, uint32_t height, const std::array<double, 9>& camera_matrix,
        const std::vector<double>& distortion, uint64_t timestamp_ns = 0);

    /**
     * @brief 批量发布坐标变换(TF)树
     * @param tf_buffer 坐标变换缓冲区
     * @param timestamp_ns 时间戳
     */
    void publish_tf(const fast_tf::CoordinateSystem& tf_buffer, uint64_t timestamp_ns) noexcept {
        if (tf_exporter_) {
            tf_exporter_->publish_all(tf_buffer, timestamp_ns);
        }
    }

private:
    /**
     * @brief 单条消息分发：根据描述符找到对应通道并输出数据
     * @tparam Def 消息描述符
     * @param m 带载荷的消息体
     */
    template <typename Def>
    void dispatch_one(const FoxgloveMsg<Def>& m) noexcept {
        // 获取对应通道
        auto& opt_ch = channels_.*(Def::member);
        if (!opt_ch)
            return;
        using Payload = typename Def::payload_type;
        // 调用日志/发送接口，完成消息输出
        detail::PayloadLogger<Payload>::log(*opt_ch, m.payload);
    }

    /**
     * @brief 统一消息分发入口：解析多态消息，路由到对应通道
     * @param msg 通用消息体
     */
    void dispatch_message(const FoxgloveMessage& msg) noexcept;

    /**
     * @brief 消息发送后台线程主函数
     * 循环从队列取消息、分发发送，隔离业务线程IO阻塞
     */
    void message_sender_thread();

    // 成员变量
    bool server_initialized_{false};                // 服务初始化完成标记
    std::atomic<bool> should_stop_{false};          // 原子标记：请求停止线程
    std::atomic<bool> shutdown_done_{false};        // 原子标记：线程已完全退出
    std::shared_ptr<std::atomic<bool>> sink_alive_; // 输出端(WebSocket/MCAP)存活状态

    ::foxglove::Context context_{};                // Foxglove SDK 全局上下文
    FoxgloveChannels channels_;                    // 所有可视化通道集合
    std::optional<::foxglove::SystemInfoPublisher> publisher_; // 系统信息发布器
    std::optional<::foxglove::WebSocketServer> websocket_server_; // 在线WebSocket服务
    std::optional<::foxglove::McapWriter> mcap_writer_;           // 离线MCAP录制器
    std::optional<fast_tf::FoxgloveExporter> tf_exporter_;        // 坐标变换导出器

    // 64字节对齐：适配CPU缓存行，提升并发队列性能
    alignas(64) tbb::concurrent_queue<FoxgloveMessage> message_queue_;
    std::thread sender_thread_; // 独立消息发送线程
};

} // namespace fcs::visualization