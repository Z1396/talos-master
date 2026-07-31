#include "runtime/stream_encode.hpp"

// 框架内部头文件
#include "core/channel_topics.hpp"        // 消息通道Topic定义（ImageChannelTopic图像流通道）
#include "core/runtime.hpp"               // runtime运行时基础接口
#include "core/types.hpp"                 // 全局类型别名、能力枚举Capability
#include "quanta/stream_encoder.hpp"      // Quanta视频编码器封装（H.264/H.265编码器）
#include "quanta/stream_transport.hpp"    // Quanta传输层：数据包组装、分片协议
#include "runtime/stream_send.hpp"        // 配套：编码后数据包网络发送系统注册函数

// 标准库 & 第三方依赖
#include <atomic>                // 无锁原子变量（序列号多线程安全自增）
#include <cstdint>               // 固定宽度整数(uint16_t等)
#include <fmt/core.h>            // 现代格式化打印，替代printf
#include <limits>                // 数值上下限（uint16最大值）
#include <memory>                // std::shared_ptr、std::optional
#include <optional>              // 可选容器，管理编码器生命周期
#include <spdlog/spdlog.h>       // 日志库
#include <string>                // 字符串

namespace fcs::runtime {
namespace { // 匿名命名空间：仅本编译单元可见，隔离内部实现

/**
 * @brief Quanta视频编码器运行时状态结构体
 * 保存编码器实例、编码参数、帧序列号、PTS时间戳、分辨率缓存、错误防抖
 */
struct QuantaEncodeState {
    quanta::EncodeParams params;                  // 编码器配置：码率、帧率、最大分辨率等
    std::optional<quanta::StreamEncoder> encoder;  // 编码器实例。std::optional方便重建/销毁
    std::atomic<uint16_t> next_seq{0};             // 视频帧全局序列号，多线程无锁自增
    int64_t next_pts = 0;                          // PTS(显示时间戳)，每送入一帧递增，用于时序同步
    int src_width    = 0;                          // 缓存上一次输入图像宽度
    int src_height   = 0;                          // 缓存上一次输入图像高度
    int framerate    = 0;                          // 缓存生效帧率，用于检测配置变更重建编码器
    std::string last_error{};                      // 防抖日志：避免同一错误疯狂刷屏
};

/**
 * @brief 防抖日志函数：相同错误仅打印一次
 * @param state 编码器状态
 * @param message 错误文本
 * @noexcept 保证不会抛出异常，编码流程崩溃隔离
 */
void log_quanta_error_once(QuantaEncodeState& state, std::string message) noexcept {
    // 和上一条错误一致，直接跳过，防止高频报错刷屏日志
    if (state.last_error == message)
        return;
    state.last_error = std::move(message);
    SPDLOG_WARN("stream_encode: {}", state.last_error);
}

/**
 * @brief 获取下一帧序列号，并原子自增（无锁CAS实现）
 * @param next_seq 原子序列号变量
 * @return 当前分配出去的序列号
 * 特性：uint16_t溢出自动循环 0~65535
 * [[nodiscard]] 强制接收返回值，防止丢弃seq
 */
[[nodiscard]] uint16_t take_next_seq(std::atomic<uint16_t>& next_seq) noexcept {
    // 宽松内存序：序列号仅用于标识帧，不需要线程间同步可见性保障
    auto current = next_seq.load(std::memory_order_relaxed);
    for (;;) { // CAS自旋循环
        // 到达uint16上限则归零循环
        const auto next = current == std::numeric_limits<uint16_t>::max()
                            ? uint16_t{0}
                            : static_cast<uint16_t>(current + 1);
        // 比较交换：如果内存值仍然等于current，则写入next，成功返回true
        if (next_seq.compare_exchange_weak(
                current, next,
                std::memory_order_relaxed,
                std::memory_order_relaxed)) {
            return current; // 返回分配出去的旧序列号
        }
        // CAS失败：current自动刷新为内存最新值，下一轮重试
    }
}

/**
 * @brief 按需创建/重建编码器
 * 触发重建条件：分辨率变化 / 帧率参数变化 / 编码器未初始化
 * @param state 编码器状态
 * @param src_width 当前原始图像宽
 * @param src_height 当前原始图像高
 * @return true编码器就绪；false创建失败
 */
[[nodiscard]] bool
ensure_quanta_encoder(QuantaEncodeState& state, int src_width, int src_height) noexcept {
    // 校验图像分辨率合法性
    if (src_width <= 0 || src_height <= 0) {
        log_quanta_error_once(
            state, fmt::format("invalid source image dimensions: {}x{}", src_width, src_height));
        return false;
    }

    // 判断：编码器已存在 + 分辨率不变 + 帧率不变 → 无需重建，直接复用
    if (state.encoder && state.src_width == src_width && state.src_height == src_height
        && state.framerate == state.params.framerate) {
        return true;
    }

    // 参数变更/无编码器 → 创建全新编码器实例
    auto encoder =
        quanta::StreamEncoder::create(state.params, src_width, src_height, state.params.framerate);
    // 创建失败
    if (!encoder) {
        log_quanta_error_once(
            state, fmt::format("quanta stream encoder create: {}", encoder.error()));
        state.encoder.reset(); // 清空optional
        return false;
    }

    // 获取编码器输出分辨率（可能下采样，小于原图尺寸）
    const auto [out_width, out_height] = encoder->dimensions();
    // 将创建成功的encoder移入optional容器
    state.encoder.emplace(std::move(*encoder));

    // 重置时序状态：新编码器从0开始计数PTS
    state.next_pts   = 0;
    // 更新缓存分辨率、帧率
    state.src_width  = src_width;
    state.src_height = src_height;
    state.framerate  = state.params.framerate;
    // 清空错误缓存，允许打印新错误
    state.last_error.clear();

    SPDLOG_INFO(
        "quanta stream encoder initialized: {}x{} -> {}x{} (limit {}x{}, {} bps @ {} fps)",
        src_width, src_height, out_width, out_height, state.params.max_width,
        state.params.max_height, state.params.target_bitrate, state.params.framerate);
    return true;
}

} // namespace 匿名实现域结束

/**
 * @brief 注册Quanta视频编码整套ECS系统入口函数
 * 在应用启动时调用，挂载图像编码系统至Talos调度器
 * @param world Talos ECS世界（存放全局资源，如数据包队列）
 * @param scheduler Talos任务调度器
 * @param encode_params 编码器静态配置参数
 * @param src_width 初始图像宽度
 * @param src_height 初始图像高度
 * @return std::expected 成功返回void；失败携带错误字符串
 */
std::expected<void, std::string> register_quanta_stream_systems(
    talos::World& world, talos::Scheduler& scheduler, const quanta::EncodeParams& encode_params,
    int src_width, int src_height) {

    // 【前置校验】尝试预创建编码器，提前捕获参数错误，不要等到运行时报错
    auto encoder = quanta::StreamEncoder::create(
        encode_params, src_width, src_height, encode_params.framerate);
    if (!encoder) {
        return std::unexpected(fmt::format("quanta stream encoder create: {}", encoder.error()));
    }

    // 创建共享状态：系统lambda捕获shared_ptr，生命周期托管，避免异步访问野指针
    auto state    = std::make_shared<QuantaEncodeState>();
    state->params = encode_params;
    state->encoder.emplace(std::move(*encoder));
    state->src_width  = src_width;
    state->src_height = src_height;
    state->framerate  = encode_params.framerate;

    // 在ECS全局资源中创建【Quanta数据包队列】
    // 编码系统产生分片包，发送系统读取该队列，生产者消费者模型
    if (!world.has_resource<quanta::QuantaPacketQueue>()) {
        auto& queue = world.emplace_resource<quanta::QuantaPacketQueue>(
            quanta::QuantaPacketQueue::kDefaultMaxPackets);
        static_cast<void>(queue); // 消除“变量未使用”编译器警告
    }

    // 向调度器注册固定频率运行的编码系统
    // talos::fixed_rate<30,3>：目标30Hz运行，允许最多3帧延时补偿
    scheduler.add_system<talos::fixed_rate<30, 3>>(
        "stream_encode", // 系统名称，日志/调试可识别
        // System执行回调函数，Talos自动注入依赖通道与资源
        [state](
            // 输入：SPMC多生产者单消费者图像通道，读取原始ImageFrame
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            // 读写资源：全局数据包队列（mut代表可修改）
            talos::res_mut<quanta::QuantaPacketQueue> packet_queue,
            // 运行时能力标志，硬件/功能开关
            core::capabilities cap) {

            // 运行时能力校验：未开启Quanta流媒体能力，直接跳过编码
            if (!core::capable(*cap, core::Capability::Quanta)) {
                return;
            }

            // 从图像通道读取一帧原始图像（阻塞/非阻塞取决于通道实现）
            auto frame = image_in.read();
            // 没有新图像，本轮无事可做
            if (!frame) {
                return;
            }

            // 检查编码器是否就绪，分辨率变化自动重建编码器
            if (!ensure_quanta_encoder(*state, frame->image.cols, frame->image.rows)) {
                return;
            }

            // 分配当前帧PTS时间戳并自增
            const int64_t pts = state->next_pts++;
            // 将OpenCV图像数据送入编码器
            // frame->image.data：像素缓冲区指针
            // frame->image.step[0]：图像单行字节跨度
            auto push_result  = state->encoder->push_frame(
                frame->image.data, static_cast<int>(frame->image.step[0]), pts);
            if (!push_result) {
                SPDLOG_WARN("stream_encode push frame: {}", push_result.error());
                return;
            }

            // 循环Poll取出编码器输出的压缩码流包（一帧图像可能产生多个NAL包）
            while (auto packet = state->encoder->poll_packet()) {
                // 获取当前帧全局序列号
                const auto seq = take_next_seq(state->next_seq);
                // 把原始码流包切割成适合UDP传输的分片包（Quanta传输协议）
                auto batch     = quanta::build_quanta_packets(*packet, seq);
                if (!batch) {
                    SPDLOG_WARN("stream_encode packetize: {}", batch.error());
                    continue;
                }

                const auto bytes     = packet->size;                // 原始码流总字节
                const auto fragments = batch->packets.size();       // 分片数量

                // 将分片数据包批次推入全局队列，供给stream_send网络发送线程
                if (!packet_queue->push_frame(std::move(*batch))) {
                    // 队列满，丢帧警告
                    SPDLOG_WARN(
                        "stream_encode dropped encoded frame: seq = {}, bytes = {}, fragments = "
                        "{}, max_fragments = {}",
                        seq, bytes, fragments, quanta::QuantaPacketQueue::kDefaultMaxPackets);
                    continue;
                }

                SPDLOG_DEBUG(
                    "Quanta frame queued: seq = {}, fragments = {}, bytes = {}", seq, fragments,
                    bytes);
            } // while poll packet
        });

    // 注册配套【数据包网络发送系统】，消费上面产生的数据包队列
    register_quanta_stream_send_system(scheduler);

    // 获取编码器输出分辨率，打印初始化成功日志
    const auto [out_width, out_height] = state->encoder->dimensions();
    SPDLOG_INFO(
        "quanta stream systems initialized: {}x{} -> {}x{} @ {}fps, packet_queue_max={}, "
        "frame_buffer={}",
        src_width, src_height, out_width, out_height, encode_params.framerate,
        quanta::QuantaPacketQueue::kDefaultMaxPackets,
        quanta::QuantaPacketQueue::kFrameBufferDepth);

    return {}; // std::expected成功，无错误
}

} // namespace fcs::runtime