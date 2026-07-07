// 引入流编码器头文件，包含 StreamEncoder 类声明、EncodeParams、EncodedPacket 等基础类型
#include "quanta/stream_encoder.hpp"
// 引入编码器后端抽象接口与工厂函数定义
#include "quanta/encode_backend/encoder_backend.hpp"

namespace quanta {

/**
 * @brief StreamEncoder 构造函数
 * 采用PIMPL设计模式，接收编码器后端抽象实例，转移智能指针所有权
 * @param backend 任意实现EncoderBackend的后端实例（FFmpeg软件/Axera硬件）
 */
StreamEncoder::StreamEncoder(std::unique_ptr<EncoderBackend> backend)
    : backend_(std::move(backend)) {}

/**
 * @brief 默认析构函数
 * backend_ 为unique_ptr，离开生命周期自动释放后端所有资源，无需手动释放
 */
StreamEncoder::~StreamEncoder() = default;

/**
 * @brief 移动构造函数，默认生成
 * 允许StreamEncoder对象转移所有权，仅移动内部unique_ptr后端句柄，无内存拷贝
 * noexcept 移动过程不会抛出异常，适配实时编码场景
 */
StreamEncoder::StreamEncoder(StreamEncoder&&) noexcept            = default;
/**
 * @brief 移动赋值运算符，默认生成
 * 覆盖原有后端资源，自动释放旧后端，接管传入对象的编码器后端
 */
StreamEncoder& StreamEncoder::operator=(StreamEncoder&&) noexcept = default;

/**
 * @brief 静态工厂创建接口，对外统一创建编码器入口
 * 自动根据平台硬件择优创建最优编码器后端（硬件优先，无硬件降级软件）
 * @param params 完整编码参数配置
 * @param src_width 原始输入图像宽度
 * @param src_height 原始输入图像高度
 * @param framerate 输入视频帧率
 * @return std::expected<StreamEncoder, std::string>
 *        成功：返回构造完成的StreamEncoder实例
 *        失败：携带可读错误字符串，描述后端创建失败原因
 * noexcept 内部捕获所有异常，不会向上抛出C++异常
 */
std::expected<StreamEncoder, std::string> StreamEncoder::create(
    EncodeParams params, int src_width, int src_height, int framerate) noexcept {
    // 调用全局最优后端工厂，转移参数所有权减少拷贝
    auto backend = create_optimal_backend(std::move(params), src_width, src_height, framerate);
    // 后端创建失败，透传错误信息
    if (!backend)
        return std::unexpected(std::move(backend.error()));
    // 后端实例合法，构造并返回StreamEncoder对象
    return StreamEncoder(std::move(*backend));
}

/**
 * @brief 推送一帧原始BGR图像进入编码流水线
 * 上层统一封装接口，内部转发至底层后端实现
 * @param data 原始像素数据内存首地址
 * @param linesize 图像单行字节跨度
 * @param pts 帧显示时间戳，用于码流时序同步
 * @return 编码推送执行结果，失败携带错误信息
 */
std::expected<void, std::string>
    StreamEncoder::push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept {
    // 转发调用后端push_frame实现
    return backend_->push_frame(data, linesize, pts);
}

/**
 * @brief 异步请求下一帧强制生成IDR关键帧
 * 仅设置标记，下一帧送入编码器时生效，无阻塞等待
 */
void StreamEncoder::request_keyframe() noexcept { backend_->request_keyframe(); }

/**
 * @brief 非阻塞轮询读取已编码NALU裸流包
 * @return std::optional<EncodedPacket>
 *        存在编码包：返回封装好的码流、时间戳、关键帧标记
 *        无待输出码流：返回std::nullopt
 */
std::optional<EncodedPacket> StreamEncoder::poll_packet() noexcept {
    return backend_->poll_packet();
}

/**
 * @brief 编码器冲刷收尾接口
 * 流结束时调用，推送空帧，输出编码器内部缓存的所有剩余码流
 * @return 冲刷操作执行结果，失败携带错误文本
 */
std::expected<void, std::string> StreamEncoder::flush() noexcept { return backend_->flush(); }

/**
 * @brief 获取当前编码器生效的编码参数只读引用
 * [[nodiscard]] 强制接收返回值，防止丢弃参数结果
 * @return 不可修改的EncodeParams参数结构体
 */
[[nodiscard]] const EncodeParams& StreamEncoder::params() const { return backend_->params(); }

/**
 * @brief 获取编码输出分辨率宽高二元组
 * @return pair<宽, 高> 输出编码尺寸
 */
[[nodiscard]] std::pair<int, int> StreamEncoder::dimensions() const {
    return backend_->dimensions();
}

} // namespace quanta