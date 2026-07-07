#pragma once
// 头文件保护宏，防止头文件重复包含引发重定义编译错误

// 上层流编码器核心定义：EncodeParams、EncodedPacket 等基础结构体
#include "quanta/stream_encoder.hpp"

// C++标准库头文件
#include <expected>   // C++23 预期返回类型，统一承载执行结果与错误信息
#include <memory>    // std::unique_ptr 独占智能指针，管理编码器后端实例
#include <string>    // 存储错误描述文本
#include <utility>   // std::move 转移对象所有权，避免拷贝开销

namespace quanta {

// ---------------------------------------------------------------------------
// 编码器后端抽象接口基类 EncoderBackend
// ---------------------------------------------------------------------------
// 设计说明：
// 1. 采用抽象接口隔离上层业务与底层编码实现，多后端统一调用规范
// 2. 当前两套实现：FFmpeg libx265 软件编码器、AX VENC 硬件编码器
// 3. StreamEncoder 采用PIMPL模式，内部持有 EncoderBackend 实例，所有编码操作转发至后端实现
// ---------------------------------------------------------------------------
class EncoderBackend {
public:
    /**
     * @brief 虚析构函数，确保派生类（FFmpegBackend/AxBackend）析构时完整调用子类析构逻辑，防止内存泄漏
     */
    virtual ~EncoderBackend() = default;

    /**
     * @brief 推送一帧原始图像数据进入编码流水线，纯虚函数，子类必须实现
     * @param data 原始图像像素内存首地址（BGR24格式）
     * @param linesize 单行像素字节宽度（输入行跨度）
     * @param pts 当前帧显示时间戳，用于码流时序同步
     * @return std::expected<void, std::string>
     *        成功：无返回值；失败：携带可读错误字符串
     * noexcept：函数不会抛出C++异常，适合实时编码线程稳定运行
     */
    virtual std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept = 0;

    /**
     * @brief 异步请求下一编码帧强制生成IDR关键帧，纯虚函数子类实现
     * 调用后仅标记请求，下一帧进入编码器时生效，无同步阻塞
     */
    virtual void request_keyframe() noexcept = 0;

    /**
     * @brief 轮询读取已完成编码的NALU数据包，纯虚函数子类实现
     * @return std::optional<EncodedPacket>
     *        有编码包：返回封装好的裸流、时间戳、关键帧标记
     *        无待输出码流：返回 std::nullopt
     */
    virtual std::optional<EncodedPacket> poll_packet() noexcept = 0;

    /**
     * @brief 编码器冲刷收尾，发送空帧输出编码器内部所有缓存码流，流结束时调用
     * @return 冲刷执行结果，失败携带错误信息
     */
    virtual std::expected<void, std::string> flush() noexcept = 0;

    /**
     * @brief 获取当前编码器生效的编码参数只读引用
     * @return const EncodeParams& 编码参数结构体，不可修改
     */
    virtual const EncodeParams& params() const noexcept = 0;

    /**
     * @brief 获取编码输出分辨率宽高二元组
     * @return std::pair<int, int> first=宽度，second=高度
     */
    virtual std::pair<int, int> dimensions() const noexcept = 0;
};

// ---------------------------------------------------------------------------
// 后端工厂创建函数声明
// ---------------------------------------------------------------------------
// 设计说明：
// 1. 各后端实现分别编译在独立编译单元，降低编译依赖耦合
// 2. 平台编译宏控制硬件编码器是否编译链接
// 3. 最优后端自动选择工厂，硬件可用优先硬件，否则降级软件

/**
 * @brief 创建 FFmpeg libx265 软件编码器后端，全平台永久可用
 * @param params 完整编码参数配置
 * @param src_width 原始输入图像宽度
 * @param src_height 原始输入图像高度
 * @param framerate 输入视频帧率
 * @return 成功返回 EncoderBackend 抽象基类智能指针；失败携带错误字符串
 * [[nodiscard]] 强制调用方接收返回值，防止丢弃创建失败结果
 */
[[nodiscard]] std::expected<std::unique_ptr<EncoderBackend>, std::string> create_ffmpeg_backend(
    EncodeParams params, int src_width, int src_height, int framerate) noexcept;

// 编译宏判断：启用Axera平台SDK时才暴露硬件编码器创建接口
#if TALOS_HAS_AXERA
/**
 * @brief 创建 Axera AX VENC 硬件编码器后端，仅编译时开启Axera SDK才可用
 * @param params 完整编码参数配置
 * @param src_width 原始输入图像宽度
 * @param src_height 原始输入图像高度
 * @param framerate 输入视频帧率
 * @return 抽象编码器后端智能指针，失败携带错误信息
 */
[[nodiscard]] std::expected<std::unique_ptr<EncoderBackend>, std::string>
    create_ax_backend(EncodeParams params, int src_width, int src_height, int framerate) noexcept;
#endif

/**
 * @brief 自动创建当前平台最优编码器后端内联函数
 * 优先级：AX硬件编码器 > FFmpeg软件编码器
 * 编译开启Axera SDK时优先硬件，否则自动降级libx265软件编码
 * @param params 编码参数，内部使用std::move转移所有权减少拷贝
 * @param src_width 原始输入宽
 * @param src_height 原始输入高
 * @param framerate 输入帧率
 * @return 抽象后端智能指针，统一上层调用接口
 */
[[nodiscard]] inline std::expected<std::unique_ptr<EncoderBackend>, std::string>
    create_optimal_backend(
        EncodeParams params, int src_width, int src_height, int framerate) noexcept {
#if TALOS_HAS_AXERA
    // 存在Axera硬件SDK，优先创建硬件编码器
    return create_ax_backend(params, src_width, src_height, framerate);
#endif
    // 无硬件支持，移动参数所有权创建软件编码器
    return create_ffmpeg_backend(std::move(params), src_width, src_height, framerate);
}

} // namespace quanta