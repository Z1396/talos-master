// 头文件保护，防止重复包含
#pragma once

// C++ 标准库
#include <cstddef>  // 通用尺寸类型 size_t
#include <cstdint>  // 定长整型 uint8_t / int64_t 等
#include <expected> // C++23 错误返回类型，承载结果或错误信息
#include <memory>   // 智能指针 std::unique_ptr
#include <optional> // 可选值，表示有无数据包
#include <span>     // 轻量连续视图，安全访问数组数据
#include <string>   // 字符串，存放编码预设、错误信息
#include <utility>  // 移动语义、转发
#include <vector>   // 动态数组，存放NALU单元列表

// 前向声明 FFmpeg C 结构体
// 仅声明不引入完整头文件，隔离FFmpeg依赖，避免污染公共头文件
extern "C" {
struct AVFormatContext; // 封装格式上下文
struct AVCodecContext;  // 编解码器上下文
struct AVFrame;         // 音视频帧数据结构
struct AVPacket;        // 编码后的数据包
struct SwsContext;      // 图像缩放/格式转换上下文
struct AVStream;        // 音视频流
}

// 数据流编码模块命名空间，负责图像预处理、HEVC编码、码流输出
namespace quanta {

// ============================================================================
// 高斯降噪参数结构体
// 用于图像亮度/色度通道的空间高斯滤波降噪
// ============================================================================
struct GaussianDenoiseParams {
    int kernel_size = 5;   // 高斯核尺寸（奇数，越大降噪越强、模糊越严重）
    double sigma_x  = 1.0; // X 方向高斯标准差
    double sigma_y  = 1.0; // Y 方向高斯标准差
};

// ============================================================================
// 图像预处理&滤波参数
// 缩放后图像的全套前处理：降噪、锐化、量化、轮廓、码率优化等
// 区分 FFmpeg 软编码 与 AX 硬件编码两条路径，部分参数仅软编码生效
// ============================================================================
struct FilterParams {
    bool enable = true; // 总开关：是否启用所有滤波预处理

    // -------------------- 饱和度调节（仅 FFmpeg 软编码生效） --------------------
    bool enable_saturation = true; // 开启饱和度调整
    double saturation      = 1.0;  // 饱和度系数，1.0 为原始值

    // -------------------- 亮度空间高斯降噪（Y通道） --------------------
    bool enable_denoise_luma = true; // 开启亮度降噪
    GaussianDenoiseParams denoise_luma{
        // 亮度降噪核参数
        .kernel_size = 3,
        .sigma_x     = 1.5,
        .sigma_y     = 1.5,
    };

    // -------------------- 色度空间高斯降噪（UV通道，NV12格式） --------------------
    bool enable_denoise_chroma = true; // 开启色度降噪
    GaussianDenoiseParams denoise_chroma{
        // 色度降噪核参数
        .kernel_size = 3,
        .sigma_x     = 1.5,
        .sigma_y     = 1.5,
    };

    // -------------------- 时域降噪（时间轴降噪，默认关闭） --------------------
    // 利用前后帧抑制随机噪声，移动目标易产生残影、拖影，实时场景默认禁用
    bool enable_denoise_tl = true; // 亮度时域降噪开关
    double denoise_tl      = 0.0;  // 亮度时域降噪强度，0 为关闭
    bool enable_denoise_tc = true; // 色度时域降噪开关
    double denoise_tc      = 0.0;  // 色度时域降噪强度，0 为关闭

    // -------------------- 亮度锐化（多级锐化，SHP 风格边缘增强） --------------------
    // 计算公式：out = 原图 + 系数 * (原图 - 5x5模糊图)，提升边缘清晰度
    bool enable_sharpen1_luma = true;
    double sharpen1_luma      = 2.0; // 第1级亮度锐化强度
    bool enable_sharpen2_luma = true;
    double sharpen2_luma      = 0.0; // 第2级亮度锐化强度，默认关闭
    bool enable_sharpen3_luma = true;
    double sharpen3_luma      = 0.0; // 第3级亮度锐化强度，默认关闭

    // -------------------- 色度锐化 --------------------
    bool enable_sharpen1_chroma = true;
    double sharpen1_chroma      = 2.0; // 第1级色度锐化强度
    bool enable_sharpen2_chroma = true;
    double sharpen2_chroma      = 0.0; // 第2级色度锐化强度，默认关闭
    bool enable_sharpen3_chroma = true;
    double sharpen3_chroma      = 0.0; // 第3级色度锐化强度，默认关闭

    // -------------------- 颜色量化（降低色彩层级，压缩码率） --------------------
    bool enable_luma_quantization   = false; // 亮度量化开关
    int luma_levels                 = 32;    // 亮度分级数量，值越小压缩越强
    bool enable_chroma_quantization = false; // 色度量化开关
    int chroma_levels               = 0;     // 0 表示保留完整8bit色度，不量化

    // -------------------- 轮廓提取/增强 --------------------
    bool enable_contour     = false; // 轮廓增强总开关
    double contour_strength = 1.0;   // 轮廓强度
    int contour_low_thresh  = 36;    // 轮廓低阈值
    int contour_high_thresh = 96;    // 轮廓高阈值
    int contour_width       = 3;     // 轮廓线条宽度

    // -------------------- 硬件编码QP偏移映射（实验性功能） --------------------
    // 对边缘/区域设置不同量化参数，优化画质与码率分布
    bool qp_delta_map     = false; // 开启QP偏移映射
    int qp_edge_delta     = 0;     // 边缘区域QP偏移
    int qp_interior_delta = 16;    // 内部区域QP偏移

    // -------------------- FFmpeg 软编码专属心理视觉优化 --------------------
    bool enable_psy_rd      = true; // 开启心理视觉率失真优化
    double psy_rd           = 0.0;  // 优化强度
    bool enable_psy_trellis = true; // 开启心理视觉格网搜索
    double psy_trellis      = 0.0;  // 优化强度
};

// ============================================================================
// 低码率实时编码核心参数（HEVC/H.265）
// 面向超低带宽传输场景，兼顾码率、延迟、抗丢包能力
// ============================================================================
struct EncodeParams {
    int max_width      = 200;    // 输出最大宽度，原图更宽则自动缩图
    int max_height     = 200;    // 输出最大高度，原图更高则自动缩图
    int gop_size       = 15;     // GOP 大小（关键帧间隔），缩短间隔降低丢包影响
    int framerate      = 30;     // 目标输出帧率
    std::string preset = "fast"; // 编码速度预设：fast 兼顾速度与压缩率
    std::string tune   = "ssim"; // 编码调优方向：基于结构相似度优化画质
    bool intra_refresh = false;  // 周期帧内刷新，丢包场景无需反馈即可恢复画面

    // ---------- CRF + VBV 码率控制（主流软编码方案） ----------
    int crf              = 52;     // 恒定画质系数，数值越大画质越低、码率越小
    int lookahead        = 0;      // 前瞻帧数，实时传输必须设为0，杜绝延迟
    int chroma_qp_offset = 0;      // 色度通道QP偏移，微调色度画质
    int target_bitrate   = 45'000; // 目标码率，单位 bps
    bool enVBR           = false;  // 仅AX硬件编码：true=可变码率VBR，false=恒定码率CBR
    int min_bit_rate     = 0;      // 最小码率（预留字段，硬件VBR暂不支持）
    int max_bit_rate     = 50'000; // 最大码率，单位 bps
    int vbv_bufsize      = 10'000; // VBV缓冲区大小，单位 bit，控制码率波动
    int refresh_num      = 4;      // 帧内刷新计数

    // ---------- libx265 专属参数 ----------
    int bframe         = 0;    // B帧数量，实时场景禁用B帧降低延迟
    bool enScenecut    = true; // 开启场景切换自动插关键帧
    bool enAqMode      = true; // 开启自适应量化
    double aq_strength = 0.4;  // 自适应量化强度（软编码生效，硬件使用QP映射）

    FilterParams filter{};     // 绑定全套图像预处理滤波参数
};

// ============================================================================
// HEVC NALU 单元描述（Annex B 标准码流格式）
// HEVC 码流由多个NALU组成，该结构体记录单个NALU位置、大小、类型
// ============================================================================
struct EncodedPacketNalu {
    size_t packet_offset = 0; // NALU 在数据包内的字节偏移
    size_t packet_size   = 0; // NALU 字节长度
    uint8_t type         = 0; // NALU 类型（关键帧、普通帧、SPS/PPS等）
};

// ============================================================================
// 编码完成后的数据包
// 承载一帧完整HEVC码流、时间戳、关键帧标记、NALU列表
// 仅支持移动语义，禁止拷贝（大数据缓冲区避免拷贝开销）
// ============================================================================
struct EncodedPacket {
    std::unique_ptr<uint8_t[]> data;      // 编码数据缓冲区，独占所有权
    size_t size   = 0;                    // 有效数据长度
    int64_t pts   = 0;                    // 显示时间戳，用于时序同步
    bool keyframe = false;                // 是否为关键帧（IDR帧）
    std::vector<EncodedPacketNalu> nalus; // 本帧包含的所有NALU单元

    // 构造/析构 & 移动/拷贝控制
    EncodedPacket()                                    = default;
    EncodedPacket(EncodedPacket&&) noexcept            = default;
    EncodedPacket& operator=(EncodedPacket&&) noexcept = default;
    EncodedPacket(const EncodedPacket&)                = delete; // 禁用拷贝构造
    EncodedPacket& operator=(const EncodedPacket&)     = delete; // 禁用拷贝赋值

    /**
     * @brief 以只读视图形式获取编码数据，用于网络传输/写入文件
     * @return 连续字节span视图
     */
    [[nodiscard]] std::span<const uint8_t> bytes() const noexcept { return {data.get(), size}; }
};

// 后端实现类前向声明（编解码底层实现，对外隐藏）
class EncoderBackend;

// ============================================================================
// 实时HEVC流编码器主类
// 输入BGR24图像帧，完成预处理、缩放、编码，输出标准HEVC码流包
// 对外统一接口，内部屏蔽 FFmpeg 软编码 / AX 硬件编码差异
// ============================================================================
class StreamEncoder {
public:
    /**
     * @brief 工厂方法：创建并初始化编码器实例
     * @param params 编码全套参数
     * @param src_width 输入图像宽度
     * @param src_height 输入图像高度
     * @param framerate 输入帧率
     * @return 成功返回编码器实例，失败返回错误字符串
     */
    [[nodiscard]] static std::expected<StreamEncoder, std::string>
        create(EncodeParams params, int src_width, int src_height, int framerate) noexcept;

    // 析构函数，释放编码资源
    ~StreamEncoder();

    // 移动语义支持，拷贝禁用
    StreamEncoder(StreamEncoder&&) noexcept;
    StreamEncoder& operator=(StreamEncoder&&) noexcept;
    StreamEncoder(const StreamEncoder&)            = delete;
    StreamEncoder& operator=(const StreamEncoder&) = delete;

    /**
     * @brief 推入一帧原始BGR24图像数据进行编码
     * @param data 图像数据指针
     * @param linesize 行字节数
     * @param pts 时间戳
     * @return 执行结果，失败返回错误信息
     */
    std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept;

    /**
     * @brief 主动请求下一帧编码为关键帧
     * 用于网络重连、画面重置等场景
     */
    void request_keyframe() noexcept;

    /**
     * @brief 轮询获取编码完成的数据包
     * @return 有数据返回数据包，无数据返回 std::nullopt
     */
    std::optional<EncodedPacket> poll_packet() noexcept;

    /**
     * @brief 刷新编码器缓冲区，输出剩余缓存数据
     * 流结束/停止编码时调用
     * @return 执行结果
     */
    std::expected<void, std::string> flush() noexcept;

    /**
     * @brief 获取当前编码参数（只读）
     */
    [[nodiscard]] const EncodeParams& params() const;

    /**
     * @brief 获取最终输出图像分辨率
     */
    [[nodiscard]] std::pair<int, int> dimensions() const;

private:
    // 私有构造函数，仅工厂方法可创建实例
    explicit StreamEncoder(std::unique_ptr<EncoderBackend> backend);
    std::unique_ptr<EncoderBackend> backend_; // 后端实现句柄，隔离软硬编码细节
};

} // namespace quanta