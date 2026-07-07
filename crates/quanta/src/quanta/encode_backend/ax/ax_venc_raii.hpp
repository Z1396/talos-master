#pragma once

#include "export.hpp"

// ---------------------------------------------------------------------------
// Axera VENC 硬件编码器 RAII 生命周期自动管理封装头文件说明
// ---------------------------------------------------------------------------
// 模块分层设计：
// 1. AxSysModule：全局AX系统底层初始化/反初始化共享引用计数封装，进程全局单例生命周期
//    全库所有AX硬件模块（IVE/IVPS/VENC）共用一套系统初始化计数，统一管理AX_SYS_Init/Deinit
// 2. VencModule：VENC编码器硬件模块全局引用计数封装，多编码通道共享同一硬件模块句柄
//    首次acquire执行AX_VENC_Init，最后一个句柄析构自动执行AX_VENC_Deinit
// 3. IvpsModule：图像预处理缩放/色彩转换硬件模块引用计数RAII
// 4. IveModule：智能视觉算子硬件引擎（降噪/锐化/二值化等）引用计数RAII
// 5. VencChannel：单个编码通道独占RAII封装，创建通道时CreateChn，析构自动DestroyChn
//    保证任何异常、提前退出场景下硬件通道资源必然释放，无泄漏
// 整体设计：共享模块句柄用std::shared_ptr做引用计数，通道独占资源禁止拷贝仅支持移动语义
// ---------------------------------------------------------------------------

// 原子类型，多线程无锁全局引用计数
#include <atomic>
// 固定宽度整数，对齐AX底层C头类型
#include <cstdint>
// C++23 std::expected 统一错误返回，替代输出参数+错误码
#include <expected>
// 智能指针，共享生命周期管理硬件模块
#include <memory>
// 存储错误描述文本
#include <string>

// C语言AX底层驱动头文件，extern C 消除C++名称修饰冲突
extern "C" {
#include "ax_base_type.h"    // AX平台基础全局类型定义（S32/U32等）
#include "ax_ive_api.h"      // IVE智能视觉算子硬件底层API
#include "ax_ivps_api.h"     // IVPS图像缩放、裁剪、色彩转换硬件API
#include "ax_venc_api.h"     // VENC视频硬件编码器完整底层API
}

namespace quanta {

// ---------------------------------------------------------------------------
// AxSysModule 全局AX系统模块 RAII 共享句柄
// 全库所有AX硬件组件共用一套系统初始化引用计数，进程全局唯一
// ---------------------------------------------------------------------------
class QUANTA_API AxSysModule {
public:
    /**
     * @brief 获取全局AX系统共享引用，线程安全
     * 首次调用：执行AX_SYS_Init硬件系统初始化；后续调用仅引用计数+1
     * @return std::expected 成功返回shared_ptr<AxSysModule>，失败携带错误字符串
     * noexcept 不会抛出C++异常，所有底层错误通过返回值传递
     */
    [[nodiscard]] static std::expected<std::shared_ptr<AxSysModule>, std::string>
        acquire() noexcept;

    /**
     * @brief 析构函数：释放当前模块持有引用，全局计数减一，计数归零则调用AX_SYS_Deinit
     */
    ~AxSysModule() noexcept;

    // 禁用拷贝构造：系统全局单例资源，禁止复制句柄
    AxSysModule(const AxSysModule&)            = delete;
    // 禁用拷贝赋值
    AxSysModule& operator=(const AxSysModule&) = delete;
    // 禁用移动构造：全局系统生命周期不可转移
    AxSysModule(AxSysModule&&)                 = delete;
    // 禁用移动赋值
    AxSysModule& operator=(AxSysModule&&)      = delete;

private:
    // 私有构造，仅静态acquire函数可以创建实例，外部无法直接构造
    AxSysModule() noexcept = default;
};

// ---------------------------------------------------------------------------
// VencModule VENC硬件编码器模块 共享引用计数RAII封装
// 多编码通道共用同一个VENC硬件模块，全局硬件资源只初始化一次
// ---------------------------------------------------------------------------
class VencModule {
public:
    /**
     * @brief 获取VENC硬件模块共享句柄，线程安全
     * @param encoder_type 编码器类型，默认视频HEVC/H264编码器
     * @return expected 共享智能指针，失败返回错误信息
     */
    [[nodiscard]] static std::expected<std::shared_ptr<VencModule>, std::string>
        acquire(AX_VENC_ENCODER_TYPE_E encoder_type = AX_VENC_VIDEO_ENCODER) noexcept;

    /**
     * @brief 析构：模块引用计数自减，计数归零时执行AX_VENC_Deinit销毁硬件模块
     */
    ~VencModule();

    // 禁止拷贝，硬件模块共享句柄仅允许shared_ptr托管，禁止复制
    VencModule(const VencModule&)            = delete;
    VencModule& operator=(const VencModule&) = delete;
    // 禁止移动，模块生命周期由shared_ptr统一管理，不允许手动转移
    VencModule(VencModule&&)                 = delete;
    VencModule& operator=(VencModule&&)      = delete;

    /**
     * @brief 获取当前模块绑定的编码器硬件类型
     * @return AX_VENC_ENCODER_TYPE_E 编码器类型枚举
     */
    [[nodiscard]] AX_VENC_ENCODER_TYPE_E encoder_type() const noexcept { return encoder_type_; }

private:
    /**
     * @brief 私有构造，仅静态acquire内部调用
     * @param ax_sys_module 全局系统共享句柄，保证系统不提前销毁
     * @param encoder_type 编码器硬件类型
     */
    explicit VencModule(
        std::shared_ptr<AxSysModule> ax_sys_module, AX_VENC_ENCODER_TYPE_E encoder_type) noexcept;

    // 持有全局系统模块，防止系统提前反初始化
    std::shared_ptr<AxSysModule> ax_sys_module_;
    // 当前模块绑定的编码器硬件类型
    AX_VENC_ENCODER_TYPE_E encoder_type_;
    // 全局VENC模块原子引用计数，多线程无锁读写
    static std::atomic<int> refcount_;
};

// ---------------------------------------------------------------------------
// IvpsModule IVPS图像预处理硬件模块 RAII 共享引用计数
// 负责缩放、裁剪、拉伸、RGB/YUV色彩空间转换硬件加速
// ---------------------------------------------------------------------------
class IvpsModule {
public:
    /**
     * @brief 获取IVPS硬件模块共享句柄，首次调用执行AX_IVPS_Init
     */
    [[nodiscard]] static std::expected<std::shared_ptr<IvpsModule>, std::string> acquire() noexcept;

    /**
     * @brief 析构：引用计数减一，归零执行AX_IVPS_Deinit
     */
    ~IvpsModule();

    // 禁用拷贝/移动，统一由shared_ptr管理生命周期
    IvpsModule(const IvpsModule&)            = delete;
    IvpsModule& operator=(const IvpsModule&) = delete;
    IvpsModule(IvpsModule&&)                 = delete;
    IvpsModule& operator=(IvpsModule&&)      = delete;

private:
    /**
     * @brief 私有构造，持有全局系统模块依赖
     */
    explicit IvpsModule(std::shared_ptr<AxSysModule> ax_sys_module) noexcept;

    std::shared_ptr<AxSysModule> ax_sys_module_;
    // 全局IVPS模块原子引用计数
    static std::atomic<int> refcount_;
};

// ---------------------------------------------------------------------------
// IveModule IVE智能视觉算子硬件模块 RAII 共享引用计数
// 提供高斯降噪、锐化、二值化、膨胀、DMA拷贝等图像硬件算子
// ---------------------------------------------------------------------------
class IveModule {
public:
    /**
     * @brief 获取IVE硬件算子模块共享句柄，首次调用AX_IVE_Init初始化硬件
     */
    [[nodiscard]] static std::expected<std::shared_ptr<IveModule>, std::string> acquire() noexcept;

    /**
     * @brief 析构：引用计数归零执行AX_IVE_Exit释放硬件
     */
    ~IveModule();

    // 禁止拷贝移动
    IveModule(const IveModule&)            = delete;
    IveModule& operator=(const IveModule&) = delete;
    IveModule(IveModule&&)                 = delete;
    IveModule& operator=(IveModule&&)      = delete;

private:
    explicit IveModule(std::shared_ptr<AxSysModule> ax_sys_module) noexcept;

    std::shared_ptr<AxSysModule> ax_sys_module_;
    // 全局IVE模块原子引用计数
    static std::atomic<int> refcount_;
};

// ---------------------------------------------------------------------------
// VencChannel 单个VENC编码通道 独占式RAII封装
// 一个实例对应硬件一个独立编码通道，资源独占，禁止拷贝，支持移动转移所有权
// ---------------------------------------------------------------------------
class VencChannel {
public:
    /**
     * @brief 创建一个HEVC编码硬件通道
     * @param module VENC硬件模块共享句柄，保证模块不提前销毁
     * @param chn_attr 通道完整配置参数（分辨率、码率、GOP、层级等）
     * @return expected 成功返回VencChannel对象，失败携带错误字符串
     */
    [[nodiscard]] static std::expected<VencChannel, std::string>
        create(std::shared_ptr<VencModule> module, const AX_VENC_CHN_ATTR_T& chn_attr) noexcept;

    /**
     * @brief 析构函数：自动调用destroy销毁硬件通道，释放硬件资源
     * 无论正常析构、异常栈展开，都会执行通道销毁，杜绝资源泄漏
     */
    ~VencChannel();

    // 禁用拷贝构造：通道硬件资源独占，不可复制
    VencChannel(const VencChannel&)            = delete;
    VencChannel& operator=(const VencChannel&) = delete;

    /**
     * @brief 移动构造：转移硬件通道所有权，原对象失效
     * @param other 待转移的通道对象
     */
    VencChannel(VencChannel&& other) noexcept;
    /**
     * @brief 移动赋值：安全释放当前持有的通道，接管传入对象通道资源
     */
    VencChannel& operator=(VencChannel&& other) noexcept;

    /**
     * @brief 获取当前硬件通道ID号
     * @return VENC_CHN 通道数字ID
     */
    [[nodiscard]] VENC_CHN id() const noexcept { return chn_; }

    /**
     * @brief 向编码器推送一帧NV12图像进行编码
     * @param frame 标准硬件帧信息结构体（包含物理地址、分辨率、格式）
     * @param timeout_ms 等待输入队列可用超时毫秒，-1无限等待
     * @return AX_S32 AX_SUCCESS(0)成功，负数AX错误码失败
     */
    [[nodiscard]] AX_S32 send_frame(const AX_VIDEO_FRAME_INFO_T& frame, AX_S32 timeout_ms) noexcept;

    /**
     * @brief 扩展推帧接口，支持逐帧自定义RC码率控制元数据（QP偏移图、ROI等）
     * @param frame 带用户自定义码率信息的帧结构体
     * @param timeout_ms 超时毫秒
     * @return AX_S32 0成功，负数错误码
     */
    [[nodiscard]] AX_S32
        send_frame_ex(const AX_USER_FRAME_INFO_T& frame, AX_S32 timeout_ms) noexcept;

    /**
     * @brief 读取编码器输出的一段压缩码流
     * @param stream 输出码流存储结构体（硬件内部缓冲区）
     * @param timeout_ms 读取超时毫秒
     * @return AX_S32 0成功，超时/失败返回负数错误码
     */
    [[nodiscard]] AX_S32 get_stream(AX_VENC_STREAM_T& stream, AX_S32 timeout_ms) noexcept;

    /**
     * @brief 释放get_stream获取的硬件内部码流缓冲区
     * 必须在get_stream成功后调用，否则硬件缓冲区泄漏卡死编码通道
     * @param stream 待释放的码流结构体
     * @return AX_S32 底层API返回码
     */
    [[nodiscard]] AX_S32 release_stream(const AX_VENC_STREAM_T& stream) noexcept;

    /**
     * @brief 启动通道接收帧流水线，创建通道后必须调用该接口才能send_frame
     * @param param 帧接收队列深度参数
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 start_recv_frame(const AX_VENC_RECV_PIC_PARAM_T& param) noexcept;

    /**
     * @brief 停止帧接收流水线，推帧接口将返回队列满/未就绪错误
     */
    [[nodiscard]] AX_S32 stop_recv_frame() noexcept;

    /**
     * @brief 强制编码器下一帧生成IDR关键帧（全帧可独立解码）
     * @param instant true：立即下一张帧强制IDR；false：等待GOP周期自然I帧
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 request_idr(bool instant = true) noexcept;

    /**
     * @brief 读取通道当前生效的码率控制RC参数
     * @return expected 成功返回完整RC配置，失败返回错误字符串
     */
    std::expected<AX_VENC_RC_PARAM_T, std::string> get_rc_param() noexcept;
    /**
     * @brief 动态更新通道码率控制参数（运行时实时调整码率、QP区间）
     * @param rc_param 新码率控制配置
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 set_rc_param(const AX_VENC_RC_PARAM_T& rc_param) noexcept;

    /**
     * @brief 设置码流VUI视频色域/色彩元数据（BT709、色域范围等）
     * @param vui_param VUI参数结构体
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 set_vui_param(const AX_VENC_VUI_PARAM_T& vui_param) noexcept;

    /**
     * @brief 设置帧内周期性刷新配置，降低画面编码闪烁、运动块效应
     * @param cfg 帧内刷新参数
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 set_intra_refresh(const AX_VENC_INTRA_REFRESH_T& cfg) noexcept;

    /**
     * @brief 设置ROI感兴趣区域编码参数（局部降低QP提升画质）
     * @param roi ROI区域配置
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 set_roi_attr(const AX_VENC_ROI_ATTR_T& roi) noexcept;

    /**
     * @brief 查询通道实时状态：缓存占用、待编码帧数量、输出队列长度
     * @param status 状态输出结构体
     * @return AX_S32 底层接口返回码
     */
    [[nodiscard]] AX_S32 query_status(AX_VENC_CHN_STATUS_T& status) noexcept;

private:
    /**
     * @brief 私有构造，仅静态create函数内部调用创建通道实例
     * @param module VENC模块共享句柄，延长模块生命周期
     * @param chn 硬件分配的通道ID
     */
    explicit VencChannel(std::shared_ptr<VencModule> module, VENC_CHN chn);

    /**
     * @brief 销毁当前编码通道硬件资源，内部析构/移动赋值自动调用
     * 逻辑：停止收帧 → 销毁通道 → 标记通道失效
     */
    void destroy() noexcept;

    // 持有VENC模块共享指针，保证模块不提前反初始化
    std::shared_ptr<VencModule> module_;
    // 硬件编码通道ID，-1代表无效已销毁通道
    VENC_CHN chn_ = -1;
    // 通道激活标记，true=通道正常创建未销毁
    bool active_  = false;
};

// ---------------------------------------------------------------------------
// IVPS 硬件工具函数 全局内联接口
// ---------------------------------------------------------------------------
/**
 * @brief 生成拉伸铺满模式IVPS缩放配置，画面完全拉伸填满输出，居中对齐，黑色填充背景
 * @return AX_IVPS_ASPECT_RATIO_T 缩放模式结构体
 */
[[nodiscard]] AX_IVPS_ASPECT_RATIO_T make_stretch_aspect_ratio() noexcept;

/**
 * @brief IVPS统一裁剪缩放分发接口，自动依次尝试VPP/VGP/TDP三条硬件流水线，任一成功直接返回
 * @param src 输入BGR/YUV原始帧
 * @param dst 缩放后输出帧
 * @param aspect_ratio 缩放填充模式配置
 * @return AX_S32 AX_SUCCESS成功，负数错误码
 */
[[nodiscard]] AX_S32 ivps_crop_resize(
    const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst,
    const AX_IVPS_ASPECT_RATIO_T* aspect_ratio) noexcept;

/**
 * @brief IVPS色彩空间转换分发接口，BGR↔NV12硬件加速转换，自动切换流水线
 * @param src 输入原始帧
 * @param dst 转换后输出帧
 * @return AX_S32 底层硬件接口返回码
 */
[[nodiscard]] AX_S32 ivps_csc(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) noexcept;

} // namespace quanta