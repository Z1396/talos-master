/**
 * @author Qzh (zihanqin2048@gmail.com)
 * @brief Hikcamera 海康工业相机上层封装头文件
 * @copyright Copyright (c) 2024 by Alliance, All Rights Reserved.
 */
#pragma once
// 头文件保护，防止重复包含

// 标准库依赖
#include <chrono>               // 时长、曝光、超时、帧率时间单位
#include <expected>             // C++23 错误处理，替代异常，分离正常/错误返回
#include <fmt/base.h>
#include <fmt/format.h>         // 格式化字符串，日志/错误拼接
#include <memory>               // unique_ptr 工厂模式、PIMPL隐藏实现
#include <opencv2/core/mat.hpp> // OpenCV 图像容器 cv::Mat
#include <optional>             // 可选错误码，可能存在/不存在
#include <ratio>
#include <string>
#include <string_view>
#include <tuple>                // 同时返回宽高 pair 替代结构体
#include <utility>              // std::move 移动语义

namespace hikcamera {

/**
 * @brief 将海康SDK原生数字错误码转换为人类可读文字描述
 * @param code 海康SDK返回错误码
 * @return 错误说明字符串
 * @noexcept 无异常抛出
 */
std::string error_code_to_message(unsigned int code) noexcept;

/**
 * @brief 图像旋转枚举，图像后处理方向
 */
enum class RotateType {
    None,               // 不旋转
    Clockwise90,        // 顺时针90°
    Clockwise180,       // 顺时针180°
    Clockwise270        // 顺时针270°
};

/**
 * @brief 图像同步模式枚举
 */
enum class SyncMode {
    NONE,               // 无同步，相机自由输出流
    SOFTWARE            // 软触发模式：软件定时器触发取图、多线程同步
};

/**
 * @brief 统一相机错误封装结构体
 * 存储错误描述文本 + 可选SDK原始错误码，重载what()拼接完整报错信息
 */
struct CameraError {
    std::string message;                      // 基础错误信息
    std::optional<unsigned int> error_code;   // 可选：海康SDK原始错误码，无则为空

    // 仅传入错误消息构造
    CameraError(std::string msg)
        : message(std::move(msg)) {}

    // C风格字符串构造
    CameraError(const char* msg)
        : message(std::string(msg)) {}

    // 消息 + SDK数字错误码
    CameraError(std::string msg, int error_code)
        : message(std::move(msg))
        , error_code(static_cast<unsigned int>(error_code)) {}

    /**
     * @brief 拼接完整错误信息，带错误码说明
     * @return 可读完整报错字符串
     */
    std::string what() const {
        if (error_code.has_value()) {
            // 存在错误码：信息 + 错误码对应描述
            return fmt::format("{}: {}", message, error_code_to_message(error_code.value()));
        }
        // 无SDK错误码，仅返回自定义消息
        return message;
    }
};

/**
 * @brief 相机对外主类：图像捕获器
 * PIMPL 模式，Impl隐藏海康SDK底层实现，对外只暴露稳定接口
 */
class ImageCapturer final {
public:
    /**
     * @brief 相机硬件/图像参数配置结构体，出厂默认预设
     */
    struct CameraProfile {
        /**
         * @brief ADC图像位深
         */
        enum ADCBitDepth : uint8_t {
            Depth8bit  = 0,  // 8位灰度图，常规使用
            Depth12bit = 3   // 12位高动态原图
        };

        // 构造函数：给出工程默认相机参数
        CameraProfile() noexcept {
            using namespace std::chrono_literals;
            trigger_mode  = false;        // 默认不开启外/软触发，连续出图
            invert_image  = true;         // 默认反转图像（适配工业黑白靶场）
            exposure_time = 10ms;         // 默认曝光10毫秒
            gain          = 16.7;         // 默认模拟增益
            rotate_type   = RotateType::None; // 默认不旋转图像
        }

        bool trigger_mode;                          // 是否开启触发模式（软/硬触发）
        bool invert_image;                          // 是否图像灰度反转

        std::chrono::duration<float, std::micro> exposure_time; // 曝光时长，微秒精度
        float gain;                                             // 模拟增益
        RotateType rotate_type;                                 // 图像后处理旋转
        ADCBitDepth adc_depth{ADCBitDepth::Depth8bit};          // ADC采样位深，默认8bit
    };

    /**
     * @brief 工厂静态方法：创建相机捕获器实例
     * 采用 std::expected 无异常错误处理，成功返回unique_ptr相机对象，失败返回CameraError
     * @param profile 相机参数配置，使用默认参数可省略
     * @param user_defined_name 用户自定义相机标识名，用于多相机区分日志，可为nullptr
     * @param sync_mode 图像同步模式，默认连续出图NONE
     * @return expected<相机独占智能指针, 错误结构体>
     * @noexcept 不抛异常
     */
    static std::expected<std::unique_ptr<ImageCapturer>, CameraError> create(
        const CameraProfile& profile = CameraProfile{}, const char* user_defined_name = nullptr,
        const SyncMode& sync_mode = SyncMode::NONE) noexcept;

    // 禁用拷贝构造、拷贝赋值（相机资源独占，不可复制）
    ImageCapturer(const ImageCapturer&)            = delete;
    ImageCapturer& operator=(const ImageCapturer&) = delete;

    /**
     * @brief 析构函数：自动停止取流、释放海康相机句柄、释放SDK资源
     */
    ~ImageCapturer() noexcept;

    /**
     * @brief 阻塞读取一帧图像
     * @param timeout 读取超时时间，默认5秒，超时返回错误
     * @return expected<cv::Mat图像, 相机错误>
     * @noexcept
     */
    [[nodiscard]] std::expected<cv::Mat, CameraError> read(
        std::chrono::duration<unsigned int, std::micro> timeout = std::chrono::seconds(5)) noexcept;

    /**
     * @brief 获取图像分辨率 宽、高
     * @return expected<tuple<width, height>, 错误>
     */
    [[nodiscard]] std::expected<std::tuple<int, int>, CameraError>
        get_width_height() const noexcept;

    /**
     * @brief 开启软件触发模式，切换为软触发取图
     */
    [[nodiscard]] std::expected<void, CameraError> software_trigger_on() noexcept;

    /**
     * @brief 内部触发模式下设置相机输出帧率（软触发限定帧率）
     * @param frame_rate 目标帧率
     */
    [[nodiscard]] std::expected<void, CameraError>
        set_frame_rate_inner_trigger_mode(float frame_rate) noexcept;

    /**
     * @brief 主动停止相机取流
     * 场景：程序关闭前提前调用，防止缓冲区图像堆积、回调内存溢出
     */
    [[nodiscard]] std::expected<void, CameraError> stop_grabbing() noexcept;

    /**
     * @brief 判断相机句柄、取流是否正常有效
     * @return true 正常可用；false 断开/未初始化/异常
     */
    [[nodiscard]] bool valid() const noexcept;

    // 允许std::make_unique构造私有构造函数（仅内部工厂使用）
    friend std::unique_ptr<ImageCapturer> std::make_unique<ImageCapturer>();

private:
    // 私有默认构造，仅内部create工厂调用，外部不可直接new
    ImageCapturer() noexcept;

    /**
     * @brief 内部初始化逻辑：打开相机、配置曝光/增益/触发/旋转等参数
     */
    [[nodiscard]] std::expected<void, CameraError> init(
        const CameraProfile& profile, const char* user_defined_name,
        const SyncMode& sync_mode) noexcept;

    // PIMPL 实现前置声明，所有海康SDK底层逻辑全部放在Impl内部，对外隐藏
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hikcamera

// ============================================================================
// fmt 格式化特化：让CameraError可以直接被fmt打印输出
// ============================================================================
namespace fmt {

template <>
struct formatter<hikcamera::CameraError> : formatter<std::string_view> {
    // 格式化CameraError，自动调用what()获取完整错误文本输出
    auto format(const hikcamera::CameraError& e, format_context& ctx) const {
        return formatter<std::string_view>::format(e.what(), ctx);
    }
};

} // namespace fmt