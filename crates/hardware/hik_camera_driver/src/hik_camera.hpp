/**
 * @author Qzh (zihanqin2048@gmail.com)
 * @brief Hikcamera
 * @copyright Copyright (c) 2024 by Alliance, All Rights Reserved.
 */
#pragma once

#include <chrono>
#include <expected>
#include <fmt/base.h>
#include <fmt/format.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <ratio>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

namespace hikcamera {

std::string error_code_to_message(unsigned int code) noexcept;

enum class RotateType {
    None,
    Clockwise90,
    Clockwise180,
    Clockwise270,
};

enum class SyncMode {
    NONE,

    SOFTWARE // use soft trigger and multi thread timer to get image
};

struct CameraError {
    std::string message;
    std::optional<unsigned int> error_code;

    CameraError(std::string msg)
        : message(std::move(msg)) {}

    CameraError(const char* msg)
        : message(std::string(msg)) {}

    CameraError(std::string msg, int error_code)
        : message(std::move(msg))
        , error_code(static_cast<unsigned int>(error_code)) {}

    std::string what() const {
        if (error_code.has_value()) {
            return fmt::format("{}: {}", message, error_code_to_message(error_code.value()));
        }
        return message;
    }
};

class ImageCapturer final {
public:
    struct CameraProfile {
        enum ADCBitDepth : uint8_t { Depth8bit = 0, Depth12bit = 3 };
        CameraProfile() noexcept {
            using namespace std::chrono_literals;
            trigger_mode  = false;
            invert_image  = true;
            exposure_time = 10ms;
            gain          = 16.7;
            rotate_type   = RotateType::None;
        }

        bool trigger_mode;
        bool invert_image;

        std::chrono::duration<float, std::micro> exposure_time;
        float gain;
        RotateType rotate_type;
        ADCBitDepth adc_depth{ADCBitDepth::Depth8bit};
    };

    // Factory method for creating ImageCapturer with std::expected
    static std::expected<std::unique_ptr<ImageCapturer>, CameraError> create(
        const CameraProfile& profile = CameraProfile{}, const char* user_defined_name = nullptr,
        const SyncMode& sync_mode = SyncMode::NONE) noexcept;

    ImageCapturer(const ImageCapturer&)            = delete;
    ImageCapturer& operator=(const ImageCapturer&) = delete;

    ~ImageCapturer() noexcept;

    [[nodiscard]] std::expected<cv::Mat, CameraError> read(
        std::chrono::duration<unsigned int, std::micro> timeout = std::chrono::seconds(5)) noexcept;

    [[nodiscard]] std::expected<std::tuple<int, int>, CameraError>
        get_width_height() const noexcept;

    [[nodiscard]] std::expected<void, CameraError> software_trigger_on() noexcept;
    [[nodiscard]] std::expected<void, CameraError>
        set_frame_rate_inner_trigger_mode(float frame_rate) noexcept;

    /// 停止取流，可在 shutdown 时提前调用以避免缓冲区溢出回调
    [[nodiscard]] std::expected<void, CameraError> stop_grabbing() noexcept;

    [[nodiscard]] bool valid() const noexcept;

    friend std::unique_ptr<ImageCapturer> std::make_unique<ImageCapturer>();

private:
    // Default constructor for internal use by create()
    ImageCapturer() noexcept;

    [[nodiscard]] std::expected<void, CameraError> init(
        const CameraProfile& profile, const char* user_defined_name,
        const SyncMode& sync_mode) noexcept;

    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hikcamera

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<hikcamera::CameraError> : formatter<std::string_view> {
    auto format(const hikcamera::CameraError& e, format_context& ctx) const {
        return formatter<std::string_view>::format(e.what(), ctx);
    }
};

} // namespace fmt
