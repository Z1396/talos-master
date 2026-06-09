#include "L1_sensor/camera_interface.hpp"

#include "core/time.hpp"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <thread>
#include <utility>

namespace fcs::L1 {

// ============================================================================
// IpcInput 实现
// ============================================================================

IpcInput::IpcInput(std::shared_ptr<ipc::ShmClient> client, CameraConfig info) noexcept
    : client_(std::move(client))
    , info_(std::move(info)) {}

IpcInput::~IpcInput() { SPDLOG_INFO("IpcInput destructed"); }

auto IpcInput::try_recv() const noexcept -> std::optional<Frame> {
    auto result = client_->recv_image();
    if (!result) {
        return std::nullopt;
    }
    // v.image is &'recv ImageData,
    // but we need to pass a &'mission ImageData
    // apparently 'mission outlives 'recv s.t. we must copy the data.
    // Use clone() to allocate once with correct size
    cv::Mat copied = result->image.clone();
    cv::cvtColor(copied, copied, cv::COLOR_RGB2BGR);
    return Frame{
        .seq          = result->seq,
        .timestamp_ns = result->timestamp_ns,
        .image        = std::move(copied),
    };
}

auto IpcInput::recv(const std::chrono::milliseconds timeout) const noexcept
    -> std::expected<Frame, InputError> {
    using clock         = std::chrono::steady_clock;
    const auto deadline = clock::now() + timeout;

    while (clock::now() < deadline) {
        if (auto frame = try_recv()) {
            return *frame;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return std::unexpected(Timeout{});
}

auto IpcInput::camera_info() const noexcept -> const CameraConfig& { return info_; }

// ============================================================================
// HikInput 实现
// ============================================================================

HikInput::HikInput(
    std::unique_ptr<hikcamera::ImageCapturer> camera, CameraConfig info,
    hikcamera::ImageCapturer::CameraProfile profile,
    std::chrono::duration<float, std::micro> exposure_time,
    std::optional<std::string> device_name) noexcept
    : camera_(std::move(camera))
    , info_(std::move(info))
    , profile_(profile)
    , exposure_time_ns_(
          static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(exposure_time).count()))
    , device_name_(std::move(device_name)) {}

auto HikInput::try_recv() noexcept -> std::optional<Frame> {
    using namespace std::chrono_literals;
    // NOTE: Using host timestamp (now_ns) instead of camera timestamp.
    // Camera-host clock sync is complex and error only visible in high-dynamic scenarios.
    auto now_   = now_ns();
    auto result = camera_->read(1ms);
    if (!result) {
        return std::nullopt;
    }
    return Frame{
        .seq          = seq_++,
        .timestamp_ns = now_,
        .image        = std::move(*result),
    };
}

auto HikInput::recv(const std::chrono::milliseconds timeout) noexcept
    -> std::expected<Frame, InputError> {
    // NOTE: Using host timestamp (now_ns) instead of camera timestamp.
    // Camera-host clock sync is complex and error only visible in high-dynamic scenarios.
    auto result = camera_->read(
        std::chrono::duration_cast<std::chrono::duration<unsigned int, std::milli>>(timeout));
    if (!result) {
        if (!camera_->valid()) {
            if (retry_ > 0) {
                --retry_;
                const char* name = device_name_ ? device_name_->c_str() : nullptr;
                auto cam         = hikcamera::ImageCapturer::create(profile_, name);
                if (!cam) {
                    return std::unexpected(cam.error());
                }
                camera_ = std::move(cam.value());
            }
            return std::unexpected(Disconnected{result.error().what()});
        }
        return std::unexpected(result.error());
    }
    retry_ = kReconnectRetryLimit;
    return Frame{
        .seq          = seq_++,
        .timestamp_ns = now_ns(),
        .image        = std::move(*result),
    };
}

auto HikInput::camera_info() const noexcept -> const CameraConfig& { return info_; }

auto HikInput::now_ns() const noexcept -> uint64_t {
    const auto now = clock::now_ns();
    // Subtract half of exposure time to get exposure midpoint
    return now - (exposure_time_ns_ / 2);
}

// ============================================================================
// CameraInterface 实现
// ============================================================================

CameraInterface::CameraInterface(InputMode mode) noexcept
    : mode_(std::move(mode)) {}

auto CameraInterface::try_recv() noexcept -> std::optional<Frame> {
    return std::visit([](auto& input) { return input.try_recv(); }, mode_);
}

auto CameraInterface::recv(std::chrono::milliseconds timeout) noexcept
    -> std::expected<Frame, InputError> {
    return std::visit([timeout](auto& input) { return input.recv(timeout); }, mode_);
}

auto CameraInterface::camera_info() const noexcept -> const CameraConfig& {
    return std::visit(
        [](const auto& input) -> const CameraConfig& { return input.camera_info(); }, mode_);
}

// ============================================================================
// CameraInterface 工厂方法
// ============================================================================

auto CameraInterface::create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept
    -> std::expected<CameraInterface, InputError> {
    const auto& ipc_info = client->camera_info();

    CameraConfig info;
    info.camera_matrix << ipc_info.fx, 0.0, ipc_info.cx, 0.0, ipc_info.fy, ipc_info.cy, 0.0, 0.0,
        1.0;
    info.distort_coefficient << ipc_info.distortion[0], ipc_info.distortion[1],
        ipc_info.distortion[2], ipc_info.distortion[3], ipc_info.distortion[4];
    info.width  = ipc_info.width;
    info.height = ipc_info.height;

    return CameraInterface(IpcInput(std::move(client), std::move(info)));
}

auto CameraInterface::create_hik(const CameraConfig& config) noexcept
    -> std::expected<CameraInterface, InputError> {
    hikcamera::ImageCapturer::CameraProfile profile;
    profile.trigger_mode = config.profile.trigger_mode;
    profile.invert_image = config.profile.invert_image;
    profile.exposure_time =
        std::chrono::duration<float, std::micro>(config.profile.exposure_time_us);
    profile.gain        = config.profile.gain;
    profile.rotate_type = config.profile.rotate_angle;

    const char* name = config.profile.device_name ? config.profile.device_name->c_str() : nullptr;
    auto result      = hikcamera::ImageCapturer::create(profile, name);
    if (!result) {
        return std::unexpected(result.error());
    }
    return CameraInterface(HikInput(
        std::move(*result), config, profile, profile.exposure_time, config.profile.device_name));
}

} // namespace fcs::L1
