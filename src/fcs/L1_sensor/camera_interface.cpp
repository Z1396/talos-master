#include "L1_sensor/camera_interface.hpp"

#include "core/time.hpp"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <thread>
#include <utility>

namespace fcs::L1 {

// ============================================================================
// IpcInput：共享内存客户端取图实现
// 场景：相机单独进程采集图像放入共享内存，视觉进程通过ShmClient读取画面
// ============================================================================

IpcInput::IpcInput(std::shared_ptr<ipc::ShmClient> client, CameraConfig info) noexcept
    : client_(std::move(client))   // 共享内存客户端智能指针
    , info_(std::move(info)) {}     // 相机内参、分辨率等配置

IpcInput::~IpcInput() {
    SPDLOG_INFO("IpcInput destructed");
}

/**
 * @brief 非阻塞尝试获取一帧图像
 * @return 有帧返回Frame，无帧返回std::nullopt
 */
auto IpcInput::try_recv() const noexcept -> std::optional<Frame> {
    // 从共享内存读取原始图像数据
    auto result = client_->recv_image();
    if (!result) {
        // 没有新图像，返回空
        return std::nullopt;
    }

    // 关键注释解读：
    // 共享内存里的image是依附recv出来的临时对象的引用
    // 外部Frame生命周期更长，如果直接引用会悬垂引用，必须拷贝图像
    // clone() 完整拷贝Mat内存，分配独立内存，安全持久持有图像
    cv::Mat copied = result->image.clone();

    // 共享内存传输默认RGB格式，OpenCV处理图像习惯BGR，做颜色通道转换
    cv::cvtColor(copied, copied, cv::COLOR_RGB2BGR);

    // 封装成视觉层标准帧结构返回
    return Frame{
        .seq          = result->seq,                // 帧序列号
        .timestamp_ns = result->timestamp_ns,     // 相机侧时间戳(纳秒)
        .image        = std::move(copied),         // 图像Mat
    };
}

/**
 * @brief 阻塞式带超时等待取帧
 * @param timeout 最大阻塞等待时长
 * @return std::expected<帧数据, 错误码> 成功返回Frame，超时返回Timeout错误
 */
auto IpcInput::recv(const std::chrono::milliseconds timeout) const noexcept
    -> std::expected<Frame, InputError> {
    using clock         = std::chrono::steady_clock;
    // 计算超时截止时间点
    const auto deadline = clock::now() + timeout;

    // 在超时时间内循环轮询
    while (clock::now() < deadline) {
        if (auto frame = try_recv()) {
            // 读到图像直接返回
            return *frame;
        }
        // 没帧休眠1ms让出CPU，避免忙等占满CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // 超时未读到图像，返回超时错误
    return std::unexpected(Timeout{});
}

/**
 * @brief 获取相机标定配置（内参、畸变系数、分辨率）
 */
auto IpcInput::camera_info() const noexcept -> const CameraConfig& {
    return info_;
}

// ============================================================================
// HikInput：海康工业相机直采实现，进程直接调用海康SDK抓图
// ============================================================================

HikInput::HikInput(
    // 海康相机捕获句柄
    std::unique_ptr<hikcamera::ImageCapturer> camera,
    // 相机标定配置
    CameraConfig info,
    // 相机硬件配置：触发模式、曝光、增益、旋转等
    hikcamera::ImageCapturer::CameraProfile profile,
    // 曝光时长（微秒）
    std::chrono::duration<float, std::micro> exposure_time,
    // 相机设备名，用于断线重连查找设备
    std::optional<std::string> device_name) noexcept
    : camera_(std::move(camera))
    , info_(std::move(info))
    , profile_(profile)
    // 曝光时间转为纳秒存储，用于修正时间戳
    , exposure_time_ns_(
          static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(exposure_time).count()))
    , device_name_(std::move(device_name)) {}

/**
 * @brief 非阻塞尝试读取海康相机图像
 */
auto HikInput::try_recv() noexcept -> std::optional<Frame> {
    using namespace std::chrono_literals;
    // 设计说明：不使用相机硬件时间戳，改用主机系统时间
    // 原因：主机与相机时钟同步难度高，高速运动场景下同步误差会造成时序错乱
    auto now_   = now_ns();
    // 阻塞1ms尝试读取
    auto result = camera_->read(1ms);
    if (!result) {
        return std::nullopt;
    }
    // 封装帧结构，seq自增作为帧序号
    return Frame{
        .seq          = seq_++,
        .timestamp_ns = now_,
        .image        = std::move(*result),
    };
}

/**
 * @brief 带超时阻塞取帧，内置断线自动重连逻辑
 */
auto HikInput::recv(const std::chrono::milliseconds timeout) noexcept
    -> std::expected<Frame, InputError> {
    // 调用海康SDK阻塞读取图像
    auto result = camera_->read(
        std::chrono::duration_cast<std::chrono::duration<unsigned int, std::milli>>(timeout));

    if (!result) {
        // 读取失败，判断相机是否已经断开
        if (!camera_->valid()) {
            // 还有重连次数，则尝试重新创建相机实例
            if (retry_ > 0) {
                --retry_;
                const char* name = device_name_ ? device_name_->c_str() : nullptr;
                // 重新打开海康相机
                auto cam         = hikcamera::ImageCapturer::create(profile_, name);
                if (!cam) {
                    // 重连失败向上抛出错误
                    return std::unexpected(cam.error());
                }
                camera_ = std::move(cam.value());
            }
            // 断线断开错误
            return std::unexpected(Disconnected{result.error().what()});
        }
        // 普通读取失败，非断线
        return std::unexpected(result.error());
    }

    // 读取成功，重置重连计数
    retry_ = kReconnectRetryLimit;
    return Frame{
        .seq          = seq_++,
        .timestamp_ns = now_ns(),
        .image        = std::move(*result),
    };
}

/**
 * 获取相机标定参数
 */
auto HikInput::camera_info() const noexcept -> const CameraConfig& {
    return info_;
}

/**
 * @brief 修正帧时间戳
 * 原理：相机曝光是一段区间 [曝光起始时刻 ~ 曝光结束时刻]
 * 图像真正成像的有效时刻是曝光中点，所以用系统当前时间减去一半曝光时长
 * 修正后时间戳更贴合画面实际成像时刻，提升PnP、滤波时序精度
 */
auto HikInput::now_ns() const noexcept -> uint64_t {
    const auto now = clock::now_ns();
    return now - (exposure_time_ns_ / 2);
}

// ============================================================================
// CameraInterface：顶层统一相机接口，使用 std::variant 做多态封装
// mode_ 是 variant，内部要么存 IpcInput，要么存 HikInput
// 上层调用完全不需要关心底层是IPC共享内存还是直连海康相机
// ============================================================================

CameraInterface::CameraInterface(InputMode mode) noexcept
    : mode_(std::move(mode)) {}

/**
 * 统一非阻塞取帧，std::visit 自动分发到对应输入类的 try_recv
 */
auto CameraInterface::try_recv() noexcept -> std::optional<Frame> {
    return std::visit([](auto& input) { return input.try_recv(); }, mode_);
}

/**
 * 统一阻塞带超时取帧
 */
auto CameraInterface::recv(std::chrono::milliseconds timeout) noexcept
    -> std::expected<Frame, InputError> {
    return std::visit([timeout](auto& input) { return input.recv(timeout); }, mode_);
}

/**
 * 统一获取相机标定参数
 */
auto CameraInterface::camera_info() const noexcept -> const CameraConfig& {
    return std::visit(
        [](const auto& input) -> const CameraConfig& { return input.camera_info(); }, mode_);
}

// ============================================================================
// 工厂函数：两种创建相机的静态接口
// ============================================================================

/**
 * 创建IPC共享内存模式相机
 * @param client 共享内存客户端
 * @return 封装好的CameraInterface
 */
auto CameraInterface::create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept
    -> std::expected<CameraInterface, InputError> {
    // 从共享内存客户端读取相机标定信息
    const auto& ipc_info = client->camera_info();

    CameraConfig info;
    // 填充相机内参矩阵 3x3
    info.camera_matrix << ipc_info.fx, 0.0, ipc_info.cx,
                         0.0, ipc_info.fy, ipc_info.cy,
                         0.0, 0.0, 1.0;
    // 5阶畸变系数 k1,k2,p1,p2,k3
    info.distort_coefficient << ipc_info.distortion[0], ipc_info.distortion[1],
        ipc_info.distortion[2], ipc_info.distortion[3], ipc_info.distortion[4];
    info.width  = ipc_info.width;
    info.height = ipc_info.height;

    // 构造上层相机接口返回
    return CameraInterface(IpcInput(std::move(client), std::move(info)));
}

/**
 * 创建海康直连模式相机
 * @param config 相机完整配置（硬件参数+标定参数）
 */
auto CameraInterface::create_hik(const CameraConfig& config) noexcept
    -> std::expected<CameraInterface, InputError> {
    // 填充海康相机硬件配置结构体
    hikcamera::ImageCapturer::CameraProfile profile;
    profile.trigger_mode = config.profile.trigger_mode;    // 触发模式：软触发/硬触发
    profile.invert_image = config.profile.invert_image;    // 是否翻转图像
    profile.exposure_time =
        std::chrono::duration<float, std::micro>(config.profile.exposure_time_us); // 曝光
    profile.gain        = config.profile.gain;             // 模拟增益
    profile.rotate_type = config.profile.rotate_angle;     // 图像旋转角度

    // 打开海康相机设备
    const char* name = config.profile.device_name ? config.profile.device_name->c_str() : nullptr;
    auto result      = hikcamera::ImageCapturer::create(profile, name);
    if (!result) {
        // 相机打开失败返回错误
        return std::unexpected(result.error());
    }

    // 封装 HikInput 再塞进顶层 CameraInterface
    return CameraInterface(HikInput(
        std::move(*result), config, profile, profile.exposure_time, config.profile.device_name));
}

} // namespace fcs::L1