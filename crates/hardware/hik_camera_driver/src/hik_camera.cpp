#include "hik_camera.hpp"
#include "CameraParams.h"
#include "MvErrorDefine.h"
#include "magic_enum.hpp"

#include <atomic>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string_view>
#include <vector>

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ratio>
#include <tuple>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <MvCameraControl.h>
#include <PixelType.h>

namespace hikcamera {

std::string error_code_to_message(unsigned int code) noexcept {
    switch (code) {
    // Success
    case MV_OK: return "Success";

    // General errors
    case MV_E_HANDLE: return "Error or invalid handle";
    case MV_E_SUPPORT: return "Not supported function";
    case MV_E_BUFOVER: return "Buffer overflow";
    case MV_E_CALLORDER: return "Function calling order error";
    case MV_E_PARAMETER: return "Incorrect parameter";
    case MV_E_RESOURCE: return "Applying resource failed";
    case MV_E_NODATA: return "No data";
    case MV_E_PRECONDITION: return "Precondition error, or running environment changed";
    case MV_E_VERSION: return "Version mismatches";
    case MV_E_NOENOUGH_BUF: return "Insufficient memory";
    case MV_E_ABNORMAL_IMAGE:
        return "Abnormal image, maybe incomplete image because of lost packet";
    case MV_E_LOAD_LIBRARY: return "Load library failed";
    case MV_E_NOOUTBUF: return "No available buffer";
    case MV_E_ENCRYPT: return "Encryption error";
    case MV_E_OPENFILE: return "Open file error";
    case MV_E_BUF_IN_USE: return "Buffer already in use";
    case MV_E_BUF_INVALID: return "Buffer address invalid";
    case MV_E_NOALIGN_BUF: return "Buffer alignment error";
    case MV_E_NOENOUGH_BUF_NUM: return "Insufficient cache count";
    case MV_E_PORT_IN_USE: return "Port is in use";
    case MV_E_IMAGE_DECODEC: return "Decoding error (SDK verification image exception)";
    case MV_E_UINT32_LIMIT: return "Image size exceeds unsigned int return limit";
    case MV_E_IMAGE_HEIGHT: return "Image height anomaly (discard incomplete image)";
    case MV_E_NOENOUGH_DDR: return "Insufficient DDR cache";
    case MV_E_NOENOUGH_STREAM: return "Insufficient stream channel";
    case MV_E_NORESPONSE: return "No response from device";
    case MV_E_UNKNOW: return "Unknown error";

    // GenICam errors
    case MV_E_GC_GENERIC: return "General error (GenICam)";
    case MV_E_GC_ARGUMENT: return "Illegal parameters (GenICam)";
    case MV_E_GC_RANGE: return "The value is out of range (GenICam)";
    case MV_E_GC_PROPERTY: return "Property error (GenICam)";
    case MV_E_GC_RUNTIME: return "Running environment error (GenICam)";
    case MV_E_GC_LOGICAL: return "Logical error (GenICam)";
    case MV_E_GC_ACCESS: return "Node accessing condition error (GenICam)";
    case MV_E_GC_TIMEOUT: return "Timeout (GenICam)";
    case MV_E_GC_DYNAMICCAST: return "Transformation exception (GenICam)";
    case MV_E_GC_UNKNOW: return "GenICam unknown error";

    // GigE errors
    case MV_E_NOT_IMPLEMENTED: return "The command is not supported by device";
    case MV_E_INVALID_ADDRESS: return "The target address being accessed does not exist";
    case MV_E_WRITE_PROTECT: return "The target address is not writable";
    case MV_E_ACCESS_DENIED: return "No permission";
    case MV_E_BUSY: return "Device is busy, or network disconnected";
    case MV_E_PACKET: return "Network data packet error";
    case MV_E_NETER: return "Network error";
    case MV_E_SUPPORT_MODIFY_DEVICE_IP: return "Current mode not support modify IP";
    case MV_E_KEY_VERIFICATION: return "Switch key verification error";
    case MV_E_IP_CONFLICT: return "Device IP conflict";

    // USB errors
    case MV_E_USB_READ: return "Reading USB error";
    case MV_E_USB_WRITE: return "Writing USB error";
    case MV_E_USB_DEVICE: return "Device exception";
    case MV_E_USB_GENICAM: return "GenICam error (USB)";
    case MV_E_USB_BANDWIDTH: return "Insufficient bandwidth";
    case MV_E_USB_DRIVER: return "Driver mismatch or unmounted drive";
    case MV_E_USB_UNKNOW: return "USB unknown error";

    // Upgrade errors
    case MV_E_UPG_FILE_MISMATCH: return "Firmware mismatches";
    case MV_E_UPG_LANGUSGE_MISMATCH: return "Firmware language mismatches";
    case MV_E_UPG_CONFLICT: return "Upgrading conflicted";
    case MV_E_UPG_INNER_ERR: return "Camera internal error during upgrade";
    case MV_E_UPG_UNKNOW: return "Unknown error during upgrade";

    default: return fmt::format("Unknown error code: 0x{:08X}", code);
    }
}

// SDK 全局初始化/清理实现
// 使用 Meyer's Singleton 确保线程安全和正确的初始化顺序
namespace {
class SDKInitializer {
public:
    static SDKInitializer& instance() noexcept {
        static SDKInitializer init;
        return init;
    }

    std::expected<void, CameraError> initialize() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            return {}; // 已经初始化，直接返回成功
        }

        auto ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"init SDK", ret});
        }

        initialized_ = true;
        return {};
    }

    void finalize() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            MV_CC_Finalize();
            initialized_ = false;
        }
    }

private:
    SDKInitializer() noexcept = default;
    ~SDKInitializer() noexcept { finalize(); }

    std::mutex mutex_;
    bool initialized_ = false;
};
} // namespace

class ImageCapturer::Impl {
public:
    std::expected<void, CameraError> init(
        const CameraProfile& profile, const char* user_defined_name,
        const SyncMode& sync_mode) noexcept {
        auto init_sdk_result = SDKInitializer::instance().initialize();
        if (!init_sdk_result) {
            return std::unexpected(fmt::format("init SDK: {}", init_sdk_result.error()));
        }

        SPDLOG_INFO("searching camera..");
        auto device_info = search_camera(user_defined_name);
        if (!device_info) {
            return std::unexpected(fmt::format("search camera: {}", device_info.error()));
        }

        SPDLOG_INFO("initializing camera..");
        auto init_result = init_camera(*device_info, profile, sync_mode);
        if (!init_result) {
            return std::unexpected(fmt::format("init camera: {}", init_result.error()));
        }
        SPDLOG_INFO("camera: {}", collect_device_info(&*device_info));

        // 清除缓冲区中的残留帧（处理程序异常退出后的脏数据）
        // 相比 read() 等待超时，ClearImageBuffer 更直接且不会阻塞
        auto clear_ret = MV_CC_ClearImageBuffer(camera_handle_);
        if (clear_ret != MV_OK) {
            SPDLOG_WARN("clear image buffer: {}", error_code_to_message(clear_ret));
            // 非致命错误，继续
        }

        return {};
    }

    ~Impl() noexcept { uninit_camera(); }

    std::tuple<int, int> get_image_size() const noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        return {image_width_, image_height_};
    }

    std::expected<cv::Mat, CameraError>
        read(std::chrono::duration<unsigned int, std::micro> timeout) noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        if (camera_handle_ == nullptr) {
            return std::unexpected(CameraError{"get image buffer: camera handle is null"});
        }
        if (!callback_state_->valid.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"get image buffer: camera is not valid"});
        }
        if (!grabbing_) {
            return std::unexpected(CameraError{"get image buffer: camera is not grabbing"});
        }

        MV_FRAME_OUT stImageInfo{};

        // MV_CC_GetImageBuffer timeout 单位是毫秒（nMsec）
        // 注意：不要乘以 1000000，timeout.count() 已经是毫秒数
        auto ret = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, timeout.count());
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"get image buffer", ret});
        }

        // RAII 包装器确保 MV_CC_FreeImageBuffer 被调用一次
        struct ImageBufferGuard {
            void* handle;
            MV_FRAME_OUT* frame;
            bool released = false;

            ~ImageBufferGuard() noexcept {
                if (!released && frame != nullptr && handle != nullptr) {
                    MV_CC_FreeImageBuffer(handle, frame);
                }
            }

            void release() noexcept { released = true; }
        };

        ImageBufferGuard buffer_guard{camera_handle_, &stImageInfo};

        // 获取 Bayer 像素格式对应的 OpenCV 解拜耳代码
        const auto bayer_code = get_bayer_conversion_code(stImageInfo.stFrameInfo.enPixelType);
        if (!bayer_code) {
            return std::unexpected(
                fmt::format(
                    "unsupported bayer format: {}",
                    std::to_string(stImageInfo.stFrameInfo.enPixelType)));
        }

        // 包装原始 Bayer 数据（不拷贝数据，只创建 cv::Mat header）
        const int width  = stImageInfo.stFrameInfo.nWidth;
        const int height = stImageInfo.stFrameInfo.nHeight;
        cv::Mat raw_bayer{height, width, CV_8UC1, stImageInfo.pBufAddr};

        cv::Mat img;
        try {
            // 使用 OpenCV 的 EA（Edge-Aware，边缘感知）算法解拜耳
            // 相比普通插值更注重边缘方向，能减少边缘处的彩边和模糊
            // 在画质与速度之间通常比 VNG 更适合实时视觉场景
            cv::cvtColor(raw_bayer, img, *bayer_code);

            // 旋转图像（如果需要）
            if (rotate_type_ != RotateType::None) {
                switch (rotate_type_) {
                case RotateType::None: break;
                case RotateType::Clockwise90: cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE); break;
                case RotateType::Clockwise180: cv::rotate(img, img, cv::ROTATE_180); break;
                case RotateType::Clockwise270:
                    cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);
                    break;
                }
            }
        } catch (const cv::Exception& e) {
            return std::unexpected(fmt::format("convert image: {}", e.what()));
        } catch (const std::exception& e) {
            return std::unexpected(fmt::format("convert image: {}", e.what()));
        }

        // 缓存图像尺寸（供 get_width_height() 使用）
        image_width_  = width;
        image_height_ = height;

        // buffer_guard 析构时会自动调用 MV_CC_FreeImageBuffer
        return img;
    }

    std::expected<void, CameraError> software_trigger_on() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_handle_ == nullptr) {
            return std::unexpected(CameraError{"send software trigger: camera handle is null"});
        }
        if (!callback_state_->valid.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"send software trigger: camera is not valid"});
        }
        if (!grabbing_) {
            return std::unexpected(CameraError{"send software trigger: camera is not grabbing"});
        }
        auto ret = MV_CC_SetCommandValue(camera_handle_, "TriggerSoftware");
        if (MV_OK != ret) {
            return std::unexpected(CameraError{"send software trigger", ret});
        }
        return {};
    }

    std::expected<void, CameraError> set_frame_rate_inner_trigger_mode(float frame_rate) noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        if (camera_handle_ == nullptr) {
            return std::unexpected(CameraError{"set frame rate: camera handle is null"});
        }
        if (!callback_state_->valid.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"set frame rate: camera is not valid"});
        }

        // 在触发模式下，帧率由触发信号控制，AcquisitionFrameRate 设置无效
        if (trigger_mode_) {
            return std::unexpected("set frame rate in trigger mode");
        }

        auto ret = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
        if (MV_OK != ret) {
            return std::unexpected(CameraError{"set frame rate control enable", ret});
        }
        ret = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
        if (MV_OK != ret) {
            return std::unexpected(CameraError{"set frame rate", ret});
        }
        return {};
    }

    std::expected<void, CameraError> stop_grabbing() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_handle_ == nullptr) {
            return {};
        }
        if (!grabbing_) {
            return {};
        }
        auto ret = MV_CC_StopGrabbing(camera_handle_);
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"stop grabbing", ret});
        }
        grabbing_ = false;
        callback_state_->valid.store(false, std::memory_order_release);
        return {};
    }

    bool valid() const noexcept { return callback_state_->valid.load(std::memory_order_acquire); }

private:
    RotateType rotate_type_ = RotateType::None;

    // 缓存图像尺寸（在第一次 read() 后填充）
    int image_width_  = 0;
    int image_height_ = 0;

    // 记录当前是否处于触发模式（在 init_camera 时确定）
    bool trigger_mode_ = false;
    bool grabbing_     = false;
    struct CallbackState {
        std::atomic_bool active{false};
        std::atomic_bool valid{false};
        std::shared_ptr<spdlog::logger> logger;
    };

#define SDK_RET_ASSERT(ret, message)                           \
    do {                                                       \
        if ((ret) != MV_OK) {                                  \
            return std::unexpected(CameraError{message, ret}); \
        }                                                      \
    } while (false)

    template <typename Char, std::size_t N>
    static std::string_view sdk_string_view(const Char (&value)[N]) noexcept {
        const auto* data = reinterpret_cast<const char*>(value);
        const auto* end  = static_cast<const char*>(std::memchr(data, '\0', N));
        return {data, static_cast<std::size_t>(end == nullptr ? N : end - data)};
    }

    template <typename Char, std::size_t N>
    static bool sdk_string_equals(const Char (&value)[N], const char* target) noexcept {
        if (target == nullptr) {
            return false;
        }
        const auto view       = sdk_string_view(value);
        const auto target_len = std::strlen(target);
        return view.size() == target_len && std::memcmp(view.data(), target, view.size()) == 0;
    }

    static std::shared_ptr<spdlog::logger> make_logger() {
        auto logger = spdlog::get("hikcamera");
        if (!logger) {
            logger = spdlog::default_logger();
        }
        return logger;
    }

    static std::shared_ptr<CallbackState>
        make_callback_state(std::shared_ptr<spdlog::logger> logger) {
        auto state    = std::make_shared<CallbackState>();
        state->logger = std::move(logger);

        // SDK callbacks only carry a raw pUser. Keep states process-lifetime so a
        // late vendor callback cannot dereference a destroyed ImageCapturer::Impl.
        static auto* states      = new std::vector<std::shared_ptr<CallbackState>>();
        static auto* states_lock = new std::mutex();
        std::lock_guard<std::mutex> lock(*states_lock);
        states->push_back(state);

        return state;
    }

    std::string collect_device_info(MV_CC_DEVICE_INFO* pDeviceInfo) noexcept {
        // 辅助 lambda：获取字符串节点
        auto get_str = [&](const char* node) -> std::string {
            MVCC_STRINGVALUE strVal = {};
            int ret                 = MV_CC_GetStringValue(camera_handle_, node, &strVal);
            if (ret == MV_OK) {
                return std::string{sdk_string_view(strVal.chCurValue)};
            }
            return fmt::format("N/A [{}]", error_code_to_message(ret));
        };

        // 辅助 lambda：获取整型节点
        auto get_int = [&](const char* node) -> std::string {
            MVCC_INTVALUE_EX intVal = {};
            int ret                 = MV_CC_GetIntValueEx(camera_handle_, node, &intVal);
            if (ret == MV_OK) {
                return std::to_string(intVal.nCurValue);
            }
            return fmt::format("N/A [{}]", error_code_to_message(ret));
        };

        // 辅助 lambda：获取枚举的符号名
        auto get_enum_sym = [&](const char* node) -> std::string {
            MVCC_ENUMVALUE enumVal = {};
            int ret                = MV_CC_GetEnumValue(camera_handle_, node, &enumVal);
            if (ret != MV_OK) {
                return fmt::format("N/A [{}]", error_code_to_message(ret));
            }
            MVCC_ENUMENTRY entry = {};
            entry.nValue         = enumVal.nCurValue;
            ret                  = MV_CC_GetEnumEntrySymbolic(camera_handle_, node, &entry);
            if (ret == MV_OK) {
                return std::string{sdk_string_view(entry.chSymbolic)};
            }
            return std::to_string(enumVal.nCurValue);
        };

        // 解析 IP（仅 GigE）
        std::string ip_str = "N/A";
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            auto& gige = pDeviceInfo->SpecialInfo.stGigEInfo;
            ip_str     = fmt::format(
                "{}.{}.{}.{}", (gige.nCurrentIp >> 24) & 0xFF, (gige.nCurrentIp >> 16) & 0xFF,
                (gige.nCurrentIp >> 8) & 0xFF, gige.nCurrentIp & 0xFF);
        }

        return fmt::format(
            "{} {} (manuf: {}, ver: {}, fw: {}, sn: {}, user: {}, type: {}, scan: {}, "
            "link: {} Mbps, ip: {})",
            get_str("DeviceVendorName"), get_str("DeviceModelName"),
            get_str("DeviceManufacturerInfo"), get_str("DeviceVersion"),
            get_str("DeviceFirmwareVersion"), get_str("DeviceSerialNumber"),
            get_str("DeviceUserID"), get_enum_sym("DeviceType"), get_enum_sym("DeviceScanType"),
            get_int("DeviceLinkSpeed"), ip_str);
    }

    bool is_same_device_name(MV_CC_DEVICE_INFO* pstMVDevInfo, const char* targetName) noexcept {
        if (nullptr == pstMVDevInfo) {
            SPDLOG_LOGGER_ERROR(logger_, "The Pointer of pstMVDevInfo is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            return sdk_string_equals(
                pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName, targetName);
        } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            return sdk_string_equals(
                pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName, targetName);
        } else {
            return false;
        }
    }

    static std::string format_device_info(MV_CC_DEVICE_INFO* info) noexcept {
        if (info->nTLayerType == MV_GIGE_DEVICE) {
            auto& gige = info->SpecialInfo.stGigEInfo;
            return fmt::format(
                "[GigE] name: {}, model: {}, sn: {}, ip: {}.{}.{}.{}",
                sdk_string_view(gige.chUserDefinedName), sdk_string_view(gige.chModelName),
                sdk_string_view(gige.chSerialNumber), (gige.nCurrentIp >> 24) & 0xFF,
                (gige.nCurrentIp >> 16) & 0xFF, (gige.nCurrentIp >> 8) & 0xFF,
                gige.nCurrentIp & 0xFF);
        } else if (info->nTLayerType == MV_USB_DEVICE) {
            auto& usb = info->SpecialInfo.stUsb3VInfo;
            return fmt::format(
                "[USB] name: {}, model: {}, sn: {}", sdk_string_view(usb.chUserDefinedName),
                sdk_string_view(usb.chModelName), sdk_string_view(usb.chSerialNumber));
        }
        return "[Unknown transport type]";
    }

    std::expected<MV_CC_DEVICE_INFO, CameraError>
        search_camera(const char* user_defined_name) noexcept {
        MV_CC_DEVICE_INFO_LIST device_list;
        memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        auto ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"enum devices", ret});
        }
        if (device_list.nDeviceNum == 0) {
            return std::unexpected("no devices");
        }

        if (user_defined_name == nullptr) {
            if (device_list.nDeviceNum > 1) {
                SPDLOG_LOGGER_INFO(logger_, "found {} devices:", device_list.nDeviceNum);
                for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
                    SPDLOG_LOGGER_INFO(
                        logger_, "  [{}]: {}", i, format_device_info(device_list.pDeviceInfo[i]));
                }
                return std::unexpected(
                    fmt::format(
                        "must pass in the device name because {} devices were found",
                        device_list.nDeviceNum));
            }
            return *device_list.pDeviceInfo[0];
        } else {
            for (auto i = 0; i < device_list.nDeviceNum; i++) {
                if (is_same_device_name(device_list.pDeviceInfo[i], user_defined_name))
                    return *device_list.pDeviceInfo[i];
            }
            SPDLOG_LOGGER_INFO(logger_, "found {} devices:", device_list.nDeviceNum);
            for (auto i = 0; i < device_list.nDeviceNum; i++) {
                SPDLOG_LOGGER_INFO(
                    logger_, "  [{}]: {}", i, format_device_info(device_list.pDeviceInfo[i]));
            }
            return std::unexpected(
                fmt::format(
                    "{} devices was found, but no device matches the name passed in: {}",
                    device_list.nDeviceNum, user_defined_name));
        }
    }

    std::expected<void, CameraError> init_camera(
        MV_CC_DEVICE_INFO& device_info, const CameraProfile& profile,
        const SyncMode& sync_mode) noexcept {
        auto pDeviceInfo = &device_info;
        rotate_type_     = profile.rotate_type;

        int ret;

        // 使用 MV_CC_CreateHandle 而非已废弃的 MV_CC_CreateHandleWithoutLog
        // 参考文档：Deprecated camera initialization related APIs: MV_CC_CreateHandleWithoutLog()
        ret = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
        SDK_RET_ASSERT(ret, "create handle");
        FinalAction destroy_handle{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_DestroyHandle(camera_handle_);
                camera_handle_ = nullptr;
            }
        }};

        ret = MV_CC_OpenDevice(camera_handle_);
        SDK_RET_ASSERT(ret, "open device");
        FinalAction close_device{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_CloseDevice(camera_handle_);
            }
        }};

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
            if (nPacketSize <= 0) {
                return std::unexpected(
                    CameraError{fmt::format("invalid packet size: {}", nPacketSize), 0});
            }

            // 使用 MV_CC_SetIntValueEx 而非 MV_CC_SetIntValue（与官方示例保持一致）
            ret = MV_CC_SetIntValueEx(camera_handle_, "GevSCPSPacketSize", nPacketSize);
            SDK_RET_ASSERT(ret, "set packet Size");

            ret = MV_CC_SetIntValueEx(camera_handle_, "GevHeartbeatTimeout", 3000);
            SDK_RET_ASSERT(ret, "set heartbeat timeout");

            ret = MV_CC_SetBoolValue(camera_handle_, "FrameTimeoutEnable", true);
            SDK_RET_ASSERT(ret, "enable frame timeout");
            ret = MV_CC_SetIntValueEx(camera_handle_, "FrameTimeoutTime", 2000); // 2秒
            SDK_RET_ASSERT(ret, "set frame timeout time");
        }

        // Set pixel format to 8-bit BayerRG for ADC depth
        ret = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
        SDK_RET_ASSERT(ret, "set pixel format to BayerRG8");

        // 根据 profile 和 sync_mode 统一设置 TriggerMode，避免重复设置
        bool trigger_on = profile.trigger_mode || (sync_mode == SyncMode::SOFTWARE);
        trigger_mode_   = trigger_on;
        ret             = MV_CC_SetEnumValue(
            camera_handle_, "TriggerMode", trigger_on ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
        SDK_RET_ASSERT(ret, "set trigger Mode");

        ret = MV_CC_SetBoolValue(camera_handle_, "ReverseX", profile.invert_image);
        SDK_RET_ASSERT(ret, "set reverse x");
        ret = MV_CC_SetBoolValue(camera_handle_, "ReverseY", profile.invert_image);
        SDK_RET_ASSERT(ret, "set reverse y");

        ret = MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        SDK_RET_ASSERT(ret, "set auto exposure");

        ret = MV_CC_SetEnumValue(camera_handle_, "ADCBitDepth", profile.adc_depth);
        if (ret != MV_OK) {
            SPDLOG_LOGGER_WARN(
                logger_, "set adc bit depth to {}: {}", magic_enum::enum_name(profile.adc_depth),
                error_code_to_message(ret));
        }

        ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", profile.exposure_time.count());
        SDK_RET_ASSERT(ret, "set exposure time");

        ret = MV_CC_SetEnumValue(camera_handle_, "GainAuto", MV_GAIN_MODE_OFF);
        SDK_RET_ASSERT(ret, "set auto gain off");

        ret = MV_CC_SetFloatValue(camera_handle_, "Gain", profile.gain);
        SDK_RET_ASSERT(ret, "set gain");

        ret = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        SDK_RET_ASSERT(ret, "set acquisition mode to continuous");

        ret = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
        SDK_RET_ASSERT(ret, "set acquisition frame rate enable");

        ret = MV_CC_SetBayerCvtQuality(camera_handle_, 1);
        SDK_RET_ASSERT(ret, "set bayer cvt quality");

        // 文档建议：在 MV_CC_StartGrabbing() 之前调用 MV_CC_SetImageNodeNum()
        // 设置合适的缓冲节点数量，影响因子包括：相机帧率、图像分辨率、计算机性能
        // 注意：此处使用默认值，如需优化可根据实际情况调整
        ret = MV_CC_SetImageNodeNum(camera_handle_, 5);
        SDK_RET_ASSERT(ret, "set image node num");

        // 设置取流策略为 LatestImagesOnly（实时场景优先取最新帧，丢弃积压）
        // 注意：必须在 StartGrabbing 之前调用；Linux 下仅 USB 设备支持
        ret = MV_CC_SetGrabStrategy(camera_handle_, MV_GrabStrategy_LatestImagesOnly);
        if (ret != MV_OK) {
            SPDLOG_LOGGER_WARN(
                logger_,
                "set grab strategy (may not be supported on this "
                "platform/device): {}",
                error_code_to_message(ret));
        }

        callback_state_->active.store(true, std::memory_order_release);
        FinalAction deactivate_callbacks{[state = callback_state_]() {
            state->valid.store(false, std::memory_order_release);
            state->active.store(false, std::memory_order_release);
        }};

        // 注册设备断连回调，及时感知相机离线（必须在 StartGrabbing 之前）
        ret = MV_CC_RegisterExceptionCallBack(
            camera_handle_, Impl::on_exception_callback, callback_state_.get());
        SDK_RET_ASSERT(ret, "register exception callback");

        FinalAction unregister_callbacks{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_RegisterExceptionCallBack(camera_handle_, nullptr, nullptr);
            }
        }};

        // 注册流异常回调，及时感知丢包/断流/图像异常（必须在 StartGrabbing 之前）
        ret = MV_CC_RegisterStreamExceptionCallBack(
            camera_handle_, Impl::on_stream_exception_callback, callback_state_.get());
        SDK_RET_ASSERT(ret, "register stream exception callback");

        // SOFTWARE 模式下设置 TriggerSource 为 SOFTWARE。SDK 示例在 StartGrabbing 前设置。
        if (sync_mode == SyncMode::SOFTWARE) {
            ret = MV_CC_SetEnumValue(camera_handle_, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
            SDK_RET_ASSERT(ret, "set trigger source: soft trigger");
        }

        callback_state_->valid.store(true, std::memory_order_release);
        ret = MV_CC_StartGrabbing(camera_handle_);
        SDK_RET_ASSERT(ret, "start grabbing");
        grabbing_ = true;

        FinalAction stop_grabbing{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_StopGrabbing(camera_handle_);
            }
            grabbing_ = false;
        }};

        stop_grabbing.disable();
        unregister_callbacks.disable();
        deactivate_callbacks.disable();
        destroy_handle.disable();
        close_device.disable();
        return {};
    }

    // 将 HIK SDK 的 Bayer 像素格式映射到 OpenCV 的解拜耳代码
    // 参考：OpenCV cvtColor 文档中 Bayer 模式定义
    static std::expected<int, CameraError>
        get_bayer_conversion_code(MvGvspPixelType enType) noexcept {
        switch (enType) {
        // 8-bit Bayer 格式（推荐使用）
        case PixelType_Gvsp_BayerRG8: return cv::COLOR_BayerBG2BGR_EA;
        case PixelType_Gvsp_BayerBG8: return cv::COLOR_BayerRG2BGR_EA;
        case PixelType_Gvsp_BayerGR8: return cv::COLOR_BayerGB2BGR_EA;
        case PixelType_Gvsp_BayerGB8: return cv::COLOR_BayerGR2BGR_EA;

        // 10-bit Bayer 格式（需要预处理）
        case PixelType_Gvsp_BayerRG10:
        case PixelType_Gvsp_BayerRG10_Packed:
        case PixelType_Gvsp_BayerBG10:
        case PixelType_Gvsp_BayerBG10_Packed:
        case PixelType_Gvsp_BayerGR10:
        case PixelType_Gvsp_BayerGR10_Packed:
        case PixelType_Gvsp_BayerGB10:
        case PixelType_Gvsp_BayerGB10_Packed:
            return std::unexpected(
                CameraError{"10-bit Bayer not supported, please set to 8-bit mode"});

        // 12-bit Bayer 格式（需要预处理）
        case PixelType_Gvsp_BayerRG12:
        case PixelType_Gvsp_BayerRG12_Packed:
        case PixelType_Gvsp_BayerBG12:
        case PixelType_Gvsp_BayerBG12_Packed:
        case PixelType_Gvsp_BayerGR12:
        case PixelType_Gvsp_BayerGR12_Packed:
        case PixelType_Gvsp_BayerGB12:
        case PixelType_Gvsp_BayerGB12_Packed:
            return std::unexpected(
                CameraError{"12-bit Bayer not supported, please set to 8-bit mode"});

        // 已解拜耳的格式（非 Bayer）
        case PixelType_Gvsp_BGR8_Packed:
        case PixelType_Gvsp_YUV422_Packed:
        case PixelType_Gvsp_YUV422_YUYV_Packed:
            return std::unexpected(CameraError{"already debayered"});

        default:
            return std::unexpected(CameraError{"unknown pixel format: " + std::to_string(enType)});
        }
    }

    void uninit_camera() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        // 如果句柄未初始化（构造失败路径），无需清理
        if (camera_handle_ == nullptr) {
            return;
        }

        callback_state_->valid.store(false, std::memory_order_release);
        callback_state_->active.store(false, std::memory_order_release);

        // 按照官方文档的顺序清理资源
        // StopGrabbing → CloseDevice → DestroyHandle

        auto nRet = MV_OK;

        if (grabbing_) {
            nRet = MV_CC_StopGrabbing(camera_handle_);
            if (MV_OK != nRet) {
                SPDLOG_LOGGER_ERROR(logger_, "stop grabbing: {}", error_code_to_message(nRet));
                // 即使失败也继续尝试清理，确保尽可能释放资源
            }
            grabbing_ = false;
        }

        nRet = MV_CC_RegisterExceptionCallBack(camera_handle_, nullptr, nullptr);
        if (MV_OK != nRet) {
            SPDLOG_LOGGER_ERROR(
                logger_, "unregister exception callback: {}", error_code_to_message(nRet));
        }

        nRet = MV_CC_CloseDevice(camera_handle_);
        if (MV_OK != nRet) {
            SPDLOG_LOGGER_ERROR(logger_, "close device: {}", error_code_to_message(nRet));
            // 即使失败也继续尝试销毁句柄
        }

        nRet = MV_CC_DestroyHandle(camera_handle_);
        if (MV_OK != nRet) {
            SPDLOG_LOGGER_ERROR(logger_, "destroy handle: {}", error_code_to_message(nRet));
            // 句柄销毁失败是最严重的情况，但我们已经尽力了
        }

        camera_handle_ = nullptr;
    }

    template <typename Func>
    struct FinalAction {
        explicit FinalAction(Func func) noexcept
            : clean_{func}
            , enabled_(true) {}

        FinalAction(const FinalAction&)            = delete;
        FinalAction& operator=(const FinalAction&) = delete;
        FinalAction(FinalAction&&)                 = delete;
        FinalAction& operator=(FinalAction&&)      = delete;

        ~FinalAction() noexcept {
            if (enabled_)
                clean_();
        }

        void disable() noexcept { enabled_ = false; }

    private:
        Func clean_;
        bool enabled_;
    };

    void* camera_handle_ = nullptr;
    mutable std::mutex camera_mutex_;
    std::shared_ptr<spdlog::logger> logger_        = make_logger();
    std::shared_ptr<CallbackState> callback_state_ = make_callback_state(logger_);

    // 设备断连回调（通过 MV_CC_RegisterExceptionCallBack 注册）
    static void __stdcall on_exception_callback(unsigned int nMsgType, void* pUser) noexcept {
        auto* state = static_cast<CallbackState*>(pUser);
        if (state == nullptr || !state->active.load(std::memory_order_acquire)) {
            return;
        }
        state->valid.store(false, std::memory_order_release);
        if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT) {
            SPDLOG_LOGGER_CRITICAL(state->logger, "device disconnected!");
        } else {
            SPDLOG_LOGGER_ERROR(state->logger, "device: {}", exception_type_to_string(nMsgType));
        }
    }

    // 流异常回调（通过 MV_CC_RegisterStreamExceptionCallBack 注册）
    static void __stdcall
        on_stream_exception_callback(MV_CC_STREAM_EXCEPTION_INFO* pstInfo, void* pUser) noexcept {
        auto* state = static_cast<CallbackState*>(pUser);
        if (state == nullptr || pstInfo == nullptr
            || !state->active.load(std::memory_order_acquire)) {
            return;
        }
        if (pstInfo->enExceptionType == MV_CC_STREAM_EXCEPTION_DISCONNECTED
            || pstInfo->enExceptionType == MV_CC_STREAM_EXCEPTION_DEVICE) {
            state->valid.store(false, std::memory_order_release);
        }
        if (pstInfo->enExceptionType != MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW) {
            const auto serial_number = std::string{sdk_string_view(pstInfo->chSerialNumber)};
            SPDLOG_LOGGER_ERROR(
                state->logger, "stream: {}, serial={}",
                stream_exception_type_to_string(pstInfo->enExceptionType), serial_number);
        }
    }

    static const char* exception_type_to_string(unsigned int nMsgType) noexcept {
        switch (nMsgType) {
        case MV_EXCEPTION_DEV_DISCONNECT: return "device disconnected";
        case MV_EXCEPTION_VERSION_CHECK: return "SDK/driver version mismatch";
        default: return "unknown";
        }
    }

    static const char* stream_exception_type_to_string(MV_CC_STREAM_EXCEPTION_TYPE type) noexcept {
        switch (type) {
        case MV_CC_STREAM_EXCEPTION_ABNORMAL_IMAGE: return "abnormal image (discarded)";
        case MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW:
            return "buffer list full (frames not consumed in time)";
        case MV_CC_STREAM_EXCEPTION_LIST_EMPTY: return "buffer list empty";
        case MV_CC_STREAM_EXCEPTION_RECONNECTION: return "stream reconnection triggered";
        case MV_CC_STREAM_EXCEPTION_DISCONNECTED:
            return "stream reconnection failed, grabbing stopped";
        case MV_CC_STREAM_EXCEPTION_DEVICE: return "device exception, grabbing stopped";
        case MV_CC_STREAM_EXCEPTION_PARTIAL_IMAGE:
            return "partial image (insufficient line height, discarded)";
        case MV_CC_STREAM_EXCEPTION_IMAGE_BUFFER_OVERFLOW:
            return "image data exceeds buffer capacity (discarded)";
        default: return "unknown";
        }
    }
};

// Factory method implementation
std::expected<std::unique_ptr<ImageCapturer>, CameraError> ImageCapturer::create(
    const CameraProfile& profile, const char* user_defined_name,
    const SyncMode& sync_mode) noexcept {
    auto capturer    = std::make_unique<ImageCapturer>();
    auto init_result = capturer->init(profile, user_defined_name, sync_mode);
    if (!init_result) {
        return std::unexpected(init_result.error());
    }
    return capturer;
}

ImageCapturer::ImageCapturer() noexcept
    : impl_(std::make_unique<ImageCapturer::Impl>()) {}

ImageCapturer::~ImageCapturer() noexcept = default;

std::expected<void, CameraError> ImageCapturer::init(
    const CameraProfile& profile, const char* user_defined_name,
    const SyncMode& sync_mode) noexcept {
    return impl_->init(profile, user_defined_name, sync_mode);
}

std::expected<void, CameraError> ImageCapturer::software_trigger_on() noexcept {
    return impl_->software_trigger_on();
}

std::expected<void, CameraError>
    ImageCapturer::set_frame_rate_inner_trigger_mode(float frame_rate) noexcept {
    return impl_->set_frame_rate_inner_trigger_mode(frame_rate);
}

std::expected<void, CameraError> ImageCapturer::stop_grabbing() noexcept {
    return impl_->stop_grabbing();
}

std::expected<cv::Mat, CameraError>
    ImageCapturer::read(std::chrono::duration<unsigned int, std::micro> timeout) noexcept {
    return impl_->read(timeout);
}

std::expected<std::tuple<int, int>, CameraError> ImageCapturer::get_width_height() const noexcept {
    return impl_->get_image_size();
}

bool ImageCapturer::valid() const noexcept { return impl_->valid(); }

} // namespace hikcamera
