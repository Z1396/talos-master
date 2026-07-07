// 引入对外统一相机头文件，包含类、结构体、错误类型、枚举声明
#include "hik_camera.hpp"
// 海康SDK相机参数结构体定义
#include "CameraParams.h"
// 海康SDK错误码枚举定义
#include "MvErrorDefine.h"
// 枚举反射工具，打印枚举名称
#include "magic_enum.hpp"

// C++标准库
#include <atomic>        // 多线程无锁状态标记
#include <cstring>       // memset、memchr、memcmp 内存操作
#include <exception>     // 标准异常基类
#include <memory>        // unique_ptr/shared_ptr 智能指针
#include <mutex>         // 线程互斥锁
#include <string_view>   // 零拷贝只读字符串视图
#include <vector>        // 动态数组存储全局回调状态

// 时间处理
#include <chrono>
// OpenCV 图像处理
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ratio>
#include <tuple>

// 日志库
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

// 海康相机核心SDK接口、像素类型定义
#include <MvCameraControl.h>
#include <PixelType.h>

namespace hikcamera {

/**
 * @brief 将海康SDK数字错误码转换为人类可读文本
 * @param code SDK返回的无符号整型错误码
 * @return 对应错误描述字符串，未知错误输出十六进制码
 * noexcept 保证函数不会抛出C++异常
 */
std::string error_code_to_message(unsigned int code) noexcept {
    switch (code) {
    // 执行成功
    case MV_OK: return "Success";

    // 通用基础错误
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

    // GenICam 相机标准寄存器相关错误
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

    // 千兆网GigE相机专属错误
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

    // USB工业相机专属错误
    case MV_E_USB_READ: return "Reading USB error";
    case MV_E_USB_WRITE: return "Writing USB error";
    case MV_E_USB_DEVICE: return "Device exception";
    case MV_E_USB_GENICAM: return "GenICam error (USB)";
    case MV_E_USB_BANDWIDTH: return "Insufficient bandwidth";
    case MV_E_USB_DRIVER: return "Driver mismatch or unmounted drive";
    case MV_E_USB_UNKNOW: return "USB unknown error";

    // 固件升级相关错误
    case MV_E_UPG_FILE_MISMATCH: return "Firmware mismatches";
    case MV_E_UPG_LANGUSGE_MISMATCH: return "Firmware language mismatches";
    case MV_E_UPG_CONFLICT: return "Upgrading conflicted";
    case MV_E_UPG_INNER_ERR: return "Camera internal error during upgrade";
    case MV_E_UPG_UNKNOW: return "Unknown error during upgrade";

    // 未匹配到任何已知错误码，输出十六进制原始码
    default: return fmt::format("Unknown error code: 0x{:08X}", code);
    }
}

// SDK全局初始化/销毁单例实现
// 采用Meyer单例：静态局部变量，C++11起线程安全，自动生命周期管理
namespace {
class SDKInitializer {
public:
    /**
     * @brief 获取单例全局实例
     * @return 静态唯一SDK初始化器引用
     */
    static SDKInitializer& instance() noexcept {
        static SDKInitializer init;
        return init;
    }

    /**
     * @brief 全局初始化海康相机SDK
     * @return 成功空，失败携带错误信息
     */
    std::expected<void, CameraError> initialize() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        // 已初始化直接返回成功，避免重复初始化
        if (initialized_) {
            return {};
        }

        // 调用海康SDK全局初始化接口
        auto ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"init SDK", ret});
        }

        initialized_ = true;
        return {};
    }

    /**
     * @brief 全局释放SDK资源
     */
    void finalize() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            MV_CC_Finalize();
            initialized_ = false;
        }
    }

private:
    // 私有构造，禁止外部实例化
    SDKInitializer() noexcept = default;
    // 程序退出自动析构，释放SDK
    ~SDKInitializer() noexcept { finalize(); }

    std::mutex mutex_;          // 多线程初始化互斥锁
    bool initialized_ = false;  // SDK是否已初始化标记
};
} // 匿名命名空间，隔离单例，仅当前文件可见

/**
 * @brief 相机采集器私有内部实现类（PIMPL模式，隐藏所有SDK底层细节）
 */
class ImageCapturer::Impl {
public:
    /**
     * @brief 相机完整初始化流程：SDK初始化→搜索设备→打开相机→配置参数→开始取流
     * @param profile 相机用户配置（曝光、增益、翻转、ADC深度等）
     * @param user_defined_name 相机自定义设备名，用于多相机区分
     * @param sync_mode 同步模式：硬件触发/软件触发/连续采集
     * @return 初始化结果
     */
    std::expected<void, CameraError> init(
        const CameraProfile& profile, const char* user_defined_name,
        const SyncMode& sync_mode) noexcept {
        // 第一步：初始化全局海康SDK
        auto init_sdk_result = SDKInitializer::instance().initialize();
        if (!init_sdk_result) {
            return std::unexpected(fmt::format("init SDK: {}", init_sdk_result.error()));
        }

        SPDLOG_INFO("searching camera..");
        // 搜索匹配名称的相机设备
        auto device_info = search_camera(user_defined_name);
        if (!device_info) {
            return std::unexpected(fmt::format("search camera: {}", device_info.error()));
        }

        SPDLOG_INFO("initializing camera..");
        // 打开设备、配置所有相机寄存器参数
        auto init_result = init_camera(*device_info, profile, sync_mode);
        if (!init_result) {
            return std::unexpected(fmt::format("init camera: {}", init_result.error()));
        }
        // 打印相机完整硬件信息
        SPDLOG_INFO("camera: {}", collect_device_info(&*device_info));

        // 清空相机内部残留图像缓冲区，防止上一次异常退出遗留脏帧
        auto clear_ret = MV_CC_ClearImageBuffer(camera_handle_);
        if (clear_ret != MV_OK) {
            SPDLOG_WARN("clear image buffer: {}", error_code_to_message(clear_ret));
            // 清空缓冲区失败属于非致命错误，继续运行
        }

        return {};
    }

    /**
     * @brief 析构函数：自动关闭相机、释放所有SDK资源
     */
    ~Impl() noexcept { uninit_camera(); }

    /**
     * @brief 获取当前相机输出图像宽高
     * @return 宽高二元组
     */
    std::tuple<int, int> get_image_size() const noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        return {image_width_, image_height_};
    }

    /**
     * @brief 同步阻塞读取一帧图像
     * @param timeout 读取超时时间（微秒）
     * @return 解码后RGB OpenCV Mat，失败返回错误
     */
    std::expected<cv::Mat, CameraError>
        read(std::chrono::duration<unsigned int, std::micro> timeout) noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        // 合法性前置校验
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

        // SDK读取接口超时单位为毫秒，转换微秒到毫秒
        auto ret = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, timeout.count() / 1000);
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"get image buffer", ret});
        }

        // RAII自动释放图像缓冲区，防止内存泄漏
        struct ImageBufferGuard {
            void* handle;
            MV_FRAME_OUT* frame;
            bool released = false;

            // 离开作用域自动释放SDK图像缓存
            ~ImageBufferGuard() noexcept {
                if (!released && frame != nullptr && handle != nullptr) {
                    MV_CC_FreeImageBuffer(handle, frame);
                }
            }

            // 手动取消自动释放（本代码不需要，预留扩展）
            void release() noexcept { released = true; }
        };

        ImageBufferGuard buffer_guard{camera_handle_, &stImageInfo};

        // 根据相机原始Bayer格式获取OpenCV解拜耳转换码
        const auto bayer_code = get_bayer_conversion_code(stImageInfo.stFrameInfo.enPixelType);
        if (!bayer_code) {
            return std::unexpected(
                fmt::format(
                    "unsupported bayer format: {}",
                    std::to_string(stImageInfo.stFrameInfo.enPixelType)));
        }

        // 构建原始Bayer单通道Mat，不拷贝数据，仅包装内存头
        const int width  = stImageInfo.stFrameInfo.nWidth;
        const int height = stImageInfo.stFrameInfo.nHeight;
        cv::Mat raw_bayer{height, width, CV_8UC1, stImageInfo.pBufAddr};

        cv::Mat img;
        try {
            // EA边缘感知插值解拜耳，实时视觉场景画质优于普通插值
            cv::cvtColor(raw_bayer, img, *bayer_code);

            // 根据配置旋转图像
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

        // 缓存图像分辨率，供外部查询接口使用
        image_width_  = width;
        image_height_ = height;

        // 缓冲区RAII自动释放，返回转换完成的RGB图像
        return img;
    }

    /**
     * @brief 发送软件触发信号（软触发模式专用）
     * @return 发送结果
     */
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
        // 发送软件触发寄存器指令
        auto ret = MV_CC_SetCommandValue(camera_handle_, "TriggerSoftware");
        if (MV_OK != ret) {
            return std::unexpected(CameraError{"send software trigger", ret});
        }
        return {};
    }

    /**
     * @brief 设置连续采集模式下的固定帧率（触发模式下帧率无效）
     * @param frame_rate 目标帧率
     * @return 配置结果
     */
    std::expected<void, CameraError> set_frame_rate_inner_trigger_mode(float frame_rate) noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        if (camera_handle_ == nullptr) {
            return std::unexpected(CameraError{"set frame rate: camera handle is null"});
        }
        if (!callback_state_->valid.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"set frame rate: camera is not valid"});
        }

        // 触发模式下帧率寄存器被相机硬件锁定，无法配置
        if (trigger_mode_) {
            return std::unexpected("set frame rate in trigger mode");
        }

        // 开启帧率控制开关
        auto ret = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
        if (MV_OK != ret) {
            return std::unexpected(CameraError{"set frame rate control enable", ret});
        }
        // 写入目标帧率
        ret = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
        if (MV_OK != ret) {
            return std::unexpected(CameraError{"set frame rate", ret});
        }
        return {};
    }

    /**
     * @brief 停止相机取流
     * @return 停止结果，未打开相机直接返回成功
     */
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

    /**
     * @brief 判断相机设备是否正常在线可用
     * @return true=正常，false=断连/未初始化
     */
    bool valid() const noexcept { return callback_state_->valid.load(std::memory_order_acquire); }

private:
    // 图像旋转配置
    RotateType rotate_type_ = RotateType::None;

    // 缓存上一帧图像宽高，避免每次读取都查询寄存器
    int image_width_  = 0;
    int image_height_ = 0;

    // 标记当前相机是否处于触发模式（软/硬触发）
    bool trigger_mode_ = false;
    // 标记是否正在持续取流
    bool grabbing_     = false;

    /**
     * @brief 全局回调共享状态结构体
     * 相机断连、流异常回调需要访问状态，生命周期全局持久化，防止野指针
     */
    struct CallbackState {
        std::atomic_bool active{false}; // 回调是否启用
        std::atomic_bool valid{false};  // 相机是否有效在线
        std::shared_ptr<spdlog::logger> logger; // 日志句柄
    };

    // 宏：SDK返回值校验，失败直接返回错误
#define SDK_RET_ASSERT(ret, message)                           \
    do {                                                       \
        if ((ret) != MV_OK) {                                  \
            return std::unexpected(CameraError{message, ret}); \
        }                                                      \
    } while (false)

    /**
     * @brief 将SDK定长字符数组转为只读字符串视图（零拷贝）
     * @tparam Char 字符类型
     * @tparam N 数组长度
     * @param value SDK字符数组
     * @return std::string_view 不分配堆内存
     */
    template <typename Char, std::size_t N>
    static std::string_view sdk_string_view(const Char (&value)[N]) noexcept {
        const auto* data = reinterpret_cast<const char*>(value);
        // 查找字符串结束符'\0'
        const auto* end  = static_cast<const char*>(std::memchr(data, '\0', N));
        return {data, static_cast<std::size_t>(end == nullptr ? N : end - data)};
    }

    /**
     * @brief 判断SDK字符数组与目标字符串是否完全相等
     * @tparam Char 字符数组类型
     * @tparam N 数组长度
     * @param value SDK字符串数组
     * @param target 对比目标字符串
     * @return 相等返回true
     */
    template <typename Char, std::size_t N>
    static bool sdk_string_equals(const Char (&value)[N], const char* target) noexcept {
        if (target == nullptr) {
            return false;
        }
        const auto view       = sdk_string_view(value);
        const auto target_len = std::strlen(target);
        return view.size() == target_len && std::memcmp(view.data(), target, view.size()) == 0;
    }

    /**
     * @brief 获取相机专用日志实例，无则使用全局默认日志
     */
    static std::shared_ptr<spdlog::logger> make_logger() {
        auto logger = spdlog::get("hikcamera");
        if (!logger) {
            logger = spdlog::default_logger();
        }
        return logger;
    }

    /**
     * @brief 创建全局持久化回调状态对象
     * SDK回调仅传递裸指针，Impl销毁后回调仍可能触发，所以全局静态容器持有
     * @param logger 日志句柄
     * @return 共享指针回调状态
     */
    static std::shared_ptr<CallbackState>
        make_callback_state(std::shared_ptr<spdlog::logger> logger) {
        auto state    = std::make_shared<CallbackState>();
        state->logger = std::move(logger);

        // 全局静态容器永久持有所有回调状态，防止回调野指针
        static auto* states      = new std::vector<std::shared_ptr<CallbackState>>();
        static auto* states_lock = new std::mutex();
        std::lock_guard<std::mutex> lock(*states_lock);
        states->push_back(state);

        return state;
    }

    /**
     * @brief 收集相机全部设备信息，格式化输出字符串
     * @param pDeviceInfo SDK设备信息结构体
     * @return 完整设备信息文本
     */
    std::string collect_device_info(MV_CC_DEVICE_INFO* pDeviceInfo) noexcept {
        // 读取字符串寄存器工具lambda
        auto get_str = [&](const char* node) -> std::string {
            MVCC_STRINGVALUE strVal = {};
            int ret                 = MV_CC_GetStringValue(camera_handle_, node, &strVal);
            if (ret == MV_OK) {
                return std::string{sdk_string_view(strVal.chCurValue)};
            }
            return fmt::format("N/A [{}]", error_code_to_message(ret));
        };

        // 读取整型寄存器工具lambda
        auto get_int = [&](const char* node) -> std::string {
            MVCC_INTVALUE_EX intVal = {};
            int ret                 = MV_CC_GetIntValueEx(camera_handle_, node, &intVal);
            if (ret == MV_OK) {
                return std::to_string(intVal.nCurValue);
            }
            return fmt::format("N/A [{}]", error_code_to_message(ret));
        };

        // 读取枚举寄存器符号名工具lambda
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

        // 解析GigE相机IP地址
        std::string ip_str = "N/A";
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            auto& gige = pDeviceInfo->SpecialInfo.stGigEInfo;
            ip_str     = fmt::format(
                "{}.{}.{}.{}", (gige.nCurrentIp >> 24) & 0xFF, (gige.nCurrentIp >> 16) & 0xFF,
                (gige.nCurrentIp >> 8) & 0xFF, gige.nCurrentIp & 0xFF);
        }

        // 拼接全部设备参数
        return fmt::format(
            "{} {} (manuf: {}, ver: {}, fw: {}, sn: {}, user: {}, type: {}, scan: {}, "
            "link: {} Mbps, ip: {})",
            get_str("DeviceVendorName"), get_str("DeviceModelName"),
            get_str("DeviceManufacturerInfo"), get_str("DeviceVersion"),
            get_str("DeviceFirmwareVersion"), get_str("DeviceSerialNumber"),
            get_str("DeviceUserID"), get_enum_sym("DeviceType"), get_enum_sym("DeviceScanType"),
            get_int("DeviceLinkSpeed"), ip_str);
    }

    /**
     * @brief 对比设备自定义名称是否匹配目标名称
     * @param pstMVDevInfo SDK设备信息
     * @param targetName 目标自定义名称
     * @return 匹配返回true
     */
    bool is_same_device_name(MV_CC_DEVICE_INFO* pstMVDevInfo, const char* targetName) noexcept {
        if (nullptr == pstMVDevInfo) {
            SPDLOG_LOGGER_ERROR(logger_, "The Pointer of pstMVDevInfo is NULL!");
            return false;
        }
        // GigE相机自定义名称存储位置
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            return sdk_string_equals(
                pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName, targetName);
        }
        // USB相机自定义名称存储位置
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            return sdk_string_equals(
                pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName, targetName);
        } else {
            return false;
        }
    }

    /**
     * @brief 格式化单台设备简要信息用于日志打印
     * @param info SDK设备信息结构体
     * @return 可读设备字符串
     */
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

    /**
     * @brief 枚举所有相机设备，根据自定义名称筛选目标相机
     * @param user_defined_name 目标相机自定义名称，nullptr取唯一设备
     * @return 匹配的相机设备信息，无匹配/多设备返回错误
     */
    std::expected<MV_CC_DEVICE_INFO, CameraError>
        search_camera(const char* user_defined_name) noexcept {
        MV_CC_DEVICE_INFO_LIST device_list;
        memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // 枚举GigE+USB两种工业相机
        auto ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        if (ret != MV_OK) {
            return std::unexpected(CameraError{"enum devices", ret});
        }
        // 未找到任何相机
        if (device_list.nDeviceNum == 0) {
            return std::unexpected("no devices");
        }

        // 未传入设备名，要求环境仅有一台相机
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
            // 仅一台相机，直接返回
            return *device_list.pDeviceInfo[0];
        } else {
            // 遍历设备匹配自定义名称
            for (auto i = 0; i < device_list.nDeviceNum; i++) {
                if (is_same_device_name(device_list.pDeviceInfo[i], user_defined_name))
                    return *device_list.pDeviceInfo[i];
            }
            // 无匹配设备，打印全部设备列表供用户选择
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

    /**
     * @brief 打开相机、配置全部寄存器参数、注册回调、启动取流
     * @param device_info 目标相机设备信息
     * @param profile 用户相机配置参数
     * @param sync_mode 采集同步模式
     * @return 相机初始化配置结果
     */
    std::expected<void, CameraError> init_camera(
        MV_CC_DEVICE_INFO& device_info, const CameraProfile& profile,
        const SyncMode& sync_mode) noexcept {
        auto pDeviceInfo = &device_info;
        rotate_type_     = profile.rotate_type;

        int ret;

        // 创建相机句柄（新版推荐API，废弃不带日志的旧接口）
        ret = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
        SDK_RET_ASSERT(ret, "create handle");
        // RAII自动销毁句柄，中途失败自动回收
        FinalAction destroy_handle{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_DestroyHandle(camera_handle_);
                camera_handle_ = nullptr;
            }
        }};

        // 打开设备
        ret = MV_CC_OpenDevice(camera_handle_);
        SDK_RET_ASSERT(ret, "open device");
        FinalAction close_device{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_CloseDevice(camera_handle_);
            }
        }};

        // GigE网口相机专属网络优化配置
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            // 获取相机最优网络数据包大小，减少分片丢包
            int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
            if (nPacketSize <= 0) {
                return std::unexpected(
                    CameraError{fmt::format("invalid packet size: {}", nPacketSize), 0});
            }
            // 设置网络包长
            ret = MV_CC_SetIntValueEx(camera_handle_, "GevSCPSPacketSize", nPacketSize);
            SDK_RET_ASSERT(ret, "set packet Size");
            // 心跳超时3秒，快速检测相机断连
            ret = MV_CC_SetIntValueEx(camera_handle_, "GevHeartbeatTimeout", 3000);
            SDK_RET_ASSERT(ret, "set heartbeat timeout");
            // 开启帧超时检测
            ret = MV_CC_SetBoolValue(camera_handle_, "FrameTimeoutEnable", true);
            SDK_RET_ASSERT(ret, "enable frame timeout");
            ret = MV_CC_SetIntValueEx(camera_handle_, "FrameTimeoutTime", 2000);
            SDK_RET_ASSERT(ret, "set frame timeout time");
        }

        // 固定像素格式为8位BayerRG原始拜耳格式
        ret = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
        SDK_RET_ASSERT(ret, "set pixel format to BayerRG8");

        // 判定触发模式：用户触发配置开启 或 软件同步模式
        bool trigger_on = profile.trigger_mode || (sync_mode == SyncMode::SOFTWARE);
        trigger_mode_   = trigger_on;
        ret             = MV_CC_SetEnumValue(
            camera_handle_, "TriggerMode", trigger_on ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
        SDK_RET_ASSERT(ret, "set trigger Mode");

        // 图像水平/垂直镜像翻转
        ret = MV_CC_SetBoolValue(camera_handle_, "ReverseX", profile.invert_image);
        SDK_RET_ASSERT(ret, "set reverse x");
        ret = MV_CC_SetBoolValue(camera_handle_, "ReverseY", profile.invert_image);
        SDK_RET_ASSERT(ret, "set reverse y");

        // 关闭自动曝光，使用固定曝光值
        ret = MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        SDK_RET_ASSERT(ret, "set auto exposure");

        // 设置ADC采样深度
        ret = MV_CC_SetEnumValue(camera_handle_, "ADCBitDepth", profile.adc_depth);
        if (ret != MV_OK) {
            SPDLOG_LOGGER_WARN(
                logger_, "set adc bit depth to {}: {}", magic_enum::enum_name(profile.adc_depth),
                error_code_to_message(ret));
        }

        // 写入曝光时间（单位微秒）
        ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", profile.exposure_time.count());
        SDK_RET_ASSERT(ret, "set exposure time");

        // 关闭自动增益，使用固定增益
        ret = MV_CC_SetEnumValue(camera_handle_, "GainAuto", MV_GAIN_MODE_OFF);
        SDK_RET_ASSERT(ret, "set auto gain off");
        ret = MV_CC_SetFloatValue(camera_handle_, "Gain", profile.gain);
        SDK_RET_ASSERT(ret, "set gain");

        // 采集模式：连续采集
        ret = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        SDK_RET_ASSERT(ret, "set acquisition mode to continuous");

        // 默认关闭帧率限制，运行时手动配置
        ret = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
        SDK_RET_ASSERT(ret, "set acquisition frame rate enable");

        // 解拜耳画质等级1（最高画质）
        ret = MV_CC_SetBayerCvtQuality(camera_handle_, 1);
        SDK_RET_ASSERT(ret, "set bayer cvt quality");

        // 设置图像缓存节点数量5，缓存多帧防止丢帧
        ret = MV_CC_SetImageNodeNum(camera_handle_, 5);
        SDK_RET_ASSERT(ret, "set image node num");

        // 取流策略：仅保留最新帧，丢弃积压旧帧（机器人实时视觉专用）
        ret = MV_CC_SetGrabStrategy(camera_handle_, MV_GrabStrategy_LatestImagesOnly);
        if (ret != MV_OK) {
            SPDLOG_LOGGER_WARN(
                logger_,
                "set grab strategy (may not be supported on this "
                "platform/device): {}",
                error_code_to_message(ret));
        }

        callback_state_->active.store(true, std::memory_order_release);
        // 退出函数前自动置回调失效RAII
        FinalAction deactivate_callbacks{[state = callback_state_]() {
            state->valid.store(false, std::memory_order_release);
            state->active.store(false, std::memory_order_release);
        }};

        // 注册设备断连异常回调
        ret = MV_CC_RegisterExceptionCallBack(
            camera_handle_, Impl::on_exception_callback, callback_state_.get());
        SDK_RET_ASSERT(ret, "register exception callback");

        // 退出自动注销回调RAII
        FinalAction unregister_callbacks{[this]() {
            if (camera_handle_ != nullptr) {
                MV_CC_RegisterExceptionCallBack(camera_handle_, nullptr, nullptr);
            }
        }};

        // 注册流异常回调（丢包、残缺帧、缓冲区溢出）
        ret = MV_CC_RegisterStreamExceptionCallBack(
            camera_handle_, Impl::on_stream_exception_callback, callback_state_.get());
        SDK_RET_ASSERT(ret, "register stream exception callback");

        // 软件触发模式设置触发源为软件寄存器
        if (sync_mode == SyncMode::SOFTWARE) {
            ret = MV_CC_SetEnumValue(camera_handle_, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
            SDK_RET_ASSERT(ret, "set trigger source: soft trigger");
        }

        // 标记相机有效可用
        callback_state_->valid.store(true, std::memory_order_release);
        // 启动持续取流
        ret = MV_CC_StartGrabbing(camera_handle_);
        SDK_RET_ASSERT(ret, "start grabbing");
        grabbing_ = true;

        // 初始化全部成功，取消所有RAII自动清理动作
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

    /**
     * @brief 将海康Bayer像素格式映射到OpenCV解拜耳转换码
     * @param enType SDK像素格式枚举
     * @return OpenCV转换码，不支持格式返回错误
     */
    static std::expected<int, CameraError>
        get_bayer_conversion_code(MvGvspPixelType enType) noexcept {
        switch (enType) {
        // 8位拜耳格式（当前代码仅支持8位）
        case PixelType_Gvsp_BayerRG8: return cv::COLOR_BayerBG2BGR_EA;
        case PixelType_Gvsp_BayerBG8: return cv::COLOR_BayerRG2BGR_EA;
        case PixelType_Gvsp_BayerGR8: return cv::COLOR_BayerGB2BGR_EA;
        case PixelType_Gvsp_BayerGB8: return cv::COLOR_BayerGR2BGR_EA;

        // 10位深度暂不支持
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

        // 12位深度暂不支持
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

        // 相机输出已解码彩色图，无需解拜耳
        case PixelType_Gvsp_BGR8_Packed:
        case PixelType_Gvsp_YUV422_Packed:
        case PixelType_Gvsp_YUV422_YUYV_Packed:
            return std::unexpected(CameraError{"already debayered"});

        // 未知像素格式
        default:
            return std::unexpected(CameraError{"unknown pixel format: " + std::to_string(enType)});
        }
    }

    /**
     * @brief 完整释放相机所有SDK资源，按官方规范顺序清理
     */
    void uninit_camera() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        // 句柄为空无需清理
        if (camera_handle_ == nullptr) {
            return;
        }

        // 标记相机失效，回调停止工作
        callback_state_->valid.store(false, std::memory_order_release);
        callback_state_->active.store(false, std::memory_order_release);

        auto nRet = MV_OK;

        // 第一步：停止取流
        if (grabbing_) {
            nRet = MV_CC_StopGrabbing(camera_handle_);
            if (MV_OK != nRet) {
                SPDLOG_LOGGER_ERROR(logger_, "stop grabbing: {}", error_code_to_message(nRet));
            }
            grabbing_ = false;
        }

        // 第二步：注销异常回调
        nRet = MV_CC_RegisterExceptionCallBack(camera_handle_, nullptr, nullptr);
        if (MV_OK != nRet) {
            SPDLOG_LOGGER_ERROR(
                logger_, "unregister exception callback: {}", error_code_to_message(nRet));
        }

        // 第三步：关闭设备
        nRet = MV_CC_CloseDevice(camera_handle_);
        if (MV_OK != nRet) {
            SPDLOG_LOGGER_ERROR(logger_, "close device: {}", error_code_to_message(nRet));
        }

        // 第四步：销毁相机句柄
        nRet = MV_CC_DestroyHandle(camera_handle_);
        if (MV_OK != nRet) {
            SPDLOG_LOGGER_ERROR(logger_, "destroy handle: {}", error_code_to_message(nRet));
        }

        camera_handle_ = nullptr;
    }

    /**
     * @brief RAII自动执行清理动作模板类，离开作用域自动执行回调
     * @tparam Func 清理回调函数类型
     */
    template <typename Func>
    struct FinalAction {
        explicit FinalAction(Func func) noexcept
            : clean_{func}
            , enabled_(true) {}

        // 禁止拷贝移动
        FinalAction(const FinalAction&)            = delete;
        FinalAction& operator=(const FinalAction&) = delete;
        FinalAction(FinalAction&&)                 = delete;
        FinalAction& operator=(FinalAction&&)      = delete;

        // 析构自动执行清理
        ~FinalAction() noexcept {
            if (enabled_)
                clean_();
        }

        // 取消自动执行
        void disable() noexcept { enabled_ = false; }

    private:
        Func clean_;
        bool enabled_;
    };

    // 海康相机SDK句柄
    void* camera_handle_ = nullptr;
    // 相机操作互斥锁，多线程读写相机接口串行化
    mutable std::mutex camera_mutex_;
    // 日志实例
    std::shared_ptr<spdlog::logger> logger_        = make_logger();
    // 全局持久化回调状态
    std::shared_ptr<CallbackState> callback_state_ = make_callback_state(logger_);

    /**
     * @brief 设备断连全局静态回调
     * @param nMsgType 异常类型
     * @param pUser 回调用户数据：CallbackState裸指针
     */
    static void __stdcall on_exception_callback(unsigned int nMsgType, void* pUser) noexcept {
        auto* state = static_cast<CallbackState*>(pUser);
        if (state == nullptr || !state->active.load(std::memory_order_acquire)) {
            return;
        }
        // 标记相机失效
        state->valid.store(false, std::memory_order_release);
        if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT) {
            SPDLOG_LOGGER_CRITICAL(state->logger, "device disconnected!");
        } else {
            SPDLOG_LOGGER_ERROR(state->logger, "device: {}", exception_type_to_string(nMsgType));
        }
    }

    /**
     * @brief 流异常回调（丢包、残缺帧、缓冲区溢出）
     * @param pstInfo 流异常信息结构体
     * @param pUser CallbackState裸指针
     */
    static void __stdcall
        on_stream_exception_callback(MV_CC_STREAM_EXCEPTION_INFO* pstInfo, void* pUser) noexcept {
        auto* state = static_cast<CallbackState*>(pUser);
        if (state == nullptr || pstInfo == nullptr
            || !state->active.load(std::memory_order_acquire)) {
            return;
        }
        // 严重断连异常直接标记相机失效
        if (pstInfo->enExceptionType == MV_CC_STREAM_EXCEPTION_DISCONNECTED
            || pstInfo->enExceptionType == MV_CC_STREAM_EXCEPTION_DEVICE) {
            state->valid.store(false, std::memory_order_release);
        }
        // 缓冲区溢出日志不打印（高频正常场景）
        if (pstInfo->enExceptionType != MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW) {
            const auto serial_number = std::string{sdk_string_view(pstInfo->chSerialNumber)};
            SPDLOG_LOGGER_ERROR(
                state->logger, "stream: {}, serial={}",
                stream_exception_type_to_string(pstInfo->enExceptionType), serial_number);
        }
    }

    /**
     * @brief 将设备异常码转为可读文本
     */
    static const char* exception_type_to_string(unsigned int nMsgType) noexcept {
        switch (nMsgType) {
        case MV_EXCEPTION_DEV_DISCONNECT: return "device disconnected";
        case MV_EXCEPTION_VERSION_CHECK: return "SDK/driver version mismatch";
        default: return "unknown";
        }
    }

    /**
     * @brief 将流异常枚举转为可读文本
     */
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

// ===================== ImageCapturer 对外接口实现 =====================
/**
 * @brief 静态工厂创建相机采集器实例
 * @param profile 相机用户配置
 * @param user_defined_name 相机自定义名称
 * @param sync_mode 同步采集模式
 * @return 相机采集器智能指针，失败返回错误
 */
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

/**
 * @brief 采集器构造，创建内部Impl实现实例（PIMPL）
 */
ImageCapturer::ImageCapturer() noexcept
    : impl_(std::make_unique<ImageCapturer::Impl>()) {}

/**
 * @brief 采集器析构，自动销毁Impl内部释放相机资源
 */
ImageCapturer::~ImageCapturer() noexcept = default;

/**
 * @brief 对外初始化接口，转发至内部Impl实现
 */
std::expected<void, CameraError> ImageCapturer::init(
    const CameraProfile& profile, const char* user_defined_name,
    const SyncMode& sync_mode) noexcept {
    return impl_->init(profile, user_defined_name, sync_mode);
}

/**
 * @brief 对外发送软件触发接口
 */
std::expected<void, CameraError> ImageCapturer::software_trigger_on() noexcept {
    return impl_->software_trigger_on();
}

/**
 * @brief 对外设置连续模式帧率接口
 */
std::expected<void, CameraError>
    ImageCapturer::set_frame_rate_inner_trigger_mode(float frame_rate) noexcept {
    return impl_->set_frame_rate_inner_trigger_mode(frame_rate);
}

/**
 * @brief 对外停止取流接口
 */
std::expected<void, CameraError> ImageCapturer::stop_grabbing() noexcept {
    return impl_->stop_grabbing();
}

/**
 * @brief 对外同步读取图像接口
 */
std::expected<cv::Mat, CameraError>
    ImageCapturer::read(std::chrono::duration<unsigned int, std::micro> timeout) noexcept {
    return impl_->read(timeout);
}

/**
 * @brief 对外获取图像宽高接口
 */
std::expected<std::tuple<int, int>, CameraError> ImageCapturer::get_width_height() const noexcept {
    return impl_->get_image_size();
}

/**
 * @brief 对外查询相机是否有效在线
 */
bool ImageCapturer::valid() const noexcept { return impl_->valid(); }

} // namespace hik_camera_driver
