/**
 * @brief 海康相机空桩(stub)实现文件
 * 场景：编译环境缺少海康官方SDK时启用，仅保留完整接口签名，不具备实际相机采集功能
 * 作用：保证项目在无海康SDK环境下依然能正常编译通过，运行时调用所有相机接口统一返回错误提示
 */
// 引入海康相机对外头文件，匹配对外接口声明
#include "hik_camera.hpp"

// 海康相机顶层命名空间，与正常SDK实现保持命名空间一致，上层代码无需改调用逻辑
namespace hikcamera {

/**
 * @brief 将海康错误码转为文本描述的空桩实现
 * @param code 海康SDK原生错误码（当前无SDK，直接返回固定提示文本）
 * @return 固定字符串 "unsupported" 表示当前环境不支持海康相机
 * noexcept 无异常抛出
 */
std::string error_code_to_message(unsigned int code) noexcept {
    return "unsupported";
}

/**
 * @brief 相机内部实现Impl私有空结构体
 * 正常完整实现中这里存放海康SDK句柄、相机参数、缓存等底层资源；
 * 无SDK桩版本仅空类，无任何成员变量，占位保证接口布局一致
 */
class ImageCapturer::Impl {};

/**
 * @brief ImageCapturer 相机采集器构造函数 桩实现
 * 初始化内部私有实现指针为空 nullptr，代表无有效相机设备
 */
ImageCapturer::ImageCapturer() noexcept
    : impl_(nullptr) {}

/**
 * @brief 相机采集器析构函数 默认实现
 * 无底层SDK资源需要释放，使用默认析构，不做额外操作
 */
ImageCapturer::~ImageCapturer() noexcept = default;

/**
 * @brief 静态工厂函数：创建相机采集器实例（桩版本）
 * @param profile 相机配置参数（桩实现完全忽略该参数，注释占位）
 * @param user_defined_name 自定义相机设备名（桩实现忽略）
 * @param sync_mode 同步触发模式（硬触发/软触发，桩实现忽略）
 * @return 永远返回错误，提示当前编译为无SDK桩版本，无法创建相机
 */
std::expected<std::unique_ptr<ImageCapturer>, CameraError> ImageCapturer::create(
    const CameraProfile& /*profile*/, const char* /*user_defined_name*/,
    const SyncMode& /*sync_mode*/) noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 相机初始化接口 桩实现
 * @param profile 相机配置
 * @param user_defined_name 设备自定义名称
 * @param sync_mode 同步模式
 * @return 固定错误：无海康SDK，初始化失败
 */
std::expected<void, CameraError> ImageCapturer::init(
    const CameraProfile& /*profile*/, const char* /*user_defined_name*/,
    const SyncMode& /*sync_mode*/) noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 读取一帧图像接口 桩实现
 * @param timeout 读取超时时间（微秒）
 * @return 固定错误，无法读取图像
 */
std::expected<cv::Mat, CameraError>
    ImageCapturer::read(std::chrono::duration<unsigned int, std::micro> /*timeout*/) noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 获取相机分辨率宽高接口 桩实现
 * @return 固定错误，无法读取相机分辨率
 */
std::expected<std::tuple<int, int>, CameraError> ImageCapturer::get_width_height() const noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 开启软触发模式接口 桩实现
 * @return 固定错误，无SDK无法配置触发
 */
std::expected<void, CameraError> ImageCapturer::software_trigger_on() noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 设置内部触发帧率接口 桩实现
 * @param frame_rate 目标帧率
 * @return 固定错误
 */
std::expected<void, CameraError>
    ImageCapturer::set_frame_rate_inner_trigger_mode(float /*frame_rate*/) noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 停止图像采集接口 桩实现
 * @return 固定错误
 */
std::expected<void, CameraError> ImageCapturer::stop_grabbing() noexcept {
    return std::unexpected("HIK SDK not available (stub build)");
}

/**
 * @brief 判断相机设备是否有效正常打开
 * @return 永远返回false，桩版本无真实相机设备
 */
bool ImageCapturer::valid() const noexcept { return false; }

} // namespace hikcamera