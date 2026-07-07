// 传统装甲板图像处理后端
#include "L2_perception/armor/backends/ort.hpp"
// ONNX Runtime 推理后端
#include "L2_perception/armor/backends/traditional.hpp"
// 装甲检测全局配置结构体、后端枚举
#include "L2_perception/armor/config.hpp"
// 装甲检测结果、基础类型定义
#include "core/armor_types.hpp"

// 条件编译：启用TensorRT推理库时引入对应后端头文件
#if TALOS_HAS_TENSORRT
# include "L2_perception/armor/backends/tensor_rt.hpp"
#endif
// 条件编译：启用Axera国产NPU推理库时引入对应后端头文件
#if TALOS_HAS_AXERA
# include "L2_perception/armor/backends/axera.hpp"
#endif

// 类型萃取工具：std::decay_t、std::is_same_v、std::is_same、std::conditional_t
#include <type_traits>

namespace fcs::L2 {

// ============================================================================
// DetectorBackend 装甲检测器顶层统一封装实现
// 使用std::variant存储多推理后端，上层业务无感知切换传统/ONNX/TensorRT/Axera
// ============================================================================

/**
 * @brief 默认构造函数，variant默认空状态
 */
DetectorBackend::DetectorBackend()  = default;

/**
 * @brief 默认析构，variant自动释放内部后端智能指针资源
 */
DetectorBackend::~DetectorBackend() = default;

/**
 * @brief 移动构造，允许后端实例所有权转移，无内存拷贝
 */
DetectorBackend::DetectorBackend(DetectorBackend&&) noexcept            = default;

/**
 * @brief 移动赋值运算符，支持后端对象移动赋值
 */
DetectorBackend& DetectorBackend::operator=(DetectorBackend&&) noexcept = default;

/**
 * @brief 构造重载：传入传统图像处理后端独占指针
 * @param backend 传统视觉算法后端unique_ptr
 */
DetectorBackend::DetectorBackend(std::unique_ptr<TraditionalBackend> backend)
    : backend_(std::move(backend)) {}

/**
 * @brief 构造重载：传入ONNX Runtime推理后端独占指针
 * @param backend ONNX推理后端unique_ptr
 */
DetectorBackend::DetectorBackend(std::unique_ptr<OrtBackend> backend)
    : backend_(std::move(backend)) {}

#if TALOS_HAS_TENSORRT
/**
 * @brief 构造重载：TensorRT GPU加速推理后端
 * 仅在编译开启TALOS_HAS_TENSORRT宏时生效
 * @param backend TensorRT后端unique_ptr
 */
DetectorBackend::DetectorBackend(std::unique_ptr<TrtBackend> backend)
    : backend_(std::move(backend)) {}
#endif

#if TALOS_HAS_AXERA
/**
 * @brief 构造重载：Axera国产NPU推理后端
 * 仅在编译开启TALOS_HAS_AXERA宏时生效
 * @param backend Axera后端unique_ptr
 */
DetectorBackend::DetectorBackend(std::unique_ptr<AxeraBackend> backend)
    : backend_(std::move(backend)) {}
#endif

/**
 * @brief 统一装甲检测接口，自动分发到底层绑定的推理后端
 * @param image 输入原始OpenCV图像帧
 * @param color 待识别装甲颜色：红色/蓝色
 * @return DetectionResult 检测结果，std::expected承载装甲目标列表或错误信息
 */
DetectorBackend::DetectionResult
    DetectorBackend::detect(const cv::Mat& image, ArmorColor color) noexcept {
    // std::visit 自动匹配variant内部存储的后端类型，调用对应detect函数
    return std::visit(
        [&image, color](auto& backend) -> DetectionResult {
            // 后端智能指针为空，返回错误信息
            if (!backend) {
                return std::unexpected("No detector backend is active");
            }
            // 调用对应后端的检测函数并返回结果
            return backend->detect(image, color);
        },
        backend_);
}

/**
 * @brief 替换当前绑定后端为传统视觉后端
 * @param backend 新传统后端独占指针，所有权转移
 */
void DetectorBackend::set_backend(std::unique_ptr<TraditionalBackend> backend) {
    backend_ = std::move(backend);
}

/**
 * @brief 替换当前绑定后端为ONNX Runtime推理后端
 * @param backend 新ONNX后端独占指针
 */
void DetectorBackend::set_backend(std::unique_ptr<OrtBackend> backend) {
    backend_ = std::move(backend);
}

#if TALOS_HAS_TENSORRT
/**
 * @brief 替换当前绑定后端为TensorRT推理后端
 * @param backend TensorRT后端独占指针
 */
void DetectorBackend::set_backend(std::unique_ptr<TrtBackend> backend) {
    backend_ = std::move(backend);
}
#endif

#if TALOS_HAS_AXERA
/**
 * @brief 替换当前绑定后端为Axera NPU推理后端
 * @param backend Axera后端独占指针
 */
void DetectorBackend::set_backend(std::unique_ptr<AxeraBackend> backend) {
    backend_ = std::move(backend);
}
#endif

/**
 * @brief 获取当前推理后端网络输入分辨率宽高
 * 不同后端分辨率规则不同，编译期if constexpr分支区分逻辑
 * @return BackendInputResolution 分辨率结构体 width/height
 */
BackendInputResolution DetectorBackend::input_resolution() const noexcept {
    return std::visit(
        [](const auto& backend) -> BackendInputResolution {
            // 萃取variant内部存储的智能指针原始类型
            using BackendPtr = std::decay_t<decltype(backend)>;
            // 获取智能指针包裹的底层后端类名
            using Backend    = typename BackendPtr::element_type;
            // 后端指针为空，返回默认空分辨率
            if (!backend) {
                return {};
            }
            // 分支1：传统图像处理后端固定分辨率 480×270
            if constexpr (std::is_same_v<Backend, TraditionalBackend>) {
                return {.width = 480, .height = 270};
#if TALOS_HAS_AXERA
            // 分支2：Axera NPU后端，从配置读取自定义输入尺寸
            } else if constexpr (std::is_same_v<Backend, AxeraBackend>) {
                return {
                    .width  = backend->get_config().input_width,
                    .height = backend->get_config().input_height};
#endif
            // 分支3：ONNX/TensorRT后端，使用类静态常量INPUT_W/INPUT_H
            } else {
                return {.width = Backend::INPUT_W, .height = Backend::INPUT_H};
            }
        },
        backend_);
}

// ============================================================================
// 全局工厂函数：根据配置自动创建对应检测器后端实例
// ============================================================================

/**
 * @brief 装甲检测器工厂创建函数
 * @param config 装甲检测完整配置（后端类型、对应后端参数）
 * @return std::expected<DetectorBackend, std::string>
 *         成功：封装好对应推理后端的顶层检测器；失败：可读错误字符串
 */
[[nodiscard]] std::expected<DetectorBackend, std::string>
    create_detector_backend(const ArmorDetectorConfig& config) noexcept {
    // 根据配置指定后端类型分支匹配
    switch (config.backend_type) {
    // 传统图像处理算法分支
    case ArmorBackendType::Traditional: {
        // 调用传统后端静态create构造函数
        auto result = TraditionalBackend::create(config.traditional);
        // 创建失败，返回错误信息
        if (!result) {
            return std::unexpected(result.error());
        }
        // 包装为unique_ptr，构造顶层DetectorBackend返回
        return DetectorBackend(std::make_unique<TraditionalBackend>(std::move(*result)));
    }
    // ONNX Runtime CPU/GPU推理分支
    case ArmorBackendType::OnnxRuntime: {
        auto result = OrtBackend::create(config.onnx_runtime);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<OrtBackend>(std::move(*result)));
    }
#if TALOS_HAS_TENSORRT
    // TensorRT GPU加速推理分支（仅编译开启宏可用）
    case ArmorBackendType::TensorRT: {
        auto result = TrtBackend::create(config.tensor_rt);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<TrtBackend>(std::move(*result)));
    }
#endif
#if TALOS_HAS_AXERA
    // Axera国产NPU推理分支（仅编译开启宏可用）
    case ArmorBackendType::Axera: {
        auto result = AxeraBackend::create(config.axera);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<AxeraBackend>(std::move(*result)));
    }
#endif
    }
    // 枚举全部分支覆盖，无匹配分支触发不可达终止
    std::unreachable();
}

} // namespace fcs::L2