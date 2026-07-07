#pragma once
// ONNX Runtime 推理后端定义
#include "L2_perception/armor/backends/ort.hpp"
// ROI 检测框后处理读取工具
#include "L2_perception/armor/readback_roi.hpp"
// 装甲检测结果结构体 ArmorDetection、ArmorColor
#include "core/armor_types.hpp"
// 基础通用类型 BackendInputResolution
#include "core/types.hpp"

// std::expected 承载检测结果/错误文本
#include <expected>
// std::unique_ptr 独占智能指针管理后端实例
#include <memory>
// std::variant 多类型后端运行时多态容器
#include <variant>
// std::vector 装甲检测结果数组
#include <vector>

// 仅前向声明cv::Mat，避免引入庞大完整OpenCV头文件，减少编译耗时
namespace cv {
class Mat;
}

// 所有推理后端统一基类接口
#include "backends/base.hpp"

namespace fcs::L2 {

// ============================================================================
// 底层具体推理后端 前向声明，解决循环头文件依赖
// ============================================================================
class TraditionalBackend;  // 传统图像处理后端
class OrtBackend;          // ONNX Runtime CPU/GPU推理后端
// 编译开启TensorRT硬件加速宏才声明
#if TALOS_HAS_TENSORRT
class TrtBackend;          // 英伟达TensorRT GPU推理后端
#endif
// 编译开启AXERA国产NPU宏才声明
#if TALOS_HAS_AXERA
class AxeraBackend;        // 国产AXERA NPU推理后端
#endif

// 配置结构体前向声明，定义在config.hpp中
struct ArmorDetectorConfig;       // 顶层装甲检测器总配置
#if TALOS_HAS_TENSORRT
struct TrtConfig;                 // TensorRT后端专属参数
#endif
#if TALOS_HAS_AXERA
struct AxeraConfig;               // Axera NPU后端专属参数
#endif
struct ArmorTraditionalConfig;    // 传统视觉算法参数
struct ArmorOrtConfig;             // ONNX Runtime推理参数

// ============================================================================
// DetectorBackendVariant 变体类型：运行时切换推理后端核心容器
// ============================================================================
/**
 * @brief 后端变体别名，存储任意一种推理后端独占智能指针
 * 编译宏控制可选后端，自动裁剪未启用硬件对应的类型
 * 内部存储std::unique_ptr，保证资源独占、自动释放
 */
using DetectorBackendVariant = std::variant<
#if TALOS_HAS_AXERA
    std::unique_ptr<AxeraBackend>,
#endif
#if TALOS_HAS_TENSORRT
    std::unique_ptr<TrtBackend>,
#endif
    std::unique_ptr<TraditionalBackend>, std::unique_ptr<OrtBackend>>;

// ============================================================================
// DetectorBackend 顶层检测器包装类
// 基于std::variant实现无虚表运行时多态，上层业务统一调用接口
// 实现分离至cpp文件，规避std::visit不完整类型编译报错
// ============================================================================
/**
 * @brief 可动态切换推理后端的装甲检测器顶层封装
 * 上层业务唯一依赖入口，屏蔽底层传统/AI推理硬件差异
 */
class DetectorBackend {
public:
    /**
     * @brief 检测结果别名：承载装甲目标数组，失败返回错误字符串
     */
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;

    /**
     * @brief 默认构造，variant初始为空无后端
     */
    DetectorBackend();

    /**
     * @brief 析构，variant自动释放内部后端智能指针资源
     */
    ~DetectorBackend();

    /**
     * @brief 移动构造，允许检测器实例所有权转移，无内存拷贝
     */
    DetectorBackend(DetectorBackend&&) noexcept;

    /**
     * @brief 移动赋值，支持后端对象移动替换
     */
    DetectorBackend& operator=(DetectorBackend&&) noexcept;

    // 删除拷贝构造、拷贝赋值，后端独占资源不可复制
    DetectorBackend(const DetectorBackend&)            = delete;
    DetectorBackend& operator=(const DetectorBackend&) = delete;

    /**
     * @brief 构造重载：传入传统视觉后端独占指针
     * @param backend TraditionalBackend独占指针，所有权转移
     */
    explicit DetectorBackend(std::unique_ptr<TraditionalBackend> backend);

    /**
     * @brief 构造重载：传入ONNX Runtime推理后端独占指针
     * @param backend OrtBackend独占指针
     */
    explicit DetectorBackend(std::unique_ptr<OrtBackend> backend);

#if TALOS_HAS_TENSORRT
    /**
     * @brief 构造重载：TensorRT GPU推理后端，仅宏开启时可用
     */
    explicit DetectorBackend(std::unique_ptr<TrtBackend> backend);
#endif

#if TALOS_HAS_AXERA
    /**
     * @brief 构造重载：AXERA NPU推理后端，仅宏开启时可用
     */
    explicit DetectorBackend(std::unique_ptr<AxeraBackend> backend);
#endif

    /**
     * @brief 执行装甲检测，自动分发至当前绑定的后端实现
     * @param image 输入原始图像帧 cv::Mat
     * @param color 目标装甲颜色 红/蓝
     * @return DetectionResult 装甲列表 / 错误信息
     * @ noexcept 函数不抛出C++异常，故障通过std::expected承载
     * @ [[nodiscard]] 强制校验返回结果，不可忽略检测失败
     */
    [[nodiscard]] DetectionResult detect(const cv::Mat& image, ArmorColor color) noexcept;

    /**
     * @brief 获取当前推理模型输入分辨率，用于ROI裁剪、图像缩放预处理
     * @return BackendInputResolution 宽高结构体
     */
    [[nodiscard]] BackendInputResolution input_resolution() const noexcept;

    /**
     * @brief 运行时替换后端为传统视觉算法
     * @param backend 新后端独占指针
     */
    void set_backend(std::unique_ptr<TraditionalBackend> backend);

    /**
     * @brief 运行时替换后端为ONNX Runtime推理
     */
    void set_backend(std::unique_ptr<OrtBackend> backend);

#if TALOS_HAS_TENSORRT
    /**
     * @brief 运行时替换后端为TensorRT GPU推理
     */
    void set_backend(std::unique_ptr<TrtBackend> backend);
#endif

#if TALOS_HAS_AXERA
    /**
     * @brief 运行时替换后端为AXERA NPU推理
     */
    void set_backend(std::unique_ptr<AxeraBackend> backend);
#endif

    /**
     * @brief 模板方法：判断当前variant存储的后端是否为指定类型
     * @tparam Backend 底层后端类名（TraditionalBackend/OrtBackend等）
     * @return true 当前绑定后端匹配该类型；false 不匹配
     */
    template <typename Backend>
    [[nodiscard]] bool holds() const noexcept {
        return std::holds_alternative<std::unique_ptr<Backend>>(backend_);
    }

    /**
     * @brief 模板方法：获取可变引用，直接操作底层后端实例
     * @tparam Backend 目标后端类型
     * @return 底层后端左值引用
     * @warning 调用前必须先用holds<>()判断类型，否则触发std::get异常
     */
    template <typename Backend>
    [[nodiscard]] Backend& get() {
        return *std::get<std::unique_ptr<Backend>>(backend_);
    }

    /**
     * @brief 模板方法：获取const只读底层后端引用，用于读取配置、参数
     */
    template <typename Backend>
    [[nodiscard]] const Backend& get() const {
        return *std::get<std::unique_ptr<Backend>>(backend_);
    }

private:
    DetectorBackendVariant backend_; ///< 存储当前激活的推理后端变体
};

// ============================================================================
// 检测器工厂函数：根据配置自动创建对应后端实例
// ============================================================================
/**
 * @brief 全局工厂入口，解析配置自动初始化匹配的推理后端
 * 实现代码放置在detector_backend.cpp
 * @param config 装甲检测器顶层总配置（包含后端类型、各后端参数）
 * @return std::expected<DetectorBackend, std::string>
 *         成功：完整封装好的检测器实例；失败：可读错误文本（模型缺失/硬件初始化失败等）
 */
[[nodiscard]] std::expected<DetectorBackend, std::string>
    create_detector_backend(const ArmorDetectorConfig& config) noexcept;

} // namespace fcs::L2