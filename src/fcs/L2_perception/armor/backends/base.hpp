#pragma once

/**
 * @file base.hpp
 * @brief 探测器后端基类与Concept接口定义
 *
 * 本文件定义了所有装甲板探测器后端的统一接口规范，通过C++20 Concept在编译期强制约束，
 * 确保所有后端实现遵循相同的契约，实现零开销抽象和类型安全的错误处理。
 *
 * ## 核心设计原则
 *
 * 1. **编译期接口约束**：使用C++20 Concept强制所有后端满足统一接口
 * 2. **零开销抽象**：CRTP模式避免虚函数开销，实现静态多态
 * 3. **异常安全**：所有方法noexcept，错误通过std::expected传播
 * 4. **RAII生命周期**：构造即初始化，无需手动init/cleanup
 *
 * ## 错误处理哲学
 *
 * - **异常是系统边界关注**：仅在外部库边界（如OpenCV、ONNXRuntime）捕获异常
 * - **内部代码禁止抛出异常**：所有内部错误通过std::expected<T, std::string>返回
 * - **完整上下文信息**：错误信息包含后端名称、操作类型、具体错误详情
 *
 * ## 架构优势
 *
 * | 特性 | 传统虚函数方案 | CRTP + Concept方案 |
 * |------|---------------|------------------|
 * | 运行时开销 | 有虚表指针开销 | 零开销 |
 * | 编译期检查 | 运行时错误 | 编译期错误 |
 * | 接口约束 | 依赖文档 | 强制编译期检查 |
 * | 性能优化 | 限制优化机会 | 编译器可完全内联 |
 *
 * @see OrtBackend ONNXRuntime后端实现
 * @see TrtBackend TensorRT后端实现
 * @see AxeraBackend AX650 NPU后端实现
 * @see TraditionalBackend 传统算法后端实现
 */

#include "core/armor_types.hpp"

#include <concepts>     // C++20 Concept支持
#include <expected>     // C++23 错误处理类型
#include <string>       // 错误信息字符串
#include <utility>      // std::move等工具
#include <vector>       // 检测结果容器

// 前向声明OpenCV类型，避免引入重量级头文件（减少编译依赖、加速编译）
namespace cv {
class Mat;
}

namespace fcs {

// ============================================================================
// DetectorBackend Concept — 编译期接口强制约束
// ============================================================================

/**
 * @brief 探测器后端Concept：定义所有后端必须满足的接口契约
 *
 * 这个Concept在编译期强制检查所有后端实现，确保它们提供统一的接口。
 * 违反契约将导致编译期硬错误（非运行时错误）。
 *
 * ## Concept强制的契约
 *
 * 1. **类型别名**：
 *    - `DetectionResult`: 检测结果类型，必须为std::expected<std::vector<ArmorDetection>, std::string>
 *    - `Config`: 配置类型，每个后端定义自己的配置结构体
 *
 * 2. **核心检测方法**：
 *    - `detect_impl(image, color) noexcept`: 执行检测，所有错误通过std::expected返回
 *
 * 3. **配置访问方法**：
 *    - `get_config() noexcept`: 返回配置的常量引用（const和非const版本）
 *
 * 4. **工厂方法**：
 *    - `static create(config) noexcept`: 创建后端实例，所有错误通过std::expected返回
 *
 * ## 错误处理规则
 *
 * - **noexcept保证**：所有方法必须noexcept，内部捕获所有异常
 * - **std::expected返回**：失败时返回std::unexpected<std::string>，包含完整错误上下文
 * - **禁止裸异常**：内部代码不允许抛出异常，所有错误必须转换为expected
 *
 * @tparam T 待检测的后端类型
 *
 * @example
 * ```cpp
 * // 编译期检查：如果OrtBackend不满足Concept，编译失败
 * template<typename Backend>
 *     requires DetectorBackend<Backend>
 * class Detector {
 *     Backend backend_;
 * public:
 *     auto detect(const cv::Mat& img) {
 *         return backend_.detect(img, ArmorColor::Red);
 *     }
 * };
 * ```
 */
template <typename T>
concept DetectorBackend = requires(T& t, const T& ct, const cv::Mat& image, ArmorColor color) {
    // ========================================
    // 类型别名约束
    // ========================================

    /// 检测结果类型：必须是std::expected<std::vector<ArmorDetection>, std::string>
    typename T::DetectionResult;

    /// 配置类型：每个后端定义自己的配置结构体
    typename T::Config;

    // ========================================
    // 核心检测方法约束
    // ========================================

    /**
     * 检测方法约束：
     * 1. 参数：const cv::Mat& image (输入图像), ArmorColor color (目标颜色)
     * 2. 返回：DetectionResult (std::expected<vector<ArmorDetection>, string>)
     * 3. 异常保证：noexcept (内部捕获所有异常，通过expected返回错误)
     */
    { t.detect_impl(image, color) } noexcept -> std::same_as<typename T::DetectionResult>;

    // ========================================
    // 配置访问方法约束
    // ========================================

    /**
     * 配置访问（非const版本）：
     * 1. 返回：const Config&
     * 2. 异常保证：noexcept
     */
    { t.get_config() } noexcept -> std::same_as<const typename T::Config&>;

    /**
     * 配置访问（const版本）：
     * 1. 返回：const Config&
     * 2. 异常保证：noexcept
     */
    { ct.get_config() } noexcept -> std::same_as<const typename T::Config&>;

    // ========================================
    // 工厂方法约束
    // ========================================

    /**
     * 工厂方法约束：
     * 1. 静态方法：static
     * 2. 参数：Config配置对象
     * 3. 返回：std::expected<T, std::string> (成功返回实例，失败返回错误信息)
     * 4. 异常保证：noexcept (所有错误通过expected返回)
     *
     * 设计哲学："构造即初始化" — 不需要单独的init()方法
     * 创建成功即可用，失败则返回错误信息
     */
    {
        T::create(std::declval<typename T::Config>())
    } noexcept -> std::same_as<std::expected<T, std::string>>;
};

// ============================================================================
// CRTP Base — 零开销抽象基类
// ============================================================================

/**
 * @brief 探测器后端CRTP基类：提供零开销的统一接口包装
 *
 * ## CRTP模式优势
 *
 * 1. **静态多态**：编译期确定调用目标，避免虚函数表开销
 * 2. **完美内联**：编译器可完全内联detect()方法，无函数调用开销
 * 3. **类型安全**：编译期检查Derived类型是否满足接口
 * 4. **零运行时开销**：与手写直接调用性能相同
 *
 * ## 错误处理机制
 *
 * - **纯转发**：detect()方法仅转发调用到Derived::detect_impl()
 * - **无try/catch**：detect_impl()已声明noexcept，无需捕获异常
 * - **异常链完全消除**：整个调用链从detect()到detect_impl()都是异常自由的
 *
 * ## 使用示例
 *
 * ```cpp
 * // 后端实现示例
 * class OrtBackend : public DetectorBackendBase<OrtBackend> {
 * public:
 *     using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;
 *     using Config = ArmorOrtConfig;
 *
 *     [[nodiscard]] static std::expected<OrtBackend, std::string>
 *         create(Config config) noexcept;
 *
 *     [[nodiscard]] DetectionResult
 *         detect_impl(const cv::Mat& image, ArmorColor color) noexcept;
 *
 *     [[nodiscard]] const Config& get_config() const noexcept;
 * };
 *
 * // 使用示例
 * auto backend = OrtBackend::create(config);
 * if (backend) {
 *     auto result = backend->detect(image, ArmorColor::Red);
 *     if (result) {
 *         for (const auto& det : *result) {
 *             // 处理检测结果
 *         }
 *     } else {
 *         SPDLOG_ERROR("检测失败: {}", result.error());
 *     }
 * }
 * ```
 *
 * @tparam Derived 派生类类型（CRTP模式要求）
 */
template <typename Derived>
class DetectorBackendBase {
public:
    /**
     * @brief 检测结果类型别名
     *
     * 所有后端统一返回此类型：
     * - 成功：std::vector<ArmorDetection> (检测到的装甲板列表)
     * - 失败：std::string (错误信息，包含后端名称、操作、具体错误)
     */
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;

    /**
     * @brief 检测装甲板 — 纯noexcept转发
     *
     * 这是用户调用的主要接口，内部转发到Derived::detect_impl()。
     *
     * ## 实现原理
     *
     * 1. **静态转型**：`static_cast<Derived*>(this)` 在编译期完成
     * 2. **直接调用**：编译器可完全内联detect_impl()
     * 3. **无运行时开销**：与直接调用detect_impl()性能相同
     *
     * ## 异常安全保证
     *
     * - **无try/catch**：因为detect_impl()已声明noexcept
     * - **编译期检查**：Concept确保detect_impl()确实noexcept
     * - **异常链消除**：整个调用链都是异常自由的
     *
     * @param image 输入图像（BGR格式，OpenCV Mat）
     * @param color 目标装甲板颜色（红/蓝）
     * @return DetectionResult 检测结果：
     *         - 成功：包含std::vector<ArmorDetection>
     *         - 失败：包含std::string错误信息
     *
     * @note noexcept保证：不会抛出任何异常
     * @note 线程安全：如果detect_impl()是线程安全的，此方法也是线程安全的
     */
    [[nodiscard]] DetectionResult detect(const cv::Mat& image, ArmorColor color) noexcept {
        // CRTP静态转型：编译期确定目标类型，零运行时开销
        // 直接调用派生类的detect_impl()方法
        // 由于Concept约束，派生类必须实现detect_impl() noexcept
        return static_cast<Derived*>(this)->detect_impl(image, color);
    }

protected:
    /**
     * @brief 受保护构造函数：仅允许派生类构造
     *
     * 防止直接实例化基类，强制使用派生类。
     */
    DetectorBackendBase() = default;

    /**
     * @brief 受保护析构函数：仅允许派生类析构
     *
     * 确保通过派生类指针删除对象时的正确析构。
     * 由于没有虚析构函数，必须确保通过正确类型的指针删除对象。
     */
    ~DetectorBackendBase() = default;
};

} // namespace fcs