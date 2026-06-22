#pragma once
// 头文件保护宏：保证该头文件在整个编译单元中只会被包含一次，替代传统 #ifndef / #define / #endif 写法

// Keep this header as a thin API surface.
// The implementation lives in `src/fcs/L2_perception/systems.cpp`.
// 注释说明：本头文件只做轻量化API对外声明，**所有函数实现逻辑全部放在 cpp 文件**
// 遵循声明与实现分离原则，减少头文件依赖、缩短编译时间、对外只暴露最小接口

// 引入调度器轻量化头文件，提供任务调度、系统注册相关基础类型（talos::Scheduler）
#include "scheduler/thin.hpp"
// C++23 标准库：std::expected，用于安全返回「成功结果 / 错误字符串」，替代裸指针/异常做错误处理
#include <expected>
// 智能指针，用于管理DetectorBackend、PnPSolver对象生命周期，自动释放内存，避免内存泄漏
#include <memory>
// 标准字符串，存储后端名称、错误描述文本
#include <string>

// 前向声明：相机配置结构体，仅告知编译器存在该类型，不引入完整头文件，降低编译耦合
namespace fcs {
struct CameraConfig;
};

// 二级命名空间：fcs工程下 L2感知层模块（L2_perception）
namespace fcs::L2 {

// 前向声明：装甲板检测器配置结构体
struct ArmorDetectorConfig;
// 前向声明：检测器底层推理后端抽象类（封装YOLO/传统图像处理等推理实现）
class DetectorBackend;
// 前向声明：PnP位姿解算器，根据装甲板像素坐标求解3D空间位姿
class PnPSolver;

// ============================================================================
// AT Legacy Armor Detection Pipeline Systems
// 传统装甲板检测流水线 任务系统注册接口
// ============================================================================

/// Register AT Legacy detection pipeline systems with the scheduler
/// 将传统装甲检测流水线的所有任务系统注册到全局调度器
///
/// Pipeline architecture: 完整数据流流水线架构注释
///   Camera System (external) → Image [SPMC]
///     相机采集系统（外部模块）输出图像数据，通过SPMC跨进程/跨线程共享内存通道分发
///         ↓
///   ArmorDetectorSystem (pool_compute) → Detection [SPMC]
///     装甲检测器系统（在线程计算池运行）：接收图像，输出装甲板检测框结果，SPMC转发
///         ↓
///   PnPSolverSystem (pool_compute) → Measurement [SPMC]
///     PnP位姿解算系统（线程池运行）：接收检测框，输出装甲三维位姿测量数据
///         ↓
///   TrackerSystem (external)
///     跟踪器系统（外部模块）：接收位姿数据做目标跟踪
///
/// ## Parameters 函数入参说明
///
/// - `scheduler`: 调度器实例，所有感知任务系统都将注册到该调度器中统一调度
/// - `config`: L2感知层全局配置（函数内部会将配置转移为调度器资源，供各系统读取）
/// noexcept：函数保证不会抛出C++异常，运行失败通过返回值/资源状态反馈
void register_detection_systems(talos::Scheduler& scheduler) noexcept;

// ============================================================================
// Configuration Helpers
// 配置工具函数：根据配置生成检测器、解算器实例
// ============================================================================

/// 检测器后端句柄结构体
/// 封装检测器底层推理实例 + 后端可读名称，统一作为函数返回载体
struct DetectorBackendHandle {
    // 共享智能指针：持有检测器后端实例，多系统共享时不会提前析构
    std::shared_ptr<DetectorBackend> backend;
    // 后端名称字符串，用于日志打印、调试区分（如"yolov8", "traditional_match"）
    std::string name;
};

/// Create detector backend resource from unified config.
/// 根据统一装甲检测配置，创建对应的检测器推理后端实例
/// Returns the backend pointer plus a human-readable backend name.
/// 返回 DetectorBackendHandle 句柄（包含实例+名称）；创建失败时返回std::string类型错误信息
/// [[nodiscard]] 编译器标记：强制调用方接收返回值，防止忽略创建失败错误
/// noexcept 不会抛出异常，错误统一封装在std::expected的错误分支中
[[nodiscard]] std::expected<DetectorBackendHandle, std::string>
    create_detector_backend_handle(const ArmorDetectorConfig& config) noexcept;

/// Create PnP solver from camera info
/// 根据相机内参配置，创建PnP位姿解算器实例
/// @param config 相机标定配置（包含内参矩阵、畸变系数、相机尺寸等）
/// [[nodiscard]] 强制接收返回值，避免丢失解算器实例
/// 返回共享智能指针，生命周期交由调用方管理
[[nodiscard]] std::shared_ptr<PnPSolver> create_pnp_solver(const CameraConfig& config) noexcept;

} // namespace fcs::L2