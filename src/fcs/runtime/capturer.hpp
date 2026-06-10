// 头文件保护，防止重复包含导致编译重定义
#pragma once

// 项目内部依赖头文件
#include "config.hpp"                // 录制器相关配置定义
#include "scheduler/thin.hpp"       // 任务调度器基础定义（Talos 调度框架）

// 标准库
#include <string>                   // 字符串类型，用于设备/模块名称

// 运行时模块命名空间
namespace fcs::runtime {

/**
 * @brief 录制器启动上下文
 * 承载启动数据录制服务所需的环境与标识信息
 */
struct CapturerLaunchContext {
    HardwareBackend backend;        // 当前硬件后端类型（直连/DAEDALUS/转发/仅相机等）
    std::string robot;              // 机器人机型/名称标识
    std::string vision;             // 视觉方案/版本标识
};

/**
 * @brief 向全局调度器注册并启动数据录制系统
 * @param scheduler 任务调度器实例，用于挂载录制相关任务
 * @param config 录制器功能配置参数
 * @param launch 启动上下文，硬件、机型、视觉版本等环境信息
 *
 * 功能说明：
 * 将图像、码流、传感器、算法中间结果等数据录制任务注册到调度器，
 * 由调度器统一管理任务生命周期、运行时机与线程。
 */
void register_runtime_capturer_system(
    talos::Scheduler& scheduler,
    const CapturerConfig& config,
    const CapturerLaunchContext& launch);

} // namespace fcs::runtime