// 头文件保护，防止重复包含导致编译错误
#pragma once

// 标准库头文件
#include <cstdint>    // 定长整数类型，这里用于端口号 uint16_t
#include <memory>     // 智能指针 std::shared_ptr
#include <string>     // 标准字符串，存储监听地址/主机名

// 项目内部头文件
#include "foxglove_server.hpp"  // Foxglove 服务端核心类定义
#include "scheduler/thin.hpp"   // Talos 调度器前置定义

// 可视化模块命名空间
namespace fcs::visualization {

/**
 * @brief 创建并初始化 Foxglove 服务端实例
 * @param config Foxglove 完整配置参数
 * @return 成功返回 FoxgloveServer 共享指针；初始化失败返回 nullptr
 *
 * [[nodiscard]]：强制调用方检查返回值，避免忽略创建失败
 */
[[nodiscard]] std::shared_ptr<FoxgloveServer> try_create_foxglove_server(FoxgloveConfig config);

/**
 * @brief 兼容旧版本的重载接口（仅 WebSocket 模式）
 * 为历史代码提供向下兼容，简化调用，直接指定端口和监听地址
 * @param port 监听端口号（16位无符号整型）
 * @param host 监听主机地址，如 "0.0.0.0"、"127.0.0.1"
 * @return 成功返回服务端共享指针，失败返回 nullptr
 */
[[nodiscard]] std::shared_ptr<FoxgloveServer>
    try_create_foxglove_server(uint16_t port, std::string host);

/**
 * @brief 向 Talos 调度器注册所有 Foxglove 消息发布系统
 * 实现位于：src/fcs/visualization/foxglove_systems.cpp
 *
 * @param daedalus 是否启用 Daedalus 设备相关可视化发布逻辑
 * @param scheduler 主调度器引用，用于注册可视化任务系统
 * @param scheduler_ptr 调度器裸指针（可选参数，兼容部分旧接口/内部逻辑）
 *
 * 作用：注册数据上报、状态发布、日志转发等可视化任务，
 * 让调度器自动驱动数据推送到 Foxglove Studio
 */
void register_foxglove_systems(
    bool daedalus, talos::Scheduler& scheduler, talos::Scheduler* scheduler_ptr = nullptr);

} // namespace fcs::visualization