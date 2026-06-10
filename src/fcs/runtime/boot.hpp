// 头文件保护，防止重复包含
#pragma once

// 引入运行时配置相关定义
#include "runtime/config_loader.hpp"
// 引入Talos调度器基础定义
#include "scheduler/thin.hpp"

// C++23 标准库：用于返回执行结果与错误信息
#include <expected>
// 标准字符串，存放错误描述文本
#include <string>

// 项目顶层命名空间
namespace fcs {

/**
 * @brief 框架核心启动入口函数，初始化并启动整套FCS业务流水线
 *
 * 功能说明：
 *  1. 向调度器注册全部运行所需共享资源
 *  2. 按分层规范注册 L1~L5 所有业务系统
 *  3. 主动调用调度器 build() 方法，构建依赖拓扑并完成合法性校验
 *
 * 模块拆分说明：
 *  Foxglove可视化相关逻辑独立在 fcs_visualization 模块中，
 *  本核心库并未链接该模块，因此**可视化服务需要调用方单独初始化**。
 *
 * @param scheduler 传入已创建的Talos调度器实例引用
 * @param config 运行时配置对象，使用右值引用+移动语义，避免大对象拷贝
 *
 * @return std::expected<void, std::string>
 *        - 执行成功：返回空void
 *        - 执行失败：返回字符串格式的错误信息
 *
 * 修饰符 [[nodiscard]]：强制调用方接收返回值，必须处理启动失败场景
 */
[[nodiscard]] std::expected<void, std::string>
    boot(talos::Scheduler& scheduler, RuntimeConfig&& config);

} // namespace fcs