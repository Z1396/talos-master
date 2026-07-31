/**
 * @file planning_systems.cpp
 * @brief L4规划层系统注册入口
 *
 * 本文件提供L4规划层的统一注册接口，是整个规划模块的入口点。
 * 所有L4子系统（瞄准、规划、决策）都在此注册到调度器。
 *
 * 设计理念：
 * - 提供单一入口点，简化系统集成
 * - 使用配置驱动，支持运行时参数调整
 * - 模块化设计，便于独立测试和扩展
 *
 * 注册流程：
 * 1. 接收L4Config配置
 * 2. 注册Aimer系统（处理机器人、前哨站、能量机关目标）
 * 3. 未来可扩展其他规划系统
 *
 * 性能考虑：
 * - 注册仅在启动时执行一次，不影响运行时性能
 * - 配置使用移动语义，避免拷贝
 */

#include "L4_planning/aimer/aimer_systems.hpp"

namespace fcs::L4 {

/**
 * @brief 注册L4规划层所有系统
 *
 * 统一注册接口，将L4规划层的所有系统注册到调度器。
 * 当前仅注册Aimer系统，未来可扩展其他规划系统。
 *
 * @param scheduler 调度器实例
 * @param config L4规划层配置（使用移动语义）
 *
 * @note 此函数在系统启动时调用一次
 * @note 配置对象会被移动到子系统，调用后不应再使用
 */
void register_l4_planning_systems(talos::Scheduler& scheduler, L4Config&& config) {
    // 注册Aimer系统（处理Robot/Outpost/Rune三种目标类型）
    // Aimer系统是L4的核心，负责目标选择、轨迹构建、控制意图生成
    register_aimer_systems(scheduler, std::move(config));
}

} // namespace fcs::L4
