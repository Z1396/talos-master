// 头文件保护指令，防止该头文件被多文件重复包含引发重复定义、编译冲突
#pragma once

// 场景构建器：用于构造 Foxglove 可视化场景实体（线条、标记、文字、3D模型等）
#include "scene_builder.hpp"
// 通用可视化辅助工具：消息发布、状态判断、类型转换等底层工具函数
#include "system_helpers.hpp"
// 战术配色调色板：统一可视化线条、装甲、目标、云台的颜色规范
#include "tactical_palette.hpp"

// 引入调度器轻量化头文件：Talos 任务调度核心，用于注册、挂载可视化任务系统
#include "scheduler/thin.hpp"

// 顶层命名空间：fcs项目 -> 可视化模块 -> Foxglove可视化插件 -> 系统注册管理
namespace fcs::visualization::foxglove::systems {

/**
 * @brief 命名空间别名简写，简化代码书写，避免超长嵌套命名
 * viz = fcs::visualization 顶层可视化总模块
 * tac = viz::tactical 战术可视化子模块（配色、战术绘制）
 */
namespace viz = fcs::visualization;
namespace tac = viz::tactical;

/**
 * 全局using导入：将底层通用工具引入当前命名空间，后续调用无需写完整长路径
 * :: 代表全局根命名空间，避免相对查找出错
 */
// 判断Foxglove服务是否就绪，未就绪时跳过可视化发布，防止空指针/网络报错
using ::fcs::visualization::detail::foxglove_ready;
// 发布JSON格式自定义消息到Foxglove面板（文本、自定义图表、状态信息）
using ::fcs::visualization::detail::publish_json_message;
// 场景发布工具：仅当场景存在有效绘制内容时才发送，空场景跳过发布节省带宽
using ::fcs::visualization::detail::publish_scene_if_nonempty;

/**
 * @brief 注册L1传感器层可视化任务系统
 * @param app Talos顶层调度器实例引用，用于挂载可视化定时任务
 * L1层级定义：原始传感器数据层，功能为图像流推送、相机原始画面可视化
 */
void register_l1_sensor_systems(talos::scheduler::Scheduler& app);

/**
 * @brief 注册L2感知层可视化任务系统
 * @param app Talos顶层调度器实例引用
 * L2层级定义：感知检测层，可视化装甲检测框、PnP位姿、相机坐标系测量结果
 */
void register_l2_perception_systems(talos::scheduler::Scheduler& app);

/**
 * @brief 注册L3估计层可视化任务系统
 * @param app Talos顶层调度器实例引用
 * L3层级定义：目标滤波估计层，可视化跟踪轨迹、目标关联匹配、卡尔曼滤波预测点
 */
void register_l3_estimation_systems(talos::scheduler::Scheduler& app);

/**
 * @brief 注册L4规划控制层可视化任务系统
 * @param app Talos顶层调度器实例引用
 * L4层级定义：运动规划控制层，可视化云台目标角度、MPC预测轨迹、控制指令曲线
 */
void register_l4_planning_systems(talos::scheduler::Scheduler& app);

/**
 * @brief 单独注册能量机关专属可视化任务
 * @param app Talos顶层调度器实例引用
 * 独立拆分原因：能量机关逻辑与常规装甲目标解算流程完全隔离，单独管理绘制逻辑
 */
void register_rune_systems(talos::scheduler::Scheduler& app);

/**
 * @brief 注册真值可视化系统（Daedalus仿真模式专用）
 * @param app Talos顶层调度器实例引用
 * Daedalus：项目配套仿真环境，可输出机器人、装甲真实世界坐标；
 * 该函数用于绘制仿真真值，和算法预测结果做对比调试
 */
void register_ground_truth_systems(talos::scheduler::Scheduler& app);

/**
 * @brief 注册通用调试可视化系统
 * @param app 主调度器引用
 * @param scheduler_ptr 额外调度器裸指针，用于挂载独立优先级/独立周期的调试任务
 * 用途：临时标记、调试线段、性能打点、异常报警可视化，开发阶段启用，正式比赛可关闭
 */
void register_debug_systems(talos::scheduler::Scheduler& app, talos::Scheduler* scheduler_ptr);

} // namespace fcs::visualization::foxglove::systems