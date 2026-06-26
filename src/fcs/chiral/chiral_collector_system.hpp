// 头文件保护宏：防止该头文件被重复多次 include，替代传统 #ifndef / #define / #endif 写法
#pragma once

// 引入调度器轻量基础头文件，仅包含Scheduler类基础声明，无沉重依赖，用于函数参数 talos::Scheduler&
#include "scheduler/thin.hpp"

/**
 * @namespace fcs::chiral
 * @brief 手性跨端数据交互模块顶层命名空间
 * 业务场景：机器人分左手、右手两套独立云台视觉火控程序；
 * 本模块实现两侧程序数据互通，通过共享内存双向同步跟踪目标、机械位姿信息，实现左右手协同作战。
 */
namespace fcs::chiral {

/**
 * @brief 注册手性数据采集同步ECS系统
 *
 * ## 完整数据流水线说明
 * 数据输出流向（本地火控 → 另一侧手性程序）：
 * 1. L4规划层通过SPMC无锁通道输出 SelectedTargetSnapshot（当前最优跟踪目标完整滤波快照）
 * 2. chiral_collector 并行系统（调度策略 pool_compute 线程池执行，不阻塞视觉主线程）读取快照
 * 3. 系统内部查询TF坐标系、转换本地自定义数据结构为统一 TalosData 协议包
 * 4. 将打包完成的 TalosData 写入共享内存，对外暴露给另一侧手性程序读取
 *
 * 数据输入流向（另一侧手性程序 → 本地火控）：
 * 共享内存实时监听 IncomingData 远端下发指令/状态，供本端其他系统读取使用
 *
 * ## 参数说明
 * @param scheduler ECS全局调度器实例引用，函数内部调用 add_system 将手性采集任务注册进调度循环
 */
void register_chiral_collector_system(talos::Scheduler& scheduler);

} // namespace fcs::chiral