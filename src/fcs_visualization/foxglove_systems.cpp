// 引入Foxglove可视化模块对外头文件
#include "foxglove_systems.hpp"

// Foxglove WebSocket服务核心实现
#include "foxglove_server.hpp"
// 可视化子系统基础基类
#include "systems/base.hpp"

// 项目调度器核心，用来注册、管理各类可视化任务系统
#include "scheduler/scheduler.hpp"

namespace fcs::visualization {

/**
 * @brief 尝试创建Foxglove可视化服务实例（完整配置重载）
 * @param config Foxglove完整配置结构体：传输协议、监听地址、端口等
 * @return 成功返回FoxgloveServer共享智能指针对象；创建失败返回空shared_ptr
 * @details 内部调用底层创建接口，捕获创建失败错误并打印日志，失败时直接禁用可视化功能
 */
std::shared_ptr<FoxgloveServer> try_create_foxglove_server(FoxgloveConfig config) {
    // 移动语义转移配置，调用底层工厂函数创建服务，返回expected包裹结果（成功存实例，失败存错误信息）
    auto server = create_foxglove_server(std::move(config));
    // 判断创建是否失败：expected无有效值代表出错
    if (!server.has_value()) {
        // 打印ERROR级日志，输出具体失败原因，提示可视化功能已关闭
        SPDLOG_ERROR("foxglove server creation failed: {}. Visualization disabled", server.error());
        // 返回空智能指针，上层据此判断可视化不可用
        return {};
    }
    // 取出expected内的服务实例，移动返回避免拷贝
    return std::move(*server);
}

/**
 * @brief 简化重载：仅通过端口+主机地址创建Foxglove服务，默认WebSocket传输
 * @param port 监听端口号 uint16
 * @param host 监听IP/主机名，例如"0.0.0.0"、"127.0.0.1"
 * @return 同上面重载，返回可视化服务实例
 * @details 内部封装默认配置，统一调用完整配置版本的创建函数，对外简化调用
 */
std::shared_ptr<FoxgloveServer> try_create_foxglove_server(uint16_t port, std::string host) {
    return try_create_foxglove_server(
        FoxgloveConfig{
            // 默认传输方式：WebSocket（Foxglove Studio客户端标准通信协议）
            .transport = FoxgloveTransport::WebSocket,
            // 转移字符串所有权，减少拷贝
            .host      = std::move(host),
            .port      = port,
        });
}

/**
 * @brief 批量注册所有Foxglove可视化子系统到全局调度器
 * @param daedalus 布尔标记，true代表启用真值仿真系统（地面真值可视化，调试专用）
 * @param scheduler 调度器引用，用于常规注册系统
 * @param scheduler_ptr 调度器裸指针，专供debug调试系统使用（需要内部访问调度器）
 * @details 分层注册全链路可视化发布任务：传感器感知->估计->规划，附加符文、调试、可选真值系统
 */
void register_foxglove_systems(
    bool daedalus, talos::Scheduler& scheduler, talos::Scheduler* scheduler_ptr) {
    // 命名空间简写，简化下方注册函数书写
    using namespace foxglove::systems;

    // ====================== 分层自动化注册可视化发布系统 ======================
    // L1 底层传感器可视化：相机、IMU、雷达、编码器等原始传感器数据发布
    register_l1_sensor_systems(scheduler);
    // L2 感知层可视化：目标检测、分割、特征点、图像处理结果
    register_l2_perception_systems(scheduler);
    // L3 状态估计层可视化：位姿、里程计、滤波结果、融合定位
    register_l3_estimation_systems(scheduler);
    // L4 规划控制层可视化：路径轨迹、速度指令、规划障碍物、代价地图
    register_l4_planning_systems(scheduler);
    // 符识别专项可视化：能量机关、装甲板、Rune符文相关检测、角度预测画面
    register_rune_systems(scheduler);

    // 仅开启Daedalus仿真模式时，注册Ground Truth地面真值可视化系统
    if (daedalus) {
        SPDLOG_INFO("register ground truth system");
        register_ground_truth_systems(scheduler);
    }

    // 全局调试可视化系统：自定义曲线、标记、文本、临时调试绘图
    // 该系统需要调度器裸指针，支持运行时动态添加/删除可视化任务
    register_debug_systems(scheduler, scheduler_ptr);
}

} // namespace fcs::visualization