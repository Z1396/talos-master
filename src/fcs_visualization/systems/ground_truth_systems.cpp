// 项目基础通用宏、类型、工具函数
#include "base.hpp"
// 调度器完整定义（Scheduler、pool_compute、spmc 等）
#include "scheduler/scheduler.hpp"
// Foxglove可视化消息结构体定义：3D场景、JSON消息封装类型
#include "foxglove_types.hpp"
// 3D场景实体构造器：快速构建球体、文字标签等SceneEntity
#include "scene_builder.hpp"

// 全局SPMC通信通道Topic常量，区分GroundTruth真值数据流
#include "core/channel_topics.hpp"
// fast_tf静态坐标系帧枚举 world/odom等
#include "frame.hpp"
// 共享内存布局、IPC数据批量结构体定义
#include "shm_layout.hpp"

// Eigen线性代数库：三维向量、坐标变换
#include <Eigen/Core>
#include <Eigen/Geometry>
// 标准数学函数
#include <cmath>
// nlohmann JSON序列化库
#include <nlohmann/json.hpp>
// 标准字符串
#include <string>
// 动态数组容器，存放批量3D实体
#include <vector>

/**
 * @namespace fcs::visualization::foxglove::systems
 * @brief Foxglove可视化系统注册模块
 * 功能：专门处理【仿真真值GroundTruth】可视化，仅Daedalus仿真模式启用
 * 输出两条Foxglove话题：
 * 1. /ground_truth/scene 3D场景：在大符、敌方装甲真值坐标渲染球体+文字标签
 * 2. /ground_truth JSON原始真值数据包：完整结构化真值，供前端数值面板绘图
 */
namespace fcs::visualization::foxglove::systems {

/**
 * @namespace 匿名文件私有命名空间
 * 存放仅本文件使用的常量、枚举转文字工具函数，隔离外部污染
 */
namespace {

// 大符渲染球体颜色：激活态主题色，0.62透明度半透明
constexpr auto kRuneColor = tac::with_alpha(tac::Semantic::STATUS_ACTIVE, 0.62);
// 3D文字标签基础白色
constexpr auto kLabelColor = tac::Text::PRIMARY;

// 大符真值球体渲染半径(m)
constexpr double kRuneSphereRadius = 0.15;
// 文字标签在目标球体上方抬高距离，避免文字覆盖球体
constexpr double kLabelOffsetZ = 0.18;
// 3D标签字体尺寸(米单位，Foxglove场景空间尺寸)
constexpr double kLabelFontSize = 0.08;

/**
 * @brief 装甲数字标签 → 可读名称字符串
 * @param label 底层数字编号
 * @return 机甲类型文字 Hero/Engineer/Infantry等
 * noexcept 无异常，实时系统安全
 */
[[nodiscard]] const char* armor_label_name(uint8_t label) noexcept {
    switch (label) {
    case 1: return "Hero";      // 英雄机甲
    case 2: return "Engineer";  // 工程机甲
    case 3: return "Infantry3"; // 三号步兵
    case 4: return "Infantry4"; // 四号步兵
    case 5: return "Infantry5"; // 五号步兵
    case 6: return "Sentry";    // 哨兵
    case 7: return "Outpost";   // 前哨站
    default: return "Unknown";  // 未知类型
    }
}

/**
 * @brief 队伍编号转阵营文字
 * @param team 0红方 / 1蓝方
 * @return Red / Blue
 */
[[nodiscard]] const char* team_name(uint8_t team) noexcept { return team == 0 ? "Red" : "Blue"; }

/**
 * @brief 大符激活状态数字 → 状态文字
 */
[[nodiscard]] const char* mechanism_state_name(uint8_t state) noexcept {
    switch (state) {
    case 0: return "Inactive";   // 未激活
    case 1: return "Activating"; // 激活中
    case 2: return "Activated";  // 已激活
    case 3: return "Failed";     // 激活失败
    default: return "Unknown";
    }
}

/**
 * @brief 大符尺寸模式转换
 * @param mode 0小符 / 1大符
 */
[[nodiscard]] const char* rune_mode_name(uint8_t mode) noexcept {
    return mode == 0 ? "Small" : "Large";
}

} // namespace

/**
 * @brief 注册仿真真值GroundTruth可视化并行系统（仅仿真Daedalus模式启用）
 * 发布两条Foxglove标准话题：
 * 1. SceneUpdate /ground_truth/scene
 *    作用：在world世界坐标系渲染3D球体标记大符位置，附带文字标签展示阵营、大小、激活状态
 * 2. JSON消息 /ground_truth
 *    作用：完整序列化所有真值数据（敌方装甲、大符全套运动参数），Foxglove图表/表格直接解析
 * @param app ECS全局调度器实例，用于注册pool_compute线程池并行任务
 */
void register_ground_truth_systems(talos::scheduler::Scheduler& app) {
    // =========================================================================
    // GroundTruth真值3D场景实体 + JSON数据包发布系统
    // 调度策略：pool_compute 线程池执行，不阻塞图像主线程
    // 输入依赖：SPMC无锁通道 ipc::GroundTruthBatch 批量真值数据包
    // 全局资源依赖：Foxglove服务句柄，用于入队发送可视化消息
    // =========================================================================
    app.add_system<talos::pool_compute>(
        // 系统唯一名称，用于日志、性能统计区分
        "foxglove_ground_truth_pub",
        // Lambda系统主逻辑
        [](
            // 只读SPMC通道：仿真输出批量真值结构体
            talos::spmc<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_in,
            // 全局只读共享资源：Foxglove服务指针，提供消息发送接口
            talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // 前置校验1：Foxglove服务未初始化就绪，直接跳过本轮
            if (!detail::foxglove_ready(*server, gt_in)) {
                return;
            }

            // 读取当前最新一帧批量真值数据包
            auto batch = gt_in.read();
            // 无新真值数据，直接返回
            if (!batch) {
                return;
            }

            // 当前帧纳秒时间戳，所有3D实体、JSON消息共用同一时间戳对齐时序
            const uint64_t timestamp_ns = batch->timestamp_ns;
            // 容器存储本帧所有3D场景实体（球体+文字标签）
            std::vector<::foxglove::schemas::SceneEntity> entities;

            // ===================== 1. 遍历所有大符真值，生成3D可视化实体 =====================
            for (uint32_t i = 0; i < batch->rune_count; ++i) {
                const auto& rune = batch->runes[i];
                // 读取odom坐标系下大符三维中心坐标，转为Eigen向量
                const Eigen::Vector3d center(
                    rune.r_center_odom[0], rune.r_center_odom[1], rune.r_center_odom[2]);

                // 构造大符中心半透明球体实体
                auto entity =
                    viz::EntityBuilder::create<fast_tf::world>(
                        "gt", fmt::format("rune_{}", i) // 命名空间gt，实体唯一ID rune_0/rune_1
                        )
                        .position(center)               // 球体中心坐标
                        .size(kRuneSphereRadius)        // 球体半径
                        .color(kRuneColor)              // 半透明激活主题色
                        .sphere()                       // 几何体类型：球体
                        .timestamp(timestamp_ns)        // 消息时序戳
                        .build();

                // 构造大符上方文字标签实体
                // 标签文本格式：阵营/大符尺寸 [激活状态]
                const std::string label = fmt::format(
                    "{}/Rune {} [{}]", team_name(rune.team), rune_mode_name(rune.rune_mode),
                    mechanism_state_name(rune.mechanism_state));

                auto label_entity =
                    viz::EntityBuilder::create<fast_tf::world>(
                        "gt", fmt::format("rune_label_{}", i))
                        .position(center)
                        .color(kLabelColor)
                        // 文字向上偏移Z，字体尺寸配置
                        .text_with_offset(label, 0, 0, kLabelOffsetZ, kLabelFontSize)
                        .timestamp(timestamp_ns)
                        .build();

                // 球体、文字加入场景实体列表
                entities.push_back(std::move(entity));
                entities.push_back(std::move(label_entity));
            }

            // 存在3D实体则打包Scene消息发送给Foxglove
            if (!entities.empty()) {
                GroundTruthSceneMessage msg;
                msg.payload.entities = std::move(entities);
                // Foxglove服务入队异步发送
                (*server)->enqueue_message(std::move(msg));
            }

            // ===================== 2. 序列化完整真值为JSON，发布结构化数值消息
            // =====================
            {
                nlohmann::json gt_json;
                // 帧序列号、时序戳顶层字段
                gt_json["frame_seq"]    = batch->frame_seq;
                gt_json["timestamp_ns"] = batch->timestamp_ns;

                // 敌方装甲目标数组
                gt_json["targets"] = nlohmann::json::array();
                for (uint32_t i = 0; i < batch->target_count; ++i) {
                    const auto& tgt = batch->targets[i];
                    // 单装甲JSON结构：阵营、类型、是否前哨、三维位置、偏航角/角速度
                    gt_json["targets"].push_back({
                        {       "team",                                 team_name(tgt.team)},
                        {"armor_label",                   armor_label_name(tgt.armor_label)},
                        { "is_outpost",                                      tgt.is_outpost},
                        {   "position", {tgt.position[0], tgt.position[1], tgt.position[2]}},
                        {       "vyaw",                                            tgt.vyaw},
                        {        "yaw",                                             tgt.yaw},
                    });
                }

                // 大符对象容器，区分Small/Large两类大符
                auto runes_obj = nlohmann::json::object();
                for (uint32_t i = 0; i < batch->rune_count; ++i) {
                    const auto& r    = batch->runes[i];
                    const char* mode = rune_mode_name(r.rune_mode);
                    // 单个大符完整运动学JSON
                    nlohmann::json rune_obj = {
                        {           "team",team_name(r.team)                                           },
                        {"mechanism_state",   mechanism_state_name(r.mechanism_state)},
                        {  "r_center_odom",
                         {r.r_center_odom[0], r.r_center_odom[1], r.r_center_odom[2]}},
                        {         "radius",                                  r.radius},
                        {  "current_angle",                           r.current_angle},
                        {         "v_roll",                                  r.v_roll},
                        {      "direction",                               r.direction},
                        {  "sin_amplitude",                           r.sin_amplitude},
                        {      "sin_omega",                               r.sin_omega},
                        {      "sin_phase",                               r.sin_phase},
                        {     "sin_offset",                              r.sin_offset},
                        {  "relative_time",                           r.relative_time},
                        {       "blade_id",                                r.blade_id},
                    };
                    // 5个装甲激活状态数组转为vector存入JSON
                    std::vector<uint8_t> activations(
                        r.target_activations, r.target_activations + 5);
                    rune_obj["target_activations"] = activations;
                    // 按大小符分类存入runes根对象
                    runes_obj[mode] = std::move(rune_obj);
                }
                gt_json["runes"] = std::move(runes_obj);

                // 通用工具：将JSON序列化为Foxglove标准JSON消息并发送
                detail::publish_json_message<GroundTruthMessage>(*server, gt_json);
            }
        });
}

} // namespace fcs::visualization::foxglove::systems