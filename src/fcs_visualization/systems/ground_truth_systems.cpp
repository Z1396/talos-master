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
    // 向talos调度器注册一个计算池系统（在talos::pool_compute线程池执行）
    app.add_system<talos::pool_compute>(
        // 系统唯一名称，用于日志、性能统计、性能埋点追踪
        "foxglove_ground_truth_pub",
        // Lambda作为系统的业务主逻辑
        [](
            // 只读SPMC通道：接收仿真输出的批量真值数据包
            // SPMC=单生产者多消费者，gt_in是只读句柄，读取仿真下发的GroundTruthBatch
            talos::spmc<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_in,
            // 全局只读资源：Foxglove服务共享指针，用于对外推送消息给前端
            talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // 前置校验1：判断Foxglove服务是否就绪，或者SPMC通道是否可用
            // 服务没启动/前端未连接，直接return，跳过本次发布，不浪费CPU
            if (!detail::foxglove_ready(*server, gt_in)) {
                return;
            }
            // 读取SPMC通道里面最新的一帧批量真值包
            // read()拿到最新的快照，返回optional类型
            auto batch = gt_in.read();
            // 没有新的真值数据包，直接退出本轮
            if (!batch) {
                return;
            }
            // 取出本帧纳秒级时间戳，3D实体、JSON调试消息共用同一个时间戳，保证时序对齐
            const uint64_t timestamp_ns = batch->timestamp_ns;
            // 容器：存放本帧所有Foxglove 3D场景实体（球体、文字标签）
            std::vector<::foxglove::schemas::SceneEntity> entities;

            // ===================== 1. 遍历所有大符真值，生成3D可视化实体 =====================
            for (uint32_t i = 0; i < batch->rune_count; ++i) // rune_count：本帧大符总数量
            {
                const auto& rune = batch->runes[i]; // runes数组：所有大符真值数组
                // 取出odom坐标系下大符三维中心坐标，封装为Eigen三维向量
                const Eigen::Vector3d center(
                    rune.r_center_odom[0], // X坐标，odom系，单位m
                    rune.r_center_odom[1], // Y坐标，odom系，单位m
                    rune.r_center_odom[2]  // Z坐标，odom系，单位m
                );

                // 用EntityBuilder流式构造Foxglove场景球体实体
                // fast_tf::world 指定坐标系为world(odom)
                // "gt" 命名空间，实体唯一ID rune_0/rune_1，前端会按ID复用/更新实体
                auto entity =
                    viz::EntityBuilder::create<fast_tf::world>(
                        "gt", fmt::format("rune_{}", i)
                        )
                        .position(center)                   // 设置球体中心三维坐标
                        .size(kRuneSphereRadius)            // 球体半径常量，3D可视化球体大小
                        .color(kRuneColor)                  // 球体半透明颜色常量
                        .sphere()                           // 声明几何体类型：球体
                        .timestamp(timestamp_ns)            // 实体时间戳，Foxglove时序同步
                        .build();

                // 构造大符上方的文字标签实体，用于在3D场景上叠加文字说明
                // 标签内容：阵营 / 大符类型 / 机构状态
                const std::string label = fmt::format(
                    "{}/Rune {} [{}]",
                    team_name(rune.team),        // rune.team：大符所属阵营
                    rune_mode_name(rune.rune_mode), // rune.rune_mode：大符类型（大/小）
                    mechanism_state_name(rune.mechanism_state) // rune.mechanism_state：机构工作状态
                );
                auto label_entity =
                    viz::EntityBuilder::create<fast_tf::world>(
                        "gt", fmt::format("rune_label_{}", i))
                        .position(center)                          // 标签基准位置和大符中心一致
                        .color(kLabelColor)                        // 文字颜色常量
                        .text_with_offset(label, 0, 0, kLabelOffsetZ, kLabelFontSize) // Z向上偏移，字体大小
                        .timestamp(timestamp_ns)                    // 标签时间戳，和球体保持一致
                        .build();

                // 将球体、文字标签移动推入实体vector，std::move避免拷贝
                entities.push_back(std::move(entity));
                entities.push_back(std::move(label_entity));
            }

            // 如果生成了3D场景实体，打包Scene消息发送给Foxglove
            if (!entities.empty())
            {
                GroundTruthSceneMessage msg; // Foxglove场景消息结构体
                msg.payload.entities = std::move(entities); // payload.entities：场景实体数组
                // 送入Foxglove服务消息队列，异步WebSocket推送到前端
                (*server)->enqueue_message(std::move(msg));
            }

            // ===================== 2. 序列化完整真值为JSON，发布结构化数值消息 =====================
            // 这一段输出结构化JSON数据，在Foxglove用Raw Message面板查看真值的全部参数
            {
                nlohmann::json gt_json; // 根json对象，存放本帧全部真值
                gt_json["frame_seq"]    = batch->frame_seq;    // 帧序列号，递增，用于对齐帧
                gt_json["timestamp_ns"] = batch->timestamp_ns; // 本帧纳秒时间戳

                gt_json["targets"] = nlohmann::json::array(); // targets数组：所有装甲真值列表
                for (uint32_t i = 0; i < batch->target_count; ++i) // target_count：本帧装甲总数量
                {
                    const auto& tgt = batch->targets[i]; // targets数组：单块装甲真值
                    gt_json["targets"].push_back({
                        {     "team",           team_name(tgt.team)},          // tgt.team：装甲所属阵营
                        {"armor_label",    armor_label_name(tgt.armor_label)}, // tgt.armor_label：装甲编号
                        { "is_outpost",                    tgt.is_outpost},    // tgt.is_outpost：是否为前哨装甲
                        {   "position", {tgt.position[0], tgt.position[1], tgt.position[2]}}, // tgt.position[XYZ]装甲三维坐标
                        {     "vyaw",                        tgt.vyaw},       // tgt.vyaw：装甲偏航角速度 rad/s
                        {      "yaw",                        tgt.yaw},        // tgt.yaw：装甲当前偏航角 rad
                    });
                }

                auto runes_obj = nlohmann::json::object(); // runes根对象，存放所有大符
                for (uint32_t i = 0; i < batch->rune_count; ++i) // rune_count：本帧大符数量
                {
                    const auto& r    = batch->runes[i]; // r：当前遍历的单个大符真值
                    const char* mode = rune_mode_name(r.rune_mode); // r.rune_mode：大符类型（Large/Small）
                    nlohmann::json rune_obj = {
                        {         "team",team_name(r.team)                          }, // r.team：大符所属阵营
                        {"mechanism_state",   mechanism_state_name(r.mechanism_state)}, // r.mechanism_state：机构状态
                        {  "r_center_odom", {r.r_center_odom[0], r.r_center_odom[1], r.r_center_odom[2]}}, // r_center_odom[XYZ]大符中心odom坐标
                        {         "radius",                    r.radius},          // r.radius：大符转动半径
                        {  "current_angle",             r.current_angle},         // r.current_angle：当前叶片角度
                        {         "v_roll",                    r.v_roll},          // r.v_roll：叶片滚动角速度
                        {      "direction",                 r.direction},         // r.direction：转动方向 +1/-1
                        {  "sin_amplitude",             r.sin_amplitude},         // r.sin_amplitude：正弦运动振幅
                        {      "sin_omega",               r.sin_omega},           // r.sin_omega：正弦角频率
                        {      "sin_phase",               r.sin_phase},           // r.sin_phase：正弦初相位
                        {     "sin_offset",              r.sin_offset},          // r.sin_offset：正弦位置偏移量
                        {  "relative_time",             r.relative_time},         // r.relative_time：相对运动时间
                        {      "blade_id",                  r.blade_id},           // r.blade_id：叶片编号
                    };
                    // 把5个装甲激活状态数组复制进json
                    std::vector<uint8_t> activations(r.target_activations, r.target_activations + 5); // target_activations：5块装甲激活标记
                    rune_obj["target_activations"] = activations;
                    runes_obj[mode] = std::move(rune_obj); // 以大符类型作为key存入runes对象
                }
                gt_json["runes"] = std::move(runes_obj);

                // 通用工具：将JSON序列化为Foxglove标准JSON消息并发送
                detail::publish_json_message<GroundTruthMessage>(*server, gt_json);
            }
        });

}

} // namespace fcs::visualization::foxglove::systems