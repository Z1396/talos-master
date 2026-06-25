#include "base.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "foxglove_types.hpp"

#include "L3_estimation/energy_meter/types.hpp"
#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include "scheduler/scheduler.hpp"

#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <system_helpers.hpp>
#include <utility.hpp>

namespace fcs::visualization::foxglove::systems {

/// @brief Register debug visualization systems
///
/// This includes all debug publishing systems for L2, L3, and L4 layers:
/// - L2: PnP solver metrics, NN confidence scores
/// - L3: Association phase, permutation, filter innovation/covariance, Viterbi, state machine
/// - L4: MPC horizon, cost, constraints
/// - Misc: Energy meter state, performance statistics
/// @param app ECS调度器实例，用于注册可视化System
/// @param scheduler_ptr 调度器裸指针，用于读取系统性能统计，可为nullptr
void register_debug_systems(talos::scheduler::Scheduler& app, talos::Scheduler* scheduler_ptr) {

    // =========================================================================
    // L2 DEBUG: 全局基础资源定时发布系统 1Hz
    // =========================================================================
    /**
     * talos::fixed_rate_silent<1>：固定1Hz周期任务，silent代表任务超时不会打印告警日志
     * 作用：定时推送机器人全局静态/基础配置给Foxglove前端
     * 依赖：Foxglove全局可视化服务、弹道速度、识别颜色、功能开关、跟随模式资源
     */
    app.add_system<talos::fixed_rate_silent<1>>(
        "foxglove_res_pub",
        [](talos::res<std::shared_ptr<FoxgloveServer>> server,
           core::trajectory::bullet_speed bullet_speed,
           core::detecting_color color,
           core::capabilities cap,
           core::following following) {
            // 解包shared_ptr，简化后续调用
            auto& server_ptr = *server;
            // 可视化服务未初始化（WebSocket未连接/Mcap未开启）直接跳过，防止空指针崩溃
            if (!server_ptr->is_initialized()) {
                return;
            }

            // 构造标准JSON数据包，适配Foxglove前端解析
            nlohmann::json j;
            // 获取高精度纳秒时间戳，前端用于多传感器时序对齐
            auto now               = clock::now_ns();
            j["timestamp"]["sec"]  = now / 1000000000L;
            j["timestamp"]["nsec"] = now % 1000000000L;
            // 子弹初速度
            j["bullet_speed"]      = bullet_speed->bullet_speed;
            // 识别敌方颜色，magic_enum将枚举转为可读字符串
            j["color"]             = magic_enum::enum_name(*color);
            // 自瞄/能量机关跟随开关状态
            j["following"]         = following->load();

            // 遍历已启用的机器人功能，转为字符串数组存入JSON
            std::vector<std::string> cap_str;
            const auto active = core::active_capabilities(*cap);
            cap_str.reserve(active.size());
            for (const auto c : active) {
                cap_str.emplace_back(magic_enum::enum_name(c));
            }
            j["capabilities"] = cap_str;

            // 通用工具：序列化JSON并通过对应Topic推送到Foxglove
            detail::publish_json_message<ResourceMessage>(*server, j);
        });

    // =========================================================================
    // L2 DEBUG: PnP解算位姿发布系统 线程池并行计算
    // =========================================================================
    /**
     * talos::pool_compute：放入调度线程池并行执行，不阻塞主线程视觉流程
     * 输入SPMC通道：装甲板批量测量数据（PnP解算输出位姿、置信度）
     */
    app.add_system<talos::pool_compute>(
        "foxglove_debug_l2_pnp_pub",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // 统一前置校验：服务就绪、输入通道有有效数据
            if (!detail::foxglove_ready(*server, meas_in)) {
                return;
            }

            // 无锁读取SPMC通道批量测量帧
            auto batch = meas_in.read();
            // 空帧直接丢弃
            if (!batch) {
                return;
            }

            // 构建PnP可视化JSON
            nlohmann::json pnp_json;
            pnp_json["timestamp_ns"] = batch->timestamp_ns;
            pnp_json["measurements"] = nlohmann::json::array();

            // 遍历当前帧所有装甲板测量结果
            for (size_t i = 0; i < batch->measurements.size(); ++i) {
                const auto& m = batch->measurements[i];
                nlohmann::json obj;
                obj["index"]      = i;
                obj["name"]       = magic_enum::enum_name(m.name); // 装甲板编号枚举转字符串
                // 平移坐标 xyz
                const auto t      = m.transform.translation();
                obj["position"]   = {t.x(), t.y(), t.z()};
                // 欧拉角 rpy
                const auto rpy    = m.transform.euler_rot().rpy();
                obj["rpy"]        = {std::get<0>(rpy), std::get<1>(rpy), std::get<2>(rpy)};
                obj["confidence"] = m.confidence; // PnP解算置信度
                pnp_json["measurements"].push_back(obj);
            }

            // 发送PnP话题数据
            detail::publish_json_message<PnPSolverMessage>(*server, pnp_json);
        });

    // =========================================================================
    // L2 DEBUG: 神经网络检测置信度发布系统 线程池并行
    // =========================================================================
    /**
     * 输入：神经网络原始检测批量结果，输出目标类别、置信度、颜色
     */
    app.add_system<talos::pool_compute>(
        "foxglove_debug_l2_nn_confidence_pub",
        [](talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, det_in)) {
                return;
            }

            auto batch = det_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json nn_json;
            nn_json["timestamp_ns"] = batch->timestamp_ns;
            nn_json["detections"]   = nlohmann::json::array();

            // 遍历所有神经网络检测框
            for (size_t i = 0; i < batch->detections.size(); ++i) {
                const auto& det = batch->detections[i];
                nlohmann::json obj;
                obj["index"]      = i;
                obj["confidence"] = det.confidence;    // 网络输出置信度
                obj["color"]      = magic_enum::enum_name(det.color); // 目标颜色
                obj["type"]       = magic_enum::enum_name(det.type);  // 装甲/能量机关类型
                obj["name"]       = magic_enum::enum_name(det.name);  // 装甲板编号
                nn_json["detections"].push_back(obj);
            }

            detail::publish_json_message<NNConfidenceMessage>(*server, nn_json);
        });

    // =========================================================================
    // L3 能量机关状态可视化发布系统 线程池并行
    // =========================================================================
    /**
     * 输入：能量机关跟踪器完整状态（圆心、半径、角速度、角度、跟踪有效性等）
     */
    app.add_system<talos::pool_compute>(
        "foxglove_debug_energy_meter_pub",
        [](talos::spmc<fcs::energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> state_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, state_in)) {
                return;
            }

            auto state = state_in.read();
            if (!state) {
                return;
            }

            nlohmann::json em_json;
            em_json["timestamp_ns"]   = state->timestamp_ns;
            em_json["tracker_state"]  = magic_enum::enum_name(state->tracker_state); // 跟踪器状态机
            em_json["tracking_valid"] = state->tracking_valid;                       // 跟踪是否有效
            em_json["r_center_odom"]  = {
                state->r_center_odom.x(), state->r_center_odom.y(), state->r_center_odom.z()};
            em_json["radius"]   = state->radius;    // 能量机关半径
            em_json["roll"]     = state->roll;
            em_json["t"]        = state->t;
            em_json["blade_id"] = state->blade_id;  // 当前叶片编号
            em_json["position"] = {state->position.x(), state->position.y(), state->position.z()};
            em_json["is_big_rune"] = state->is_big_rune;
            em_json["model_valid"] = state->model_valid;
            em_json["direction"]   = state->direction;
            em_json["a"]           = state->a;
            em_json["omega"]       = state->omega;  // 旋转角速度
            em_json["b"]           = state->b;
            em_json["r"]           = state->radius;
            em_json["obs_valid"]   = state->obs_valid;
            em_json["yaw"]         = state->yaw;

            detail::publish_json_message<EnergyMeterMessage>(*server, em_json);
        });

    // =========================================================================
    // 全系统性能统计发布系统 100Hz固定周期
    // =========================================================================
    /**
     * 仅当传入有效调度器指针时注册；
     * 读取ECS所有System运行耗时、调度延迟统计，转为JSON推送到前端
     */
    if (scheduler_ptr) {
        app.add_system<talos::fixed_rate_silent<100>>(
            "foxglove_debug_perf_stats_pub",
            [scheduler_ptr](talos::res<std::shared_ptr<FoxgloveServer>> server) {
                auto& server_ptr = *server;
                if (!server_ptr->is_initialized()) {
                    return;
                }

                // 从调度器获取性能统计完整JSON字符串
                std::string json_str = scheduler_ptr->get_stats_json();

                // 填充性能消息，二进制序列化JSON
                PerfStatsMessage msg;
                msg.payload = json_to_bytes(json_str);
                // 消息入队，异步推送到WebSocket前端
                server_ptr->enqueue_message(std::move(msg));
            });
    }
}

} // namespace fcs::visualization::foxglove::systems