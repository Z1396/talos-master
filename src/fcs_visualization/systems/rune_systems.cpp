// 基础工具、Foxglove消息类型、3D场景构建工具、调度器定义
#include "base.hpp"
#include "foxglove_types.hpp"
#include "scene_builder.hpp"
#include "scheduler/scheduler.hpp"

// L2感知层：风车检测器配置、检测输出数据结构
#include "L2_perception/rune/rune_config.hpp"
#include "L2_perception/rune/types.hpp"
// L3滤波层：能量机关EKF状态结构体
#include "L3_estimation/energy_meter/types.hpp"
// 全局通道话题定义、帧时间戳工具
#include "core/channel_topics.hpp"
#include "frame.hpp"

// 高性能字符串格式化库，替代sprintf/nlohmann流式拼接
#include <fmt/format.h>

// 顶层命名空间：火控系统 - 可视化 - Foxglove上位机 - 可视化系统注册入口
namespace fcs::visualization::foxglove::systems {

/**
 * @brief 注册全部风车能量机关可视化调度任务
 * @param app Talos全局调度器实例，所有任务挂载到此调度器
 * @detail 注册4个独立计算任务，分工：
 *  1. foxglove_rune_debug_images：30Hz定时推送相机原图调试截图（JPG图片流）
 *  2. foxglove_rune_debug_json：感知帧就绪时推送检测结构化JSON日志
 *  3. foxglove_rune_scene：原始观测有效时绘制3D球体（检测到的风车中心、扇叶目标）
 *  4. foxglove_rune_ekf_scene：EKF滤波状态有效时绘制完整风车模型（5扇叶+旋转外圈+运动参数文本）
 */
void register_rune_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // 任务1：风车视觉调试图像发布 30Hz固定频率定时任务
    // 类型：talos::fixed_rate<30> 固定30Hz执行，不受感知帧速率影响，稳定输出图像流
    // 输入通道：RuneDebugFrameChannelTopic 风车感知调试帧（内含3路JPG压缩图像）
    // 全局资源：FoxgloveServer 上位机连接服务句柄
    // =========================================================================
    app.add_system<talos::fixed_rate<30>>(
        "foxglove_rune_debug_images", // 任务唯一名称，日志/监控可区分
        [](
            // SPMC多生产者单消费者通道：读取L2感知输出的风车调试帧
            talos::spmc<::fcs::rune::RuneDebugFrame, RuneDebugFrameChannelTopic> dbg_in,
            // 共享资源句柄：Foxglove服务实例，用于入队消息推送上位机
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            // 工具函数：校验Foxglove服务是否就绪、输入通道是否合法，未就绪直接返回不执行
            if (!detail::foxglove_ready(*server, dbg_in)) {
                return;
            }

            // 读取通道最新一帧调试数据（智能指针，无数据返回空）
            auto dbg = dbg_in.read();
            // 无有效感知帧，跳过本次图像发布
            if (!dbg) {
                return;
            }

            // 局部lambda复用代码：统一封装JPG图像发布逻辑
            // 参数1：图像二进制字节数组；参数2：消息构造闭包，返回对应图像消息类型
            auto publish = [&](const std::vector<uint8_t>& bytes, auto builder) {
                // 图像为空直接跳过，避免推送空消息
                if (bytes.empty()) {
                    return;
                }
                // 调用闭包生成对应图像消息结构体（箭头图/目标图/中心图）
                auto msg              = builder();
                // 填充消息时间戳，转换纳秒帧时间戳为Foxglove标准时间对象
                msg.payload.timestamp = timestamp_from_ns(dbg->timestamp_ns);
                // 相机坐标系标识，上位机TF坐标系匹配
                msg.payload.frame_id  = "camera_optical_frame";
                // 图像压缩格式固定JPG
                msg.payload.format    = "jpeg";
                // 类型转换：uint8_t图像字节流转为std::byte，填充消息数据段
                msg.payload.data      = std::vector<std::byte>(
                    reinterpret_cast<const std::byte*>(bytes.data()),
                    reinterpret_cast<const std::byte*>(bytes.data() + bytes.size()));
                // 将消息送入Foxglove发送队列，异步推送上位机
                (*server)->enqueue_message(std::move(msg));
            };

            // 分别推送三路调试图像：箭头识别图、目标扇叶图、风车中心ROI图
            publish(dbg->arrow_jpeg, [] { return RuneArrowImageMessage{}; });
            publish(dbg->target_jpeg, [] { return RuneTargetImageMessage{}; });
            publish(dbg->rcenter_jpeg, [] { return RuneCenterImageMessage{}; });
        });

    // =========================================================================
    // 任务2：风车检测调试JSON结构化数据发布 事件驱动任务
    // 类型：talos::pool_compute 池化计算，通道有新数据自动触发一次执行
    // 输入1：RuneDebugFrame 感知调试帧原始数据
    // 输入2：RuneDetectorConfig 全局风车检测阈值配置（静态资源）
    // 输出：RuneDebugMessage JSON字符串消息
    // 实现特点：手动fmt拼接JSON，避免nlohmann::json对象序列化性能开销
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_rune_debug_json",
        [](
            talos::spmc<::fcs::rune::RuneDebugFrame, RuneDebugFrameChannelTopic> dbg_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server,
            // 全局静态资源：风车检测器参数配置（阈值、尺寸等）
            talos::res<::fcs::rune::RuneDetectorConfig> cfg
        ) {
            if (!detail::foxglove_ready(*server, dbg_in)) {
                return;
            }

            auto dbg = dbg_in.read();
            if (!dbg) {
                return;
            }

            // fmt内存缓冲区，分段拼接JSON字符串，减少内存多次分配
            fmt::memory_buffer buf;
            // 拼接JSON头部基础字段：时间戳、帧ID、各模块状态布尔值、ROI全局框、中心框
            fmt::format_to(
                std::back_inserter(buf),
                "{{\"timestamp_ns\":{},\"frame_id\":{},\"detect_ok\":{},\"tf_ok\":{}"
                ",\"solve_ok\":{},\"observation_valid\":{},\"status_code\":{},\"arrows\":{},"
                "\"targets\":{},\"global_roi\":{{\"x\":{:.3f},\"y\":{:.3f},\"w\":{:.3f},\"h\":{:."
                "3f}}},\"center_roi\":{{\"x\":{:.3f},\"y\":{:.3f},\"w\":{:.3f},\"h\":{:.3f}}},"
                "\"target_rois\":[",
                dbg->timestamp_ns,
                dbg->frame_id,
                dbg->detect_reversed ? "true" : "false", // 是否反向识别
                dbg->tf_ok ? "true" : "false",           // 坐标变换是否成功
                dbg->solve_ok ? "true" : "false",        // 风车解算是否正常
                dbg->observation_valid ? "true" : "false", // 观测值是否有效
                dbg->status_code,                        // 检测状态码
                dbg->arrows_count,                        // 识别到的箭头数量
                dbg->targets_count,                      // 识别到的扇叶目标数量
                dbg->global_roi.x, dbg->global_roi.y, dbg->global_roi.w, dbg->global_roi.h, // 全图ROI
                dbg->center_roi.x, dbg->center_roi.y, dbg->center_roi.w, dbg->center_roi.h  // 风车中心ROI
            );

            // 循环拼接所有扇叶目标ROI矩形数组
            for (size_t i = 0; i < dbg->target_rois.size(); ++i) {
                const auto& r = dbg->target_rois[i];
                // 非第一个元素前置逗号，JSON数组语法规范
                if (i != 0) {
                    fmt::format_to(std::back_inserter(buf), ",");
                }
                fmt::format_to(
                    std::back_inserter(buf),
                    "{{\"x\":{:.3f},\"y\":{:.3f},\"w\":{:.3f},\"h\":{:.3f}}}", r.x, r.y, r.w, r.h
                );
            }

            // 拼接JSON尾部：闭合ROI数组，追加三路检测阈值配置，闭合根对象
            fmt::format_to(
                std::back_inserter(buf),
                "],\"thresholds\":{{\"arrow\":{},\"target\":{},\"rcenter\":{}}}}}",
                cfg->arrow_threshold, cfg->target_threshold, cfg->rcenter_threshold
            );

            // 构造JSON消息，将缓冲区字符串转为二进制字节载荷
            RuneDebugMessage msg;
            msg.payload = json_to_bytes(fmt::to_string(buf));
            // 消息入队发送
            (*server)->enqueue_message(std::move(msg));
        });

    // =========================================================================
    // 任务3：原始观测风车3D场景发布 事件驱动
    // 输入通道：RuneObservationChannelTopic L2感知原始观测（未滤波的扇叶世界坐标）
    // 功能：在odom世界坐标系绘制3D球体，标记风车中心、所有检测到的扇叶目标
    // 适用场景：对比原始视觉观测与滤波后EKF模型的偏差
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_rune_scene",
        [](
            talos::spmc<::fcs::rune::RuneObservation, RuneObservationChannelTopic> obs_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            if (!detail::foxglove_ready(*server, obs_in)) {
                return;
            }

            auto obs = obs_in.read();
            // 无观测数据 / 观测值无效直接跳过
            if (!obs || !obs->valid) {
                return;
            }

            // Foxglove 3D场景实体数组，存储球体、线条、文本等渲染对象
            std::vector<::foxglove::schemas::SceneEntity> entities;

            // 1. 添加风车中心球体实体（参考色，尺寸0.08m）
            entities.push_back(
                viz::EntityBuilder::create<fast_tf::odom>("rune", "r_center")
                    .position(obs->r_center_odom.translation()) // 风车中心世界坐标
                    .size(0.08)                                 // 球体半径 0.08米
                    .color(tac::L4::MPC_REFERENCE)              // 配色：参考基准色
                    .sphere()                                   // 渲染类型：实心球体
                    .timestamp(obs->timestamp_ns)               // 实体绑定帧时间戳
                    .build()
            );

            // 2. 遍历所有视觉检测到的扇叶目标，逐个生成球体标记
            for (size_t i = 0; i < obs->target_positions_odom.size(); ++i) {
                const auto& p = obs->target_positions_odom[i];
                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>("rune", fmt::format("target_{}", i))
                        .position(p)
                        .size(0.06)
                        .color(tac::L4::MPC_PRESENT) // 配色：当前有效目标色
                        .sphere()
                        .timestamp(obs->timestamp_ns)
                        .build()
                );
            }

            // 实体数组非空时推送3D场景消息，空数组直接跳过
            publish_scene_if_nonempty<RuneSceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 任务4：EKF滤波后完整风车模型3D场景（核心可视化任务）
    // 输入通道：EnergyMeterStateChannelTopic L3能量机关EKF滤波状态
    // 功能：基于滤波后的中心、半径、三轴旋转角，数学还原完整5扇叶风车模型
    // 渲染内容：
    //  1. 风车中心大球体
    //  2. 5个扇叶三维点位（当前跟踪扇叶高亮放大，其余半透明缩小）
    //  3. 闭合环形线：扇叶旋转轨迹外圈
    //  4. 顶部文本：风车运动模型参数（角加速度、转速、旋转方向）
    // 坐标变换逻辑：局部扇叶坐标 → Pitch俯仰旋转 → Yaw水平旋转 → 叠加风车中心世界坐标
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_rune_ekf_scene",
        [](
            talos::spmc<::fcs::energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> state_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            if (!detail::foxglove_ready(*server, state_in)) {
                return;
            }

            auto state = state_in.read();
            // EKF未启动跟踪/滤波状态无效，跳过渲染
            if (!state || !state->tracking_valid) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;

            // 实体1：滤波后的风车中心点球体，尺寸更大便于区分原始观测
            entities.push_back(
                viz::EntityBuilder::create<fast_tf::odom>("rune_ekf", "r_center")
                    .position(state->r_center_odom)
                    .size(0.10)
                    .color(tac::L4::MPC_REFERENCE)
                    .sphere()
                    .timestamp(state->timestamp_ns)
                    .build()
            );

            // 提取EKF滤波输出的风车姿态与尺寸参数
            const double base_roll = state->roll;       // 风车自身旋转初始相位角
            const double yaw       = state->yaw;         // 风车整体水平偏航（底盘旋转）
            const double pitch     = state->pitch;       // 风车俯仰倾斜角
            const double radius    = state->radius;      // 扇叶旋转半径
            const int tracked_id   = state->blade_id;     // 当前正在跟踪识别的扇叶ID(0~4)

            // 预计算三角函数，循环内复用减少算力消耗
            const double cy = std::cos(yaw);
            const double sy = std::sin(yaw);
            const double sp = std::sin(pitch);
            const double cp = std::cos(pitch);

            // 循环生成5个扇叶的世界坐标实体（标准能量机关固定5扇叶）
            for (int i = 0; i < 5; ++i) {
                // 单个扇叶自身旋转相位：均分360°，5扇叶间隔72°
                const double roll_i = base_roll + static_cast<double>(i) * 2.0 * M_PI / 5.0;
                const double sr     = std::sin(roll_i);
                const double cr     = std::cos(roll_i);

                // 步骤1：扇叶在风车自身局部坐标系点位（未做俯仰/水平旋转）
                Eigen::Vector3d blade_local(0.0, -radius * sr, radius * cr);

                // 步骤2：绕Y轴Pitch俯仰旋转，适配风车倾斜姿态
                blade_local = Eigen::Vector3d(
                    blade_local.x() * cp + blade_local.z() * sp,
                    blade_local.y(),
                    -blade_local.x() * sp + blade_local.z() * cp
                );

                // 步骤3：绕Z轴Yaw水平旋转，叠加风车中心世界坐标，得到最终odom坐标系点位
                const Eigen::Vector3d pos =
                    state->r_center_odom
                    + Eigen::Vector3d(
                        blade_local.x() * cy - blade_local.y() * sy,
                        blade_local.x() * sy + blade_local.y() * cy,
                        blade_local.z()
                    );

                // 区分当前跟踪扇叶与普通扇叶：高亮不透明/半透明缩小
                const bool is_tracked = (i == tracked_id);
                const auto color =
                    is_tracked ? tac::L4::MPC_PRESENT : tac::with_alpha(tac::Semantic::INERT, 0.30);
                const double size = is_tracked ? 0.08 : 0.04;

                // 生成扇叶球体实体，附带ID文本标签b0~b4
                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>(
                        "rune_ekf", fmt::format("blade_{}", i))
                        .position(pos)
                        .size(size)
                        .color(color)
                        .sphere()
                        .text_with_offset(fmt::format("b{}", i), 0, 0, 0.06, tac::Text::SIZE_SMALL)
                        .timestamp(state->timestamp_ns)
                        .build()
                );
            }

            // 实体组：绘制扇叶旋转外圈闭合环线，直观展示旋转轨迹范围
            {
                constexpr int arc_points = 24; // 环线分段数，24段平滑圆形
                std::vector<::foxglove::schemas::Point3> circle_points;
                circle_points.reserve(arc_points);
                for (int k = 0; k < arc_points; ++k) {
                    // 0~2π均分角度
                    const double angle =
                        static_cast<double>(k) / static_cast<double>(arc_points) * 2.0 * M_PI;
                    const double sr = std::sin(angle);
                    const double cr = std::cos(angle);
                    // 局部点位+俯仰+水平旋转+世界坐标平移，与扇叶坐标变换逻辑完全一致
                    Eigen::Vector3d local(0.0, -radius * sr, radius * cr);
                    local = Eigen::Vector3d(
                        local.x() * cp + local.z() * sp,
                        local.y(),
                        -local.x() * sp + local.z() * cp
                    );
                    Eigen::Vector3d world(
                        state->r_center_odom.x() + local.x() * cy - local.y() * sy,
                        state->r_center_odom.y() + local.x() * sy + local.y() * cy,
                        state->r_center_odom.z() + local.z()
                    );
                    circle_points.push_back(viz::make_point3(world));
                }
                // 构建闭合环线（line_loop首尾自动相连），半透明灰色
                auto arc_builder =
                    viz::EntityBuilder::create<fast_tf::odom>("rune_ekf", "arc")
                        .line_loop(
                            circle_points, tac::with_alpha(tac::Semantic::INERT, 0.24), 0.005)
                        .timestamp(state->timestamp_ns);
                entities.push_back(std::move(arc_builder).build());
            }

            // 实体：风车运动模型参数文本标签，悬浮在风车上方
            {
                std::string info;
                // 模型收敛：输出角加速度a、角速度w、偏置b、旋转方向
                if (state->model_valid) {
                    info = fmt::format(
                        "a={:.3f} w={:.3f} b={:.3f} dir={}",
                        state->a, state->omega, state->b, state->direction
                    );
                } else {
                    // 模型未收敛，提示滤波收敛中
                    info = "model: converging";
                }
                // 文本位置：风车中心Z轴向上偏移，避免球体遮挡
                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>("rune_ekf", "info")
                        .position(state->r_center_odom + Eigen::Vector3d(0, 0, radius + 0.15))
                        .text(info, tac::Text::SIZE_SMALL)
                        .timestamp(state->timestamp_ns)
                        .build()
                );
            }

            // 推送EKF完整风车3D场景消息
            publish_scene_if_nonempty<RuneEkfSceneMessage>(*server, std::move(entities));
        });
}

} // namespace fcs::visualization::foxglove::systems