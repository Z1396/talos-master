#include "L3_estimation/tracker/types.hpp"          // L3目标跟踪器输出数据结构定义
#include "L3_estimation/tracker/vis_helpers.hpp"   // 跟踪可视化通用绘图构建工具
#include "base.hpp"                                 // 项目基础宏、类型别名、工具函数
#include "foxglove_types.hpp"                       // Foxglove前端消息结构体定义(SceneEntity/各类可视化消息)
#include "scene_builder.hpp"                        // Foxglove 3D场景实体构建器EntityBuilder

#include "L2_perception/ldm/types.hpp"              // L2层能量机关原始测量数据
#include "L3_estimation/ldm_naive/types.hpp"        // L3层能量机关跟踪状态结构体
#include "L3_estimation/tracker/util.hpp"           // 跟踪器工具函数（提取位置/速度）
#include "core/channel_topics.hpp"                 // SPMC消息通道Topic常量定义
#include "core/types.hpp"                           // 项目通用基础数据类型

#include <Eigen/Core>                               // Eigen线性代数基础矩阵/向量
#include <Eigen/Geometry>                           // Eigen四元数、旋转矩阵、位姿变换
#include <cmath>                                    // 标准数学库、pi、数值判断
#include <magic_enum.hpp>                           // 枚举值自动转可读字符串，无需手写映射表
#include <nlohmann/json.hpp>                        // JSON序列化库，用于结构化调试数据下发
#include <opencv2/core.hpp>                         // OpenCV基础矩阵、点、尺寸类型
#include <opencv2/imgcodecs.hpp>                    // OpenCV图像编码（PNG/JPG内存压缩）
#include <opencv2/imgproc.hpp>                      // OpenCV绘图、色彩映射、缩放、文字绘制
#include <optional>                                 // std::optional 空安全包装类型
#include <string>                                   // 标准字符串
#include <vector>                                   // 动态数组容器

// 命名空间分层：项目fcs -> 可视化模块 -> Foxglove网页可视化 -> 可视化系统注册逻辑
namespace fcs::visualization::foxglove::systems {

/**
 * @brief 格式化跟踪中心距离文本，非法数值输出n/a
 * @param value 像素距离浮点值
 * @return 格式化字符串，有限值保留1位小数，NaN/无穷返回n/a
 * [[nodiscard]] 强制接收返回值，禁止丢弃格式化文本结果
 */
[[nodiscard]] inline std::string tracker_image_center_text(double value) {
    // std::isfinite 判断是否为合法有限浮点数（过滤NaN/Inf）
    return std::isfinite(value) ? fmt::format("{:.1f}", value) : "n/a";
}

/**
 * @brief 生成目标跟踪状态标签文本（颜色+装甲类型+跟踪状态）
 * @param output L3跟踪器单目标输出结果
 * @return 格式化标签字符串，用于3D实体上方文字显示
 */
[[nodiscard]] inline std::string tracker_label(const ::fcs::L3::TrackerOutput& output) {
    return fmt::format(
        "{}/{} · {}",
        magic_enum::enum_name(output.target_color),  // 敌方颜色 Red/Blue
        magic_enum::enum_name(output.target_name),    // 目标类型 Armor1~4/Outpost
        magic_enum::enum_name(output.status)          // 跟踪状态 Lost/Tracking/Init等
    );
}

/**
 * @brief 判断两个时间戳差值是否在允许最大偏移内（帧同步工具）
 * @param lhs_timestamp_ns 左帧纳秒时间戳
 * @param rhs_timestamp_ns 右帧纳秒时间戳
 * @param max_skew_ns 允许最大时间偏差阈值（单位ns）
 * @ noexcept 函数不抛出C++异常
 * @return true 两帧时间接近，可以叠加可视化图层；false 偏差过大，丢弃图层
 */
[[nodiscard]] inline bool timestamp_close(
    uint64_t lhs_timestamp_ns, uint64_t rhs_timestamp_ns, uint64_t max_skew_ns) noexcept {
    // 无符号减法防溢出：大数减小数获取绝对时间差
    const uint64_t skew = lhs_timestamp_ns > rhs_timestamp_ns ? lhs_timestamp_ns - rhs_timestamp_ns
                                                              : rhs_timestamp_ns - lhs_timestamp_ns;
    // 差值小于等于阈值则判定帧同步
    return skew <= max_skew_ns;
}

/**
 * @brief 生成LDM能量机关跟踪状态文本标签
 * @param state L3层能量机关完整跟踪状态
 * @return 固定前缀+跟踪状态字符串
 */
[[nodiscard]] inline std::string ldm_tracker_label(const ::fcs::L3::ldm::LdmState& state) {
    return fmt::format("LDM · {}", magic_enum::enum_name(state.status));
}

/**
 * @brief 构建能量机关坐标系XYZ三轴3D可视化实体
 * @param state LDM跟踪状态（包含世界坐标系位姿、时间戳）
 * @return Foxglove SceneEntity 3D线框实体，用于显示局部坐标系
 */
[[nodiscard]] inline ::foxglove::schemas::SceneEntity
    make_ldm_axes_entity(const ::fcs::L3::ldm::LdmState& state) {
    // 三轴可视化常量：轴长度0.18m，线条粗细0.006m
    constexpr double kAxisLength = 0.18;
    constexpr double kThickness  = 0.006;

    // 创建Scene实体构建器：父坐标系odom，分组l3，实体唯一名称ldm_axes
    auto builder = viz::EntityBuilder::create<fast_tf::odom>("l3", "ldm_axes");
    // 绑定当前帧时间戳、平移坐标、旋转四元数
    builder.timestamp(state.timestamp_ns)
        .position(state.position())
        .orientation(Eigen::Quaterniond(state.rotation()));

    // X轴线段：原点→X正方向，X轴标准配色
    builder.line_strip(
        {viz::make_point3(0.0, 0.0, 0.0), viz::make_point3(kAxisLength, 0.0, 0.0)}, tac::Axis::X,
        kThickness);
    // Y轴线段：原点→Y正方向，Y轴标准配色
    builder.line_strip(
        {viz::make_point3(0.0, 0.0, 0.0), viz::make_point3(0.0, kAxisLength, 0.0)}, tac::Axis::Y,
        kThickness);
    // Z轴线段：原点→Z正方向，Z轴标准配色
    builder.line_strip(
        {viz::make_point3(0.0, 0.0, 0.0), viz::make_point3(0.0, 0.0, kAxisLength)}, tac::Axis::Z,
        kThickness);

    // 生成并返回完整3D实体对象
    return builder.build();
}

/**
 * @brief 构建LDM跟踪器主可视化实体（彩色立方体+状态文字）
 * @param state LDM跟踪状态
 * @return 带颜色立方体、顶部状态文字、调试元数据的SceneEntity
 */
[[nodiscard]] inline ::foxglove::schemas::SceneEntity
    make_ldm_state_entity(const ::fcs::L3::ldm::LdmState& state) {
    return viz::EntityBuilder::create<fast_tf::odom>("l3", "ldm_tracker")
        .timestamp(state.timestamp_ns)
        .position(state.position())
        .orientation(Eigen::Quaterniond(state.rotation()))
        // 根据跟踪状态匹配对应显示颜色（丢失/稳定/初始化区分色调）
        .color(tac::tracker_status_color(static_cast<int>(state.status)))
        // 0.08m边长立方体代表能量机关跟踪中心
        .cube(0.08, 0.08, 0.08)
        // 立方体上方偏移文字，显示LDM跟踪状态
        .text_with_offset(
            ldm_tracker_label(state), 0.0, 0.0, tac::L3::LABEL_OFFSET_Z, tac::Text::SIZE_SMALL)
        // 附加元数据，Foxglove前端可悬停查看
        .metadata("status", std::string(magic_enum::enum_name(state.status)))
        .metadata(
            "last_observation_timestamp_ns", std::to_string(state.last_observation_timestamp_ns))
        .build();
}

/**
 * @brief 构建LDM原始观测测量值实体（半透明灰色立方体，区分预测值）
 * @param measurement L2层LDM PnP原始测量结果
 * @return 半透明灰色小立方体，代表当前帧原始观测点
 */
[[nodiscard]] inline ::foxglove::schemas::SceneEntity
    make_ldm_measurement_entity(const ::fcs::L2::ldm::LdmMeasurement& measurement) {
    // 测量值位姿（odom坐标系）
    const auto& pose = *measurement.transform_odom;

    return viz::EntityBuilder::create<fast_tf::odom>("l3", "ldm_measurement")
        .timestamp(measurement.timestamp_ns)
        .position(pose.translation())
        .orientation(pose.quaternion())
        // 半透明灰色，作为“虚影”区分跟踪预测值
        .color(tac::L2::MEASUREMENT_GHOST)
        // 0.05m小立方体
        .cube(0.05, 0.05, 0.05)
        .text_with_offset("LDM meas", 0.0, 0.0, 0.08, tac::Text::SIZE_SMALL)
        // 观测置信度、深度质量元数据
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .build();
}

/// @brief 注册全部L3估计算法可视化ECS系统
/// 覆盖跟踪目标3D场景、能量机关跟踪、观测关联对比、EKF协方差热力图、LDM结构化JSON调试数据
/// 包含4类可视化System：
/// 1. foxglove_l3_tracker_scene：敌方机器人/前哨站目标3D跟踪可视化（球体、速度箭头、装甲立方体）
/// 2. foxglove_l3_ldm_tracker_scene：能量机关LDM跟踪3D场景（坐标系、跟踪立方体、原始观测虚影）
/// 3. foxglove_l3_association_scene：数据关联可视化（预测装甲框vs图像原始观测点对比）
/// 4. foxglove_l3_ekf_heatmap：EKF卡尔曼滤波矩阵热力图（协方差P/增益K/过程噪声Q/观测噪声R）
/// 5. foxglove_l3_ldm_tracker_json：LDM跟踪完整状态JSON结构化数据下发
/// @param app ECS调度器实例，用于注册并行可视化任务
void register_l3_estimation_systems(talos::scheduler::Scheduler& app) {

    // 导入L3层可视化工具命名空间，简化绘图API调用
    using namespace L3::vis;

    // =========================================================================
    // 系统1：foxglove_l3_tracker_scene 通用目标跟踪3D场景发布
    // 调度策略：talos::pool_compute 线程池并行执行，不阻塞视觉主线程
    // 输入通道：TrackerOutput批量跟踪结果数组（多目标并行跟踪输出）
    // 全局资源：Foxglove可视化服务单例
    // 功能：遍历所有有效跟踪目标，生成3D球体、速度箭头、装甲模型、文字标签，批量下发Scene场景消息
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l3_tracker_scene",
        [](
            // SPMC输入通道：批量多目标跟踪输出数组
            talos::spmc<std::vector<::fcs::L3::TrackerOutput>, TrackerOutputChannelTopic> tracker_in,
            // 全局只读资源：Foxglove服务智能指针，提供场景消息发送接口
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            // 前置校验：Foxglove服务已初始化、输入通道存在可读数据
            if (!detail::foxglove_ready(*server, tracker_in))
                return;
            // 读取最新一批跟踪结果
            auto outputs = tracker_in.read();
            // 空数据直接跳过本轮渲染
            if (!outputs || outputs->empty())
                return;

            // 存储本帧所有3D场景实体，统一批量发送（减少IO频繁调用）
            std::vector<::foxglove::schemas::SceneEntity> entities;

            // 遍历全部跟踪目标
            for (const auto& output : *outputs) {
                // 仅处理处于有效跟踪状态的目标，跳过丢失/未初始化目标
                if (!output.is_tracking())
                    continue;

                // 构造目标唯一标识字符串（颜色_类型，区分多个目标实体）
                const std::string target_key =
                    fmt::format("{}_{}", output.target_color, output.target_name);
                // 提取跟踪器输出的世界坐标、平移速度、偏航角速度
                const auto pos   = get_tracker_position(output);
                const auto vel   = get_tracker_velocity(output);
                const auto v_yaw = get_tracker_v_yaw(output);

                // 1. 目标主体球体：根据跟踪状态切换颜色（稳定/丢失/初始化）
                entities.push_back(
                    viz::patterns::tracker_target<fast_tf::odom>(
                        Eigen::Vector3d(pos.x, pos.y, pos.z), static_cast<int>(output.status),
                        output.timestamp_ns, fmt::format("target_{}", target_key)));

                // 2. 目标文字标签实体（目标颜色、类型、跟踪状态、调试元数据）
                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>(
                        "l3",
                        fmt::format("tracker_info_{}_{}", output.target_color, output.target_name))
                        .timestamp(output.timestamp_ns)
                        .position(pos)
                        .color(tac::Text::PRIMARY)
                        // 球体上方偏移文字
                        .text_with_offset(
                            tracker_label(output), tac::L3::TARGET_SIZE * 1.4, 0.0,
                            tac::L3::LABEL_OFFSET_Z, tac::Text::SIZE_SMALL)
                        // 前端悬停可查看的调试元信息
                        .metadata(
                            "target_name", std::string(magic_enum::enum_name(output.target_name)))
                        .metadata(
                            "target_color", std::string(magic_enum::enum_name(output.target_color)))
                        .metadata("status", std::string(magic_enum::enum_name(output.status)))
                        .metadata(
                            "last_image_center_distance_px",
                            tracker_image_center_text(output.last_image_center_distance_px))
                        .metadata(
                            "last_observation_timestamp_ns",
                            std::to_string(output.last_observation_timestamp_ns))
                        .build());

                // 3. 速度矢量箭头实体：X/Y平移速度 + 旋转偏航可视化箭头
                entities.push_back(
                    viz::patterns::velocity_arrows<fast_tf::odom>(
                        Eigen::Vector3d(pos.x, pos.y, pos.z), vel, v_yaw, output.timestamp_ns,
                        fmt::format("velocity_{}", target_key)));

                // 4. 装甲板立方体模型绘制（区分机器人/前哨站两种目标）
                if (output.is_robot()) {
                    // 机器人目标：4块装甲板立方体
                    auto builder = viz::EntityBuilder::create<fast_tf::odom>(
                                       "l3", fmt::format(
                                                 "robot_armors_{}_{}", output.target_color,
                                                 output.target_name))
                                       .timestamp(output.timestamp_ns);
                    // 填充四块装甲3D立方体实体
                    add_robot_armor_cubes(builder, *output.robot_state(), output.target_name);
                    entities.push_back(std::move(builder).build());
                } else if (output.is_outpost()) {
                    // 前哨站目标：3块环形装甲板立方体
                    auto builder = viz::EntityBuilder::create<fast_tf::odom>(
                                       "l3", fmt::format("outpost_armors_{}", output.target_color))
                                       .timestamp(output.timestamp_ns);
                    add_outpost_armor_cubes(builder, *output.outpost_state());
                    entities.push_back(std::move(builder).build());
                }
            }

            // 实体数组非空才发送Scene消息，空则跳过；使用移动语义转移容器，无拷贝
            publish_scene_if_nonempty<TrackSceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 系统2：foxglove_l3_ldm_tracker_scene 能量机关LDM跟踪3D可视化
    // 输入：LDM跟踪状态通道、LDM原始PnP测量通道
    // 功能：绘制LDM跟踪立方体、局部坐标系三轴、速度箭头、原始观测虚影立方体（帧同步200ms容忍窗口）
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l3_ldm_tracker_scene",
        [](
            // LDM跟踪状态数据流
            talos::spmc<::fcs::L3::ldm::LdmState> ldm_in,
            // LDM原始观测测量数据流
            talos::spmc<::fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> ldm_meas_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            // 校验服务与输入通道就绪
            if (!detail::foxglove_ready(*server, ldm_in)) {
                return;
            }

            // 读取最新LDM跟踪状态
            const auto state = ldm_in.read();
            // 无数据 / 未处于跟踪状态直接返回
            if (!state || !state->is_tracking()) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;
            // 添加LDM主跟踪立方体实体
            entities.push_back(make_ldm_state_entity(*state));
            // 添加XYZ局部坐标系三轴线框
            entities.push_back(make_ldm_axes_entity(*state));
            // 添加LDM世界坐标系速度矢量箭头
            entities.push_back(
                viz::patterns::velocity_arrows<fast_tf::odom>(
                    state->position(), state->velocity_world(), 0.0, state->timestamp_ns,
                    "ldm_velocity"));

            // 帧同步最大允许时间偏移200ms，轻微不同步仍可叠加原始观测虚影
            constexpr uint64_t kMaxMeasurementSkewNs = 200'000'000;
            // 读取缓存的最新一帧LDM原始测量数据
            const auto measurement                   = ldm_meas_in.read_current();
            // 测量帧有效、位姿存在、时间戳偏差在阈值内，则绘制半透明观测立方体
            if (measurement && measurement->transform_odom.has_value()
                && timestamp_close(
                    measurement->timestamp_ns,
                    state->timestamp_ns, kMaxMeasurementSkewNs)) {
                entities.push_back(make_ldm_measurement_entity(*measurement));
            }

            // 批量下发LDM跟踪场景消息
            publish_scene_if_nonempty<LdmTrackSceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 系统3：foxglove_l3_association_scene 数据关联可视化（预测装甲vs图像观测对比）
    // 输入：多目标跟踪输出、L2层装甲批量观测帧
    // 功能：绘制滤波预测装甲3D球体 + 图像原始观测半透明球体，直观展示关联匹配效果
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l3_association_scene",
        [](
            talos::spmc<std::vector<::fcs::L3::TrackerOutput>, TrackerOutputChannelTopic> tracker_in,
            talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            if (!detail::foxglove_ready(*server, tracker_in))
                return;

            auto outputs = tracker_in.read();
            if (!outputs || outputs->empty())
                return;

            // 读取当前最新图像装甲观测帧
            auto batch = meas_in.read_current();
            if (!batch)
                return;

            std::vector<::foxglove::schemas::SceneEntity> entities;

            // 遍历所有有效跟踪目标，绘制滤波预测装甲框
            for (const auto& output : *outputs) {
                if (!output.is_tracking())
                    continue;

                const std::string target_key =
                    fmt::format("{}_{}", output.target_color, output.target_name);
                // 机器人目标：4个预测装甲点球体
                if (output.is_robot()) {
                    const auto& state = *output.robot_state();
                    const auto armors = state.armor_poses();
                    for (int i = 0; i < 4; i++) {
                        entities.push_back(
                            viz::patterns::predicted_armor<fast_tf::odom>(
                                Eigen::Vector3d(armors[i][0], armors[i][1], armors[i][2]), i,
                                output.timestamp_ns, fmt::format("pred_{}_{}", target_key, i)));
                    }
                } else if (output.is_outpost()) {
                    // 前哨站目标：环形3个预测装甲点，均匀120度分布
                    const auto& state           = *output.outpost_state();
                    constexpr double kAngleStep = 2.0 * std::numbers::pi / 3.0;
                    for (int i = 0; i < 3; i++) {
                        const double armor_yaw = state.yaw + i * kAngleStep;
                        const auto& pos        = state.position;
                        constexpr double r     = L3::OutpostTargetState::radius;
                        // 极坐标转笛卡尔世界坐标
                        const Eigen::Vector3d armor_pos{
                            pos.x() + r * std::cos(armor_yaw), pos.y() + r * std::sin(armor_yaw),
                            state.z[i]};

                        // 绘制前哨站预测装甲半透明球体
                        entities.push_back(
                            viz::EntityBuilder::create<fast_tf::odom>(
                                "l3", fmt::format("pred_{}_{}", target_key, i))
                                .position(armor_pos)
                                .size(tac::L3::ARMOR_PLATE_SIZE)
                                .color(tac::L3::PREDICTION_CONTEXT)
                                .sphere()
                                .text_with_offset(
                                    fmt::format("{}:{}", target_key, i), 0, 0,
                                    tac::L3::LABEL_OFFSET_Z)
                                .timestamp(output.timestamp_ns)
                                .build());
                    }
                }
            }

            // 绘制本帧图像原始观测装甲（灰色虚影，区分滤波预测值）
            for (const auto& m : batch->measurements) {
                const auto t = m.transform.translation();
                entities.push_back(
                    viz::patterns::measurement_sphere<fast_tf::odom>(
                        Eigen::Vector3d(t.x(), t.y(), t.z()), m.confidence, m.timestamp_ns));
            }

            // 下发关联对比场景消息
            publish_scene_if_nonempty<AssociationSceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 系统4：foxglove_l3_ekf_heatmap EKF卡尔曼矩阵热力图可视化（调试滤波收敛、噪声参数）
    // 功能：将协方差P、卡尔曼增益K、过程噪声Q、观测噪声R矩阵渲染为伪彩色热力图，拼接大图编码PNG下发Foxglove
    // 注：代码第一行return临时关闭热力图渲染，可注释启用
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l3_ekf_heatmap",
        [](
            talos::spmc<std::vector<::fcs::L3::TrackerOutput>, TrackerOutputChannelTopic> tracker_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            // 临时禁用热力图渲染，直接return退出，调试时注释此行开启
            return;
            if (!detail::foxglove_ready(*server, tracker_in))
                return;
            auto outputs = tracker_in.read();
            if (!outputs || outputs->empty())
                return;

            // 筛选当前处于跟踪状态的目标
            std::vector<const ::fcs::L3::TrackerOutput*> active;
            for (const auto& o : *outputs) {
                if (o.is_tracking())
                    active.push_back(&o);
            }
            if (active.empty())
                return;

            // 内部闭包：单矩阵热力图渲染函数
            // 输入：Eigen矩阵、单元格像素尺寸、行/列标签数组；输出带文字网格的彩色热力图Mat
            const auto render_heatmap =
                [](const Eigen::MatrixXd& mat, int cell,
                   const std::vector<std::string_view>& row_labels,
                   const std::vector<std::string_view>& col_labels) -> cv::Mat {
                const int rows = static_cast<int>(mat.rows());
                const int cols = static_cast<int>(mat.cols());
                // 空矩阵直接返回空白图
                if (rows == 0 || cols == 0)
                    return {};

                // 1. 矩阵数值归一化到0~255灰度区间
                double vmin  = mat.minCoeff();
                double vmax  = mat.maxCoeff();
                double range = vmax - vmin;
                // 防止全相同数值除0异常
                if (range < 1e-15)
                    range = 1.0;

                // 灰度单通道图
                cv::Mat gray(rows, cols, CV_8U);
                for (int r = 0; r < rows; ++r)
                    for (int c = 0; c < cols; ++c)
                        gray.at<uint8_t>(r, c) = static_cast<uint8_t>(
                            std::clamp(255.0 * (mat(r, c) - vmin) / range, 0.0, 255.0));

                // 应用Viridis伪彩色映射，转为彩色热力图
                cv::Mat colored;
                cv::applyColorMap(gray, colored, cv::COLORMAP_VIRIDIS);

                // 放大到指定单元格尺寸，最近邻插值无模糊
                cv::Mat big;
                cv::resize(
                    colored, big, cv::Size(cols * cell, rows * cell), 0, 0, cv::INTER_NEAREST);

                // 绘制网格分隔线
                for (int r = 0; r <= rows; ++r)
                    cv::line(
                        big, {0, r * cell}, {cols * cell, r * cell},
                        tac::to_cv_bgr(tac::Semantic::CONTEXT), 1);
                for (int c = 0; c <= cols; ++c)
                    cv::line(
                        big, {c * cell, 0}, {c * cell, rows * cell},
                        tac::to_cv_bgr(tac::Semantic::CONTEXT), 1);

                // 每个单元格内打印矩阵原始数值
                constexpr double kValFontScale = 0.38;
                const cv::Scalar kValColor     = tac::to_cv_bgr(tac::Text::PRIMARY);
                for (int r = 0; r < rows; ++r) {
                    for (int c = 0; c < cols; ++c) {
                        const double v = mat(r, c);
                        char buf[32];
                        // 数值自适应格式化：极小/极大值科学计数，普通数值固定小数
                        if (std::abs(v) < 1e-2 || std::abs(v) >= 1e4)
                            std::snprintf(buf, sizeof(buf), "%.1e", v);
                        else
                            std::snprintf(buf, sizeof(buf), "%.4f", v);
                        int baseline = 0;
                        cv::Size ts  = cv::getTextSize(
                            buf, cv::FONT_HERSHEY_SIMPLEX, kValFontScale, 1, &baseline);
                        // 文字居中绘制在单元格
                        cv::putText(
                            big, buf,
                            cv::Point(
                                c * cell + cell / 2 - ts.width / 2,
                                r * cell + cell / 2 + ts.height / 2),
                            cv::FONT_HERSHEY_SIMPLEX, kValFontScale, kValColor, 1, cv::LINE_AA);
                    }
                }

                // 预留左侧、顶部边距用于行列标签
                const int kLeftMargin        = cell + 8;
                const int kTopMargin         = cell + 10;
                const cv::Scalar kLabelColor = tac::to_cv_bgr(tac::Text::SECONDARY);
                constexpr double kFontScale  = 0.45;

                // 创建带边距的画布
                cv::Mat canvas(
                    kTopMargin + big.rows, kLeftMargin + big.cols, CV_8UC3,
                    tac::to_cv_bgr(tac::Semantic::SURFACE));
                // 将热力图拷贝到画布主体区域
                big.copyTo(canvas(cv::Rect(kLeftMargin, kTopMargin, big.cols, big.rows)));

                // 左侧绘制行状态枚举标签（右对齐）
                for (int r = 0; r < rows && r < static_cast<int>(row_labels.size()); ++r) {
                    const std::string label(row_labels[r]);
                    int baseline = 0;
                    cv::Size ts =
                        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, kFontScale, 1, &baseline);
                    cv::putText(
                        canvas, label,
                        cv::Point(
                            kLeftMargin - ts.width - 4,
                            kTopMargin + r * cell + cell / 2 + ts.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, kFontScale, kLabelColor, 1, cv::LINE_AA);
                }

                // 顶部绘制列状态枚举标签（居中）
                for (int c = 0; c < cols && c < static_cast<int>(col_labels.size()); ++c) {
                    const std::string label(col_labels[c]);
                    int baseline = 0;
                    cv::Size ts =
                        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, kFontScale, 1, &baseline);
                    const int cx = kLeftMargin + c * cell + cell / 2;
                    cv::putText(
                        canvas, label, cv::Point(cx - ts.width / 2, kTopMargin - 8),
                        cv::FONT_HERSHEY_SIMPLEX, kFontScale, kLabelColor, 1, cv::LINE_AA);
                }

                return canvas;
            };

            // 热力图全局渲染常量：单元格像素、画布边距、标题高度
            constexpr int kCell     = 60;
            constexpr int kMargin   = 8;
            const cv::Scalar kBg    = tac::to_cv_bgr(tac::Semantic::SURFACE);
            const cv::Scalar kWhite = tac::to_cv_bgr(tac::Text::PRIMARY);
            constexpr int kTitleH   = 22;

            // 单目标热力图行结构体：目标名称 + 多张子热力图(P/K/Q/R)
            struct TargetRow {
                std::string label;
                struct MatPanel {
                    cv::Mat img;
                    std::string tag; // 矩阵标识 P/K/Q/R
                };
                std::vector<MatPanel> mats;
            };
            std::vector<TargetRow> target_rows;

            // 预生成状态枚举字符串标签数组（仅初始化一次static）
            const auto make_robo_labels = [] {
                std::vector<std::string_view> labels;
                labels.reserve(L3::STATE_MAX);
                for (uint8_t i = 0; i < L3::STATE_MAX; ++i)
                    labels.push_back(magic_enum::enum_name(static_cast<L3::RoboState>(i)));
                return labels;
            };
            const auto make_outpost_labels = [] {
                std::vector<std::string_view> labels;
                labels.reserve(L3::O_STATE_MAX);
                for (uint8_t i = 0; i < L3::O_STATE_MAX; ++i)
                    labels.push_back(magic_enum::enum_name(static_cast<L3::OutpostState>(i)));
                return labels;
            };
            const auto make_measure_labels = [] {
                std::vector<std::string_view> labels;
                labels.reserve(L3::MEASURE_MAX);
                for (uint8_t i = 0; i < L3::MEASURE_MAX; ++i)
                    labels.push_back(magic_enum::enum_name(static_cast<L3::Measure>(i)));
                return labels;
            };

            // 静态全局标签数组，程序生命周期只构造一次
            static const auto kRoboLabels    = make_robo_labels();
            static const auto kOutpostLabels = make_outpost_labels();
            static const auto kMeasLabels    = make_measure_labels();

            // 遍历所有有效跟踪目标，生成对应矩阵热力图面板
            for (const auto* output : active) {
                const std::string key = fmt::format(
                    "{} {}", magic_enum::enum_name(output->target_color),
                    magic_enum::enum_name(output->target_name));

                TargetRow row;
                row.label = key;

                // 区分机器人/前哨站两套状态维度标签
                if (output->is_robot()) {
                    const auto& s  = *output->robot_state();
                    const auto& rl = kRoboLabels;
                    // 协方差矩阵P热力图
                    if (s.P.size() > 0)
                        row.mats.push_back({render_heatmap(s.P, kCell, rl, rl), "P"});
                    // 卡尔曼增益矩阵K热力图
                    if (s.K.size() > 0)
                        row.mats.push_back({render_heatmap(s.K, kCell, rl, kMeasLabels), "K"});
                    // 过程噪声协方差Q
                    if (s.Q.size() > 0)
                        row.mats.push_back({render_heatmap(s.Q, kCell, rl, rl), "Q"});
                    // 观测噪声协方差R
                    if (s.R.size() > 0)
                        row.mats.push_back(
                            {render_heatmap(s.R, kCell, kMeasLabels, kMeasLabels), "R"});
                } else if (output->is_outpost()) {
                    const auto& s  = *output->outpost_state();
                    const auto& rl = kOutpostLabels;
                    if (s.P.size() > 0)
                        row.mats.push_back({render_heatmap(s.P, kCell, rl, rl), "P"});
                    if (s.K.size() > 0)
                        row.mats.push_back({render_heatmap(s.K, kCell, rl, kMeasLabels), "K"});
                    if (s.Q.size() > 0)
                        row.mats.push_back({render_heatmap(s.Q, kCell, rl, rl), "Q"});
                    if (s.R.size() > 0)
                        row.mats.push_back(
                            {render_heatmap(s.R, kCell, kMeasLabels, kMeasLabels), "R"});
                }

                // 当前目标存在有效矩阵则存入行容器
                if (!row.mats.empty())
                    target_rows.push_back(std::move(row));
            }

            // 无任何热力图面板直接退出
            if (target_rows.empty())
                return;

            // 计算拼接大图总宽高
            int max_w   = 0;
            int total_h = kMargin;
            for (const auto& tr : target_rows) {
                int row_w     = kMargin;
                int max_mat_h = 0;
                for (const auto& m : tr.mats) {
                    row_w += m.img.cols + kMargin;
                    max_mat_h = std::max(max_mat_h, m.img.rows);
                }
                max_w = std::max(max_w, row_w);
                total_h += 2 * kTitleH + max_mat_h + kMargin;
            }

            // 创建空白总画布
            cv::Mat composite(total_h, max_w, CV_8UC3, kBg);

            // 逐行拼接每个目标的热力图面板
            int y_off = kMargin;
            for (const auto& tr : target_rows) {
                // 绘制目标名称标题
                cv::putText(
                    composite, tr.label, cv::Point(kMargin, y_off + kTitleH - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.52, kWhite, 1, cv::LINE_AA);
                y_off += kTitleH;

                // 横向并排P/K/Q/R热力图
                int x_off = kMargin;
                int mat_h = 0;
                for (const auto& m : tr.mats) {
                    // 矩阵标识文字(P/K)
                    cv::putText(
                        composite, m.tag, cv::Point(x_off + 4, y_off + 14),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, tac::to_cv_bgr(tac::Text::SECONDARY), 1,
                        cv::LINE_AA);
                    // 拷贝热力图到大画布对应位置
                    if (!m.img.empty())
                        m.img.copyTo(
                            composite(cv::Rect(x_off, y_off + kTitleH, m.img.cols, m.img.rows)));
                    x_off += m.img.cols + kMargin;
                    mat_h = std::max(mat_h, m.img.rows);
                }
                y_off += kTitleH + mat_h + kMargin;
            }

            // 将完整画布编码为PNG二进制内存数据
            std::vector<uint8_t> compressed;
            if (!cv::imencode(".png", composite, compressed))
                return;

            // 填充EKF热力图消息结构体下发Foxglove
            EkfHeatmapMessage msg;
            msg.payload.timestamp = timestamp_from_ns((*outputs)[0].timestamp_ns);
            msg.payload.frame_id  = "ekf_heatmap";
            msg.payload.format    = "png";
            // 二进制字节流转换存储
            msg.payload.data      = std::vector<std::byte>(
                reinterpret_cast<const std::byte*>(compressed.data()),
                reinterpret_cast<const std::byte*>(compressed.data() + compressed.size()));
            // 消息入队异步发送
            (*server)->enqueue_message(std::move(msg));
        });

    // =========================================================================
    // 系统5：foxglove_l3_ldm_tracker_json LDM跟踪完整状态JSON结构化数据下发
    // 功能：序列化LDM全部跟踪参数、位姿、速度、预测坐标为JSON，前端可查看完整数值调试
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l3_ldm_tracker_json",
        [](
            talos::spmc<::fcs::L3::ldm::LdmState> ldm_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server
        ) {
            // 校验服务与输入通道就绪
            if (!detail::foxglove_ready(*server, ldm_in)) {
                return;
            }

            const auto state = ldm_in.read();
            // 无跟踪数据直接返回
            if (!state) {
                return;
            }

            // JSON根对象
            nlohmann::json j;
            // 原始纳秒时间戳
            j["timestamp_ns"]                  = state->timestamp_ns;
            // 上一帧观测时间戳
            j["last_observation_timestamp_ns"] = state->last_observation_timestamp_ns;
            // 拆分sec/nsec标准时间格式
            j["timestamp"]                     = nlohmann::json::object();
            j["timestamp"]["sec"]              = state->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"]             = state->timestamp_ns % 1000000000L;
            // 跟踪状态枚举字符串
            j["status"]                        = std::string(magic_enum::enum_name(state->status));
            // 跟踪精度标记
            j["accurate"]                      = state->accurate;

            // odom坐标系中心三维坐标
            j["position"] = {state->position().x(), state->position().y(), state->position().z()};

            // 旋转矩阵行优先平铺数组下发
            const auto& R = state->rotation();
            j["rotation"] = nlohmann::json::array();
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    j["rotation"].push_back(R(row, col));
                }
            }

            // 机体坐标系速度
            j["velocity_body"] = {
                state->velocity_body().x(), state->velocity_body().y(), state->velocity_body().z()};

            // 世界odom坐标系速度
            j["velocity_world"] = {
                state->velocity_world().x(), state->velocity_world().y(),
                state->velocity_world().z()};

            // 滤波预测未来时刻坐标与预测时长
            j["predicted_position_odom"] = {
                state->predicted_position_odom.x(), state->predicted_position_odom.y(),
                state->predicted_position_odom.z()};
            j["predicted_future_ns"] = state->predicted_future_ns;

            // 通用JSON消息发送工具，下发LdmStateMessage结构化话题
            detail::publish_json_message<LdmStateMessage>(*server, j);
        });
}

} // namespace fcs::visualization::foxglove::systems