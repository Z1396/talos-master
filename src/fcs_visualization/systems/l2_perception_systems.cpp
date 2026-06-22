// L3跟踪器工具函数（协方差、坐标转换）
#include "L3_estimation/tracker/util.hpp"
// Foxglove可视化系统基类
#include "base.hpp"
// Foxglove标准消息结构：SceneEntity、颜色、点、线、立方体等
#include "foxglove_types.hpp"
// 3D场景实体构造工具封装（EntityBuilder）
#include "scene_builder.hpp"

// LDM八边形检测配置、几何尺寸定义
#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/ldm/ldm_geometry.hpp"
// LDM检测、位姿测量数据结构
#include "L2_perception/ldm/types.hpp"
// 全局消息话题常量
#include "core/channel_topics.hpp"
// 调度器通用基础类型
#include "core/types.hpp"
// 图像帧基础结构
#include "frame.hpp"

// Eigen矩阵、四元数、位姿变换
#include <Eigen/Geometry>
// 数学常量、数值判断
#include <cmath>
// 枚举字符串反射（magic_enum）
#include <magic_enum.hpp>
// JSON序列化库
#include <nlohmann/json.hpp>
// 调度器工具辅助函数
#include <system_helpers.hpp>

namespace fcs::visualization::foxglove::systems {

// 内部私有工具命名空间，仅本文件可见
namespace {

// 3D场景实体存活时长：5ms，超时自动消失，避免旧帧残留在界面
constexpr uint64_t kLdmSceneLifetimeNs = 5'000'000;

/**
 * @brief Eigen4x4矩阵转为JSON二维数组
 * @param mat 4×4变换/协方差矩阵
 * @return [[r0c0,r0c1...],[r1c0...],...] JSON数组
 */
[[nodiscard]] nlohmann::json matrix4_to_json(const Eigen::Matrix4d& mat) {
    nlohmann::json rows = nlohmann::json::array();
    for (int r = 0; r < 4; ++r) {
        nlohmann::json row = nlohmann::json::array();
        for (int c = 0; c < 4; ++c) {
            row.push_back(mat(r, c));
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

/**
 * @brief 提取4×4矩阵对角线元素转为一维JSON数组
 * @param mat 输入4阶方阵
 * @return [m00,m11,m22,m33]
 */
[[nodiscard]] nlohmann::json matrix4_diag_to_json(const Eigen::Matrix4d& mat) {
    nlohmann::json diag = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) {
        diag.push_back(mat(i, i));
    }
    return diag;
}

/**
 * @brief 格式化矩阵单行字符串，用于SceneEntity附加文本元数据
 * @param mat 4×4矩阵
 * @param row 目标行号0~3
 * @return 科学计数法格式化字符串 "[1.23e-3, ...]"
 */
[[nodiscard]] std::string matrix4_row_string(const Eigen::Matrix4d& mat, int row) {
    return fmt::format(
        "[{:.3e}, {:.3e}, {:.3e}, {:.3e}]", mat(row, 0), mat(row, 1), mat(row, 2), mat(row, 3));
}

/**
 * @brief 根据LDM深度质量等级返回对应3D实体颜色
 * Stable(稳定深度)=亮色，None无深度=灰色
 */
[[nodiscard]] ::foxglove::schemas::Color
    ldm_scene_color(const fcs::L2::ldm::LdmMeasurement& measurement) noexcept {
    switch (measurement.depth_quality) {
    case fcs::L2::ldm::LdmDepthQuality::Stable: return tac::L2::LDM_STABLE;
    case fcs::L2::ldm::LdmDepthQuality::Constrained: return tac::L2::LDM_CONSTRAINED;
    case fcs::L2::ldm::LdmDepthQuality::BearingOnly: return tac::L2::LDM_BEARING_ONLY;
    case fcs::L2::ldm::LdmDepthQuality::None: return tac::L2::LDM_NONE;
    }
    return tac::L2::LDM_NONE;
}

/**
 * @brief 生成LDM 3D实体顶部文本标签：配对数/总光斑数+深度质量
 * @return 示例 "LDM 4/8 Stable"
 */
[[nodiscard]] std::string ldm_label(const fcs::L2::ldm::LdmMeasurement& measurement) {
    return fmt::format(
        "LDM {}/{} {}", measurement.selected_pair_count, measurement.pair_count_total,
        measurement.depth_quality);
}

// ============================================================================
// LDM八边形立面可见性判断（基于匹配观测面，非相机几何剔除）
// ============================================================================
/**
 * @brief 从最优网格解算结果提取观测立面，生成8个面可见标记
 * 核心逻辑：仅detector实际匹配到的立面标记为可见，其余背面虚线绘制；
 * 区别于L1图像层：L1是几何法向量背面剔除，L2 3D场景使用**检测匹配状态**，更贴合算法真实观测信息
 * @param measurement LDM完整位姿测量结果
 * @return array[8] 每个立面是否被观测到
 */
[[nodiscard]] std::array<bool, 8>
    ldm_volume_assigned_faces(const fcs::L2::ldm::LdmMeasurement& measurement) noexcept {
    std::array<bool, 8> visible{};
    // 存在有效最优网格候选解
    if (measurement.selected_candidate_idx.has_value() && *measurement.selected_candidate_idx >= 0
        && static_cast<size_t>(*measurement.selected_candidate_idx)
               < measurement.mesh_candidates.size()) {
        const auto& selected =
            measurement.mesh_candidates[static_cast<size_t>(*measurement.selected_candidate_idx)];
        // 遍历该解匹配到的所有立面编号，标记为可见
        for (const int face_idx : selected.octagon_face_indices) {
            if (face_idx >= 0 && face_idx < 8) {
                visible[static_cast<size_t>(face_idx)] = true;
            }
        }
    }
    return visible;
}

// ============================================================================
// Foxglove 3D虚线模拟（Foxglove Scene不原生支持虚线，分段短实线拼接）
// ============================================================================
/**
 * @brief 向实体构建器添加一段3D虚线，拆分为多段短实线line_strip
 * @param builder 3D场景实体构造器
 * @param p1 起点3D坐标
 * @param p2 终点3D坐标
 * @param color 虚线颜色
 * @param thickness 线条粗细(m)
 * @param dash_len 每段实线长度(m)
 * @param gap_len 间隔空白长度(m)
 */
inline void add_ldm_dashed_edge(
    viz::EntityBuilder& builder, const ::foxglove::schemas::Point3& p1,
    const ::foxglove::schemas::Point3& p2, const ::foxglove::schemas::Color& color,
    double thickness, double dash_len, double gap_len) {
    const double dx  = p2.x - p1.x;
    const double dy  = p2.y - p1.y;
    const double dz  = p2.z - p1.z;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    // 线段非法/长度过短直接跳过
    if (!std::isfinite(len) || len <= 1e-6) {
        return;
    }

    const double step  = dash_len + gap_len;
    const int num_segs = std::max(1, static_cast<int>(len / step));

    // 循环分段绘制实线片段，中间留空实现虚线效果
    for (int s = 0; s < num_segs; ++s) {
        const double t0 = static_cast<double>(s) * step / len;
        const double t1 = std::min(1.0, (static_cast<double>(s) * step + dash_len) / len);
        if (t0 >= 1.0) {
            break;
        }
        builder.line_strip(
            {viz::make_point3(p1.x + dx * t0, p1.y + dy * t0, p1.z + dz * t0),
             viz::make_point3(p1.x + dx * t1, p1.y + dy * t1, p1.z + dz * t1)},
            color, thickness);
    }
}

// ============================================================================
// 绘制完整LDM八边形3D外轮廓（上层八角、下层八角、垂直棱线）
// 根据立面可见性自动切换实线/虚线
// ============================================================================
/**
 * @brief 批量添加八边形全部12条棱线到3D实体
 * 规则：
 * 1. 上层环、下层环侧边：相邻后立面可见则实线，否则虚线
 * 2. 垂直棱线：左右任一立面可见则实线，否则虚线
 * @param builder 场景实体构建器
 * @param config LDM几何尺寸配置
 * @param visible_faces 8个立面可见标记数组
 * @param solid_color 可见实线主色
 * @param dashed_color 不可见虚线灰色
 * @param thickness 线条粗细(m)
 */
inline void add_ldm_volume_edges(
    viz::EntityBuilder& builder, const fcs::L2::ldm::LdmDetectorConfig& config,
    const std::array<bool, 8>& visible_faces, const ::foxglove::schemas::Color& solid_color,
    const ::foxglove::schemas::Color& dashed_color, double thickness) {
    // 获取八边形16个3D顶点：上层8点、下层8点（模型坐标系）
    const auto outline = fcs::L2::ldm::volume_outline_points(config.geometry);

    // OpenCV Point3f 转换为Foxglove 3D点结构
    std::array<::foxglove::schemas::Point3, 16> pts;
    for (size_t i = 0; i < 16; ++i) {
        pts[i] = viz::make_point3(
            static_cast<double>(outline[i].x), static_cast<double>(outline[i].y),
            static_cast<double>(outline[i].z));
    }

    // 虚线分段参数：实线段2cm，空白间隔1.2cm
    constexpr double kDashLen = 0.020;
    constexpr double kGapLen  = 0.012;

    // 遍历8个立面，绘制所有棱线
    for (size_t i = 0; i < 8; ++i) {
        const size_t next = (i + 1) % 8;

        // 上层环边 i → next：可见性由next立面决定
        if (visible_faces[next]) {
            builder.line_strip({pts[i], pts[next]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i], pts[next], dashed_color, thickness, kDashLen, kGapLen);
        }

        // 下层环边 i+8 → next+8，可见性同上
        if (visible_faces[next]) {
            builder.line_strip({pts[i + 8], pts[next + 8]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i + 8], pts[next + 8], dashed_color, thickness, kDashLen, kGapLen);
        }

        // 垂直棱线 i ↔ i+8：左右相邻立面任一可见即为实线
        const bool vert_visible = visible_faces[i] || visible_faces[next];
        if (vert_visible) {
            builder.line_strip({pts[i], pts[i + 8]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i], pts[i + 8], dashed_color, thickness, kDashLen, kGapLen);
        }
    }
}

// ============================================================================
// 3D实体工厂函数：生成不同坐标系下LDM八边形SceneEntity
// ============================================================================
/**
 * @brief 生成odom里程计坐标系下完整LDM八边形3D实体
 * 包含：轮廓线、中心标记球、顶部文本标签、各类算法元数据
 * @param measurement LDM位姿测量结果（含odom系变换）
 * @param config LDM几何尺寸配置
 * @return Foxglove标准SceneEntity 3D场景对象
 */
[[nodiscard]] ::foxglove::schemas::SceneEntity make_ldm_volume_entity_odom(
    const fcs::L2::ldm::LdmMeasurement& measurement,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
    // 根据深度质量获取主色
    const auto color = ldm_scene_color(measurement);
    // odom世界坐标系位姿
    const auto& pose = *measurement.transform_odom;

    // 获取本帧观测立面可见标记
    const auto visible_faces = ldm_volume_assigned_faces(measurement);

    // 创建实体构建器，绑定odom坐标系，分类l2，实体标识ldm
    auto builder = viz::EntityBuilder::create<fast_tf::odom>("l2", "ldm");
    builder.timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        .position(pose.translation())
        .orientation(pose.quaternion())
        .color(color);

    // 虚线使用主色45%亮度变暗
    const auto dimmed_color = tac::scaled_rgb(color, 0.45);

    // 绘制八边形全部轮廓棱线（自动虚实切换）
    add_ldm_volume_edges(builder, config, visible_faces, color, dimmed_color, 0.0025);

    // 中心小球标记 + 顶部文字标签
    builder.size(0.025)
        .sphere()
        .text_with_offset(
            ldm_label(measurement), 0.0, 0.0, config.geometry.volume_height_m * 0.8,
            tac::L2::LABEL_FONT_SIZE)
        // 附加元数据，鼠标悬浮Foxglove可查看
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count));
    return builder.build();
}

/**
 * @brief 相机光学坐标系LDM八边形3D实体
 * 用于观察相机视角下模型位置，与odom实体分开发布，两套坐标系互不干扰
 */
[[nodiscard]] ::foxglove::schemas::SceneEntity make_ldm_volume_entity_camera(
    const fcs::L2::ldm::LdmMeasurement& measurement,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
    const auto color = ldm_scene_color(measurement);
    // 相机坐标系位姿
    const auto& pose = *measurement.transform_cam;

    const auto visible_faces = ldm_volume_assigned_faces(measurement);
    // 绑定camera_optical光学坐标系
    auto builder             = viz::EntityBuilder::create<fast_tf::camera_optical>("l2", "ldm_cam");
    builder.timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        .position(pose.translation())
        .orientation(pose.quaternion())
        .color(color);

    const auto dimmed_color = tac::scaled_rgb(color, 0.45);
    add_ldm_volume_edges(builder, config, visible_faces, color, dimmed_color, 0.0025);

    builder.size(0.0025)
        .sphere()
        .text_with_offset(
            ldm_label(measurement), 0.0, 0.0, config.geometry.volume_height_m * 0.8,
            tac::L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count));
    return builder.build();
}

/**
 * @brief 仅存在方位角无深度时：绘制相机原点指向目标的视线射线
 * 深度质量=BearingOnly/None时使用，仅展示观测方向，无立体八边形
 */
[[nodiscard]] ::foxglove::schemas::SceneEntity
    make_ldm_bearing_entity(const fcs::L2::ldm::LdmMeasurement& measurement) {
    const auto color          = ldm_scene_color(measurement);
    // 归一化视线方向，延长至2米
    const Eigen::Vector3d end = measurement.bearing_cam.normalized() * 2.0;

    return viz::EntityBuilder::create<fast_tf::camera_optical>("l2", "ldm_bearing")
        .timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        // 原点→2米远处线段
        .line_strip({viz::make_point3(0.0, 0.0, 0.0), viz::make_point3(end)}, color, 0.0025)
        .position(end)
        .color(color)
        // 射线末端小球标记
        .size(0.025)
        .sphere()
        .text_with_offset(ldm_label(measurement), 0.0, 0.0, 0.08, tac::L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count))
        .build();
}

} // 内部匿名namespace结束

/**
 * @brief 批量注册L2感知层所有可视化调度系统
 * 包含6套独立pool_compute线程池系统：
 * 1. foxglove_l2_detection_pub：装甲检测框JSON
 * 2. foxglove_l2_measurement_scene：装甲3D立方体Scene
 * 3. foxglove_l2_ldm_scene：LDM八边形3D立体实体/视线射线
 * 4. foxglove_l2_ldm_detection_json：LDM原始光斑检测JSON
 * 5. foxglove_l2_ldm_measurement_json：LDM完整位姿测量JSON
 * 6. foxglove_l2_measurement_pub：装甲PnP位姿+协方差JSON
 */
void register_l2_perception_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // 系统1：foxglove_l2_detection_pub 装甲原始检测框JSON发布
    // 输入：装甲检测批通道，输出DebugArmorsMessage结构化JSON
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_detection_pub",
        [](talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // Foxglove服务未就绪/通道无数据直接退出
            if (!foxglove_ready(*server, det_in)) {
                return;
            }

            auto batch = det_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json armors_json;
            // 填充时间戳（秒+纳秒标准ROS格式）
            armors_json["timestamp_ns"]      = batch->timestamp_ns;
            armors_json["timestamp"]         = nlohmann::json::object();
            armors_json["timestamp"]["sec"]  = batch->timestamp_ns / 1000000000L;
            armors_json["timestamp"]["nsec"] = batch->timestamp_ns % 1000000000L;
            armors_json["armors"]            = nlohmann::json::array();

            // 遍历所有装甲检测结果序列化
            for (size_t i = 0; i < batch->detections.size(); ++i) {
                const auto& det = batch->detections[i];
                nlohmann::json obj;
                // 图像中心像素坐标
                obj["center"]["x"] = det.center().x;
                obj["center"]["y"] = det.center().y;
                // 四个角点像素数组
                obj["corners"] = nlohmann::json::array();
                for (const auto& pt : det.corners) {
                    obj["corners"].push_back({pt.x, pt.y});
                }
                obj["confidence"] = det.confidence;
                // 枚举转字符串
                obj["color"]      = magic_enum::enum_name(det.color);
                obj["type"]       = magic_enum::enum_name(det.type);
                obj["name"]       = magic_enum::enum_name(det.name);
                armors_json["armors"].push_back(obj);
            }

            // 封装JSON通过Foxglove发送
            detail::publish_json_message<DebugArmorsMessage>(*server, armors_json);
        });

    // =========================================================================
    // 系统2：foxglove_l2_measurement_scene 装甲3D立方体场景发布
    // 输入：装甲PnP位姿批数据，生成odom系立方体SceneEntity
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_measurement_scene",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!foxglove_ready(*server, meas_in))
                return;

            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;

            // 逐个生成装甲3D立方体实体
            for (size_t i = 0; i < batch->measurements.size(); ++i) {
                const auto& m = batch->measurements[i];
                const auto t  = m.transform.translation();
                const auto q  = m.transform.quaternion();

                // 创建odom坐标系立方体实体
                auto builder =
                    viz::EntityBuilder::create<fast_tf::odom>("l2", fmt::format("armor_{}", i));
                builder.position(t.x(), t.y(), t.z())
                    .orientation(q.x(), q.y(), q.z(), q.w())
                    .color(tac::L2::MEASUREMENT_GHOST)
                    // 立方体尺寸：厚度、高度、宽度（装甲物理尺寸）
                    .cube(
                        tac::Geometry::ARMOR_THICKNESS, tac::Geometry::ARMOR_HEIGHT_SMALL,
                        tac::Geometry::ARMOR_WIDTH)
                    // 立方体下方文本标签：装甲名称+类型
                    .text_with_offset(
                        fmt::format(
                            "{} {}", magic_enum::enum_name(m.name), magic_enum::enum_name(m.type)),
                        0, -tac::Text::SIZE_MEDIUM, 0, tac::L2::LABEL_FONT_SIZE)
                    // 附加PnP解算元数据：置信度、协方差、条件数
                    .metadata("confidence", fmt::format("{:.3f}", m.confidence))
                    .metadata("color", std::string(magic_enum::enum_name(m.color)))
                    .metadata("pnp_cov_order", "bearing_yaw,bearing_pitch,log_distance,armor_yaw")
                    .metadata("pnp_condition", fmt::format("{:.3e}", m.pnp_condition_number))
                    .metadata("pnp_cov[0]", matrix4_row_string(m.pnp_cov_ypdr, 0))
                    .metadata("pnp_cov[1]", matrix4_row_string(m.pnp_cov_ypdr, 1))
                    .metadata("pnp_cov[2]", matrix4_row_string(m.pnp_cov_ypdr, 2))
                    .metadata("pnp_cov[3]", matrix4_row_string(m.pnp_cov_ypdr, 3))
                    .timestamp(m.timestamp_ns);

                auto entity = builder.build();
                entities.push_back(std::move(entity));
            }

            // 批量发布3D场景消息，无实体则跳过
            publish_scene_if_nonempty<SceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 系统3：foxglove_l2_ldm_scene LDM八边形3D可视化
    // 分支逻辑：
    // 1. 存在odom位姿 → 发布odom立体八边形
    // 2. 仅相机位姿 → 发布相机系立体八边形
    // 3. 无深度仅有方位 → 发布视线射线
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_ldm_scene",
        [](talos::spmc<fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> ldm_meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server,
           talos::res<fcs::L2::ldm::LdmDetectorConfig> ldm_config) {
            if (!foxglove_ready(*server, ldm_meas_in)) {
                return;
            }

            auto measurement = ldm_meas_in.read();
            if (!measurement) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;
            // 根据可用位姿选择实体类型
            if (measurement->transform_odom.has_value()) {
                entities.push_back(make_ldm_volume_entity_odom(*measurement, *ldm_config));
            } else if (measurement->transform_cam.has_value()) {
                entities.push_back(make_ldm_volume_entity_camera(*measurement, *ldm_config));
            } else {
                entities.push_back(make_ldm_bearing_entity(*measurement));
            }

            publish_scene_if_nonempty<SceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 系统4：foxglove_l2_ldm_detection_json LDM原始光斑检测JSON
    // 输出所有blob光斑、配对、网格候选、匹配立面索引原始数据
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_ldm_detection_json",
        [](talos::spmc<fcs::L2::ldm::LdmDetection, LdmDetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, det_in)) {
                return;
            }

            auto det = det_in.read();
            if (!det) {
                return;
            }

            nlohmann::json j;
            j["timestamp_ns"]      = det->timestamp_ns;
            j["frame_id"]          = det->frame_id;
            j["timestamp"]         = nlohmann::json::object();
            j["timestamp"]["sec"]  = det->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"] = det->timestamp_ns % 1000000000L;
            j["color"]             = std::string(magic_enum::enum_name(det->color));
            j["accurate"]          = det->accurate;
            // 图像ROI矩形
            j["rect"]              = nlohmann::json::object();
            j["rect"]["x"]         = det->rect.x;
            j["rect"]["y"]         = det->rect.y;
            j["rect"]["w"]         = det->rect.width;
            j["rect"]["h"]         = det->rect.height;

            // 图像中心像素
            if (det->center_image_px.has_value()) {
                j["center_image_px"]      = nlohmann::json::object();
                j["center_image_px"]["x"] = det->center_image_px->x;
                j["center_image_px"]["y"] = det->center_image_px->y;
            }

            // 光斑、配对、候选解数量
            j["blob_count"]      = det->blobs.size();
            j["pair_count"]      = det->pairs.size();
            j["candidate_count"] = det->mesh_candidates.size();

            // 序列化所有网格候选解（分数、RMSE、匹配立面）
            j["candidates"] = nlohmann::json::array();
            for (const auto& cand : det->mesh_candidates) {
                nlohmann::json cj;
                cj["cluster_id"]           = cand.cluster_id;
                cj["solved"]               = cand.solved;
                cj["depth_valid"]          = cand.depth_valid;
                cj["preliminary_score"]    = cand.preliminary_score;
                cj["reprojection_rmse_px"] = cand.reprojection_rmse_px;
                cj["score"]                = cand.score;
                cj["pair_count"]           = cand.pair_indices.size();
                cj["face_indices"]         = nlohmann::json::array();
                for (const int fi : cand.octagon_face_indices) {
                    cj["face_indices"].push_back(fi);
                }
                j["candidates"].push_back(cj);
            }

            // 最优候选解下标
            if (det->selected_candidate_idx.has_value()) {
                j["selected_candidate_idx"] = *det->selected_candidate_idx;
            }

            detail::publish_json_message<LdmDetectionMessage>(*server, j);
        });

    // =========================================================================
    // 系统5：foxglove_l2_ldm_measurement_json LDM完整位姿测量JSON
    // 输出相机系/odom系变换、距离、置信度、深度质量、视线向量
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_ldm_measurement_json",
        [](talos::spmc<fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, meas_in)) {
                return;
            }

            auto meas = meas_in.read();
            if (!meas) {
                return;
            }

            nlohmann::json j;
            j["timestamp_ns"]         = meas->timestamp_ns;
            j["frame_id"]             = meas->frame_id;
            j["timestamp"]            = nlohmann::json::object();
            j["timestamp"]["sec"]     = meas->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"]    = meas->timestamp_ns % 1000000000L;
            j["color"]                = std::string(magic_enum::enum_name(meas->color));
            j["accurate"]             = meas->accurate;
            j["pair_count_total"]     = meas->pair_count_total;
            j["selected_pair_count"]  = meas->selected_pair_count;
            j["center_image_px"]      = nlohmann::json::object();
            j["center_image_px"]["x"] = meas->center_image_px.x;
            j["center_image_px"]["y"] = meas->center_image_px.y;
            // 相机归一化视线向量
            j["bearing_cam"]          = {
                meas->bearing_cam.x(), meas->bearing_cam.y(), meas->bearing_cam.z()};
            j["depth_quality"]   = std::string(magic_enum::enum_name(meas->depth_quality));
            j["confidence"]      = meas->confidence;
            j["candidate_count"] = meas->mesh_candidates.size();

            if (meas->selected_candidate_idx.has_value()) {
                j["selected_candidate_idx"] = *meas->selected_candidate_idx;
            }

            // 相机坐标系位姿、直线距离
            if (meas->transform_cam.has_value()) {
                const auto& t                     = meas->transform_cam->translation();
                const auto& q                     = meas->transform_cam->quaternion();
                j["transform_cam"]                = nlohmann::json::object();
                j["transform_cam"]["dist"]        = t.norm();
                j["transform_cam"]["position"]    = {t.x(), t.y(), t.z()};
                j["transform_cam"]["orientation"] = {q.x(), q.y(), q.z(), q.w()};
            }

            // 里程计世界坐标系位姿、直线距离
            if (meas->transform_odom.has_value()) {
                const auto& t                      = meas->transform_odom->translation();
                const auto& q                      = meas->transform_odom->quaternion();
                j["transform_odom"]                = nlohmann::json::object();
                j["transform_odom"]["dist"]        = t.norm();
                j["transform_odom"]["position"]    = {t.x(), t.y(), t.z()};
                j["transform_odom"]["orientation"] = {q.x(), q.y(), q.z(), q.w()};
            }

            detail::publish_json_message<LdmMeasurementMessage>(*server, j);
        });

    // =========================================================================
    // 系统6：foxglove_l2_measurement_pub 装甲PnP测量完整JSON（协方差、欧拉角、角度偏差）
    // 包含4×4 PnP协方差矩阵、RPY欧拉角、odom与相机间角度差
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_measurement_pub",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<fast_tf::CoordinateSystem> tf_system,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, meas_in)) {
                return;
            }

            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json j;
            j["timestamp_ns"]      = batch->timestamp_ns;
            j["timestamp"]         = nlohmann::json::object();
            j["timestamp"]["sec"]  = batch->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"] = batch->timestamp_ns % 1000000000L;
            j["measurements"]      = nlohmann::json::array();

            // 查询当前帧odom→相机光学坐标变换
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, batch->timestamp_ns);

            for (const auto& m : batch->measurements) {
                const auto t   = m.transform.translation();
                const auto q   = m.transform.quaternion();
                // 分解RPY欧拉角
                auto [r, p, y] = m.transform.euler_rot().rpy();

                nlohmann::json mj;
                mj["timestamp_ns"]          = m.timestamp_ns;
                mj["timestamp"]             = nlohmann::json::object();
                mj["timestamp"]["sec"]      = m.timestamp_ns / 1000000000L;
                mj["timestamp"]["nsec"]     = m.timestamp_ns % 1000000000L;
                mj["name"]                  = std::string(magic_enum::enum_name(m.name));
                mj["type"]                  = std::string(magic_enum::enum_name(m.type));
                mj["color"]                 = std::string(magic_enum::enum_name(m.color));
                mj["confidence"]            = m.confidence;
                mj["position"]              = {t.x(), t.y(), t.z()};
                mj["orientation"]           = {q.x(), q.y(), q.z(), q.w()};
                mj["euler"]["roll"]         = r;
                mj["euler"]["pitch"]        = p;
                mj["euler"]["yaw"]          = y;
                mj["distance"]              = t.norm();
                // PnP协方差维度说明：方位偏航、俯仰、对数距离、装甲偏航
                mj["pnp_geometry"]["order"] = {
                    "bearing_yaw", "bearing_pitch", "log_distance", "armor_yaw"};
                mj["pnp_geometry"]["cov_ypdr"]  = matrix4_to_json(m.pnp_cov_ypdr);
                mj["pnp_geometry"]["diag"]      = matrix4_diag_to_json(m.pnp_cov_ypdr);
                mj["pnp_geometry"]["condition"] = m.pnp_condition_number;
                mj["pnp_geometry"]["note"] =
                    "normalized-plane PnP covariance propagated to measurement coordinates";

                // TF树查询有效时，计算相机观测yaw与世界yaw角度偏差（度）
                if (tf_lookup) {
                    auto target_in_ref = (m.transform);

                    auto target_pos_yaw = fcs::L3::xyz2ypd(target_in_ref.translation())[0];
                    auto [roll, pitch, target_yaw] = target_in_ref.euler_rot().rpy();

                    mj["delta_angle"] =
                        L3::shortest_rad(target_pos_yaw, target_yaw) * 180.0 / std::numbers::pi;
                }

                j["measurements"].push_back(mj);
            }

            detail::publish_json_message<MeasurementMessage>(*server, j);
        });
}

} // namespace fcs::visualization::foxglove::systems