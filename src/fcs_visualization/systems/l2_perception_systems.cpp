// L3跟踪器工具函数（协方差、坐标转换）
#include "L3_estimation/tracker/util.hpp"
// Foxglove可视化系统基类，foxglove_ready、detail命名空间辅助函数定义
#include "base.hpp"
// 调度器完整定义（Scheduler、pool_compute、spmc 等）
// pool_compute：线程池执行器；spmc：单生产者多消费者无锁通道，框架内部数据总线
#include "scheduler/scheduler.hpp"
// Foxglove标准消息结构：SceneEntity、颜色、点、线、立方体等3D场景实体数据结构
#include "foxglove_types.hpp"
// 3D场景实体构造工具封装（EntityBuilder），链式API快速构建SceneEntity对象
#include "scene_builder.hpp"

// LDM八边形检测配置、几何尺寸定义，LDM物理参数常量
#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/ldm/ldm_geometry.hpp"
// LDM检测、位姿测量数据结构：LdmDetection、LdmMeasurement结构体定义
#include "L2_perception/ldm/types.hpp"
// 全局消息话题常量，各个spmc通道的Topic标签类型
#include "core/channel_topics.hpp"
// 调度器通用基础类型，时间戳、单位别名等基础类型别名
#include "core/types.hpp"
// 图像帧基础结构，帧时间戳、帧id定义
#include "frame.hpp"

// Eigen矩阵、四元数、位姿变换，矩阵运算、平移旋转四元数
#include <Eigen/Geometry>
// 数学常量、数值判断，std::isfinite 数值有效性判断
#include <cmath>
// 枚举字符串反射（magic_enum），enum转字符串，不用手写字符串映射
#include <magic_enum.hpp>
// JSON序列化库，nlohmann json，构造调试输出JSON消息
#include <nlohmann/json.hpp>
// 调度器工具辅助函数
#include <system_helpers.hpp>

namespace fcs::visualization::foxglove::systems {

// 内部私有工具命名空间，仅本翻译单元(.cpp文件)内部可见，对外不暴露接口
namespace {

// 3D场景实体存活时长：5ms，超时自动消失，避免旧帧残留在Foxglove界面
// 单位纳秒，5'000'000 ns = 5ms
constexpr uint64_t kLdmSceneLifetimeNs = 5'000'000;

/**
 * @brief Eigen4x4矩阵转为JSON二维数组
 * @param mat 4×4变换/协方差矩阵
 * @return [[r0c0,r0c1...],[r1c0...],...] JSON二维数组
 * @note [[nodiscard]] 禁止忽略返回值，防止忘记接收json结果
 */
[[nodiscard]] nlohmann::json matrix4_to_json(const Eigen::Matrix4d& mat) {
    // 创建外层json数组，用来存放每一行
    nlohmann::json rows = nlohmann::json::array();
    // 遍历矩阵行 r=0~3
    for (int r = 0; r < 4; ++r) {
        // 创建单行json数组
        nlohmann::json row = nlohmann::json::array();
        // 遍历矩阵列 c=0~3
        for (int c = 0; c < 4; ++c) {
            // 将矩阵元素填入单行数组
            row.push_back(mat(r, c));
        }
        // 将单行压入外层行数组
        rows.push_back(std::move(row));
    }
    return rows;
}

/**
 * @brief 提取4×4矩阵对角线元素转为一维JSON数组
 * @param mat 输入4阶方阵
 * @return [m00,m11,m22,m33]，对角线元素，协方差对角元代表各个维度方差
 */
[[nodiscard]] nlohmann::json matrix4_diag_to_json(const Eigen::Matrix4d& mat) {
    nlohmann::json diag = nlohmann::json::array();
    // 只取 i == i 对角线位置
    for (int i = 0; i < 4; ++i) {
        diag.push_back(mat(i, i));
    }
    return diag;
}

/**
 * @brief 格式化矩阵单行字符串，用于SceneEntity附加文本元数据，鼠标悬浮Foxglove显示
 * @param mat 4×4矩阵
 * @param row 目标行号0~3
 * @return 科学计数法格式化字符串 "[1.23e-3, ...]"
 */
[[nodiscard]] std::string matrix4_row_string(const Eigen::Matrix4d& mat, int row) {
    // fmt格式化，4个元素使用科学计数保留3位有效数字
    return fmt::format(
        "[{:.3e}, {:.3e}, {:.3e}, {:.3e}]", mat(row, 0), mat(row, 1), mat(row, 2), mat(row, 3));
}

/**
 * @brief 根据LDM深度质量等级返回对应3D实体颜色
 * Stable(稳定深度)=亮色，None无深度=灰色
 * @param measurement LDM测量结果结构体
 * @return foxglove color RGBA0‑1
 * @note noexcept 函数不会抛出异常
 */
[[nodiscard]] ::foxglove::schemas::Color
    ldm_scene_color(const fcs::L2::ldm::LdmMeasurement& measurement) noexcept {
    // 根据深度质量枚举选择配色，配色来源tactical.hpp L2命名空间
    switch (measurement.depth_quality) {
    case fcs::L2::ldm::LdmDepthQuality::Stable: return tac::L2::LDM_STABLE;
    case fcs::L2::ldm::LdmDepthQuality::Constrained: return tac::L2::LDM_CONSTRAINED;
    case fcs::L2::ldm::LdmDepthQuality::BearingOnly: return tac::L2::LDM_BEARING_ONLY;
    case fcs::L2::ldm::LdmDepthQuality::None: return tac::L2::LDM_NONE;
    }
    // switch兜底返回无深度灰色
    return tac::L2::LDM_NONE;
}

/**
 * @brief 生成LDM 3D实体顶部文本标签：配对数/总光斑数+深度质量
 * @param measurement LDM测量结果
 * @return 示例 "LDM 4/8 Stable"
 */
[[nodiscard]] std::string ldm_label(const fcs::L2::ldm::LdmMeasurement& measurement) {
    // fmt拼接字符串：选中配对数 / 总配对数 + 深度质量枚举名字
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
 * @return array[8] 每个立面是否被观测到，下标0‑7对应八边形8个立面
 */
[[nodiscard]] std::array<bool, 8>
    ldm_volume_assigned_faces(const fcs::L2::ldm::LdmMeasurement& measurement) noexcept {
    // 初始化大小为8的bool数组，全部初始化为false(不可见)
    std::array<bool, 8> visible{};
    // 判断：存在有效最优网格候选解
    // has_value()：std::optional是否有有效值；*取值；索引合法；索引不超过候选数组大小
    if (measurement.selected_candidate_idx.has_value() && *measurement.selected_candidate_idx >= 0
        && static_cast<size_t>(*measurement.selected_candidate_idx)
               < measurement.mesh_candidates.size()) {
        // 获取当前选中最优网格候选解
        const auto& selected =
            measurement.mesh_candidates[static_cast<size_t>(*measurement.selected_candidate_idx)];
        // 遍历该解匹配到的所有立面编号
        for (const int face_idx : selected.octagon_face_indices) {
            // 边界校验，下标合法0‑7
            if (face_idx >= 0 && face_idx < 8) {
                // 将该立面标记为true，代表算法匹配观测到这个面
                visible[static_cast<size_t>(face_idx)] = true;
            }
        }
    }
    // 返回8个立面可见标记数组
    return visible;
}

// ============================================================================
// Foxglove 3D虚线模拟（Foxglove Scene不原生支持虚线，分段短实线拼接）
// ============================================================================
/**
 * @brief 向实体构建器添加一段3D虚线，拆分为多段短实线line_strip模拟虚线效果
 * @param builder 3D场景实体构造器，EntityBuilder链式对象
 * @param p1 起点3D坐标 foxglove Point3
 * @param p2 终点3D坐标 foxglove Point3
 * @param color 虚线颜色
 * @param thickness 线条粗细(m)
 * @param dash_len 每段实线长度(m)
 * @param gap_len 间隔空白长度(m)
 */
inline void add_ldm_dashed_edge(
    viz::EntityBuilder& builder, const ::foxglove::schemas::Point3& p1,
    const ::foxglove::schemas::Point3& p2, const ::foxglove::schemas::Color& color,
    double thickness, double dash_len, double gap_len) {
    // 计算两点之间的三轴差值 dx dy dz
    const double dx  = p2.x - p1.x;
    const double dy  = p2.y - p1.y;
    const double dz  = p2.z - p1.z;
    // 欧氏距离，线段总长度
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    // std::isfinite 判断数值不是nan/infinite；长度过小直接返回，不绘制
    if (!std::isfinite(len) || len <= 1e-6) {
        return;
    }

    // 一个周期 = 实线段长度 + 空白间隔长度
    const double step  = dash_len + gap_len;
    // 计算总共有多少个虚实周期，至少1段
    const int num_segs = std::max(1, static_cast<int>(len / step));

    // 循环分段绘制实线片段，中间留空实现虚线效果
    for (int s = 0; s < num_segs; ++s) {
        // t0：当前片段起点归一化系数 [0,1]
        const double t0 = static_cast<double>(s) * step / len;
        // t1：当前片段终点归一化系数，不超过1.0，防止超出线段终点
        const double t1 = std::min(1.0, (static_cast<double>(s) * step + dash_len) / len);
        // t0>=1，已经走完线段，直接break退出循环
        if (t0 >= 1.0) {
            break;
        }
        // 插值计算起点终点三维坐标，调用builder绘制一小段实线
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
    // 获取八边形16个3D顶点：上层8点索引0‑7、下层8点索引8‑15（模型坐标系）
    const auto outline = fcs::L2::ldm::volume_outline_points(config.geometry);

    // OpenCV Point3f 转换为Foxglove 3D点结构 Point3
    std::array<::foxglove::schemas::Point3, 16> pts;
    for (size_t i = 0; i < 16; ++i) {
        pts[i] = viz::make_point3(
            static_cast<double>(outline[i].x), static_cast<double>(outline[i].y),
            static_cast<double>(outline[i].z));
    }

    // 虚线分段参数：实线段2cm，空白间隔1.2cm，单位米
    constexpr double kDashLen = 0.020;
    constexpr double kGapLen  = 0.012;

    // 遍历8个立面，绘制所有棱线
    for (size_t i = 0; i < 8; ++i) {
        // 下一个顶点索引，i+1，i=7时取0，八边形闭环
        const size_t next = (i + 1) % 8;

        // --------上层环边 i → next：可见性由next立面决定--------
        if (visible_faces[next]) {
            // 立面可见，绘制实线line_strip
            builder.line_strip({pts[i], pts[next]}, solid_color, thickness);
        } else {
            // 立面不可见，调用函数绘制模拟虚线
            add_ldm_dashed_edge(
                builder, pts[i], pts[next], dashed_color, thickness, kDashLen, kGapLen);
        }

        // --------下层环边 i+8 → next+8，可见性同上--------
        if (visible_faces[next]) {
            builder.line_strip({pts[i + 8], pts[next + 8]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i + 8], pts[next + 8], dashed_color, thickness, kDashLen, kGapLen);
        }

        // --------垂直棱线 i ↔ i+8：左右相邻立面任一可见即为实线--------
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
    // 根据深度质量获取主绘制颜色
    const auto color = ldm_scene_color(measurement);
    // odom世界坐标系位姿，std::optional解引用，外部调用保证has_value
    const auto& pose = *measurement.transform_odom;

    // 获取本帧观测立面可见标记数组
    const auto visible_faces = ldm_volume_assigned_faces(measurement);

    // 创建实体构建器，绑定odom坐标系，分类l2，实体标识ldm
    auto builder = viz::EntityBuilder::create<fast_tf::odom>("l2", "ldm");
    // 设置时间戳、实体存活5ms、平移、旋转姿态、基础颜色
    builder.timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        .position(pose.translation())
        .orientation(pose.quaternion())
        .color(color);

    // 虚线使用主色45%亮度变暗，调用tactical.hpp scaled_rgb工具函数
    const auto dimmed_color = tac::scaled_rgb(color, 0.45);

    // 绘制八边形全部轮廓棱线（自动虚实切换），线条粗细0.0025米
    add_ldm_volume_edges(builder, config, visible_faces, color, dimmed_color, 0.0025);

    // 中心小球标记 + 顶部文字标签
    builder.size(0.025)          // 球体半径0.025m
        .sphere()                 // 添加球体图元
        .text_with_offset(        // 添加3D文本，z方向向上偏移
            ldm_label(measurement), 0.0, 0.0, config.geometry.volume_height_m * 0.8,
            tac::L2::LABEL_FONT_SIZE)
        // 附加元数据，鼠标悬浮Foxglove可查看这些key‑value调试信息
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count));
    // 构建返回完整SceneEntity消息对象
    return builder.build();
}

/**
 * @brief 相机光学坐标系LDM八边形3D实体
 * 用于观察相机视角下模型位置，与odom实体分开发布，两套坐标系互不干扰
 * @param measurement LDM测量结果
 * @param config LDM检测器配置
 * @return SceneEntity，坐标系 camera_optical
 */
[[nodiscard]] ::foxglove::schemas::SceneEntity make_ldm_volume_entity_camera(
    const fcs::L2::ldm::LdmMeasurement& measurement,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
    const auto color = ldm_scene_color(measurement);
    // 相机坐标系位姿，std::optional解引用
    const auto& pose = *measurement.transform_cam;

    const auto visible_faces = ldm_volume_assigned_faces(measurement);
    // 绑定camera_optical光学坐标系
    auto builder = viz::EntityBuilder::create<fast_tf::camera_optical>("l2", "ldm_cam");
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
 * @param measurement LDM测量结果
 * @return SceneEntity，camera_optical坐标系下视线+末端小球
 */
[[nodiscard]] ::foxglove::schemas::SceneEntity
    make_ldm_bearing_entity(const fcs::L2::ldm::LdmMeasurement& measurement) {
    const auto color = ldm_scene_color(measurement);
    // 归一化视线方向向量，放大到2米长度作为射线终点
    const Eigen::Vector3d end = measurement.bearing_cam.normalized() * 2.0;

    // 链式builder：原点(0,0,0)到end的线段，末端球体，悬浮文本，附加元数据
    return viz::EntityBuilder::create<fast_tf::camera_optical>("l2", "ldm_bearing")
        .timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        // line_strip：相机原点 → 2米远处射线
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

} // namespace 内部匿名工具命名空间结束

/**
 * @brief 批量注册L2感知层所有可视化调度系统
 * 包含6套独立pool_compute线程池系统：
 * 1. foxglove_l2_detection_pub：装甲检测框JSON
 * 2. foxglove_l2_measurement_scene：装甲3D立方体Scene
 * 3. foxglove_l2_ldm_scene：LDM八边形3D立体实体/视线射线
 * 4. foxglove_l2_ldm_detection_json：LDM原始光斑检测JSON
 * 5. foxglove_l2_ldm_measurement_json：LDM完整位姿测量JSON
 * 6. foxglove_l2_measurement_pub：装甲PnP位姿+协方差JSON
 * @param app 调度器Scheduler实例，注册system到调度器
 */
void register_l2_perception_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // 系统1：foxglove_l2_detection_pub 装甲原始检测框JSON发布
    // 输入：装甲检测批通道 spmc<ArmorDetectionBatch, DetectionChannelTopic>
    // 输出DebugArmorsMessage结构化JSON，发送给Foxglove服务
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_detection_pub",
        [](talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // foxglove_ready：检查Foxglove服务是否就绪、spmc通道有无新数据；不满足直接return退出system
            if (!foxglove_ready(*server, det_in)) {
                return;
            }

            // spmc读取一批检测数据，read()返回std::optional，无数据则为空
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
            // armors数组存放每一个装甲检测
            armors_json["armors"]            = nlohmann::json::array();

            // 遍历本帧所有装甲检测结果序列化写入json
            for (const auto& det : batch->detections) {
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
                // magic_enum 将枚举转为字符串，方便前端调试查看
                obj["color"] = magic_enum::enum_name(det.color);
                obj["type"]  = magic_enum::enum_name(det.type);
                obj["name"]  = magic_enum::enum_name(det.name);
                // 将单个装甲json压入armors数组
                armors_json["armors"].push_back(obj);
            }

            // 封装JSON通过Foxglove服务发送出去，模板参数DebugArmorsMessage自定义消息类型
            detail::publish_json_message<DebugArmorsMessage>(*server, armors_json);
        });

    // =========================================================================
    // 系统2：foxglove_l2_measurement_scene 装甲3D立方体场景发布
    // 输入：装甲PnP位姿批数据 MeasurementChannelTopic
    // 生成odom系立方体SceneEntity批量发布
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l2_measurement_scene",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // foxglove就绪检查，未就绪直接返回
            if (!foxglove_ready(*server, meas_in))
                return;

            // spmc读取测量批数据
            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            // 存放本帧所有装甲SceneEntity实体
            std::vector<::foxglove::schemas::SceneEntity> entities;

            // 逐个生成装甲3D立方体实体
            for (size_t i = 0; i < batch->measurements.size(); ++i) {
                const auto& m = batch->measurements[i];
                // 取出平移向量、四元数旋转
                const auto t  = m.transform.translation();
                const auto q  = m.transform.quaternion();

                // 创建odom坐标系下实体builder，id为armor_0 armor_1...
                auto builder =
                    viz::EntityBuilder::create<fast_tf::odom>("l2", fmt::format("armor_{}", i));
                // 设置位姿、基础颜色(测量虚影灰色)
                builder.position(t.x(), t.y(), t.z())
                    .orientation(q.x(), q.y(), q.z(), q.w())
                    .color(tac::L2::MEASUREMENT_GHOST)
                    // 立方体尺寸：厚度、高度、宽度（装甲物理尺寸，来自tactical.hpp Geometry命名空间）
                    .cube(
                        tac::Geometry::ARMOR_THICKNESS, tac::Geometry::ARMOR_HEIGHT_SMALL,
                        tac::Geometry::ARMOR_WIDTH)
                    // 立方体下方文本标签：装甲名称+类型，z负方向偏移
                    .text_with_offset(
                        fmt::format(
                            "{} {}", magic_enum::enum_name(m.name), magic_enum::enum_name(m.type)),
                        0, -tac::Text::SIZE_MEDIUM, 0, tac::L2::LABEL_FONT_SIZE)
                    // 附加PnP解算元数据：置信度、协方差、条件数，鼠标悬浮查看
                    .metadata("confidence", fmt::format("{:.3f}", m.confidence))
                    .metadata("color", std::string(magic_enum::enum_name(m.color)))
                    .metadata("pnp_cov_order", "bearing_yaw,bearing_pitch,log_distance,armor_yaw")
                    .metadata("pnp_condition", fmt::format("{:.3e}", m.pnp_condition_number))
                    .metadata("pnp_cov[0]", matrix4_row_string(m.pnp_cov_ypdr, 0))
                    .metadata("pnp_cov[1]", matrix4_row_string(m.pnp_cov_ypdr, 1))
                    .metadata("pnp_cov[2]", matrix4_row_string(m.pnp_cov_ypdr, 2))
                    .metadata("pnp_cov[3]", matrix4_row_string(m.pnp_cov_ypdr, 3))
                    .timestamp(m.timestamp_ns);

                // 构建SceneEntity，移动push_back进vector
                auto entity = builder.build();
                entities.push_back(std::move(entity));
            }

            // 批量发布3D场景消息，vector为空则跳过不发送
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
            // 就绪检查
            if (!foxglove_ready(*server, ldm_meas_in)) {
                return;
            }

            // spmc读取LDM测量结果
            auto measurement = ldm_meas_in.read();
            if (!measurement) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;
            // 根据可用位姿选择实体类型
            if (measurement->transform_odom.has_value()) {
                // 有odom世界位姿，生成odom坐标系八边形实体
                entities.push_back(make_ldm_volume_entity_odom(*measurement, *ldm_config));
            } else if (measurement->transform_cam.has_value()) {
                // 只有相机坐标系位姿，生成相机坐标系八边形实体
                entities.push_back(make_ldm_volume_entity_camera(*measurement, *ldm_config));
            } else {
                // 没有任何有效位姿，只有方位角，绘制视线射线
                entities.push_back(make_ldm_bearing_entity(*measurement));
            }

            // 发布场景实体，空则跳过
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
            j["rect"]      = nlohmann::json::object();
            j["rect"]["x"] = det->rect.x;
            j["rect"]["y"] = det->rect.y;
            j["rect"]["w"] = det->rect.width;
            j["rect"]["h"] = det->rect.height;

            // 图像中心像素，std::optional判断是否有值
            if (det->center_image_px.has_value()) {
                j["center_image_px"]      = nlohmann::json::object();
                j["center_image_px"]["x"] = det->center_image_px->x;
                j["center_image_px"]["y"] = det->center_image_px->y;
            }

            // 光斑、配对、候选解数量统计
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
                // 把该候选匹配到的立面下标全部写入json数组
                for (const int fi : cand.octagon_face_indices) {
                    cj["face_indices"].push_back(fi);
                }
                j["candidates"].push_back(cj);
            }

            // 最优候选解下标，optional存在才写入
            if (det->selected_candidate_idx.has_value()) {
                j["selected_candidate_idx"] = *det->selected_candidate_idx;
            }

            // 发送LdmDetectionMessage类型json消息
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
            // 相机归一化视线向量xyz
            j["bearing_cam"] = {
                meas->bearing_cam.x(), meas->bearing_cam.y(), meas->bearing_cam.z()};
            j["depth_quality"]   = std::string(magic_enum::enum_name(meas->depth_quality));
            j["confidence"]      = meas->confidence;
            j["candidate_count"] = meas->mesh_candidates.size();

            // 最优候选下标，optional存在写入json
            if (meas->selected_candidate_idx.has_value()) {
                j["selected_candidate_idx"] = *meas->selected_candidate_idx;
            }

            // 相机坐标系位姿、直线距离，判断optional是否有效值
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

            // 查询当前帧odom→相机光学坐标变换，带时间戳插值查找
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, batch->timestamp_ns);

            // 遍历每一个装甲PnP测量结果
            for (const auto& m : batch->measurements) {
                const auto t = m.transform.translation();
                const auto q = m.transform.quaternion();
                // 位姿分解得到roll pitch yaw欧拉角，结构化绑定
                auto [r, p, y] = m.transform.euler_rot().rpy();

                nlohmann::json mj;
                mj["timestamp_ns"]      = m.timestamp_ns;
                mj["timestamp"]         = nlohmann::json::object();
                mj["timestamp"]["sec"]  = m.timestamp_ns / 1000000000L;
                mj["timestamp"]["nsec"] = m.timestamp_ns % 1000000000L;
                mj["name"]              = std::string(magic_enum::enum_name(m.name));
                mj["type"]              = std::string(magic_enum::enum_name(m.type));
                mj["color"]             = std::string(magic_enum::enum_name(m.color));
                mj["confidence"]        = m.confidence;
                mj["position"]          = {t.x(), t.y(), t.z()};
                mj["orientation"]       = {q.x(), q.y(), q.z(), q.w()};
                mj["euler"]["roll"]     = r;
                mj["euler"]["pitch"]    = p;
                mj["euler"]["yaw"]      = y;
                mj["distance"]          = t.norm();
                // PnP协方差维度说明：方位偏航、俯仰、对数距离、装甲偏航
                mj["pnp_geometry"]["order"] = {
                    "bearing_yaw", "bearing_pitch", "log_distance", "armor_yaw"};
                mj["pnp_geometry"]["cov_ypdr"]  = matrix4_to_json(m.pnp_cov_ypdr);
                mj["pnp_geometry"]["diag"]      = matrix4_diag_to_json(m.pnp_cov_ypdr);
                mj["pnp_geometry"]["condition"] = m.pnp_condition_number;
                mj["pnp_geometry"]["note"] =
                    "normalized‑plane PnP covariance propagated to measurement coordinates";

                // TF树查询有效时，计算相机观测yaw与世界yaw角度偏差（度）
                if (tf_lookup) {
                    auto target_in_ref = (m.transform);

                    auto target_pos_yaw = fcs::L3::xyz2ypd(target_in_ref.translation())[0];
                    auto [roll, pitch, target_yaw] = target_in_ref.euler_rot().rpy();

                    // shortest_rad：最短角度差，弧度转角度存入json
                    mj["delta_angle"] =
                        L3::shortest_rad(target_pos_yaw, target_yaw) * 180.0 / std::numbers::pi;
                }

                j["measurements"].push_back(mj);
            }

            detail::publish_json_message<MeasurementMessage>(*server, j);
        });
}

} // namespace fcs::visualization::foxglove::systems