// LDM八边形符文检测配置结构体定义
#include "L2_perception/ldm/ldm_config.hpp"
// LDM检测、测量结果数据结构
#include "L2_perception/ldm/types.hpp"
// Foxglove可视化系统基类
#include "base.hpp"
// 调度器运行时核心
#include "core/runtime.hpp"
// Foxglove服务全局配置
#include "foxglove_config.hpp"
// Foxglove消息类型定义（图像、视频、TF、标定等）
#include "foxglove_types.hpp"
// H265视频编码器封装
#include "quanta/stream_encoder.hpp"

// 相机内外参配置结构体
#include "camera_config.hpp"
// 全局话题名常量定义
#include "core/channel_topics.hpp"
// 调度器全局通用数据类型
#include "core/types.hpp"
// 图像帧基础结构体（携带原图、时间戳、帧ID、检测ROI）
#include "frame.hpp"

// 标准库容器/算法/数学
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <numbers>
#include <optional>
// OpenCV图像、绘图、编解码
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <utility>

namespace fcs::visualization::foxglove::systems {

// 内部工具函数、状态结构体隔离在detail命名空间，对外不可见
namespace detail {

/**
 * @brief 视频编码器帧时间戳映射缓存
 * @param pts 编码器内部递增帧计数
 * @param timestamp_ns 原始图像硬件纳秒时间戳
 */
struct QuantaFrameStamp {
    int64_t pts{0};
    uint64_t timestamp_ns{0};
};

/**
 * @brief LDM符文可视化缓存状态
 * 保存最新一帧LDM检测、测量结果，跨帧复用绘图，避免频繁拷贝
 */
struct LdmOverlayState {
    // LDM原始blob、配对检测结果
    std::optional<L2::ldm::LdmDetection> latest_detection{};
    // 位姿解算后的八边形网格测量结果（含投影轮廓、RMSE、相机坐标系位姿）
    std::optional<L2::ldm::LdmMeasurement> latest_measurement{};
};

/**
 * @brief H265视频编码器全局状态
 * 管理编码器实例、待匹配时间戳队列、尺寸/帧率缓存、错误防抖
 */
struct QuantaPublisherState {
    // H265编码器实例，optional延迟初始化
    std::optional<quanta::StreamEncoder> encoder{};
    // 编码器pts → 原始图像时间戳 等待匹配队列
    std::deque<QuantaFrameStamp> pending_frames{};
    // 上一条错误信息，用于防抖重复打印
    std::string last_error{};
    // 输入图像分辨率
    int src_width{0};
    int src_height{0};
    // 编码帧率
    int framerate{0};
    // 下一个待分配的编码器pts序号，自增
    int64_t next_pts{0};
};

/**
 * @brief 计算八边形投影轮廓包围矩形
 * @param projected_outline 8边形16个投影像素点（上下两层各8点）
 * @return 轮廓最小包围矩形
 */
[[nodiscard]] inline cv::Rect2f
    projected_outline_bounds(const std::vector<cv::Point2f>& projected_outline) noexcept {
    if (projected_outline.empty()) {
        return {};
    }
    return cv::boundingRect(projected_outline);
}

/**
 * @brief 获取八边形单一面外法向量（模型坐标系）
 * @param face_idx 0~7 八边形8个立面编号
 * @return 模型坐标系朝外单位法向量
 */
[[nodiscard]] inline Eigen::Vector3d ldm_face_outward_normal_model(size_t face_idx) {
    // 每个面间隔45°，XZ平面分布，Y轴垂直地面
    const double angle = static_cast<double>(face_idx) * (std::numbers::pi_v<double> / 4.0);
    return Eigen::Vector3d(std::sin(angle), 0.0, -std::cos(angle));
}

/**
 * @brief 判断八边形单个立面是否在相机视野可见（背面剔除）
 * @param candidate 单套八边形位姿解算结果
 * @param geometry 八边形几何尺寸配置（外接圆半径）
 * @param face_idx 立面编号0~7
 * @return true 正面朝向相机，需要绘制实线；false 背面，绘制虚线
 */
[[nodiscard]] inline bool ldm_face_visible_from_camera(
    const L2::ldm::LdmMeshCandidate& candidate, const L2::ldm::LdmGeometryConfig& geometry,
    size_t face_idx) {
    // 位姿未求解完成，不做剔除，全部显示
    if (!candidate.solved) {
        return true;
    }

    // 物体→相机变换矩阵
    const auto& pose = candidate.pose.camera;
    // 模型法向量转到相机坐标系
    const Eigen::Vector3d normal   = ldm_face_outward_normal_model(face_idx);
    const Eigen::Vector3d normal_c = pose.rotation() * normal;
    // 立面中心点相机坐标
    const Eigen::Vector3d face_c = pose.rotation()
                                     * (geometry.octagon_circumradius_m
                                        * std::cos(std::numbers::pi_v<double> / 8.0) * normal)
                                 + pose.translation();
    const double face_range = face_c.norm();
    // 深度非法/距离过近/在相机后方(Z<=0) → 不可见
    if (!std::isfinite(face_range) || face_range <= 1e-9 || face_c.z() <= 1e-6) {
        return false;
    }

    // 法向量与视线夹角点积 >0 正面可见
    return normal_c.dot(-face_c / face_range) > 1e-6;
}

/**
 * @brief 批量获取八边形8个立面可见标记数组
 * @return array[8] 每个面是否可见
 */
[[nodiscard]] inline std::array<bool, 8> ldm_visible_faces_from_camera(
    const L2::ldm::LdmMeshCandidate& candidate, const L2::ldm::LdmGeometryConfig& geometry) {
    std::array<bool, 8> visible{};
    for (size_t face_idx = 0; face_idx < visible.size(); ++face_idx) {
        visible[face_idx] = ldm_face_visible_from_camera(candidate, geometry, face_idx);
    }
    return visible;
}

/**
 * @brief 绘制自定义虚线（OpenCV无原生虚线API）
 * @param image 画布
 * @param start 起点
 * @param end 终点
 * @param color 线条颜色
 * @param thickness 线宽
 */
inline void draw_dashed_line(
    cv::Mat& image, cv::Point2f start, cv::Point2f end, const cv::Scalar& color, int thickness) {
    const double length = cv::norm(end - start);
    // 线段长度非法/过短直接跳过
    if (!std::isfinite(length) || length <= 1.0) {
        return;
    }

    constexpr double kDashPx = 7.0; // 实线段像素长度
    constexpr double kGapPx  = 5.0; // 空白间隔像素
    constexpr double kStep   = kDashPx + kGapPx;
    // 单位方向向量
    const cv::Point2f direction = (end - start) * static_cast<float>(1.0 / std::max(length, 1.0));
    const int num_segments      = static_cast<int>(length / kStep) + 1;
    // 分段循环绘制实线片段
    for (int seg = 0; seg < num_segments; ++seg) {
        const double offset   = static_cast<double>(seg) * kStep;
        const double dash_end = std::min(length, offset + kDashPx);
        cv::line(
            image, start + direction * static_cast<float>(offset),
            start + direction * static_cast<float>(dash_end), color, thickness, cv::LINE_AA);
    }
}

/**
 * @brief 根据可见性绘制线段：可见实线，不可见灰色虚线
 * @param visible 立面是否朝向相机
 */
inline void draw_visibility_line(
    cv::Mat& image, cv::Point2f start, cv::Point2f end, const cv::Scalar& color, bool visible) {
    if (visible) {
        // 可见：主色实线
        cv::line(image, start, end, color, tac::Image::LINE_MEDIUM, cv::LINE_AA);
        return;
    }
    // 不可见：次级灰色虚线
    const cv::Scalar dashed_color = tac::to_cv_bgr(tac::Image::LDM_SECONDARY);
    draw_dashed_line(image, start, end, dashed_color, tac::Image::LINE_THIN);
}

/**
 * @brief 绘制完整LDM八边形投影外轮廓
 * 上下两层八角顶点，立面背面自动虚线剔除
 * @param image 原图画布
 * @param candidate 位姿解算网格结果
 * @param geometry 八边形尺寸配置
 * @param color 轮廓主色
 */
inline void draw_ldm_projected_outline(
    cv::Mat& image, const L2::ldm::LdmMeshCandidate& candidate,
    const L2::ldm::LdmGeometryConfig& geometry, const cv::Scalar& color) {
    const auto& projected_outline = candidate.projected_outline_image;
    // 合法轮廓必须16个点：上层8点+下层8点
    if (projected_outline.size() != 16) {
        return;
    }

    // 预计算8个立面可见标记
    const auto visible_faces = ldm_visible_faces_from_camera(candidate, geometry);
    for (size_t i = 0; i < 8; ++i) {
        const size_t next = (i + 1) % 8;
        // 立面侧边：上层相邻点连线、下层相邻点连线
        const bool side_visible = visible_faces[next];
        draw_visibility_line(
            image, projected_outline[i], projected_outline[next], color, side_visible);
        draw_visibility_line(
            image, projected_outline[i + 8], projected_outline[next + 8], color, side_visible);
        // 垂直棱线：上下对应顶点连线，相邻任一立面可见即实线
        draw_visibility_line(
            image, projected_outline[i], projected_outline[i + 8], color,
            visible_faces[i] || visible_faces[next]);
    }
}

/**
 * @brief 编码器错误日志防抖：相同错误只打印一次，避免刷屏
 * @param state 编码器状态
 * @param message 错误文本
 */
inline void log_quanta_error_once(QuantaPublisherState& state, std::string message) noexcept {
    if (state.last_error == message)
        return;
    state.last_error = std::move(message);
    SPDLOG_WARN("Foxglove quanta encoder: {}", state.last_error);
}

/**
 * @brief 初始化/复用H265编码器，分辨率/帧率变化自动重建
 * @return true 编码器就绪可用
 */
[[nodiscard]] inline bool ensure_quanta_encoder(
    QuantaPublisherState& state, const quanta::EncodeParams& cfg, int src_width,
    int src_height) noexcept {
    // 编码器已存在、分辨率/帧率完全匹配，直接复用
    if (state.encoder && state.src_width == src_width && state.src_height == src_height
        && state.framerate == cfg.framerate) {
        return true;
    }

    // 创建新编码器实例
    auto enc = quanta::StreamEncoder::create(cfg, src_width, src_height, cfg.framerate);
    if (!enc) {
        // 创建失败，记录错误并清空编码器
        log_quanta_error_once(state, std::move(enc.error()));
        state.encoder.reset();
        return false;
    }
    // 保存编码器、重置帧pts与时间戳队列
    state.encoder.emplace(std::move(*enc));
    state.pending_frames.clear();
    state.next_pts   = 0;
    state.src_width  = src_width;
    state.src_height = src_height;
    state.framerate  = cfg.framerate;

    state.last_error.clear();
    SPDLOG_INFO(
        "Foxglove quanta encoder initialized: {}x{} -> <= {}x{}, {} bps @ {} fps", src_width,
        src_height, cfg.max_width, cfg.max_height, cfg.target_bitrate, cfg.framerate);
    return true;
}

/**
 * @brief 根据编码器pts匹配原始图像纳秒时间戳
 * 编码包输出晚于入帧，通过pts映射找回原图时间戳
 * @param pts 编码帧序号
 * @param fallback_timestamp_ns 队列找不到时备用时间戳
 * @return 对应图像硬件时间戳
 */
[[nodiscard]] inline uint64_t take_timestamp_for_pts(
    QuantaPublisherState& state, int64_t pts, uint64_t fallback_timestamp_ns) noexcept {
    for (auto it = state.pending_frames.begin(); it != state.pending_frames.end(); ++it) {
        if (it->pts != pts)
            continue;
        // 匹配成功，取出时间戳并删除缓存记录
        const uint64_t timestamp_ns = it->timestamp_ns;
        state.pending_frames.erase(it);
        return timestamp_ns;
    }
    // 无匹配记录，返回当前帧时间戳兜底
    return fallback_timestamp_ns;
}

/**
 * @brief 推送原图进入H265编码器，轮询输出编码包并通过Foxglove发送视频流
 * @param server Foxglove服务句柄
 * @param state 编码器状态
 * @param cfg 编码参数（码率、缩放、帧率）
 * @param image_bgr 原始BGR图像
 * @param timestamp_ns 图像纳秒时间戳
 */
inline void publish_quanta_video(
    FoxgloveServer& server, QuantaPublisherState& state, const quanta::EncodeParams& cfg,
    const cv::Mat& image_bgr, uint64_t timestamp_ns) noexcept {
    // 编码器未就绪直接返回
    if (!ensure_quanta_encoder(state, cfg, image_bgr.cols, image_bgr.rows))
        return;

    // 分配递增pts帧序号
    const int64_t pts = state.next_pts++;
    // 送入原始图像像素数据
    auto push_result =
        state.encoder->push_frame(image_bgr.data, static_cast<int>(image_bgr.step[0]), pts);
    if (!push_result) {
        log_quanta_error_once(state, std::move(push_result.error()));
        return;
    }

    // 缓存pts与原图时间戳映射，等待编码包输出匹配
    state.pending_frames.push_back(QuantaFrameStamp{.pts = pts, .timestamp_ns = timestamp_ns});

    // 循环取出所有已编码完成H265数据包
    while (auto packet = state.encoder->poll_packet()) {
        // 匹配该包对应的原始图像时间戳
        const uint64_t packet_timestamp_ns =
            take_timestamp_for_pts(state, packet->pts, timestamp_ns);

        // 构造Foxglove标准VideoMessage
        VideoMessage msg;
        msg.payload.timestamp = timestamp_from_ns(packet_timestamp_ns);
        msg.payload.frame_id  = "camera_optical_frame"; // 相机光学坐标系
        msg.payload.format    = "h265";
        // 拷贝H265码流二进制数据
        msg.payload.data = std::vector<std::byte>(
            reinterpret_cast<const std::byte*>(packet->data.get()),
            reinterpret_cast<const std::byte*>(packet->data.get() + packet->size));

        // 消息入队发送
        server.enqueue_message(std::move(msg));
    }
}

/**
 * @brief WebSocket实时模式：原图压缩JPEG单帧图像发布
 * @param server Foxglove服务
 * @param image_bgr 叠加完绘制的图像
 * @param timestamp_ns 图像时间戳
 */
inline void publish_jpeg_image(
    FoxgloveServer& server, const cv::Mat& image_bgr, uint64_t timestamp_ns) noexcept {
    std::vector<uint8_t> compressed;
    // JPEG质量45，平衡带宽与清晰度
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 45};
    // 图像编码失败直接丢弃
    if (!cv::imencode(".jpg", image_bgr, compressed, params)) {
        return;
    }

    // 构造图像消息
    ImageMessage msg;
    msg.payload.timestamp = timestamp_from_ns(timestamp_ns);
    msg.payload.frame_id  = "camera_optical_frame";
    msg.payload.format    = "jpeg";
    msg.payload.data      = std::vector<std::byte>(
        reinterpret_cast<const std::byte*>(compressed.data()),
        reinterpret_cast<const std::byte*>(compressed.data() + compressed.size()));

    server.enqueue_message(std::move(msg));
}

/**
 * @brief 帧同步校验：判断两帧时间戳偏差是否在允许范围内
 * @param producer_timestamp_ns 检测帧时间戳
 * @param consumer_timestamp_ns 图像帧时间戳
 * @param max_skew_ns 最大允许纳秒偏差
 * @return true 同步可用，允许叠加绘图
 */
[[nodiscard]] inline bool frame_sync_ok(
    uint64_t producer_timestamp_ns, uint64_t consumer_timestamp_ns, uint64_t max_skew_ns) noexcept {
    const uint64_t skew = (producer_timestamp_ns > consumer_timestamp_ns)
                            ? (producer_timestamp_ns - consumer_timestamp_ns)
                            : (consumer_timestamp_ns - producer_timestamp_ns);
    return skew <= max_skew_ns;
}

} // namespace detail

/**
 * @brief 注册L1底层传感器可视化系统
 * 功能：相机原图叠加装甲/LDM检测框、发布图像流、TF坐标、相机标定内参
 * 系统名：foxglove_l1_image_pub，线程池并行调度
 */
void register_l1_sensor_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // 图像发布系统：同步渲染原图+各类检测叠加图层
    // 输入通道：装甲检测批数据、LDM检测、LDM位姿测量
    // 共享资源：Foxglove服务实例、相机参数、可视化全局配置、TF树、LDM几何配置
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l1_image_pub",
        // 捕获编码器状态、LDM绘图缓存，mutable允许修改内部状态
        [video_state = detail::QuantaPublisherState{}, ldm_state = detail::LdmOverlayState{}](
            // 输入SPMC多生产者单消费者通道
            talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
            talos::spmc<L2::ldm::LdmDetection, LdmDetectionChannelTopic> ldm_in,
            talos::spmc<L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> ldm_meas_in,
            // 全局共享资源：Foxglove服务指针
            talos::res<std::shared_ptr<FoxgloveServer>> server,
            // 相机标定配置
            talos::res<CameraConfig> cam,
            // Foxglove全局配置（传输方式、编码参数）
            talos::res<FoxgloveConfig> foxglove_cfg,
            [[maybe_unused]] core::detecting_color detecting_color_,
            // TF坐标变换树
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            // LDM八边形几何尺寸配置
            talos::res<L2::ldm::LdmDetectorConfig> ldm_config) mutable {
            // 校验Foxglove服务是否正常就绪、输入通道有数据
            if (!foxglove_ready(*server, det_in)) {
                return;
            }

            // 读取最新一帧相机图像批数据
            auto batch = det_in.read();
            // 无数据/空图像直接退出本轮
            if (!batch || batch->image.empty()) {
                return;
            }

            // 1. 发布当前帧所有TF坐标变换
            (*server)->publish_tf(*tf_buffer, batch->timestamp_ns);

            // 拷贝原图用于叠加绘图，不修改原始图像帧数据
            cv::Mat img_bgr = batch->image.clone();

            // 绘制检测ROI有效区域矩形
            const cv::Scalar roi_color = tac::to_cv_bgr(
                batch->has_detector_roi ? tac::Image::ROI_VALID : tac::Image::ROI_MISSING);
            cv::rectangle(
                img_bgr, batch->detector_roi, roi_color, tac::Image::LINE_MEDIUM, cv::LINE_AA);

            // 绘制相机光心十字标记（内参cx cy）
            const double cx = cam->camera_matrix(0, 2);
            const double cy = cam->camera_matrix(1, 2);
            cv::drawMarker(
                img_bgr, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                tac::to_cv_bgr(tac::Image::OPTICAL_CENTER), cv::MARKER_CROSS,
                tac::Image::MARKER_SIZE, tac::Image::LINE_MEDIUM);

            // 装甲框文字、线条配色
            const cv::Scalar box_color  = tac::to_cv_bgr(tac::Image::DETECTION_BOX);
            const cv::Scalar text_color = tac::to_cv_bgr(tac::Image::DETECTION_TEXT);
            // 四个装甲角点配色：RT红、LT绿、LB蓝、RB黄
            std::array<cv::Scalar, 4> colors = {
                cv::Scalar(255, 0, 0),  // RT
                cv::Scalar(0, 255, 0),  // LT
                cv::Scalar(0, 0, 255),  // LB
                cv::Scalar(0, 255, 255) // RB
            };
            // 遍历所有装甲检测结果
            for (const auto& det : batch->detections) {
                // 绘制4个角点+带箭头边线
                for (size_t j = 0; j < det.corners.size(); j++) {
                    auto pp1 = det.corners[j];
                    auto pp2 = det.corners[(j + 1) % 4];
                    cv::circle(img_bgr, pp1, 2, colors[(j + 1) % 4], tac::Image::LINE_MEDIUM);
                    cv::arrowedLine(
                        img_bgr, pp1, pp2, colors[(j + 1) % 4], tac::Image::LINE_MEDIUM,
                        cv::LINE_AA, 0, 10.0 / cv::norm(pp1 - pp2));
                }

                // 装甲类型、颜色文字
                cv::Point2f text_pos(det.rect.x, det.rect.y + det.rect.height + 20);
                std::string name_type = fmt::format(
                    "{} {} {}", magic_enum::enum_name(det.name), magic_enum::enum_name(det.type),
                    magic_enum::enum_name(det.color));
                cv::putText(
                    img_bgr, name_type, text_pos, cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                    text_color, tac::Image::TEXT_THIN, cv::LINE_AA);

                // 置信度文字
                cv::Point2f conf_pos(det.rect.x, det.rect.y + det.rect.height + 40);
                std::string conf_str = fmt::format("conf: {:.2f}", det.confidence);
                cv::putText(
                    img_bgr, conf_str, conf_pos, cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                    text_color, tac::Image::TEXT_THIN, cv::LINE_AA);
            }

            // LDM帧同步最大允许200ms时间偏移
            constexpr uint64_t kMaxLdmOverlaySkewNs = 200'000'000;
            // 读取当前最新LDM检测、测量结果缓存
            const auto maybe_ldm_det  = ldm_in.read_current();
            const auto maybe_ldm_meas = ldm_meas_in.read_current();
            if (maybe_ldm_det) {
                ldm_state.latest_detection = *maybe_ldm_det;
            }
            if (maybe_ldm_meas) {
                ldm_state.latest_measurement = *maybe_ldm_meas;
            }

            // 判断LDM检测帧是否与当前图像帧同步，允许叠加绘图
            const auto use_ldm_det = ldm_state.latest_detection.has_value()
                                  && (ldm_state.latest_detection->frame_id == batch->frame_id
                                      || detail::frame_sync_ok(
                                          ldm_state.latest_detection->timestamp_ns,
                                          batch->timestamp_ns, kMaxLdmOverlaySkewNs));

            // 绘制LDM原始blob、配对中点、配对连线
            if (use_ldm_det) {
                const auto& ldm_det           = *ldm_state.latest_detection;
                const cv::Scalar blob_color   = tac::to_cv_bgr(tac::Image::LDM_SECONDARY);
                const cv::Scalar pair_color   = tac::to_cv_bgr(tac::Image::LDM_PRIMARY);
                const cv::Scalar center_color = tac::to_cv_bgr(tac::Image::LDM_CENTER);
                cv::Rect2f ldm_rect           = ldm_det.rect;

                // 绘制单个光斑blob矩形框
                for (const auto& blob : ldm_det.blobs) {
                    cv::rectangle(
                        img_bgr, blob.rect, blob_color, tac::Image::LINE_THIN, cv::LINE_AA);
                }

                // 绘制光斑配对连线、中点标记、配对编号
                for (size_t i = 0; i < ldm_det.pairs.size(); ++i) {
                    const auto& pair = ldm_det.pairs[i];
                    cv::line(
                        img_bgr, pair.top_center_px, pair.bottom_center_px, pair_color,
                        tac::Image::LINE_MEDIUM, cv::LINE_AA);
                    cv::circle(img_bgr, pair.top_center_px, 4, colors[0], tac::Image::LINE_MEDIUM);
                    cv::circle(
                        img_bgr, pair.bottom_center_px, 4, colors[2], tac::Image::LINE_MEDIUM);
                    cv::circle(img_bgr, pair.midpoint_px, 3, center_color, -1);
                    cv::putText(
                        img_bgr, fmt::format("p{}", i), pair.midpoint_px + cv::Point2f(4.0f, -4.0f),
                        cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL, center_color,
                        tac::Image::TEXT_THIN, cv::LINE_AA);
                }

                // 根据所有配对中点计算LDM整体包围框
                {
                    std::vector<cv::Point2f> pair_pts;
                    for (const auto& pair : ldm_det.pairs) {
                        pair_pts.push_back(pair.top_center_px);
                        pair_pts.push_back(pair.bottom_center_px);
                    }
                    if (!pair_pts.empty()) {
                        ldm_rect = cv::boundingRect(pair_pts);
                    }
                }

                // 绘制LDM整体包围框与配对数量文字
                if (ldm_rect.width > 0.0f && ldm_rect.height > 0.0f) {
                    cv::rectangle(
                        img_bgr, ldm_rect, pair_color, tac::Image::LINE_MEDIUM, cv::LINE_AA);
                    cv::putText(
                        img_bgr, fmt::format("LDM {}", ldm_det.pairs.size()),
                        cv::Point(
                            static_cast<int>(ldm_rect.x),
                            std::max(20, static_cast<int>(ldm_rect.y) - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_MEDIUM, pair_color,
                        tac::Image::TEXT_MEDIUM_PX, cv::LINE_AA);
                }
            }

            // 判断LDM位姿解算测量帧是否同步
            const auto use_ldm_meas = ldm_state.latest_measurement.has_value()
                                   && (ldm_state.latest_measurement->frame_id == batch->frame_id
                                       || detail::frame_sync_ok(
                                           ldm_state.latest_measurement->timestamp_ns,
                                           batch->timestamp_ns, kMaxLdmOverlaySkewNs));
            if (use_ldm_meas) {
                const auto& ldm_meas = *ldm_state.latest_measurement;
                // 左上角打印配对数、深度质量、置信度
                cv::putText(
                    img_bgr,
                    fmt::format(
                        "LDM pairs={}/{} {} conf={:.2f}", ldm_meas.selected_pair_count,
                        ldm_meas.pair_count_total, ldm_meas.depth_quality, ldm_meas.confidence),
                    cv::Point(30, 32), cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_MEDIUM,
                    tac::to_cv_bgr(tac::Image::LDM_PRIMARY), tac::Image::TEXT_MEDIUM_PX,
                    cv::LINE_AA);

                // 打印解算后相机坐标系XYZ坐标
                if (ldm_meas.transform_cam.has_value()) {
                    const auto center = ldm_meas.transform_cam->translation();
                    cv::putText(
                        img_bgr,
                        fmt::format(
                            "cam=[{:.2f},{:.2f},{:.2f}]m", center.x(), center.y(), center.z()),
                        cv::Point(30, 56), cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                        tac::to_cv_bgr(tac::Image::LDM_SECONDARY), tac::Image::TEXT_THIN,
                        cv::LINE_AA);
                }

                // 绘制八边形完整投影轮廓（带背面剔除虚线）
                const auto& selected_idx = ldm_meas.selected_candidate_idx;
                if (selected_idx.has_value() && *selected_idx >= 0
                    && static_cast<size_t>(*selected_idx) < ldm_meas.mesh_candidates.size()) {
                    const auto& selected =
                        ldm_meas.mesh_candidates[static_cast<size_t>(*selected_idx)];
                    if (selected.projected_outline_image.size() == 16) {
                        detail::draw_ldm_projected_outline(
                            img_bgr, selected, ldm_config->geometry, box_color);
                    }

                    // 绘制八边形中心十字标记
                    cv::drawMarker(
                        img_bgr, ldm_meas.center_image_px, tac::to_cv_bgr(tac::Image::LDM_CENTER),
                        cv::MARKER_CROSS, 18, tac::Image::LINE_MEDIUM);

                    // 打印可见立面编号、重投影RMSE误差
                    std::string faces_text;
                    for (size_t i = 0; i < selected.octagon_face_indices.size(); ++i) {
                        if (!faces_text.empty()) {
                            faces_text += ",";
                        }
                        faces_text += std::to_string(selected.octagon_face_indices[i]);
                    }
                    const std::string rmse_text =
                        std::isfinite(selected.reprojection_rmse_px)
                            ? fmt::format("{:.2f}", selected.reprojection_rmse_px)
                            : "n/a";

                    const cv::Rect2f meas_rect =
                        detail::projected_outline_bounds(selected.projected_outline_image);
                    cv::putText(
                        img_bgr, fmt::format("ldm faces={} rmse={}", faces_text, rmse_text),
                        cv::Point(
                            static_cast<int>(meas_rect.x),
                            std::max(20, static_cast<int>(meas_rect.y) - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                        tac::to_cv_bgr(tac::Image::LDM_PRIMARY), tac::Image::TEXT_THIN,
                        cv::LINE_AA);
                }
            }

            // 根据Foxglove传输模式选择推流方式
            if (foxglove_cfg->transport == FoxgloveTransport::Mcap) {
                // MCAP离线录包：H265视频流编码发送
                detail::publish_quanta_video(
                    *(*server), video_state, foxglove_cfg->quanta, img_bgr, batch->timestamp_ns);
            } else {
                // WebSocket实时可视化：单帧JPEG图片发送
                detail::publish_jpeg_image(*(*server), img_bgr, batch->timestamp_ns);
            }

            // 发布相机内参标定消息，客户端可读取畸变、内参用于3D可视化
            std::array<double, 9> camera_matrix_arr;
            std::copy_n(cam->camera_matrix.data(), 9, camera_matrix_arr.begin());
            std::vector<double> dist_coeffs(
                cam->distort_coefficient.data(),
                cam->distort_coefficient.data() + cam->distort_coefficient.size());

            (*server)->publish_camera_calibration(
                cam->width, cam->height, camera_matrix_arr, dist_coeffs, batch->timestamp_ns);
        });
}

} // namespace fcs::visualization::foxglove::systems