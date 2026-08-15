// LDM八边形符文检测配置结构体定义
#include "L2_perception/ldm/ldm_config.hpp"
// LDM检测、测量结果数据结构
#include "L2_perception/ldm/types.hpp"
// Foxglove可视化系统基类
#include "base.hpp"
// 调度器完整定义（Scheduler、fixed_rate、spmc 等）
#include "scheduler/scheduler.hpp"
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
 * @brief 确保Quanta视频编码器可用，参数不匹配则重建编码器
 * @param state 编码器状态结构体（持有编码器实例、分辨率、pts、缓存队列、错误信息）
 * @param cfg 编码全局配置：码率、最大缩放分辨率、帧率等
 * @param src_width 原始图像宽度
 * @param src_height 原始图像高度
 * @return bool true=编码器就绪可用；false=编码器创建失败
 * @ noexcept 函数不抛出C++异常
 * @ [[nodiscard]] 禁止忽略返回值，必须判断编码器是否创建成功
 */
[[nodiscard]] inline bool ensure_quanta_encoder(
    QuantaPublisherState& state, const quanta::EncodeParams& cfg, int src_width,
    int src_height) noexcept {
    // 分支1：已有编码器，且分辨率、帧率完全一致 → 直接复用，无需重建
    if (state.encoder && state.src_width == src_width && state.src_height == src_height
        && state.framerate == cfg.framerate) {
        return true;
    }

    // 分支2：无编码器 / 分辨率/帧率发生变化，需要新建编码器
    // 工厂创建视频流编码器，传入编码配置、原图分辨率、输出帧率
    auto enc = quanta::StreamEncoder::create(cfg, src_width, src_height, cfg.framerate);

    // 判断编码器创建是否失败（硬件编码不支持、参数非法、内存不足等）
    if (!enc) {
        // 只打印一次本次同类错误，避免刷屏
        log_quanta_error_once(state, std::move(enc.error()));
        // 释放原有编码器资源
        state.encoder.reset();
        // 返回失败，上层不可执行编码
        return false;
    }

    // 把创建成功的编码器移动存入state的optional容器，接管所有权
    state.encoder.emplace(std::move(*enc));
    // 清空待编码帧缓存队列，旧帧分辨率不匹配直接丢弃
    state.pending_frames.clear();
    // 重置PTS（显示时间戳）计数器，从0重新计数
    state.next_pts = 0;
    // 更新状态里记录的当前图像分辨率、帧率
    state.src_width  = src_width;
    state.src_height = src_height;
    state.framerate  = cfg.framerate;

    // 清空上次保存的错误标记
    state.last_error.clear();
    // 打印日志提示编码器初始化完成，输出原图尺寸、缩放上限、目标码率、帧率
    SPDLOG_INFO(
        "Foxglove quanta encoder initialized: {}x{} -> <= {}x{}, {} bps @ {} fps", src_width,
        src_height, cfg.max_width, cfg.max_height, cfg.target_bitrate, cfg.framerate);
    // 编码器就绪，返回成功
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

/// @brief 注册L1传感器图像可视化系统
/// 功能：读取相机原图、装甲检测、LDM能量机关检测/位姿数据，在图像上绘制各类标注框、文字、特征点
/// 再根据配置选择 JPEG单帧(WebSocket实时) / H265视频流(Mcap录包) 推送到Foxglove前端
/// 额外同步发布TF坐标变换、相机标定内参，支撑前端3D可视化
/// @param app ECS调度器实例，用于注册pool_compute并行图像渲染任务
void register_l1_sensor_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // L1 图像可视化发布系统：原图叠加检测图层推流至Foxglove
    // 调度类型：talos::pool_compute 线程池并行执行，不阻塞视觉主线程
    // 持久捕获：video_state视频编码器状态、ldm_state LDM绘图缓存状态
    // 输入SPMC通道：装甲检测批量帧、LDM光斑检测、LDM八边形位姿解算结果
    // 全局资源：Foxglove服务、相机标定、可视化配置、TF变换树、LDM几何参数
    // =========================================================================
    //add_system:节点           里面的lambda表达式里的参数是每个通道类型
    app.add_system<talos::pool_compute>(
        "foxglove_l1_image_pub",
        // 系统持久状态捕获：lambda每次执行复用这两个状态，不会重复创建编码器/缓存
        // mutable
        // 允许修改捕获的局部状态变量（QuantaPublisherState/LdmOverlayState内部有可修改成员）
        /*    video_state H.265 视频编码器状态（避免每帧重建编码器） ldm_state
           LDM（能量机关）数据缓存（保留上一帧数据用于叠加）*/
        [video_state = detail::QuantaPublisherState{}, ldm_state = detail::LdmOverlayState{}](
            // 输入通道1：装甲板神经网络检测批量帧（原图+所有装甲检测结果）
            talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
            // 输入通道2：LDM能量机关光斑blob原始检测数据
            talos::spmc<L2::ldm::LdmDetection, LdmDetectionChannelTopic> ldm_in,
            // 输入通道3：LDM八边形PnP位姿解算结果（3D坐标、重投影误差、可见立面）
            talos::spmc<L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> ldm_meas_in,

            // 全局只读资源：Foxglove可视化服务单例智能指针，提供消息发送、TF发布接口
            talos::res<std::shared_ptr<FoxgloveServer>> server,
            // 全局相机标定资源：内参矩阵、畸变系数、图像原始分辨率
            talos::res<CameraConfig> cam,
            // Foxglove全局配置：传输模式(Mcap/WebSocket)、视频编码参数、码率、分辨率上限
            talos::res<FoxgloveConfig> foxglove_cfg,
            // 当前敌方识别颜色，本函数未使用，[[maybe_unused]]消除未使用告警
            [[maybe_unused]] core::detecting_color detecting_color_,
            // TF坐标变换树：相机/底盘/世界坐标系位姿关系
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            // LDM八边形几何尺寸配置（外接圆、立面长度，用于投影轮廓绘制）
            /*1. 默认规则
            lambda 捕获的变量，在 lambda 内部默认是 const，不能修改里面的成员、不能赋值。
            不加 mutable，写 video_state.encoder.reset() 直接编译报错。
            2. mutable 作用
            解除捕获变量的 const 限制，允许在函数体内修改捕获的对象。*/
            talos::res<L2::ldm::LdmDetectorConfig> ldm_config) mutable {
            // 前置统一校验：Foxglove服务已初始化、输入通道存在可读数据
            // foxglove_ready：封装逻辑：服务非空+通道有数据，无数据直接跳过本轮渲染
            if (!foxglove_ready(*server, det_in)) {
                return;
            }

            // 读取装甲检测批量帧（包含原图+所有装甲检测框）
            auto batch = det_in.read();
            // 空帧 / 图像数据为空，无需渲染，直接退出
            if (!batch || batch->image.empty()) {
                return;
            }

            // 1. 同步发布当前帧全部TF坐标变换
            // 前端Foxglove依赖TF实现3D坐标、目标空间位置可视化
            (*server)->publish_tf(*tf_buffer, batch->timestamp_ns);

            // clone深拷贝，避免修改原始图像帧缓存，不影响其他系统读取原图
            cv::Mat img_bgr = batch->image.clone();

            // -------------------------- 绘制ROI有效检测区域矩形 --------------------------
            // 根据帧是否启用检测器ROI切换矩形颜色
            const cv::Scalar roi_color = tac::to_cv_bgr(
                batch->has_detector_roi ? tac::Image::ROI_VALID : tac::Image::ROI_MISSING);
            cv::rectangle(
                img_bgr, batch->detector_roi, roi_color, tac::Image::LINE_MEDIUM, cv::LINE_AA);

            // -------------------------- 绘制相机光心十字标记 --------------------------
            // 从相机内参取出主点cx cy（图像光学中心）
            const double cx = cam->camera_matrix(0, 2);
            const double cy = cam->camera_matrix(1, 2);
            // 在图像光学中心(主点cx,cy)绘制十字标记，便于前端对齐坐标系原点
            // 参数说明：
            //   img_bgr: 绘制画布（原图拷贝）
            //   cv::Point(cx, cy): 相机内参主点坐标（图像光学中心），浮点转整型像素点
            //   OPTICAL_CENTER: 十字标记颜色（BGR格式转换后的光学中心专属色）
            //   cv::MARKER_CROSS: 标记类型为十字形
            //   MARKER_SIZE: 十字标记尺寸（像素）
            //   LINE_MEDIUM: 十字线线宽
            cv::drawMarker(
                img_bgr, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                tac::to_cv_bgr(tac::Image::OPTICAL_CENTER), cv::MARKER_CROSS,
                tac::Image::MARKER_SIZE, tac::Image::LINE_MEDIUM);

            // -------------------------- 装甲板绘图配色定义 --------------------------
            const cv::Scalar box_color  = tac::to_cv_bgr(tac::Image::DETECTION_BOX);
            const cv::Scalar text_color = tac::to_cv_bgr(tac::Image::DETECTION_TEXT);
            // 四个装甲角点配色：RT右上红、LT左上绿、LB左下蓝、RB右下黄
            std::array<cv::Scalar, 4> corner_colors = {
                cv::Scalar(255, 0, 0),  // RT
                cv::Scalar(0, 255, 0),  // LT
                cv::Scalar(0, 0, 255),  // LB
                cv::Scalar(0, 255, 255) // RB
            };

            // -------------------------- 遍历绘制所有装甲检测结果 --------------------------
            for (const auto& det : batch->detections) {
                // 循环绘制4个角点 + 带箭头边线
                for (size_t j = 0; j < det.corners.size(); j++) {
                    auto pp1 = det.corners[j];
                    auto pp2 = det.corners[(j + 1) % 4];
                    // 绘制角点小圆点
                    cv::circle(
                        img_bgr, pp1, 2, corner_colors[(j + 1) % 4], tac::Image::LINE_MEDIUM);
                    // 带箭头边线，箭头大小随线段长度自适应
                    cv::arrowedLine(
                        img_bgr, pp1, pp2, corner_colors[(j + 1) % 4], tac::Image::LINE_MEDIUM,
                        cv::LINE_AA, 0, 10.0 / cv::norm(pp1 - pp2));
                }

                // 装甲名称/类型/敌方颜色文字
                cv::Point2f text_pos(det.rect.x, det.rect.y + det.rect.height + 20);
                std::string name_type = fmt::format(
                    "{} {} {}", magic_enum::enum_name(det.name), magic_enum::enum_name(det.type),
                    magic_enum::enum_name(det.color));
                cv::putText(
                    img_bgr, name_type, text_pos, cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                    text_color, tac::Image::TEXT_THIN, cv::LINE_AA);

                // 网络置信度文字
                cv::Point2f conf_pos(det.rect.x, det.rect.y + det.rect.height + 40);
                std::string conf_str = fmt::format("conf: {:.2f}", det.confidence);
                cv::putText(
                    img_bgr, conf_str, conf_pos, cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                    text_color, tac::Image::TEXT_THIN, cv::LINE_AA);
            }

            // -------------------------- LDM能量机关帧同步阈值配置 --------------------------
            // 允许图像帧与LDM检测帧最大时间偏差200ms，超过则不叠加LDM图层防止画面错位
            constexpr uint64_t kMaxLdmOverlaySkewNs = 200'000'000;
            // 读取当前最新LDM光斑检测、LDM位姿解算数据缓存
            const auto maybe_ldm_det  = ldm_in.read_current();
            const auto maybe_ldm_meas = ldm_meas_in.read_current();
            // 更新系统持久缓存ldm_state，缓存最新一帧LDM数据，无新数据则保留上一帧
            if (maybe_ldm_det) {
                ldm_state.latest_detection = *maybe_ldm_det;
            }
            if (maybe_ldm_meas) {
                ldm_state.latest_measurement = *maybe_ldm_meas;
            }

            // -------------------------- 判断LDM光斑检测帧是否可叠加 --------------------------
            // 同步规则：帧ID完全相同 / 时间戳差值小于200ms
            const auto use_ldm_det = ldm_state.latest_detection.has_value()
                                  && (ldm_state.latest_detection->frame_id == batch->frame_id
                                      || detail::frame_sync_ok(
                                          ldm_state.latest_detection->timestamp_ns,
                                          batch->timestamp_ns, kMaxLdmOverlaySkewNs));

            // 绘制LDM光斑、配对连线、中点标记
            if (use_ldm_det) {
                const auto& ldm_det           = *ldm_state.latest_detection;
                const cv::Scalar blob_color   = tac::to_cv_bgr(tac::Image::LDM_SECONDARY);
                const cv::Scalar pair_color   = tac::to_cv_bgr(tac::Image::LDM_PRIMARY);
                const cv::Scalar center_color = tac::to_cv_bgr(tac::Image::LDM_CENTER);
                cv::Rect2f ldm_rect           = ldm_det.rect;

                // 绘制单个光斑blob包围矩形
                for (const auto& blob : ldm_det.blobs) {
                    cv::rectangle(
                        img_bgr, blob.rect, blob_color, tac::Image::LINE_THIN, cv::LINE_AA);
                }

                // 遍历光斑配对，绘制上下光斑连线、端点、中点、配对编号
                for (size_t i = 0; i < ldm_det.pairs.size(); ++i) {
                    const auto& pair = ldm_det.pairs[i];
                    cv::line(
                        img_bgr, pair.top_center_px, pair.bottom_center_px, pair_color,
                        tac::Image::LINE_MEDIUM, cv::LINE_AA);
                    // 上端点圆点
                    cv::circle(
                        img_bgr, pair.top_center_px, 4, corner_colors[0], tac::Image::LINE_MEDIUM);
                    // 下端点圆点
                    cv::circle(
                        img_bgr, pair.bottom_center_px, 4, corner_colors[2],
                        tac::Image::LINE_MEDIUM);
                    // 配对中点实心圆
                    cv::circle(img_bgr, pair.midpoint_px, 3, center_color, -1);
                    // 配对编号文字
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

                // 绘制LDM整体包围框 + 配对数量文字
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

            // -------------------------- 判断LDM位姿解算帧是否可叠加 --------------------------
            const auto use_ldm_meas = ldm_state.latest_measurement.has_value()
                                   && (ldm_state.latest_measurement->frame_id == batch->frame_id
                                       || detail::frame_sync_ok(
                                           ldm_state.latest_measurement->timestamp_ns,
                                           batch->timestamp_ns, kMaxLdmOverlaySkewNs));
            if (use_ldm_meas) {
                const auto& ldm_meas = *ldm_state.latest_measurement;
                // 左上角打印配对总数、有效配对、深度质量、解算置信度
                cv::putText(
                    img_bgr,
                    fmt::format(
                        "LDM pairs={}/{} {} conf={:.2f}", ldm_meas.selected_pair_count,
                        ldm_meas.pair_count_total, ldm_meas.depth_quality, ldm_meas.confidence),
                    cv::Point(30, 32), cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_MEDIUM,
                    tac::to_cv_bgr(tac::Image::LDM_PRIMARY), tac::Image::TEXT_MEDIUM_PX,
                    cv::LINE_AA);

                // 打印八边形相机坐标系XYZ三维坐标（单位m）
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

                // 绘制八边形3D模型投影轮廓、可见立面信息
                const auto& selected_idx = ldm_meas.selected_candidate_idx;
                if (selected_idx.has_value() && *selected_idx >= 0
                    && static_cast<size_t>(*selected_idx) < ldm_meas.mesh_candidates.size()) {
                    const auto& selected =
                        ldm_meas.mesh_candidates[static_cast<size_t>(*selected_idx)];
                    // 投影轮廓点数量为16个，绘制八边形外框（背面虚线、正面实线）
                    if (selected.projected_outline_image.size() == 16) {
                        detail::draw_ldm_projected_outline(
                            img_bgr, selected, ldm_config->geometry, box_color);
                    }

                    // 绘制八边形图像中心十字标记
                    cv::drawMarker(
                        img_bgr, ldm_meas.center_image_px, tac::to_cv_bgr(tac::Image::LDM_CENTER),
                        cv::MARKER_CROSS, 18, tac::Image::LINE_MEDIUM);

                    // 拼接可见立面编号字符串
                    std::string faces_text;
                    for (size_t i = 0; i < selected.octagon_face_indices.size(); ++i) {
                        if (!faces_text.empty()) {
                            faces_text += ",";
                        }
                        faces_text += std::to_string(selected.octagon_face_indices[i]);
                    }
                    // 重投影误差格式化，非法数值显示n/a
                    const std::string rmse_text =
                        std::isfinite(selected.reprojection_rmse_px)
                            ? fmt::format("{:.2f}", selected.reprojection_rmse_px)
                            : "n/a";

                    // 在八边形包围框上方打印可见立面、重投影RMSE误差
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

            // -------------------------- 根据传输模式分两种推流逻辑 --------------------------
            if (foxglove_cfg->transport == FoxgloveTransport::Mcap) {
                // Mcap离线录包模式：使用Quanta H265硬件编码器输出视频流
                // video_state持久保存编码器，分辨率不变时复用编码器，无需重复创建
                detail::publish_quanta_video(
                    *(*server), video_state, foxglove_cfg->quanta, img_bgr, batch->timestamp_ns);
            } else {
                // WebSocket实时调试模式：单帧压缩为JPEG图片逐帧发送，延迟低
                detail::publish_jpeg_image(*(*server), img_bgr, batch->timestamp_ns);
            }

            // -------------------------- 发布相机标定消息给Foxglove前端 --------------------------
            // 把Eigen内参矩阵、畸变系数转为标准数组格式
            // 定义固定长度数组，存储相机内参矩阵（3×3，共9个double浮点元素）
            // std::array<double,9> 栈内存分配，长度固定，对应OpenCV相机内参矩阵3行3列9个参数
            std::array<double, 9> camera_matrix_arr;

            // std::copy_n：从源地址复制指定数量的数据到目标容器
            // 参数1：cam->camera_matrix.data() 源指针，OpenCV
            // Mat/矩阵的数据首地址，指向9个内参浮点数 参数2：9
            // 需要复制的元素总个数（3*3相机矩阵固定9个值） 参数3：camera_matrix_arr.begin()
            // 目标数组起始迭代器，写入到std::array中
            std::copy_n(cam->camera_matrix.data(), 9, camera_matrix_arr.begin());

            // 构建畸变系数std::vector<double>动态数组
            // vector构造函数重载：传入【起始迭代器/数据指针】、【末尾数据指针】，自动拷贝区间内所有元素
            // cam->distort_coefficient.data()：畸变系数数组首地址
            // cam->distort_coefficient.data() +
            // cam->distort_coefficient.size()：畸变系数尾部边界（不包含）
            // 自动适配任意长度畸变系数（k1,k2,p1,p2,k3等，长度不固定）
            std::vector<double> dist_coeffs(
                cam->distort_coefficient.data(),
                cam->distort_coefficient.data() + cam->distort_coefficient.size());

            // 发送CameraCalibration消息，前端可完成图像去畸变、3D点云投影
            (*server)->publish_camera_calibration(
                cam->width, cam->height, camera_matrix_arr, dist_coeffs, batch->timestamp_ns);
        });
}

} // namespace fcs::visualization::foxglove::systems