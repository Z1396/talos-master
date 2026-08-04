#include "L2_perception/rune/rune_systems.hpp"

// 小符检测器、配置、数据结构
#include "L2_perception/rune/rune_config.hpp"
#include "L2_perception/rune/rune_detector.hpp"
#include "L2_perception/rune/types.hpp"
// 相机标定内参配置
#include "camera_config.hpp"
// 全局消息通道Topic定义
#include "core/channel_topics.hpp"
// 运行时能力管理、系统生命周期
#include "core/runtime.hpp"
// 全局通用数据结构
#include "core/types.hpp"
// 图像帧结构
#include "frame.hpp"
// DAG任务调度器 talos
#include "scheduler/scheduler.hpp"

#include <magic_enum.hpp>    // 枚举转字符串，打印状态码
#include <spdlog/spdlog.h>   // 日志系统

#include <opencv2/core/eigen.hpp> // Eigen矩阵 ↔ OpenCV Mat互相转换
#include <opencv2/imgcodecs.hpp>  // imencode JPEG压缩

#include <cmath>
#include <memory>
#include <utility>

namespace fcs::rune {
namespace {
// 匿名命名空间：仅本文件内可用，防止全局命名污染

/// @brief OpenCV Rect2f → 框架自定义RectF结构体
[[nodiscard]] auto to_rectf(const cv::Rect2f& r) noexcept -> RectF {
    return RectF{.x = r.x, .y = r.y, .w = r.width, .h = r.height};
}

/// @brief 将图像压缩为JPEG二进制字节流，用于调试画面网络传输
/// @param img 原始图像
/// @return 压缩后的uint8_t数组
[[nodiscard]] auto encode_jpeg(const cv::Mat& img) -> std::vector<uint8_t> {
    if (img.empty()) {
        return {};
    }
    std::vector<uint8_t> encoded;
    // JPEG画质80，平衡体积与清晰度
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
    // 内存编码，无需写入磁盘
    if (!cv::imencode(".jpg", img, encoded, params)) {
        return {};
    }
    return encoded;
}

/// @brief 纳秒时间戳转为秒，用于解算时序计算
[[nodiscard]] auto ns_to_sec(uint64_t ns) -> double {
    return static_cast<double>(ns) * 1e-9;
}

} // namespace

/**
 * @brief 向全局调度器注册【小符检测任务系统】
 * @param scheduler talos DAG调度器实例
 * @param config 小符检测器配置参数
 */
void register_rune_detection_systems(talos::Scheduler& scheduler, RuneDetectorConfig&& config) {
    // 把小符配置存入调度器全局资源池，整个系统各处均可读取
    scheduler.world().insert_resource(config);

    // 注册一个并行计算系统：线程池调度运行
    scheduler.add_system<talos::pool_compute>(
        // 系统名称，日志/监控区分各个模块
        "l2_rune_detector",
        // 系统执行入口lambda，捕获检测器实例
        [detector = std::make_shared<::fyt::rune::RuneDetector>(config)](
            // 输入：多生产者单消费者通道，读取图像帧（只读）
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            // 输出：小符观测结果，可修改写入
            talos::spmc_mut<RuneObservation, RuneObservationChannelTopic> obs_out,
            // 输出：可视化调试画面
            talos::spmc_mut<RuneDebugFrame, RuneDebugFrameChannelTopic> debug_out,
            // 只读资源：小符配置
            talos::res<RuneDetectorConfig> cfg,
            // 只读资源：相机内参、畸变系数、分辨率
            talos::res<CameraConfig> cam_cfg,
            // 系统能力标记，用来开关功能
            core::capabilities cap,
            // 当前己方颜色（红/蓝）
            core::detecting_color color,
            // TF坐标变换系统：camera ↔ odom里程计坐标系转换
            talos::res<fast_tf::CoordinateSystem> tf_system) mutable
        {
            // 能力判断：如果运行配置关闭了小符识别，直接退出，不执行检测
            if (!core::capable(*cap, core::Capability::Rune)) {
                return;
            }

            // 阻塞读取一帧图像，无图像则返回
            auto frame = image_in.read();
            if (!frame) {
                return;
            }

            // 逻辑：我方是红色，则识别蓝色小符；我方蓝色识别红色小符（敌方能量机关颜色）
            auto detect_color = *color == ArmorColor::Red ? ArmorColor::Blue : ArmorColor::Red;

            // 初始化观测结果结构体，默认无效
            RuneObservation out;
            out.timestamp_ns = frame->timestamp_ns;
            out.frame_id     = frame->frame_id;
            out.valid        = false;

            // 初始化调试帧结构体
            RuneDebugFrame dbg;
            dbg.timestamp_ns = frame->timestamp_ns;
            dbg.frame_id     = frame->frame_id;

            // ========== 核心检测：送入检测器做箭头、靶区、中心识别 ==========
            // reversed：是否成功检测到完整小符结构
            const auto reversed = detector->detect(frame->image, detect_color);

            // 如果检测到有效靶标、中心不为原点，提取关键点用于PnP
            if (!detector->targets.empty() && detector->rcenter.center != cv::Point2f(0, 0)) {
                detector->setKeyPoints();
            }

            // ========== 开启绘图调试模式时，绘制可视化画面 ==========
            if (cfg->draw) {
                cv::Mat arrow_vis;
                // 存在箭头轮廓，则在灰度箭头图上绘制框与中心点
                if (!detector->arrows.empty()) {
                    cv::cvtColor(detector->arrowImg, arrow_vis, cv::COLOR_GRAY2BGR);
                    // 遍历所有箭头，绘制旋转包围矩形
                    for (const auto& arrow : detector->arrows) {
                        cv::RotatedRect box = arrow.rotated;
                        cv::Point2f vertices[4];
                        box.points(vertices);
                        // 四条边线连成矩形
                        for (int j = 0; j < 4; ++j) {
                            cv::line(
                                arrow_vis, vertices[j], vertices[(j + 1) % 4],
                                cv::Scalar(255, 0, 255), 2);
                        }
                        // 绘制箭头中心点（黄实心圆点）
                        cv::circle(arrow_vis, arrow.center, 3, cv::Scalar(0, 255, 255), -1);
                    }
                }

                // 绘制小符靶的四个角关键点
                if (!detector->targets.empty()) {
                    for (size_t i = 0; i < detector->targets.size(); ++i) {
                        const auto& target = detector->targets[i];
                        // 靶中心黄色圆点
                        cv::circle(
                            arrow_vis, detector->targets[i].center, 6, cv::Scalar(255, 255, 0), -1);

                        // 初始化关键点后绘制四角特征点
                        if (target.initKey) {
                            const auto& kp = target.keypnt;
                            // 四角配色：左上蓝、左下黄、右下红、右上绿
                            cv::Scalar color_lu(255, 0, 0);
                            cv::Scalar color_ru(0, 255, 0);
                            cv::Scalar color_rd(0, 0, 255);
                            cv::Scalar color_ld(0, 255, 255);

                            cv::circle(arrow_vis, kp.lu, 6, color_lu, -1);
                            cv::circle(arrow_vis, kp.ru, 6, color_ru, -1);
                            cv::circle(arrow_vis, kp.rd, 6, color_rd, -1);
                            cv::circle(arrow_vis, kp.ld, 6, color_ld, -1);

                            // 存在第二个配对靶标，同步绘制四点
                            if (detector->targets.size() == 1 && target.other) {
                                cv::circle(arrow_vis, target.other_point2f[0], 6, color_lu, -1);
                                cv::circle(arrow_vis, target.other_point2f[1], 6, color_ru, -1);
                                cv::circle(arrow_vis, target.other_point2f[2], 6, color_rd, -1);
                                cv::circle(arrow_vis, target.other_point2f[3], 6, color_ld, -1);
                            }

                            // 四条边连成四边形
                            std::vector<cv::Point> quad = {kp.lu, kp.ru, kp.rd, kp.ld};
                            cv::polylines(arrow_vis, quad, true, cv::Scalar(255, 255, 255), 1);
                        }
                    }
                }

                // 绘制小符旋转中心红点
                if (!detector->rCenterImg.empty()) {
                    if (detector->rcenter.center != cv::Point2f(0, 0)) {
                        cv::circle(
                            arrow_vis, detector->rcenter.center, 5, cv::Scalar(0, 0, 255), -1);
                    }
                }
                // 将箭头可视化画面压缩为JPEG存入调试帧
                dbg.arrow_jpeg = encode_jpeg(arrow_vis.empty() ? detector->arrowImg : arrow_vis);

                // 【注释块】重投影误差可视化：红色检测点、绿色PnP重投影点，用来排查标定精度
                cv::Mat reproj_vis;
                dbg.target_jpeg =
                    encode_jpeg(reproj_vis.empty() ? detector->targetImg : reproj_vis);

                // 【注释块】中心ROI可视化绘制
                cv::Mat rcenter_vis;
                dbg.rcenter_jpeg =
                    encode_jpeg(rcenter_vis.empty() ? detector->rCenterImg : rcenter_vis);
            }

            // 填充调试信息字段
            dbg.detect_reversed = reversed;
            dbg.status_code     = static_cast<uint32_t>(detector->status);
            dbg.arrows_count    = static_cast<uint32_t>(detector->arrows.size());
            dbg.targets_count   = static_cast<uint32_t>(detector->targets.size());
            dbg.center_roi      = to_rectf(detector->centerRoi);
            // 把所有靶标ROI存入调试结构
            dbg.target_rois.reserve(detector->targetROIs.size());
            for (const auto& r : detector->targetROIs) {
                dbg.target_rois.emplace_back(to_rectf(r));
            }

            // 分支1：检测失败，只发布调试帧，不发布有效观测
            if (!dbg.detect_reversed) {
                debug_out.write(std::move(dbg));
                return;
            }

            // ========== 坐标变换：查询 odom ↔ 相机光心坐标系变换矩阵 ==========
            auto tf_odom_camera = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            // TF查找失败（云台运动过快、时间同步偏差）
            if (!tf_odom_camera) {
                SPDLOG_DEBUG(
                    "rune: TF lookup failed at {}: {}", frame->timestamp_ns,
                    tf_odom_camera.error());
                dbg.tf_ok = false;
                debug_out.write(std::move(dbg));
                return;
            }
            dbg.tf_ok = true;

            // Eigen内参矩阵 → OpenCV Mat，供给solvePnP使用
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
            cv::eigen2cv(cam_cfg->camera_matrix, camera_matrix);
            cv::eigen2cv(cam_cfg->distort_coefficient, dist_coeffs);

            const auto odom_T_cam      = tf_odom_camera.value();
            const double timestamp_sec = ns_to_sec(frame->timestamp_ns);

            // ========== PnP解算：求出【小符中心在里程计坐标系下的位姿 odom_T_center】 ==========
            auto pose_result =
                detector->solve(camera_matrix, dist_coeffs, timestamp_sec, odom_T_cam);
            dbg.solve_ok = pose_result.has_value();
            // PnP求解失败，上报调试信息后退出
            if (!pose_result) {
                debug_out.write(std::move(dbg));
                SPDLOG_ERROR("solve rune: {}", pose_result.error());
                return;
            }

            // 拿到小符中心在世界(odom)坐标系的位姿
            const auto odom_T_center = *pose_result;
            out.valid                = true;
            out.r_center_odom        = odom_T_center;

            // ========== 推算每个旋转叶片在里程计下的三维坐标 & 姿态 ==========
            // RUNE_R2PANCENTER：小符中心到装甲叶片的固定物理距离（毫米/米，实体建模参数）
            const double r2pancenter = fyt::rune::RUNE_R2PANCENTER;
            for (size_t i = 0; i < detector->targets.size(); ++i) {
                // droll：当前叶片相对基准角度
                const double dtheta = detector->targets[i].droll;
                const double sr     = std::sin(dtheta);
                const double cr     = std::cos(dtheta);

                // 在小符自身坐标系下，旋转后的叶片中心点
                const Eigen::Vector3d blade_center_rotated(
                    0.0, -r2pancenter * sr, r2pancenter * cr);

                // 坐标变换：小符坐标系 → odom里程计坐标系
                out.target_positions_odom.emplace_back(
                    odom_T_center.rotation() * blade_center_rotated
                    + out.r_center_odom.translation());

                // 叶片自身旋转四元数
                const Eigen::Quaterniond roll_quat(
                    std::cos(dtheta / 2), std::sin(dtheta / 2), 0.0, 0.0);
                // 叠加小符整体旋转，得到叶片在odom下最终姿态
                out.target_quats_odom.emplace_back(odom_T_center.quaternion() * roll_quat);
            }

            // 调试帧标记观测有效，发布调试包与正式观测数据包
            dbg.observation_valid = true;
            debug_out.write(std::move(dbg));
            obs_out.write(out);
        });
}

} // namespace fcs::rune