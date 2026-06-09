#include "L2_perception/rune/rune_systems.hpp"

#include "L2_perception/rune/rune_config.hpp"
#include "L2_perception/rune/rune_detector.hpp"
#include "L2_perception/rune/types.hpp"
#include "camera_config.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/types.hpp"
#include "frame.hpp"
#include "scheduler/scheduler.hpp"

#include <magic_enum.hpp>
#include <spdlog/spdlog.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <memory>
#include <utility>

namespace fcs::rune {
namespace {

[[nodiscard]] auto to_rectf(const cv::Rect2f& r) noexcept -> RectF {
    return RectF{.x = r.x, .y = r.y, .w = r.width, .h = r.height};
}

[[nodiscard]] auto encode_jpeg(const cv::Mat& img) -> std::vector<uint8_t> {
    if (img.empty()) {
        return {};
    }

    std::vector<uint8_t> encoded;
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", img, encoded, params)) {
        return {};
    }
    return encoded;
}

[[nodiscard]] auto ns_to_sec(uint64_t ns) -> double { return static_cast<double>(ns) * 1e-9; }

} // namespace

void register_rune_detection_systems(talos::Scheduler& scheduler, RuneDetectorConfig&& config) {
    scheduler.world().insert_resource(config);

    scheduler.add_system<talos::pool_compute>(
        "l2_rune_detector",
        [detector = std::make_shared<::fyt::rune::RuneDetector>(config)](
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            talos::spmc_mut<RuneObservation, RuneObservationChannelTopic> obs_out,
            talos::spmc_mut<RuneDebugFrame, RuneDebugFrameChannelTopic> debug_out,
            talos::res<RuneDetectorConfig> cfg, talos::res<CameraConfig> cam_cfg,
            core::capabilities cap, core::detecting_color color,
            talos::res<fast_tf::CoordinateSystem> tf_system) mutable {
            if (!core::capable(*cap, core::Capability::Rune)) {
                return;
            }
            auto frame = image_in.read();
            if (!frame) {
                return;
            }

            // self color for rune
            auto detect_color = *color == ArmorColor::Red ? ArmorColor::Blue : ArmorColor::Red;

            RuneObservation out;
            out.timestamp_ns = frame->timestamp_ns;
            out.frame_id     = frame->frame_id;
            out.valid        = false;

            RuneDebugFrame dbg;
            dbg.timestamp_ns = frame->timestamp_ns;
            dbg.frame_id     = frame->frame_id;

            const auto reversed = detector->detect(frame->image, detect_color);

            if (!detector->targets.empty() && detector->rcenter.center != cv::Point2f(0, 0)) {
                detector->setKeyPoints();
            }

            if (cfg->draw) {
                cv::Mat arrow_vis;
                if (!detector->arrows.empty()) {
                    // 绘制 arrow 可视化：绘制轮廓和边界框
                    cv::cvtColor(detector->arrowImg, arrow_vis, cv::COLOR_GRAY2BGR);
                    // 用绿色绘制所有检测到的 arrow 轮廓
                    for (const auto& arrow : detector->arrows) {

                        // 绘制旋转矩形
                        cv::RotatedRect box = arrow.rotated;

                        cv::Point2f vertices[4];
                        box.points(vertices);
                        for (int j = 0; j < 4; ++j) {
                            cv::line(
                                arrow_vis, vertices[j], vertices[(j + 1) % 4],
                                cv::Scalar(255, 0, 255), 2);
                        }
                        // 绘制中心点

                        cv::circle(arrow_vis, arrow.center, 3, cv::Scalar(0, 255, 255), -1);
                    }
                    // // 显示检测状态
                    // cv::putText(
                    //     arrow_vis,
                    //     cv::format(
                    //         "Arrows: %zu | Status: %d", detector->arrows.size(),
                    //         static_cast<int>(detector->status)),
                    //     cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255,
                    //     255), 2);
                }

                if (!detector->targets.empty()) {
                    // 绘制靶的四个 keypoint（用不同颜色）
                    for (size_t i = 0; i < detector->targets.size(); ++i) {
                        const auto& target = detector->targets[i];
                        cv::circle(
                            arrow_vis, detector->targets[i].center, 6, cv::Scalar(255, 255, 0), -1);

                        if (target.initKey) {
                            const auto& kp = target.keypnt;
                            // 转换坐标到全局（考虑 global ROI 偏移）

                            // 用不同颜色绘制四个角点：左上(蓝)、左下(绿)、右下(红)、右上(黄)
                            cv::Scalar color_lu(255, 0, 0);   // 蓝色
                            cv::Scalar color_ru(0, 255, 0);   // 绿色
                            cv::Scalar color_rd(0, 0, 255);   // 红色
                            cv::Scalar color_ld(0, 255, 255); // 黄色

                            cv::circle(arrow_vis, kp.lu, 6, color_lu, -1);
                            cv::circle(arrow_vis, kp.ru, 6, color_ru, -1);
                            cv::circle(arrow_vis, kp.rd, 6, color_rd, -1);
                            cv::circle(arrow_vis, kp.ld, 6, color_ld, -1);

                            if (detector->targets.size() == 1 && target.other) {

                                cv::circle(arrow_vis, target.other_point2f[0], 6, color_lu, -1);
                                cv::circle(arrow_vis, target.other_point2f[1], 6, color_ru, -1);
                                cv::circle(arrow_vis, target.other_point2f[2], 6, color_rd, -1);
                                cv::circle(arrow_vis, target.other_point2f[3], 6, color_ld, -1);
                            }

                            // // 绘制角点标签
                            // cv::putText(
                            //     arrow_vis, "LU", kp.lu  + cv::Point2f(-10, -10),
                            //     cv::FONT_HERSHEY_SIMPLEX, 0.4, color_lu, 1);
                            // cv::putText(
                            //     arrow_vis, "RU", kp.ru  + cv::Point2f(5, -10),
                            //     cv::FONT_HERSHEY_SIMPLEX, 0.4, color_ru, 1);
                            // cv::putText(
                            //     arrow_vis, "RD", kp.rd  + cv::Point2f(5, 15),
                            //     cv::FONT_HERSHEY_SIMPLEX, 0.4, color_rd, 1);
                            // cv::putText(
                            //     arrow_vis, "LD", kp.ld  + cv::Point2f(-10, 15),
                            //     cv::FONT_HERSHEY_SIMPLEX, 0.4, color_ld, 1);

                            // 绘制连接线显示四边形轮廓
                            std::vector<cv::Point> quad = {kp.lu, kp.ru, kp.rd, kp.ld};
                            cv::polylines(arrow_vis, quad, true, cv::Scalar(255, 255, 255), 1);
                        }
                    }
                }

                if (!detector->rCenterImg.empty()) {
                    // 绘制中心区域矩形
                    if (detector->rcenter.center != cv::Point2f(0, 0)) {
                        cv::circle(
                            arrow_vis, detector->rcenter.center, 5, cv::Scalar(0, 0, 255), -1);
                    }
                }
                dbg.arrow_jpeg = encode_jpeg(arrow_vis.empty() ? detector->arrowImg : arrow_vis);

                // // 画重投影对比图：检测点(红) vs 重投影点(绿) + 四个 keypoint
                cv::Mat reproj_vis;
                // if (!detector->targetImg.empty()&&!detector->debug_detected_points_.empty()) {
                //     cv::cvtColor(detector->targetImg, reproj_vis, cv::COLOR_GRAY2BGR);
                //     for (const auto& pt : detector->debug_detected_points_) {
                //         cv::circle(reproj_vis, pt, 4, cv::Scalar(0, 0, 255), -1); // 红色：检测点
                //     }
                //     for (const auto& pt : detector->debug_reprojected_points_) {
                //         cv::circle(reproj_vis, pt, 4, cv::Scalar(0, 255, 0), -1);
                //         //绿色：重投影点
                //     }
                //     // 画连线显示误差
                //     for (size_t i = 0; i < detector->debug_detected_points_.size()
                //                        && i < detector->debug_reprojected_points_.size();
                //          ++i) {
                //         cv::line(
                //             reproj_vis, detector->debug_detected_points_[i],
                //             detector->debug_reprojected_points_[i], cv::Scalar(255, 255, 0), 1);
                //     }
                //     // 显示平均误差
                //     cv::putText(
                //         reproj_vis, cv::format("Reproj Err: %.2f px",
                //         detector->debug_reproj_error_), cv::Point(10, 25),
                //         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                // }

                dbg.target_jpeg =
                    encode_jpeg(reproj_vis.empty() ? detector->targetImg : reproj_vis);

                // // 绘制 rcenter 可视化：中心区域 + 检测框
                cv::Mat rcenter_vis;
                // if (!detector->rCenterImg.empty()) {
                //     // 绘制中心区域矩形
                //     if (detector->rcenter.center != cv::Point2f(0, 0)) {
                //         cv::cvtColor(detector->rCenterImg, rcenter_vis, cv::COLOR_GRAY2BGR);
                //         cv::Rect rc = detector->rcenter.boundingRect;
                //         cv::rectangle(rcenter_vis, rc.tl(), rc.br(), cv::Scalar(0, 255, 0), 2);
                //         // 绘制中心点
                //         cv::circle(
                //             rcenter_vis, detector->rcenter.center, 5, cv::Scalar(0, 0, 255), -1);
                //         // 显示信息
                //         cv::putText(
                //             rcenter_vis, "R Center", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                //             0.6, cv::Scalar(255, 255, 0), 2);
                //     }
                // }
                dbg.rcenter_jpeg =
                    encode_jpeg(rcenter_vis.empty() ? detector->rCenterImg : rcenter_vis);
            }

            dbg.detect_reversed = reversed;
            dbg.status_code     = static_cast<uint32_t>(detector->status);
            dbg.arrows_count    = static_cast<uint32_t>(detector->arrows.size());
            dbg.targets_count   = static_cast<uint32_t>(detector->targets.size());
            dbg.center_roi      = to_rectf(detector->centerRoi);
            dbg.target_rois.reserve(detector->targetROIs.size());
            for (const auto& r : detector->targetROIs) {
                dbg.target_rois.emplace_back(to_rectf(r));
            }

            if (!dbg.detect_reversed) {
                // SPDLOG_WARN(
                //     "rune detector : detect fail :{}", magic_enum::enum_name(detector->status));
                debug_out.write(std::move(dbg));
                // obs_out.write(out);
                return;
            }

            auto tf_odom_camera = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            if (!tf_odom_camera) {
                SPDLOG_DEBUG(
                    "rune: TF lookup failed at {}: {}", frame->timestamp_ns,
                    tf_odom_camera.error());
                dbg.tf_ok = false;
                debug_out.write(std::move(dbg));
                // obs_out.write(out);
                return;
            }
            dbg.tf_ok = true;

            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
            cv::eigen2cv(cam_cfg->camera_matrix, camera_matrix);
            cv::eigen2cv(cam_cfg->distort_coefficient, dist_coeffs);

            const auto odom_T_cam      = tf_odom_camera.value();
            const double timestamp_sec = ns_to_sec(frame->timestamp_ns);

            auto pose_result =
                detector->solve(camera_matrix, dist_coeffs, timestamp_sec, odom_T_cam);
            dbg.solve_ok = pose_result.has_value();
            if (!pose_result) {
                debug_out.write(std::move(dbg));
                // obs_out.write(out);
                SPDLOG_ERROR("solve rune: {}", pose_result.error());
                return;
            }

            const auto odom_T_center = *pose_result;
            out.valid                = true;
            out.r_center_odom        = odom_T_center;

            // 传递所有检测到的靶（大符 2 个，小符 1 个），L3 靶数投票依赖 .size()
            // 叶片在竖直平面 (Y-Z) 内绕水平轴 (X) 旋转
            const double r2pancenter = fyt::rune::RUNE_R2PANCENTER;
            for (size_t i = 0; i < detector->targets.size(); ++i) {
                const double dtheta = detector->targets[i].droll;
                const double sr     = std::sin(dtheta);
                const double cr     = std::cos(dtheta);
                const Eigen::Vector3d blade_center_rotated(
                    0.0, -r2pancenter * sr, r2pancenter * cr);

                out.target_positions_odom.emplace_back(
                    odom_T_center.rotation() * blade_center_rotated
                    + out.r_center_odom.translation());

                const Eigen::Quaterniond roll_quat(
                    std::cos(dtheta / 2), std::sin(dtheta / 2), 0.0, 0.0);
                out.target_quats_odom.emplace_back(odom_T_center.quaternion() * roll_quat);
            }

            dbg.observation_valid = true;
            debug_out.write(std::move(dbg));
            obs_out.write(out);
        });
}

} // namespace fcs::rune
