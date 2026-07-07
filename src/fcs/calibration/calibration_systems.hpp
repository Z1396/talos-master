#pragma once
// 标定板抽象接口、工厂创建头文件
#include "calibration_board.hpp"
// 顶层标定总配置 CalibrationConfig
#include "calibration_config.hpp"
// 标定基础类型、枚举、通道Topic类型、状态结构体
#include "calibration_types.hpp"
// ChArUco标定板子检测器实现
#include "charuco_detector.hpp"
// 棋盘格标定板子检测器实现
#include "chessboard_detector.hpp"
// 手眼标定求解器实现
#include "handeye_calibrator.hpp"
// 内参标定求解器实现
#include "intrinsic_calibrator.hpp"

// L1层传感器数据包结构定义
#include "L1_sensor/parcel.hpp"
// 全局通道、消息Topic常量定义
#include "core/channel_topics.hpp"
// Foxglove Web可视化服务
#include "foxglove_server.hpp"
// Foxglove可视化消息结构、时间戳转换工具
#include "foxglove_types.hpp"
// 图像帧数据结构 ImageFrame
#include "frame.hpp"
// Talos实时调度器、系统注册API
#include "scheduler/scheduler.hpp"

// 智能指针 std::shared_ptr
#include <memory>
// OpenCV 标定核心函数 solvePnP、投影、畸变相关
#include <opencv2/calib3d.hpp>
// OpenCV Eigen矩阵互转工具
#include <opencv2/core/eigen.hpp>
// OpenCV 图像编码保存
#include <opencv2/imgcodecs.hpp>
// OpenCV 图像处理、绘图、文字渲染
#include <opencv2/imgproc.hpp>
// spdlog 日志打印
#include <spdlog/spdlog.h>
// 标准线程休眠
#include <thread>
// 通用工具函数（fmt、移动、类型转换等）
#include <utility.hpp>

namespace fcs::calibration {

/**
 * @brief 向Talos实时调度器注册全套标定业务系统
 * 模板函数，适配调度器应用类型
 * 注册4套独立实时任务系统：角点检测、内参样本采集、手眼样本采集、Foxglove可视化
 * @tparam App 调度器顶层应用类型（talos::scheduler::Scheduler）
 * @param app 调度器实例
 * @param config 全局标定配置共享指针
 * @param board 标定板检测器多态接口（棋盘格/ChArUco）
 * @param intrinsic_calibrator 内参采集/求解器
 * @param handeye_calibrator 手眼标定采集/求解器
 * @param intrinsic_result 已求解完成的相机内参结果（用于手眼PnP）
 */
template <typename App>
void register_calibration_systems(
    talos::scheduler::Scheduler& app, std::shared_ptr<CalibrationConfig> config,
    std::shared_ptr<CalibrationBoard> board,
    std::shared_ptr<IntrinsicCalibrator> intrinsic_calibrator,
    std::shared_ptr<HandEyeCalibrator> handeye_calibrator,
    std::shared_ptr<IntrinsicResult> intrinsic_result) {

    // ====================== 系统1：标定板角点检测池化计算系统 ======================
    // 类型：pool_compute 线程池异步计算，不阻塞主线程
    // 功能：接收图像帧，检测标定板角点，输出角点检测结果
    app.add_system<talos::pool_compute>(
        "calibration_corner_detector",
        [board](
            // 输入：图像帧多生产者单消费者通道
            talos::spmc<ImageFrame, ImageChannelTopic> img_in,
            // 输出：角点检测结果，可修改通道
            talos::spmc_mut<CornerDetection, CalibrationCornerChannelTopic> corner_out,
            // 共享只读资源：全局标定状态
            talos::res<std::shared_ptr<CalibrationStatus>> status) {
            // 无新图像帧直接返回
            if (!img_in.has_new()) {
                return;
            }

            // 读取最新图像帧
            auto frame = img_in.read();
            // 帧为空 / 图像无数据直接跳过
            if (!frame || frame->image.empty()) {
                return;
            }

            // 仅在【采集中】状态执行检测，空闲/求解阶段不检测
            if ((*status)->state != CalibrationState::Capturing) {
                return;
            }

            // 调用标定板多态detect接口执行角点检测
            auto detection = board->detect(frame->image, frame->timestamp_ns);
            // 检测成功则写入角点输出通道
            if (detection) {
                corner_out.write(std::move(*detection));
                SPDLOG_DEBUG("Corner detection successful");
            }
        });

    // ====================== 系统2：内参标定样本采集池化计算系统 ======================
    // 功能：接收角点结果，校验姿态多样性，满足条件加入内参样本集
    app.add_system<talos::pool_compute>(
        "calibration_intrinsic_collector",
        [intrinsic_calibrator, config](
            talos::spmc<CornerDetection, CalibrationCornerChannelTopic> corner_in,
            // 可修改共享资源：标定状态（更新样本计数）
            talos::res_mut<std::shared_ptr<CalibrationStatus>> status) {
            // 无新角点数据直接返回
            if (!corner_in.has_new()) {
                return;
            }

            // 仅内参模式 / 全量标定模式才采集内参样本
            if (config->mode != CalibrationMode::Intrinsic
                && config->mode != CalibrationMode::Full) {
                return;
            }

            // 仅采集中状态处理样本
            if ((*status)->state != CalibrationState::Capturing) {
                return;
            }

            // 读取角点检测结果
            auto detection = corner_in.read();
            // 检测失败无有效角点直接丢弃
            if (!detection || !detection->success) {
                return;
            }

            // 模拟采集间隔冷却3秒（真实业务替换为配置里的interval）
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(3000ms);
            // 校验当前帧姿态与已有样本是否足够差异，姿态重复则丢弃
            if (!intrinsic_calibrator->is_diverse_enough(*detection)) {
                SPDLOG_DEBUG("Sample rejected: not diverse enough");
                return;
            }

            // 将有效角点样本加入内参求解器样本库
            if (auto result = intrinsic_calibrator->add_sample(*detection); !result) {
                SPDLOG_WARN("Failed to add sample: {}", result.error());
                return;
            }

            // 更新全局状态当前采集样本数
            (*status)->sample_count = intrinsic_calibrator->sample_count();
            SPDLOG_INFO(
                "Intrinsic sample collected: {}/{}", (*status)->sample_count,
                (*status)->target_samples);
        });

    // ====================== 系统3：手眼标定样本采集池化计算系统 ======================
    // 功能：结合TF机器人末端位姿 + 图像角点，求解板到相机位姿，存入手眼样本
    app.add_system<talos::pool_compute>(
        "calibration_handeye_collector",
        [handeye_calibrator, intrinsic_result, config](
            talos::spmc<CornerDetection, CalibrationCornerChannelTopic> corner_in,
            // 只读共享资源：TF坐标变换缓存
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            talos::res_mut<std::shared_ptr<CalibrationStatus>> status) {
            // 无新角点数据直接返回
            if (!corner_in.has_new()) {
                return;
            }

            // 仅手眼模式 / 全量标定模式采集手眼样本
            if (config->mode != CalibrationMode::Handeye && config->mode != CalibrationMode::Full) {
                return;
            }

            if ((*status)->state != CalibrationState::Capturing) {
                return;
            }

            auto detection = corner_in.read();
            if (!detection || !detection->success) {
                return;
            }

            // 查询TF：odom世界坐标系到云台末端gimbal_pitch变换
            auto tf_result = fast_tf::lookup<fast_tf::odom, fast_tf::gimbal_pitch>(
                *tf_buffer, detection->timestamp_ns);

            // 无有效TF变换丢弃样本
            if (!tf_result) {
                SPDLOG_WARN("TF lookup failed for handeye sample");
                return;
            }

            // 校验机器人姿态多样性，重复姿态不采集
            if (!handeye_calibrator->is_diverse_enough(*tf_result)) {
                SPDLOG_DEBUG("Handeye sample rejected: not diverse enough");
                return;
            }

            // 从已求解内参构造OpenCV相机矩阵、畸变系数
            cv::Mat camera_matrix, dist_coeffs;
            // Eigen矩阵转OpenCV Mat
            cv::eigen2cv(intrinsic_result->camera_matrix, camera_matrix);

            // 5阶径向+切向畸变数组初始化
            dist_coeffs = cv::Mat(1, 5, CV_64F);
            for (int i = 0; i < 5; ++i) {
                dist_coeffs.at<double>(0, i) = intrinsic_result->distort_coefficient(0, i);
            }

            // PnP求解：标定板三维点 + 图像二维角点 → 板在相机下的旋转平移
            cv::Vec3d rvec, tvec;
            bool pnp_success = cv::solvePnP(
                detection->object_points, detection->image_points, camera_matrix, dist_coeffs, rvec,
                tvec);

            // PnP求解失败丢弃样本
            if (!pnp_success) {
                SPDLOG_WARN("PnP solve failed");
                return;
            }

            // OpenCV旋转平移向量转换为ROS标准位姿
            const auto board_pose = HandEyeCalibrator::opencv_to_ros(rvec, tvec);

            // 机器人末端位姿 + 板到相机位姿 存入手眼样本库
            if (auto result =
                    handeye_calibrator->add_sample(*tf_result, board_pose, detection->timestamp_ns);
                !result) {
                SPDLOG_WARN("Failed to add handeye sample: {}", result.error());
                return;
            }

            // 更新全局采集计数
            (*status)->sample_count = handeye_calibrator->sample_count();
            SPDLOG_INFO(
                "Handeye sample collected: {}/{}", (*status)->sample_count,
                (*status)->target_samples);
        });

    // ====================== 系统4：Foxglove可视化定频渲染系统 ======================
    // 类型：fixed_rate<30> 固定30Hz定时执行，推送可视化图像到Web
    app.add_system<talos::fixed_rate<30>>(
        "calibration_visualization",
        [board, config](
            talos::spmc<CornerDetection, CalibrationCornerChannelTopic> corner_in,
            talos::spmc<ImageFrame, ImageChannelTopic> img_in,
            talos::res<std::shared_ptr<visualization::FoxgloveServer>> server,
            talos::res<std::shared_ptr<CalibrationStatus>> status) {
            // Foxglove服务未初始化直接退出
            if (!(*server)->is_initialized()) {
                return;
            }

            cv::Mat vis_img;
            timestamp_ns_t ts = 0;

            // 优先使用带角点绘制的检测图
            if (corner_in.has_new()) {
                auto detection = corner_in.read();
                if (detection && detection->success) {
                    // 调用标定板绘制接口，在原图叠加角点
                    vis_img = board->draw_corners(detection->image, *detection);
                    ts      = detection->timestamp_ns;
                }
            } else if (img_in.has_new()) {
                // 无新检测结果，使用原始相机图像
                auto frame = img_in.read();
                if (frame && !frame->image.empty()) {
                    vis_img = frame->image.clone();
                    ts      = frame->timestamp_ns;
                }
            }

            // 无图像不推送
            if (vis_img.empty()) {
                return;
            }

            // 在图像左上角叠加采集样本计数文字
            cv::putText(
                vis_img,
                fmt::format("Samples: {}/{}", (*status)->sample_count, (*status)->target_samples),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // JPEG压缩图像，85画质
            std::vector<uint8_t> compressed;
            std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 85};
            if (!cv::imencode(".jpg", vis_img, compressed, params)) {
                return;
            }

            // 构造Foxglove图像消息
            visualization::CalibrationImageMessage msg;
            msg.payload.timestamp = visualization::timestamp_from_ns(ts);
            msg.payload.frame_id  = "calibration_view";
            msg.payload.format    = "jpeg";
            // 二进制图像载荷转std::byte
            msg.payload.data      = std::vector<std::byte>(
                reinterpret_cast<const std::byte*>(compressed.data()),
                reinterpret_cast<const std::byte*>(compressed.data() + compressed.size()));

            // 送入Foxglove发送队列异步推送Web前端
            (*server)->enqueue_message(std::move(msg));
        });
}

} // namespace fcs::calibration