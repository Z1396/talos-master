#include "L2_perception/ldm/ldm_systems.hpp"

// LDM检测器配置结构体
#include "L2_perception/ldm/ldm_config.hpp"
// LDM大符图像检测类
#include "L2_perception/ldm/ldm_detector.hpp"
// LDM PnP姿态求解器，用于三维位姿解算
#include "L2_perception/ldm/ldm_solver.hpp"
// L3层跟踪器状态、测量数据类型定义
#include "L3_estimation/ldm_naive/types.hpp"
// 相机内参、畸变、标定配置
#include "camera_config.hpp"
// 消息通道枚举定义，图像/检测结果/测量结果话题ID
#include "core/channel_topics.hpp"
// 运行时核心资源、全局运行上下文
#include "core/runtime.hpp"
// 项目通用基础数据类型别名
#include "core/types.hpp"
// 图像帧结构体，存储原图、时间戳、帧编号
#include "frame.hpp"
// Talos实时调度器、系统注册、固定频率任务封装
#include "scheduler/scheduler.hpp"

// OpenCV标定、PnP、罗德里格斯旋转转换工具
#include <opencv2/calib3d.hpp>
// 日志打印库
#include <spdlog/spdlog.h>

namespace fcs::L2::ldm {

namespace { // 内部匿名命名空间，仅本文件可见工具函数

/**
 * @brief 紫色大符特殊语义修正逻辑
 * 紫色大符识别精度更高，强制标记accurate=true，同步修正输出颜色为当前目标识别色
 * @param detection 图像层大符检测结果，原地修改字段
 * @param measurement 位姿测量结果指针，可为nullptr，同步修正
 * @param detecting_color 当前整车识别目标装甲颜色（红/蓝/紫）
 * noexcept 无异常抛出，实时流水线稳定
 */
void apply_detector_boundary_semantics(
    LdmDetection& detection, std::optional<LdmMeasurement>* measurement,
    ArmorColor detecting_color) noexcept {
    // 仅紫色大符走特殊修正逻辑，其他颜色直接返回
    if (detection.color != ArmorColor::Purple) {
        return;
    }

    // 标记检测结果高精度，覆盖原始颜色为当前目标色
    detection.accurate = true;
    detection.color    = detecting_color;

    // 若存在有效测量结果，同步同步高精度标记与颜色
    if (measurement != nullptr && measurement->has_value()) {
        (*measurement)->accurate = true;
        (*measurement)->color    = detecting_color;
    }
}

/**
 * @brief 从LDM跟踪器历史状态构建PnP先验位姿PosePrior
 * 作用：给PnP求解提供上一帧位姿初值，加速收敛、抑制远距离畸变跳变
 * 流程：
 * 1. 获取里程计→相机变换矩阵T_odom_camera，求逆得到相机→里程计变换
 * 2. 跟踪器odom坐标系下SE2(3)位姿，转换至相机光学坐标系
 * 3. Eigen旋转/平移转为OpenCV rvec/tvec，封装为先验结构
 * @param state L3跟踪器输出的上一帧大符状态（旋转、平移、跟踪状态标记）
 * @param T_odom_camera 里程计坐标系到相机光学坐标系变换
 * @return 有效跟踪返回PosePrior，未跟踪/数值非法返回std::nullopt
 */
[[nodiscard]] std::optional<LdmSolver::PosePrior> make_ldm_pose_prior(
    const fcs::L3::ldm::LdmState& state,
    const LdmSolver::OdomCameraTransform& T_odom_camera) noexcept {
    // 跟踪器未处于有效跟踪状态，无历史位姿，返回空
    if (!state.is_tracking()) {
        return std::nullopt;
    }

    // T_odom_camera：p_odom = R_oc * p_cam + t_oc
    // 求逆变换 T_cam_odom：p_cam = R_oc^T * (p_odom - t_oc)
    const auto T_camera_odom = T_odom_camera.inverse();

    // state.X.R()：odom坐标系下大符自身旋转矩阵 R_odom_body
    // state.X.p()：odom坐标系下大符原点三维平移
    // 复合变换：相机→odom × odom→大符 = 相机→大符
    const Eigen::Matrix3d R_camera_body = T_camera_odom.rotation() * state.X.R();
    const Eigen::Vector3d t_camera_body =
        T_camera_odom.rotation() * state.X.p() + T_camera_odom.translation();

    // 平移向量存在NaN/无穷 或 相机前向Z距离过近(<=1mm)，先验失效丢弃
    if (!t_camera_body.allFinite() || t_camera_body.z() <= 1e-3) {
        return std::nullopt;
    }

    // Eigen3d旋转矩阵 转为 OpenCV Mat
    Eigen::Matrix3d R_cv = R_camera_body;
    cv::Mat R_mat;
    cv::eigen2cv(R_cv, R_mat);
    // 罗德里格斯变换：旋转矩阵 → 旋转向量 rvec
    cv::Mat rvec;
    cv::Rodrigues(R_mat, rvec);

    // 封装PnP先验结构体返回
    return LdmSolver::PosePrior{
        .rvec = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)),
        .tvec = cv::Vec3d(t_camera_body.x(), t_camera_body.y(), t_camera_body.z()),
    };
}

} // namespace 内部工具函数结束

/**
 * @brief 注册LDM大符完整感知流水线系统到Talos调度器
 * 流水线链路：图像输入 → LdmDetector灯条/大符检测 → LdmSolver PnP位姿解算 → 输出检测框+三维测量
 * 任务固定200Hz运行，全局资源注入检测器配置、相机标定参数
 * @param scheduler Talos实时调度器实例
 * @param config 检测器阈值配置，右值移动存入全局资源池
 * @param camera_config 相机内参、畸变、分辨率标定参数
 * noexcept 无抛异常，保障机器人实时任务稳定
 */
void register_ldm_systems(
    talos::Scheduler& scheduler, LdmDetectorConfig&& config,
    const CameraConfig& camera_config) noexcept {
    // 将检测器配置存入调度器全局资源池，供系统任务只读获取
    scheduler.world().insert_resource(config);

    // 构造LDM姿态求解器共享指针，传入相机标定与检测器几何配置
    auto solver_ptr = std::make_shared<LdmSolver>(camera_config, config);

    // 注册200Hz固定频率系统任务：ldm_detector 完整感知流水线
    scheduler.add_system<talos::fixed_rate<200>>(
        "ldm_detector",
        // 任务捕获：检测器实例、求解器共享指针，mutable允许修改捕获的智能指针
        [detector = std::make_shared<LdmDetector>(config), solver_ptr](
            // 输入：图像帧SPMC只读通道，多生产者单消费者图像消息
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            // 输出：大符检测结果可写通道
            talos::spmc_mut<LdmDetection, LdmDetectionChannelTopic> detection_out,
            // 输出：三维位姿测量结果可写通道
            talos::spmc_mut<LdmMeasurement, LdmMeasurementChannelTopic> measurement_out,
            // 输入：L3跟踪器历史状态只读通道，用于生成PnP先验
            talos::spmc<fcs::L3::ldm::LdmState> ldm_state_in,
            // 全局只读资源：检测器配置
            talos::res<LdmDetectorConfig> cfg_,
            // 全局运行参数：当前识别目标装甲颜色（红/蓝/紫）
            core::detecting_color detecting_color_,
            // 整机能力标记，判断是否启用大符识别模块
            core::capabilities cap,
            // 全局TF坐标变换系统资源
            talos::res<fast_tf::CoordinateSystem> tf_system) mutable {
            // 整机未开启LDM大符识别能力，直接跳过本帧处理
            if (!core::capable(*cap, core::Capability::Ldm)) {
                return;
            }
            // 读取最新图像帧，无图像直接返回
            auto frame = image_in.read();
            if (!frame) {
                return;
            }

            // 执行图像大符检测，返回std::expected，区分执行错误/空结果/有效检测
            auto det_result = detector->detect(frame->image);
            // 检测底层执行失败（图像非法、参数错误等），打印日志退出
            if (!det_result) {
                SPDLOG_ERROR("ldm detect failed: {}", magic_enum::enum_name(det_result.error()));
                return;
            }

            // 初始化空检测结果，填充基础时间戳、帧ID、默认参数
            LdmDetection detection;
            detection.timestamp_ns = frame->timestamp_ns;
            detection.frame_id     = frame->frame_id;
            detection.color        = *detecting_color_;
            detection.accurate     = false;

            // 图像内检测到有效大符，覆盖初始化空数据
            if (det_result->has_value()) {
                detection              = std::move(**det_result);
                detection.timestamp_ns = frame->timestamp_ns;
                detection.frame_id     = frame->frame_id;
                // 紫色大符特殊语义修正
                apply_detector_boundary_semantics(detection, nullptr, *detecting_color_);
            }

            // 时序插值查找当前帧时间戳下 里程计→相机 坐标变换矩阵
            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            // TF变换查询成功，执行PnP三维位姿求解
            if (tf_lookup) {
                // 从跟踪器历史状态生成PnP先验位姿
                std::optional<LdmSolver::PosePrior> prior;
                if (auto ldm_state = ldm_state_in.read()) {
                    prior = make_ldm_pose_prior(*ldm_state, *tf_lookup);
                }

                // 调用求解器PnP解算三维位姿测量值
                auto meas_result = solver_ptr->solve(detection, *tf_lookup, prior);
                // PnP求解成功
                if (meas_result) {
                    auto measurement                                   = std::move(*meas_result);
                    std::optional<LdmMeasurement> boundary_measurement = std::move(measurement);
                    // 同步修正紫色高精度标记与目标颜色
                    apply_detector_boundary_semantics(
                        detection, &boundary_measurement, *detecting_color_);
                    // 将测量结果写入消息通道
                    measurement_out.write(std::move(*boundary_measurement));
                } else {
                    // PnP求解失败，打印错误日志，输出空占位测量包保证消息时序对齐
                    SPDLOG_ERROR("ldm solve failed: {}", meas_result.error());
                    measurement_out.write(
                        LdmMeasurement{
                            .timestamp_ns = detection.timestamp_ns,
                            .frame_id     = detection.frame_id,
                            .color        = detection.color,
                            .accurate     = detection.accurate,
                        });
                }
            }

            // 无论是否解算位姿，都输出图像层检测结果至通道
            detection_out.write(std::move(detection));
        });
}

} // namespace fcs::L2::ldm