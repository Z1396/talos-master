#include "L2_perception/ldm/ldm_systems.hpp"

#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/ldm/ldm_detector.hpp"
#include "L2_perception/ldm/ldm_solver.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "camera_config.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/types.hpp"
#include "frame.hpp"
#include "scheduler/scheduler.hpp"

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

namespace fcs::L2::ldm {

namespace {

void apply_detector_boundary_semantics(
    LdmDetection& detection, std::optional<LdmMeasurement>* measurement,
    ArmorColor detecting_color) noexcept {
    if (detection.color != ArmorColor::Purple) {
        return;
    }

    detection.accurate = true;
    detection.color    = detecting_color;

    if (measurement != nullptr && measurement->has_value()) {
        (*measurement)->accurate = true;
        (*measurement)->color    = detecting_color;
    }
}

/// Build a PosePrior from the last known LDM tracker state.
/// Extracts the odom-frame pose (R, p) from the SE2(3) state, transforms to
/// camera frame via T_odom_camera, and converts to OpenCV rvec/tvec.
/// Returns std::nullopt if the tracker is not in a tracking state.
[[nodiscard]] std::optional<LdmSolver::PosePrior> make_ldm_pose_prior(
    const fcs::L3::ldm::LdmState& state,
    const LdmSolver::OdomCameraTransform& T_odom_camera) noexcept {
    if (!state.is_tracking()) {
        return std::nullopt;
    }

    // T_odom_camera : FrameTransform<odom, camera_optical>
    //   p_odom = R_oc * p_camera + t_oc
    // Inverse gives camera←odom:
    //   p_camera = R_oc^T * (p_odom - t_oc)
    const auto T_camera_odom = T_odom_camera.inverse();

    // Build camera→ldm_body transform from tracker state.
    // state.X.R() = R_odom_body  (world→body, expressed in odom)
    // state.X.p() = position of body origin in odom
    const Eigen::Matrix3d R_camera_body = T_camera_odom.rotation() * state.X.R();
    const Eigen::Vector3d t_camera_body =
        T_camera_odom.rotation() * state.X.p() + T_camera_odom.translation();

    if (!t_camera_body.allFinite() || t_camera_body.z() <= 1e-3) {
        return std::nullopt;
    }

    // Convert to OpenCV rvec/tvec format expected by PosePrior.
    Eigen::Matrix3d R_cv = R_camera_body;
    cv::Mat R_mat;
    cv::eigen2cv(R_cv, R_mat);
    cv::Mat rvec;
    cv::Rodrigues(R_mat, rvec);

    return LdmSolver::PosePrior{
        .rvec = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)),
        .tvec = cv::Vec3d(t_camera_body.x(), t_camera_body.y(), t_camera_body.z()),
    };
}

} // namespace

void register_ldm_systems(
    talos::Scheduler& scheduler, LdmDetectorConfig&& config,
    const CameraConfig& camera_config) noexcept {
    scheduler.world().insert_resource(config);

    auto solver_ptr = std::make_shared<LdmSolver>(camera_config, config);

    scheduler.add_system<talos::fixed_rate<200>>(
        "ldm_detector",
        [detector = std::make_shared<LdmDetector>(config), solver_ptr](
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            talos::spmc_mut<LdmDetection, LdmDetectionChannelTopic> detection_out,
            talos::spmc_mut<LdmMeasurement, LdmMeasurementChannelTopic> measurement_out,
            talos::spmc<fcs::L3::ldm::LdmState> ldm_state_in, talos::res<LdmDetectorConfig> cfg_,
            core::detecting_color detecting_color_, core::capabilities cap,
            talos::res<fast_tf::CoordinateSystem> tf_system) mutable {
            if (!core::capable(*cap, core::Capability::Ldm)) {
                return;
            }
            auto frame = image_in.read();
            if (!frame) {
                return;
            }

            auto det_result = detector->detect(frame->image);
            if (!det_result) {
                SPDLOG_ERROR("ldm detect failed: {}", magic_enum::enum_name(det_result.error()));
                return;
            }

            LdmDetection detection;
            detection.timestamp_ns = frame->timestamp_ns;
            detection.frame_id     = frame->frame_id;
            detection.color        = *detecting_color_;
            detection.accurate     = false;

            if (det_result->has_value()) {
                detection              = std::move(**det_result);
                detection.timestamp_ns = frame->timestamp_ns;
                detection.frame_id     = frame->frame_id;
                apply_detector_boundary_semantics(detection, nullptr, *detecting_color_);
            }

            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            if (tf_lookup) {
                // Build prior from last known tracker state for distance bounding.
                std::optional<LdmSolver::PosePrior> prior;
                if (auto ldm_state = ldm_state_in.read()) {
                    prior = make_ldm_pose_prior(*ldm_state, *tf_lookup);
                }

                auto meas_result = solver_ptr->solve(detection, *tf_lookup, prior);
                if (meas_result) {
                    auto measurement                                   = std::move(*meas_result);
                    std::optional<LdmMeasurement> boundary_measurement = std::move(measurement);
                    apply_detector_boundary_semantics(
                        detection, &boundary_measurement, *detecting_color_);
                    measurement_out.write(std::move(*boundary_measurement));
                } else {
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

            detection_out.write(std::move(detection));
        });
}

} // namespace fcs::L2::ldm
