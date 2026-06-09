#pragma once

/// PnP construction helpers for ArmorMeasurementT.
/// Include this only in files that perform solvePnP and create camera-optical measurements.
/// Separated from types.hpp to avoid propagating calib3d.hpp to all downstream TUs.

#include "core/types.hpp"

namespace fcs {

/// Construct a camera-optical measurement from detection + PnP result.
///
/// NOTE: from_pnp returns the pose in camera_optical frame (OpenCV convention):
/// - X axis: right
/// - Y axis: down
/// - Z axis: forward (pointing into the scene)
///
/// OpenCV returns object -> camera pose, so this constructor is only valid for
/// camera_optical-frame measurements before TF reframe.
inline CameraArmorMeasurement make_camera_measurement(
    const ArmorDetection& det, const cv::Mat& rvec, const cv::Mat& tvec, uint64_t ts,
    float dist_to_center = 0.0f) {
    const cv::Vec3d rvec_v(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    const cv::Vec3d tvec_v(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    CameraArmorMeasurement m;
    m.name                     = det.name;
    m.color                    = det.color;
    m.type                     = det.type;
    m.confidence               = det.confidence;
    m.distance_to_image_center = dist_to_center;
    m.timestamp_ns             = ts;
    m.transform                = CameraArmorMeasurement::Transform::from_pnp(rvec_v, tvec_v);
    return m;
}

} // namespace fcs
