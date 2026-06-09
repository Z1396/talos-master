#include "validation.hpp"
#include "frame.hpp"

namespace fast_tf {

// ============================================================================
// Explicit instantiations for all edge transforms in the coordinate system
// ============================================================================

// world -> odom
template std::expected<void, std::string>
    validate_transform(const TransformMatrix<double, world_fuxk_frame, odom_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, world_fuxk_frame, odom_fuxk_frame>&) noexcept;

// odom -> gimbal_yaw
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, odom_fuxk_frame, gimbal_yaw_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, odom_fuxk_frame, gimbal_yaw_fuxk_frame>&) noexcept;

// gimbal_yaw -> gimbal_pitch
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, gimbal_yaw_fuxk_frame, gimbal_pitch_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, gimbal_yaw_fuxk_frame, gimbal_pitch_fuxk_frame>&) noexcept;

// gimbal_pitch -> camera_link
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, camera_link_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, camera_link_fuxk_frame>&) noexcept;

// camera_link -> camera_optical
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, camera_link_fuxk_frame, camera_optical_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, camera_link_fuxk_frame, camera_optical_fuxk_frame>&) noexcept;

// gimbal_pitch -> muzzle_link
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, muzzle_link_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, muzzle_link_fuxk_frame>&) noexcept;

} // namespace fast_tf
