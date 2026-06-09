#pragma once

#include "frame.hpp"

#include <cstdint>
#include <expected>
#include <fmt/format.h>

namespace fcs::L4 {

// ============================================================================
// Transform Lookup Result
// ============================================================================

/// Result of gimbal/muzzle transform lookup
///
struct TransformLookupResult {
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch> gimbal;
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::muzzle> muzzle;
    uint64_t timestamp_ns;
};

// ============================================================================
// Transform Lookup Utilities
// ============================================================================

/// Lookup both gimbal and muzzle transforms from tf buffer
///
/// @param tf_buffer Coordinate system buffer (shared_ptr)
/// @param current_ns Query timestamp in nanoseconds
/// @return TransformLookupResult on success, error message on failure
///
/// @note Uses fast_tf::lookup_clamped which clamps to buffer time range
/// @note Error messages include detailed diagnostic information
[[nodiscard]] inline std::expected<TransformLookupResult, std::string>
    lookup_gimbal_muzzle_transforms(
        const fast_tf::CoordinateSystem& tf_buffer, uint64_t current_ns) noexcept {
    using namespace fast_tf;

    auto muzzle_tf = lookup_clamped<odom, muzzle>(tf_buffer, current_ns);
    if (!muzzle_tf) {
        return std::unexpected(muzzle_tf.error());
    }

    auto gimbal_tf = lookup_clamped<odom, gimbal_pitch>(tf_buffer, current_ns);
    if (!gimbal_tf) {
        return std::unexpected(gimbal_tf.error());
    }

    return TransformLookupResult{
        .gimbal       = *gimbal_tf,
        .muzzle       = *muzzle_tf,
        .timestamp_ns = current_ns,
    };
}

/// Lookup only gimbal transform from tf buffer
///
/// @param tf_buffer Coordinate system buffer (shared_ptr)
/// @param current_ns Query timestamp in nanoseconds
[[nodiscard]] inline std::expected<
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>, std::string>
    lookup_gimbal_transform(
        const fast_tf::CoordinateSystem& tf_buffer, uint64_t current_ns) noexcept {
    using namespace fast_tf;

    auto gimbal_tf = lookup_clamped<odom, gimbal_pitch>(tf_buffer, current_ns);
    if (!gimbal_tf) {
        return std::unexpected(gimbal_tf.error());
    }

    return *gimbal_tf;
}

} // namespace fcs::L4
