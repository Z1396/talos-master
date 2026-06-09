#include "chiral/chiral_collector_system.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/selected_target_snapshot.hpp"
#include "chiral/chiral_endpoint.hpp"
#include "chiral/navigation.hpp"
#include "core/channel_topics.hpp"
#include "frame.hpp"
#include "matrix.hpp"
#include "scheduler/scheduler.hpp"

#include <magic_enum.hpp>
#include <spdlog/spdlog.h>
#include <vector>

namespace fcs::chiral {
using namespace talos::chiral;
namespace {
// Convert Eigen Vector3d to chiral::Vector3d
template <navigation::tag From, navigation::tag To = navigation::untyped>
navigation::Vector3d<From, To> to_chiral_vector3d(const Eigen::Vector3d& v) noexcept {
    return {v.x(), v.y(), v.z()};
}

// Convert FCS TrackerStatus to chiral::TrackerStatus
navigation::TrackerStatus to_chiral_tracker_status(L3::TrackerStatus status) noexcept {
    switch (status) {
    case L3::TrackerStatus::Idle: return navigation::TrackerStatus::Idle;
    case L3::TrackerStatus::Detecting: return navigation::TrackerStatus::Detecting;
    case L3::TrackerStatus::Tracking: return navigation::TrackerStatus::Tracking;
    case L3::TrackerStatus::TempLost: return navigation::TrackerStatus::TempLost;
    }
}

// Convert FCS ArmorName to chiral::ArmorName
navigation::ArmorName to_chiral_armor_name(ArmorName name) noexcept {
    switch (name) {
    case ArmorName::Sentry: return navigation::ArmorName::Sentry;
    case ArmorName::One: return navigation::ArmorName::One;
    case ArmorName::Two: return navigation::ArmorName::Two;
    case ArmorName::Three: return navigation::ArmorName::Three;
    case ArmorName::Four: return navigation::ArmorName::Four;
    case ArmorName::Five: return navigation::ArmorName::Five;
    case ArmorName::Outpost: return navigation::ArmorName::Outpost;
    case ArmorName::Base: return navigation::ArmorName::Base;
    case ArmorName::BaseLarge: return navigation::ArmorName::Invalid;
    case ArmorName::Invalid: return navigation::ArmorName::Invalid;
    }
}

// Convert FCS ArmorColor to chiral::ArmorColor
[[maybe_unused]] navigation::ArmorColor to_chiral_armor_color(ArmorColor color) noexcept {
    switch (color) {
    case ArmorColor::Blue: return navigation::ArmorColor::Blue;
    case ArmorColor::Red: return navigation::ArmorColor::Red;
    case ArmorColor::Neutral: return navigation::ArmorColor::Neutral;
    case ArmorColor::Purple: return navigation::ArmorColor::Purple;
    }
}

} // namespace

void register_chiral_collector_system(talos::Scheduler& scheduler) {
    // Create bidirectional endpoint: Talos writes TalosData, reads IncomingData
    auto endpoint = navigation::TalosEndpoint::create();
    if (!endpoint) {
        SPDLOG_ERROR(
            "Failed to create ChiralEndpoint (TalosSide): {}",
            magic_enum::enum_name(endpoint.error()));
        return;
    }

    scheduler.add_system<talos::pool_compute>(
        "chiral_collector",
        [ep_storage = std::move(endpoint.value())](
            talos::spmc<L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
                selected_target_in,
            talos::res<fast_tf::CoordinateSystem> tf_buffer) mutable {
            if (!selected_target_in.has_new()) {
                return;
            }

            auto selected_target = selected_target_in.read();
            if (!selected_target) {
                return;
            }

            navigation::TalosData talos_data{};
            talos_data.state_kind   = navigation::TargetStateKind::Robot;
            talos_data.state.status = navigation::TrackerStatus::Idle;
            talos_data.state.color  = navigation::ArmorColor::Neutral;
            talos_data.state.name   = navigation::ArmorName::Invalid;

            const uint64_t current_ns = selected_target->timestamp_ns;
            using namespace fast_tf;
            auto gimbal_yaw_tf =
                lookup_clamped<odom, fast_tf::gimbal_yaw_fuxk_frame>(*tf_buffer, current_ns);
            if (!gimbal_yaw_tf) {
                SPDLOG_WARN("Failed to lookup gimbal_yaw transform: {}", gimbal_yaw_tf.error());
                return;
            }
            auto gimbal_yaw_fix = gimbal_yaw_tf->inverse();

            // Lookup gimbal transform (odom -> gimbal)
            auto gimbal_tf = lookup_clamped<odom, gimbal_pitch>(*tf_buffer, current_ns);
            if (gimbal_tf) {
                auto translation                     = gimbal_tf.value().translation();
                auto rotation                        = gimbal_tf.value().quaternion();
                talos_data.gimbal_link.translation.x = translation.x();
                talos_data.gimbal_link.translation.y = translation.y();
                talos_data.gimbal_link.translation.z = translation.z();
                talos_data.gimbal_link.rotation.x    = rotation.x();
                talos_data.gimbal_link.rotation.y    = rotation.y();
                talos_data.gimbal_link.rotation.z    = rotation.z();
                talos_data.gimbal_link.rotation.w    = rotation.w();
            } else {
                SPDLOG_WARN("Failed to lookup gimbal transform: {}", gimbal_tf.error());
                return;
            }

            // Lookup camera transform (gimbal -> camera)
            auto camera_tf = lookup_clamped<gimbal, camera>(*tf_buffer, current_ns);
            if (camera_tf) {
                auto translation                     = camera_tf.value().translation();
                auto rotation                        = camera_tf.value().quaternion();
                talos_data.camera_link.translation.x = translation.x();
                talos_data.camera_link.translation.y = translation.y();
                talos_data.camera_link.translation.z = translation.z();
                talos_data.camera_link.rotation.x    = rotation.x();
                talos_data.camera_link.rotation.y    = rotation.y();
                talos_data.camera_link.rotation.z    = rotation.z();
                talos_data.camera_link.rotation.w    = rotation.w();
            } else {
                SPDLOG_WARN("Failed to lookup camera transform: {}", camera_tf.error());
                return;
            }

            // Lookup muzzle transform (odom -> muzzle)
            auto muzzle_tf = lookup_clamped<odom, muzzle>(*tf_buffer, current_ns);
            if (muzzle_tf) {
                auto translation                     = muzzle_tf.value().translation();
                auto rotation                        = muzzle_tf.value().quaternion();
                talos_data.muzzle_link.translation.x = translation.x();
                talos_data.muzzle_link.translation.y = translation.y();
                talos_data.muzzle_link.translation.z = translation.z();
                talos_data.muzzle_link.rotation.x    = rotation.x();
                talos_data.muzzle_link.rotation.y    = rotation.y();
                talos_data.muzzle_link.rotation.z    = rotation.z();
                talos_data.muzzle_link.rotation.w    = rotation.w();
            } else {
                SPDLOG_WARN("Failed to lookup muzzle transform: {}", muzzle_tf.error());
                return;
            }

            const bool has_valid_armor_target =
                selected_target->has_target() && selected_target->tracker.is_tracking();
            const L3::TrackerOutput* tracker_output =
                has_valid_armor_target ? std::addressof(selected_target->tracker) : nullptr;

            if (tracker_output) {
                talos_data.state.status = to_chiral_tracker_status(tracker_output->status);
                talos_data.state.color  = to_chiral_armor_color(tracker_output->target_color);
                talos_data.state.name   = to_chiral_armor_name(tracker_output->target_name);
            }

            if (tracker_output && tracker_output->is_robot()) {
                talos_data.state_kind   = navigation::TargetStateKind::Robot;
                const auto* robot_state = tracker_output->robot_state();
                if (robot_state) {
                    auto pos      = robot_state->position;
                    auto vel      = robot_state->velocity;
                    auto pose_fix = gimbal_yaw_fix
                                  * TransformMatrixd<odom, void>::from_rpy(
                                        0.0, 0.0, robot_state->yaw, pos.x(), pos.y(), pos.z());
                    auto vel_fix = gimbal_yaw_fix
                                 * TransformMatrixd<odom, void>::from_rpy(
                                       0.0, 0.0, robot_state->v_yaw, vel.x(), vel.y(), vel.z());

                    talos_data.state.robot.position =
                        to_chiral_vector3d<navigation::gimbal_yaw>(pose_fix.translation());
                    talos_data.state.robot.velocity =
                        to_chiral_vector3d<navigation::gimbal_yaw>(vel_fix.translation());
                    talos_data.state.robot.yaw     = pose_fix.euler_rot().yaw;
                    talos_data.state.robot.v_yaw   = vel_fix.euler_rot().yaw;
                    talos_data.state.robot.radius0 = robot_state->radius0;
                    talos_data.state.robot.radius1 = robot_state->radius1;
                    talos_data.state.robot.z1      = robot_state->z1;
                    talos_data.state.robot.armor_num =
                        static_cast<uint32_t>(robot_state->armors_num);
                }
            } else if (tracker_output && tracker_output->is_outpost()) {
                talos_data.state_kind     = navigation::TargetStateKind::Outpost;
                const auto* outpost_state = tracker_output->outpost_state();
                if (outpost_state) {
                    auto pos = outpost_state->position;
                    auto vel = outpost_state->velocity;
                    auto z   = outpost_state->z;
                    for (auto& i : z) {
                        i = (gimbal_yaw_fix
                             * TransformMatrixd<odom, void>::from_rpy(
                                 0.0, 0.0, outpost_state->yaw, pos.x(), pos.y(), i))
                                .translation()
                                .z();
                    }
                    auto vel_fix = gimbal_yaw_fix
                                 * TransformMatrixd<odom, void>::from_rpy(
                                       0.0, 0.0, outpost_state->v_yaw, vel.x(), vel.y(), vel.z());
                    auto pos_fix =
                        (gimbal_yaw_fix
                         * TransformMatrixd<odom, void>::from_rpy(
                             0.0, 0.0, outpost_state->yaw, pos.x(), pos.y(), 0.0));
                    talos_data.state.outpost.position.x = pos_fix.translation().x();
                    talos_data.state.outpost.position.y = pos_fix.translation().y();
                    talos_data.state.outpost.position.z = 0.0; // 2D position
                    talos_data.state.outpost.velocity =
                        to_chiral_vector3d<navigation::gimbal_yaw>(vel_fix.translation());
                    talos_data.state.outpost.yaw   = pos_fix.euler_rot().yaw;
                    talos_data.state.outpost.v_yaw = vel_fix.euler_rot().yaw;
                    talos_data.state.outpost.z     = z;
                }
            }
            ep_storage->write(talos_data);
        });
}

} // namespace fcs::chiral
