#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <foxglove/schemas.hpp>
#include <opencv2/core/types.hpp>

namespace fcs::visualization::tactical {

namespace Semantic {

// Restrained, Apple-like semantic hues. Neutral context carries most of the scene;
// saturated colors are reserved for decisions and the current focus.
constexpr ::foxglove::schemas::Color STATUS_ACTIVE     = {0.00, 0.44, 0.90, 1.00};
constexpr ::foxglove::schemas::Color DECISION_POSITIVE = {0.20, 0.70, 0.32, 1.00};
constexpr ::foxglove::schemas::Color DECISION_NEGATIVE = {0.92, 0.22, 0.20, 1.00};
constexpr ::foxglove::schemas::Color ALERT             = {0.95, 0.58, 0.10, 1.00};
constexpr ::foxglove::schemas::Color INERT             = {0.58, 0.62, 0.68, 1.00};

constexpr ::foxglove::schemas::Color CONTEXT = {0.36, 0.39, 0.44, 1.00};
constexpr ::foxglove::schemas::Color SURFACE = {0.12, 0.13, 0.15, 1.00};

} // namespace Semantic

namespace Team {

constexpr ::foxglove::schemas::Color RED  = {0.86, 0.20, 0.18, 0.70};
constexpr ::foxglove::schemas::Color BLUE = {0.10, 0.38, 0.86, 0.70};

} // namespace Team

namespace Axis {

constexpr ::foxglove::schemas::Color X = {0.84, 0.24, 0.22, 0.72};
constexpr ::foxglove::schemas::Color Y = {0.22, 0.64, 0.32, 0.72};
constexpr ::foxglove::schemas::Color Z = {0.10, 0.38, 0.86, 0.72};

} // namespace Axis

[[nodiscard]] constexpr ::foxglove::schemas::Color
    with_alpha(::foxglove::schemas::Color color, double alpha) noexcept {
    color.a = static_cast<float>(alpha);
    return color;
}

[[nodiscard]] constexpr ::foxglove::schemas::Color
    scaled_rgb(::foxglove::schemas::Color color, double scale) noexcept {
    color.r = static_cast<float>(color.r * scale);
    color.g = static_cast<float>(color.g * scale);
    color.b = static_cast<float>(color.b * scale);
    return color;
}

[[nodiscard]] inline cv::Scalar to_cv_bgr(const ::foxglove::schemas::Color& color) noexcept {
    const auto channel = [](double value) { return std::clamp(value, 0.0, 1.0) * 255.0; };
    return cv::Scalar(channel(color.b), channel(color.g), channel(color.r), channel(color.a));
}

// ============================================================================
// L1 - IMAGE OVERLAY
// ============================================================================
namespace Image {

constexpr ::foxglove::schemas::Color ROI_VALID      = with_alpha(Semantic::ALERT, 0.78);
constexpr ::foxglove::schemas::Color ROI_MISSING    = with_alpha(Semantic::DECISION_NEGATIVE, 0.78);
constexpr ::foxglove::schemas::Color DETECTION_BOX  = with_alpha(Semantic::STATUS_ACTIVE, 0.92);
constexpr ::foxglove::schemas::Color DETECTION_TEXT = with_alpha(Semantic::STATUS_ACTIVE, 0.90);
constexpr ::foxglove::schemas::Color LDM_PRIMARY    = with_alpha(Semantic::STATUS_ACTIVE, 0.92);
constexpr ::foxglove::schemas::Color LDM_SECONDARY  = with_alpha(Semantic::INERT, 0.72);
constexpr ::foxglove::schemas::Color LDM_CENTER     = with_alpha(Semantic::ALERT, 0.90);
constexpr ::foxglove::schemas::Color OPTICAL_CENTER = with_alpha(Semantic::INERT, 0.82);
constexpr ::foxglove::schemas::Color CORNER_PRIMARY = with_alpha(Semantic::STATUS_ACTIVE, 0.92);
constexpr ::foxglove::schemas::Color CORNER_SECONDARY = with_alpha(Semantic::INERT, 0.72);

constexpr int LINE_THIN      = 1;
constexpr int LINE_MEDIUM    = 2;
constexpr int MARKER_SIZE    = 14;
constexpr double TEXT_SMALL  = 0.45;
constexpr double TEXT_MEDIUM = 0.55;
constexpr int TEXT_THIN      = 1;
constexpr int TEXT_MEDIUM_PX = 2;

} // namespace Image

// ============================================================================
// L2 - PERCEPTION
// ============================================================================
namespace L2 {

constexpr ::foxglove::schemas::Color MEASUREMENT_GHOST      = {0.62, 0.66, 0.72, 0.22};
constexpr ::foxglove::schemas::Color MEASUREMENT_CONFIDENCE = {0.62, 0.66, 0.72, 0.48};
constexpr ::foxglove::schemas::Color LDM_STABLE       = with_alpha(Semantic::STATUS_ACTIVE, 0.78);
constexpr ::foxglove::schemas::Color LDM_CONSTRAINED  = with_alpha(Semantic::ALERT, 0.68);
constexpr ::foxglove::schemas::Color LDM_BEARING_ONLY = with_alpha(Semantic::INERT, 0.52);
constexpr ::foxglove::schemas::Color LDM_NONE         = with_alpha(Semantic::CONTEXT, 0.30);

constexpr double ARMOR_SIZE      = 0.055;
constexpr double LABEL_FONT_SIZE = 0.042;

} // namespace L2

// ============================================================================
// L3 - ESTIMATION
// ============================================================================
namespace L3 {

constexpr ::foxglove::schemas::Color TRACKING_LOCKED    = with_alpha(Semantic::STATUS_ACTIVE, 0.92);
constexpr ::foxglove::schemas::Color TRACKING_ACQUIRING = with_alpha(Semantic::STATUS_ACTIVE, 0.58);
constexpr ::foxglove::schemas::Color TRACKING_WARNING   = with_alpha(Semantic::ALERT, 0.62);
constexpr ::foxglove::schemas::Color TRACKING_LOST = with_alpha(Semantic::DECISION_NEGATIVE, 0.48);
constexpr ::foxglove::schemas::Color PREDICTION_AMBER   = with_alpha(Semantic::ALERT, 0.62);
constexpr ::foxglove::schemas::Color UNCERTAINTY_ORANGE = with_alpha(Semantic::ALERT, 0.18);
constexpr ::foxglove::schemas::Color PREDICTION_CONTEXT = with_alpha(Semantic::INERT, 0.42);

constexpr double TARGET_SIZE         = 0.095;
constexpr double ARMOR_PLATE_SIZE    = 0.085;
constexpr double UNCERTAINTY_SCALE   = 0.18;
constexpr double ROBOT_CENTER_SIZE   = 0.15;
constexpr double OUTPOST_CENTER_SIZE = 0.17;
constexpr double LABEL_OFFSET_Z      = 0.115;

} // namespace L3

// ============================================================================
// L4 - PLANNING
// ============================================================================
namespace L4 {

constexpr ::foxglove::schemas::Color GIMBAL_AIM_FIRE =
    with_alpha(Semantic::DECISION_POSITIVE, 0.92);
constexpr ::foxglove::schemas::Color GIMBAL_AIM_HOLD = with_alpha(Semantic::ALERT, 0.78);

constexpr ::foxglove::schemas::Color FUTURE_ARMOR   = with_alpha(Semantic::INERT, 0.34);
constexpr ::foxglove::schemas::Color SPATIAL_LINK   = with_alpha(Semantic::CONTEXT, 0.32);
constexpr ::foxglove::schemas::Color SELECTION_LINK = with_alpha(Semantic::STATUS_ACTIVE, 0.42);

constexpr ::foxglove::schemas::Color CANDIDATE_SELECTED = with_alpha(Semantic::STATUS_ACTIVE, 0.94);
constexpr ::foxglove::schemas::Color CANDIDATE_RUNNER_UP  = with_alpha(Semantic::ALERT, 0.68);
constexpr ::foxglove::schemas::Color CANDIDATE_ELIMINATED = with_alpha(Semantic::INERT, 0.22);

constexpr ::foxglove::schemas::Color TRAJECTORY_FIRE =
    with_alpha(Semantic::DECISION_POSITIVE, 0.42);
constexpr ::foxglove::schemas::Color TRAJECTORY_HOLD = with_alpha(Semantic::INERT, 0.18);

constexpr ::foxglove::schemas::Color MPC_PRESENT   = with_alpha(Semantic::STATUS_ACTIVE, 0.58);
constexpr ::foxglove::schemas::Color MPC_REFERENCE = with_alpha(Semantic::ALERT, 0.42);

constexpr double SPATIAL_LINK_THICKNESS    = 0.006;
constexpr double TRAJECTORY_LINE_THICKNESS = 0.006;
constexpr double SELECTION_SIZE            = 0.058;
constexpr double PREDICTION_SIZE           = 0.036;
constexpr double TRAJECTORY_DOT            = 0.014;
constexpr double RING_HEIGHT               = 0.004;
constexpr double RING_SCALE                = 1.55;

} // namespace L4

// ============================================================================
// L5 - WEAPON
// ============================================================================
namespace L5 {

constexpr ::foxglove::schemas::Color FIRE_EXECUTE  = with_alpha(Semantic::DECISION_POSITIVE, 0.98);
constexpr ::foxglove::schemas::Color FIRE_READY    = with_alpha(Semantic::DECISION_POSITIVE, 0.84);
constexpr ::foxglove::schemas::Color FIRE_COOLDOWN = with_alpha(Semantic::ALERT, 0.66);
constexpr ::foxglove::schemas::Color FIRE_ABORTED  = with_alpha(Semantic::DECISION_NEGATIVE, 0.72);

} // namespace L5

namespace Velocity {

constexpr ::foxglove::schemas::Color LINEAR  = with_alpha(Semantic::STATUS_ACTIVE, 0.78);
constexpr ::foxglove::schemas::Color ANGULAR = with_alpha(Semantic::ALERT, 0.72);

constexpr double ARROW_SHAFT_DIAMETER = 0.016;
constexpr double ARROW_HEAD_DIAMETER  = 0.032;
constexpr double ARROW_MIN_LENGTH     = 0.0001;
constexpr double ARROW_SHAFT_RATIO    = 0.8;
constexpr double ARROW_HEAD_RATIO     = 0.2;

} // namespace Velocity

namespace Text {

constexpr ::foxglove::schemas::Color PRIMARY   = {0.92, 0.94, 0.96, 0.88};
constexpr ::foxglove::schemas::Color SECONDARY = {0.72, 0.75, 0.78, 0.66};
constexpr ::foxglove::schemas::Color WARNING   = with_alpha(Semantic::ALERT, 0.78);
constexpr ::foxglove::schemas::Color ERROR     = with_alpha(Semantic::DECISION_NEGATIVE, 0.78);

constexpr double SIZE_SMALL      = 0.035;
constexpr double SIZE_MEDIUM     = 0.052;
constexpr double SIZE_LARGE      = 0.070;
constexpr double SIZE_DEFAULT    = SIZE_MEDIUM;
constexpr bool BILLBOARD_ENABLED = true;

} // namespace Text

namespace Geometry {

constexpr double ARMOR_THICKNESS    = 0.03;
constexpr double ARMOR_HEIGHT_SMALL = 0.135;
constexpr double ARMOR_HEIGHT_BIG   = 0.23;
constexpr double ARMOR_WIDTH        = 0.125;
constexpr double ARMOR_TILT_ANGLE   = 0.2618;

} // namespace Geometry

namespace Temporal {

constexpr uint64_t ENTITY_LIFETIME_NS  = 100'000'000ULL;
constexpr double MPC_EXTRAPOLATION_VEL = 5.0;

} // namespace Temporal

namespace Legacy {

constexpr ::foxglove::schemas::Color COLOR_YELLOW = L4::GIMBAL_AIM_HOLD;
constexpr ::foxglove::schemas::Color COLOR_CYAN   = L3::TRACKING_LOCKED;
constexpr ::foxglove::schemas::Color COLOR_GREEN  = L4::GIMBAL_AIM_FIRE;
constexpr ::foxglove::schemas::Color COLOR_WHITE  = L2::MEASUREMENT_GHOST;
constexpr ::foxglove::schemas::Color COLOR_RED    = L3::TRACKING_LOST;
constexpr ::foxglove::schemas::Color COLOR_ORANGE = L3::PREDICTION_AMBER;

constexpr double SPHERE_SIZE_DEFAULT = L3::TARGET_SIZE;
constexpr double SPHERE_SIZE_SMALL   = L3::ARMOR_PLATE_SIZE;
constexpr double SPHERE_SIZE_TINY    = L4::TRAJECTORY_DOT;

} // namespace Legacy

enum class SelectionTier { Selected, RunnerUp, Eliminated };

struct SelectionStyle {
    ::foxglove::schemas::Color color;
    double size_scale;
    double alpha;
    bool show_label;
};

[[nodiscard]] constexpr SelectionStyle selection_style(SelectionTier tier) noexcept {
    switch (tier) {
    case SelectionTier::Selected:
        return {
            .color = L4::CANDIDATE_SELECTED, .size_scale = 1.25, .alpha = 0.94, .show_label = true};
    case SelectionTier::RunnerUp:
        return {
            .color = L4::CANDIDATE_RUNNER_UP, .size_scale = 1.0, .alpha = 0.68, .show_label = true};
    case SelectionTier::Eliminated:
        return {
            .color      = L4::CANDIDATE_ELIMINATED,
            .size_scale = 0.68,
            .alpha      = 0.22,
            .show_label = false};
    }
    return {
        .color = L4::CANDIDATE_ELIMINATED, .size_scale = 0.68, .alpha = 0.22, .show_label = false};
}

[[nodiscard]] inline ::foxglove::schemas::Color tracker_status_color(int status_int) noexcept {
    switch (status_int) {
    case 2: return L3::TRACKING_LOCKED;
    case 1: return L3::TRACKING_ACQUIRING;
    case 3: return L3::TRACKING_WARNING;
    case 0:
    default: return L2::MEASUREMENT_GHOST;
    }
}

[[nodiscard]] inline ::foxglove::schemas::Color fire_decision_color(bool can_fire) noexcept {
    return can_fire ? L4::TRAJECTORY_FIRE : L4::TRAJECTORY_HOLD;
}

[[nodiscard]] inline ::foxglove::schemas::Color
    temporal_fade(const ::foxglove::schemas::Color& base_color, int temporal_distance) noexcept {
    const int dist = std::abs(temporal_distance);
    double alpha   = 1.0;
    if (dist == 0) {
        alpha = 1.00;
    } else if (dist <= 2) {
        alpha = 0.76;
    } else if (dist <= 4) {
        alpha = 0.48;
    } else {
        alpha = 0.22;
    }

    return with_alpha(base_color, alpha * base_color.a);
}

} // namespace fcs::visualization::tactical
