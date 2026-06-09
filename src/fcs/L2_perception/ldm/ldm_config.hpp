#pragma once

#include "core/armor_types.hpp"

#include <array>

namespace fcs::L2::ldm {

struct LdmGeometryConfig {
    double octagon_side_length_m{0.020711};
    double octagon_circumradius_m{0.02706};
    double pair_center_separation_m{0.036514};
    double window_length_m{0.012};
    double window_width_m{0.009618};
    double volume_height_m{0.067};
    std::array<double, 2> detectable_center_z_range_m{-0.01, 0.01};
};

struct LdmDetectorConfig {
    ArmorColor target_color{ArmorColor::Red};
    int min_blob_area_px{5};
    int min_sparse_blob_pixel_count{8};
    double min_blob_fill_ratio{0.35};
    double min_blob_aspect_ratio{0.35};
    double max_blob_aspect_ratio{4.0};
    double min_pair_center_dy_ratio{1.0};
    double max_pair_center_dy_ratio{12.0};
    double max_pair_size_delta_ratio{0.75};
    double max_gap_cv{0.65};
    double min_preliminary_candidate_score{0.80};
    double min_preliminary_candidate_score_two_pair{0.87};
    double min_two_pair_mean_center_dy_px{12.0};
    double min_isolated_two_pair_order_span_ratio{0.25};
    double max_adjacent_face_order_gap_ratio{1.20};
    double max_resolved_window_length_fraction{1.45};
    double max_merged_window_pair_separation_px{45.0};
    int min_pairs_for_detection{2};
    double rmse_stable_threshold_px{8.0};
    double rmse_constrained_threshold_px{8.0};
    double max_pose_angle_rad{0.872664626};
    LdmGeometryConfig geometry{};
};

} // namespace fcs::L2::ldm
