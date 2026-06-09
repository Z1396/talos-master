#pragma once

namespace fcs::rune {

struct RuneDetectorConfig {
    bool draw{false};
    bool debug_mode{false}; // L2层调试模式，启用时输出详细错误信息
    int arrow_threshold{170};
    int target_threshold{130};
    int rcenter_threshold{120};

    // 流水灯参数
    double min_roundness{0.6};
    double max_roundness{1.0};
    double min_arrow_light_area{10.0};
    double max_arrow_light_area{250.0};
    double min_arrow_light_aspect_ratio{1.5};
    double max_arrow_light_aspect_ratio{5.0};

    // ROI参数
    double global_roi_length_ratio{1.5};
    double local_roi_distance_ratio{6.0};
    int local_roi_width{150};

    // R中心参数
    double max_center_aspect_ratio{3.0};

    // 靶参数
    int min_target_light_area{400};
    int max_target_light_area{30000};
    int min_target_light_contour_area{300};
    int max_target_light_contour_area{10000};
    double min_target_light_aspect_ratio{0.5};
    double max_target_light_aspect_ratio{1.5};
    float min_target_roi_area{100.0f};
};

} // namespace fcs::rune
