#include "core/types.hpp"
#include <opencv2/imgproc.hpp>

namespace fcs {

ArmorDetection::ArmorDetection(
    std::array<cv::Point2f, 4> pts, ArmorName armor_name, ArmorColor color, float conf)
    : corners(pts)
    , name(armor_name)
    , color(color)
    , type(cls_to_armor_type(armor_name))
    , confidence(conf) {
    std::vector<cv::Point2f> pts_vec(corners.begin(), corners.end());
    rect = cv::boundingRect(pts_vec);
    area = static_cast<int>(cv::contourArea(pts_vec));
}

} // namespace fcs
