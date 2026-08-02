#include "L2_perception/rune/rune_detector.hpp"

#include "L2_perception/rune/rune_config.hpp"
#include "L2_perception/rune/rune_detector_types.hpp"
#include "core/armor_types.hpp"
#include "frame.hpp"
#include "matrix.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <ranges>
#include <spdlog/spdlog.h>
#include <utility>
#include <vector>

namespace angles {
inline auto shortest_angular_distance(double from, double to) -> double {
    const double diff = std::remainder(to - from, 2.0 * M_PI);
    return diff;
}
} // namespace angles

namespace fyt::rune {

bool inRect(const cv::Point2f& p, const cv::Rect2f& rect) {
    return (
        p.x >= rect.x && p.x <= rect.x + rect.width && p.y >= rect.y
        && p.y <= rect.y + rect.height);
}

void resetRoi(cv::Rect2f& rect, int rows, int cols) {
    // 调整左上角点的坐标
    rect.x = rect.x < 0 ? 0.0f : rect.x >= static_cast<float>(cols) ? static_cast<float>(cols - 1) : rect.x;
    rect.y = rect.y < 0 ? 0.0f : rect.y >= static_cast<float>(rows) ? static_cast<float>(rows - 1) : rect.y;
    // 调整长宽
    rect.width =
        rect.x + rect.width >= static_cast<float>(cols) ? static_cast<float>(cols) - rect.x - 1.0f : rect.width;
    rect.height =
        rect.y + rect.height >= static_cast<float>(rows) ? static_cast<float>(rows) - rect.y - 1.0f : rect.height;
    // 此时可能出现 width 或 height 小于 0 的情况，因此需要将其置为 0
    if (rect.width < 0) {
        rect.width = 0;
    }
    if (rect.height < 0) {
        rect.height = 0;
    }
}

/*处理图片*/
void RuneDetector::processPictures(const cv::Mat& src, fcs::ArmorColor color) {
    // 二值化+开闭运算
    std::vector<cv::Mat> ch;
    cv::split(src, ch);               // B,G,R
    cv::Mat img;
    cv::Mat blue{ch[0]}, red{ch[2]};
    if (color == fcs::ArmorColor::Red) {
        cv::subtract(red, blue, img); // R - B
    } else if (color == fcs::ArmorColor::Blue) {
        cv::subtract(blue, red, img); // B - R
    }

    cv::threshold(img, arrowImg, params_.arrow_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(img, targetImg, params_.target_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(img, rCenterImg, params_.rcenter_threshold, 255, cv::THRESH_BINARY);
}

/*求点到两个点拟合出直线的距离*/
float pointLineDistance(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
    float A = b.y - a.y;
    float B = a.x - b.x;
    float C = b.x * a.y - a.x * b.y;
    return std::fabs(A * p.x + B * p.y + C) / std::sqrt(A * A + B * B);
}

/*求点到一条直线的距离*/
inline float pointLineDistance(const cv::Point2f& point, const cv::Vec4f& line) {
    float vx = line[0], vy = line[1];
    float x0 = line[2], y0 = line[3];
    float px = point.x, py = point.y;

    // 点到直线距离公式
    float distance = std::abs((x0 - px) * vy - (y0 - py) * vx);
    return distance;
}

/*设置流水灯的参数*/
bool Arrow::set(const Light& light) {
    std::vector<cv::Point2f> arrowPoints;
    contour = light.contour;
    rotated = cv::minAreaRect(contour);

    center = rotated.center;
    // 较长的一边为height，较短的一边为width
    size.height = rotated.size.height;
    size.width  = rotated.size.width;
    // RotatedRect::angle 范围为 -90~0. 这里根据长宽长度关系，将角度扩展到 -90~90
    if (size.height < size.width) {
        angle = rotated.angle;
        // 长的为 length
        std::swap(size.height, size.width);
    } else {
        angle = rotated.angle + 90;
    }
    ratio = size.height / size.width;
    area  = size.height * size.width;
    return true;
}

/*设置R标中心的参数*/
void CenterR::set(const Light& light) {
    this->light  = light;
    boundingRect = cv::boundingRect(light.contour);
    center       = light.center;
}

/*设置靶的参数*/
void Target::set(const Light& l) {
    center  = l.center;
    contour = l.contour;
}

Light::Light(const std::vector<cv::Point>& cnt, const cv::Rect2f& localRoi)
    : contour(cnt)
    , contourArea(cv::contourArea(cnt))
    , rotated(cv::minAreaRect(cnt)) {

    // 长的为 height ，短的为 width
    size.width = rotated.size.width, size.height = rotated.size.height;
    if (size.width > size.height) {
        std::swap(size.width, size.height);
    }
    ratio     = size.height / size.width;
    center    = rotated.center;
    angle     = rotated.angle;
    area      = rotated.size.width * rotated.size.height;
    float len = static_cast<float>(cv::arcLength(contour, true));
    roundness = (4 * CV_PI * contourArea) / (len * len);

    center += localRoi.tl();
}

void RuneDetector::setLocalRoi() {
    targetROIs.clear();

    // 临时保存所有 arrow 的上下 ROI
    std::vector<cv::Rect2f> rawUpROIs;
    std::vector<cv::Rect2f> rawDownROIs;

    for (const auto& arrow : arrows) {
        double distance{arrow.size.width * params_.local_roi_distance_ratio};
        float width{static_cast<float>(params_.local_roi_width)};

        float x = static_cast<float>(distance * std::cos(arrow.angle * CV_PI / 180.0f));
        float y = static_cast<float>(distance * std::sin(arrow.angle * CV_PI / 180.0f));

        cv::Point2f centerUp{arrow.center.x + x, arrow.center.y + y};
        cv::Point2f centerDown{arrow.center.x - x, arrow.center.y - y};

        cv::RotatedRect rectUp{centerUp, cv::Size2f(width, width), (float)arrow.angle};
        cv::RotatedRect rectDown{centerDown, cv::Size2f(width, width), (float)arrow.angle};

        std::array<std::array<cv::Point2f, 4>, 2> roiPoints;
        rectUp.points(roiPoints.at(0).begin());
        rectDown.points(roiPoints.at(1).begin());

        for (const auto& pts : roiPoints) {
            std::vector<cv::Point> ip;
            for (auto& p : pts)
                ip.emplace_back((int)p.x, (int)p.y);
            cv::fillConvexPoly(localMask, ip, cv::Scalar(255));
        }

        cv::Rect2f targetRoi(centerUp.x - width * 0.5f, centerUp.y - width * 0.5f, width, width);
        cv::Rect2f rcenterRoi(
            centerDown.x - width * 0.5f, centerDown.y - width * 0.5f, width, width);

        resetRoi(targetRoi, image_height_, image_width_);
        resetRoi(rcenterRoi, image_height_, image_width_);

        rawUpROIs.emplace_back(targetRoi);
        rawDownROIs.emplace_back(rcenterRoi);
    }

    const int N = static_cast<int>(arrows.size());

    if (N == 1) {
        cv::Rect2f upROI   = rawUpROIs[0];
        cv::Rect2f downROI = rawDownROIs[0];

        cv::Rect2f downGlobal(downROI.x, downROI.y, downROI.width, downROI.height);

        if (inRect(rcenter.center, downGlobal)) {
            centerRoi  = downROI;
            targetROIs = {upROI};
        } else {
            centerRoi  = upROI;
            targetROIs = {downROI};
        }
        return;
    }

    if (N == 2) {
        cv::Rect2f A = rawUpROIs[0];
        cv::Rect2f B = rawDownROIs[0];
        cv::Rect2f C = rawUpROIs[1];
        cv::Rect2f D = rawDownROIs[1];

        double maxArea = 0.0;
        cv::Rect2f best1, best2;

        // Pair 1: A & C
        {
            cv::Rect2f inter = A & C;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = A;
                best2   = C;
            }
        }

        // Pair 2: A & D
        {
            cv::Rect2f inter = A & D;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = A;
                best2   = D;
            }
        }

        // Pair 3: B & C
        {
            cv::Rect2f inter = B & C;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = B;
                best2   = C;
            }
        }

        // Pair 4: B & D
        {
            cv::Rect2f inter = B & D;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = B;
                best2   = D;
            }
        }

        cv::Rect2f inter = best1 & best2;

        float w    = inter.width;
        float h    = inter.height;
        float side = std::max(w, h); // 正方形边长 = 最大边

        // 以 inter 的中心为正方形中心
        cv::Point2f center = (inter.tl() + inter.br()) * 0.5f;

        // 生成正方形 ROI
        centerRoi = cv::Rect2f(center.x - side / 2, center.y - side / 2, side, side);

        resetRoi(centerRoi, image_height_, image_width_);

        std::vector<cv::Rect2f> all = {A, B, C, D};

        for (auto& r : all) {
            if (r != best1 && r != best2) {
                targetROIs.push_back(r);
            }
        }
    }
}

bool findArrow(std::vector<Arrow>& arrows, const std::vector<Light>& lights, const cv::Mat& img) {
    if (lights.empty())
        return false;

    for (const auto& light : lights) {

        Arrow arrow;

        cv::Rect roi = light.rotated.boundingRect();

        roi &= cv::Rect(0, 0, img.cols, img.rows);

        if (roi.area() == 0)
            continue;

        cv::Mat roi_img = img(roi);

        std::vector<std::vector<cv::Point>> cnts;

        cv::findContours(roi_img, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (cnts.size() <= 5)
            continue;

        bool ok = arrow.set(light);

        if (!ok)
            continue;

        arrows.emplace_back(std::move(arrow));
    }

    return !arrows.empty();
}

void findArrowLights(
    cv::Mat& binary, std::vector<Light>& lights, const fcs::rune::RuneDetectorConfig& params) {

    cv::Rect2f roi = {0, 0, static_cast<float>(binary.cols), static_cast<float>(binary.rows)};

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto& contour : contours) {

        cv::RotatedRect rect = cv::minAreaRect(contour);

        double area = rect.size.width * rect.size.height;

        double ratio = std::max(rect.size.width, rect.size.height)
                     / std::min(rect.size.width, rect.size.height);

        if (area < 0.25 * params.min_arrow_light_area || area > 0.25 * params.max_arrow_light_area)
            continue;

        if (ratio < params.min_arrow_light_aspect_ratio
            || ratio > params.max_arrow_light_aspect_ratio)
            continue;

        // map contour back to original resolution
        for (auto& pt : contour)
            pt *= 2;

        lights.emplace_back(contour, roi);
    }
}

bool RuneDetector::detectAllArrows() {
    cv::Mat binImg;
    cv::resize(arrowImg, binImg, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    cv::morphologyEx(binImg, binImg, cv::MORPH_CLOSE, close_kernel_);
    cv::threshold(binImg, binImg, 50, 255, cv::THRESH_BINARY);

    arrows.clear();

    std::vector<Light> lights;
    findArrowLights(binImg, lights, params_);

    bool found = findArrow(arrows, lights, arrowImg);

    return found;
}

static std::vector<cv::Point> normalizeContour(const std::vector<cv::Point>& cnt) {
    cv::Rect box = cv::boundingRect(cnt);
    std::vector<cv::Point> out;
    out.reserve(cnt.size());

    for (auto& p : cnt) {
        float nx = float(p.x - box.x) / float(box.width);
        float ny = float(p.y - box.y) / float(box.height);
        out.emplace_back(int(nx * 1000), int(ny * 1000));
    }
    return out;
}

bool findCenterLights(
    const cv::Mat& image, Light& light, const cv::Rect2f& localRoi,
    const std::vector<cv::Point>& r_template_norm, const fcs::rune::RuneDetectorConfig& params) {

    if (r_template_norm.empty()) {
        if (params.debug_mode) {
            SPDLOG_ERROR("findCenterLights: R template not loaded");
        }
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = 1e9;
    std::vector<cv::Point> best_cnt;

    for (auto& cnt : contours) {
        if (cv::contourArea(cnt) < 50 || cv::contourArea(cnt) > 1000)
            continue;
        auto cnt_norm = normalizeContour(cnt);
        double score =
            cv::matchShapes(cnt_norm, r_template_norm, cv::CONTOURS_MATCH_I1, 0); // 差异度
        if (score < best_score) {
            best_score = score;
            best_cnt   = cnt;
        }
    }

    if (!best_cnt.empty() && best_score < 0.5) {
        light = Light(best_cnt, localRoi);
        if (light.roundness > 0.8) {
            return false;
        }
        return true;

    } else {
        if (params.debug_mode) {
            SPDLOG_ERROR(
                "findCenterLights: Best contour match score too high. score = {}", best_score);
        }
        return false;
    }
}

/*寻找R中心*/
bool RuneDetector::detectCenterR() {
    cv::Mat imageCenter = (rCenterImg & localMask)(centerRoi);
    Light light;

    if (findCenterLights(imageCenter, light, centerRoi, r_template_norm_, params_) == false) {
        return false;
    }

    rcenter.set(light);

    return true;
}

bool sameTarget(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2) {
    // 判断面积比
    double areaRatio{cv::contourArea(contour1) / cv::contourArea(contour2)};
    if ((areaRatio >= 0.8 && areaRatio <= 1.20) == false) {
        return false;
    }

    cv::RotatedRect rotated1 = cv::minAreaRect(contour1);
    cv::RotatedRect rotated2 = cv::minAreaRect(contour2);

    if (rotated1.size.height < rotated1.size.width) {
        cv::swap(rotated1.size.height, rotated1.size.width);
    }
    if (rotated2.size.height < rotated2.size.width) {
        cv::swap(rotated2.size.height, rotated2.size.width);
    }

    // 判断距离
    double distance{cv::norm(rotated1.center - rotated2.center)};
    double maxDistance{1.8 * (rotated1.size.width + rotated2.size.width)};
    if (distance > maxDistance) {
        return false;
    }

    // 判断轮廓旋转矩形的长宽
    float w1 = rotated1.size.width;
    float w2 = rotated2.size.width;
    float h1 = rotated1.size.height;
    float h2 = rotated2.size.height;

    float w_ratio = std::max(w1, w2) / std::min(w1, w2);
    float h_ratio = std::max(h1, h2) / std::min(h1, h2);

    if (w_ratio > 1.25f || h_ratio > 1.25f) {
        return false;
    }

    float c1_ratio = h1 / w1;
    float c2_ratio = h2 / w2;

    if (c1_ratio <= 1.2f || c2_ratio <= 1.2f) {
        return false;
    }
    return true;
}

void addReferRuneCenter(const cv::Point2f& rc, Points& target, bool debug_mode = false) {

    if (target.corners.size() != 4) {
        if (debug_mode) {
            SPDLOG_ERROR(
                "rune : addReferRuneCenter: target.corners.size() != 4. size={}",
                target.corners.size());
        }
        return;
    }

    cv::Point2f down_vec = rc - target.center;
    float norm           = std::sqrt(down_vec.x * down_vec.x + down_vec.y * down_vec.y);
    if (norm < 1e-6f) {
        if (debug_mode) {
            SPDLOG_ERROR("rune : addReferRuneCenter: norm too small. norm={}", norm);
        }
        return;
    }

    float angle_ref = std::atan2(down_vec.y, down_vec.x);

    // 获取4个点在旋转后的角度
    struct Node {
        float ang;
        cv::Point2f p;
    };
    std::vector<Node> arr;
    arr.reserve(4);

    for (auto& p : target.corners) {
        cv::Point2f v = p - target.center;

        // 旋转坐标，使 down_vec 对齐 angle=0
        float ang = std::atan2(v.y, v.x) - angle_ref;

        // 归一化到 (-π, π]
        while (ang <= -CV_PI)
            ang += 2 * CV_PI;
        while (ang > CV_PI)
            ang -= 2 * CV_PI;

        arr.push_back({ang, p});
    }

    // 按角度排序（从 -π 到 π）
    std::sort(arr.begin(), arr.end(), [](const Node& a, const Node& b) { return a.ang < b.ang; });

    // 准备象限变量并标记
    cv::Point2f lu(0, 0), ru(0, 0), rd(0, 0), ld(0, 0);
    bool has_lu = false, has_ru = false, has_rd = false, has_ld = false;

    for (const auto& n : arr) {
        float a = n.ang;

        if (a > CV_PI / 2 && a <= CV_PI) {
            lu     = n.p;
            has_lu = true;
        } else if (a > 0 && a <= CV_PI / 2) {
            ru     = n.p;
            has_ru = true;
        } else if (a > -CV_PI / 2 && a <= 0) {
            rd     = n.p;
            has_rd = true;
        } else {                           // a > -CV_PI && a <= -CV_PI/2
            ld     = n.p;
            has_ld = true;
        }
    }

    std::array<cv::Point2f, 4> ordered;

    if (has_lu && has_ru && has_rd && has_ld) {
        ordered[0] = lu;
        ordered[1] = ru;
        ordered[2] = rd;
        ordered[3] = ld;
        target.corners.assign(ordered.begin(), ordered.end());
        return;
    }

    float angle     = 3.0f * CV_PI / 4.0f; // 135°
    int best_idx    = 0;
    float best_diff = std::numeric_limits<float>::max();
    for (int i = 0; i < (int)arr.size(); ++i) {
        float d =
            static_cast<float>(std::fabs(angles::shortest_angular_distance(angle, arr[i].ang)));
        if (d < best_diff) {
            best_diff = d;
            best_idx  = i;
        }
    }

    for (int i = 0; i < 4; ++i) {
        int idx    = (best_idx + i) % 4;
        ordered[i] = arr[idx].p;
    }

    target.corners.assign(ordered.begin(), ordered.end());
}

inline bool markRuneTarget(
    const std::vector<cv::Point>& arrow, const cv::Point2f& rc, cv::Point2f target,
    const std::vector<std::vector<cv::Point>>& contours, const std::vector<cv::Vec4i>& hierarchy,
    Target& result, bool debug_mode = false) {

    if (hierarchy.empty()) {
        return false;
    }

    struct ContourInfo {
        int index;
        double area;
        float ratio;
        double width;
        double height;
        double dist;
    };

    std::vector<int> originalIndices;
    std::vector<std::vector<cv::Point>> c;
    for (size_t k = 0; k < contours.size(); k++) {
        bool including = false;
        for (auto& p : contours[k]) {
            if (cv::pointPolygonTest(arrow, p, false) >= 0) {
                including = true;
                break;
            }
        }
        if (!including) {
            c.emplace_back(contours[k]);
            originalIndices.push_back(static_cast<int>(k));
        }
    }

    std::vector<int> labels;
    cv::partition(c, labels, sameTarget);

    std::map<int, std::vector<ContourInfo>> groups;
    for (size_t i = 0; i < c.size(); i++) {
        auto rect = cv::minAreaRect(c[i]);
        if (rect.size.height < rect.size.width) {
            cv::swap(rect.size.height, rect.size.width);
        }
        double area = cv::contourArea(c[i]);
        float ratio = rect.size.height / rect.size.width;
        double dist = cv::norm(rect.center - target);
        groups[labels[i]].push_back(
            {originalIndices[i], area, ratio, rect.size.width, rect.size.height, dist});
    }

    if (groups.empty()) {
        SPDLOG_ERROR("[ERROR] markRuneTarget: groups empty");
        return false;
    }

    // std::string out = "[ ";
    // for (auto& [label, vec] : groups) {
    //     out += "[ ";
    //     for (auto& info : vec) {
    //         out += fmt::format("{}: area={:.1f}, ratio={:.2f}, dist={:.1f}", info.index,
    //         info.area, info.ratio, info.dist); if (&info != &vec.back())
    //             out += ", ";
    //     }
    //     out += " ]";
    //     out += fmt::format(" {} ", vec.size());
    //     if (label != groups.rbegin()->first)
    //         out += ", ";
    // }
    // out += " ]";
    // SPDLOG_INFO("{}", out);

    std::vector<std::vector<cv::Point>> res;
    for (auto& [label, vec] : groups) {
        if (vec.size() > 3 && vec.size() < 5) {
            for (size_t i = 0; i < c.size(); i++) {
                if (labels[i] == label) {
                    res.emplace_back(c[i]);
                }
            }
        }
    }

    if (res.size() < 4) {
        SPDLOG_ERROR("res.size() < 4, not enough contours to define key points");
        return false;
    }

    Points points;

    for (const auto& cnt : res) {

        cv::Moments m = cv::moments(cnt);
        if (m.m00 == 0) {
            SPDLOG_WARN("m.m00 == 0");
            continue;
        }

        // 质心
        cv::Point2f center(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
        points.corners.emplace_back(center);
    }
    points.center = target;

    if (points.corners.size() < 4) {
        SPDLOG_ERROR("points.corners.size() < 4, cannot assign key points");
        return false;
    }

    addReferRuneCenter(rc, points, debug_mode);

    result.keypnt.lu = points.corners[0];
    result.keypnt.ru = points.corners[1];
    result.keypnt.rd = points.corners[2];
    result.keypnt.ld = points.corners[3];

    return true;
}

void RuneDetector::setKeyPoints() {
    for (size_t i = 0; i < targets.size(); i++) {

        cv::Mat detect = (targetImg & localMask)(targetROIs[i]);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(detect, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> arrow_contour;
        for (const auto& cnt_pt : arrows[i].contour) {
            cv::Point pt = cnt_pt - cv::Point(targetROIs[i].tl());
            arrow_contour.emplace_back(pt);
        }

        if (contours.empty()) {
            targets[i].initKey = false;
            continue;
        }

        cv::Point2f target_center = targets[i].center - targetROIs[i].tl();
        cv::Point2f r_center      = rcenter.center - targetROIs[i].tl();
        Target rune_target;
        bool ok = markRuneTarget(
            arrow_contour, r_center, target_center, contours, hierarchy, rune_target,
            params_.debug_mode);

        if (!ok) {
            targets[i].initKey = false;
            continue;
        }
        targets[i].keypnt.lu = rune_target.keypnt.lu + targetROIs[i].tl();
        targets[i].keypnt.ru = rune_target.keypnt.ru + targetROIs[i].tl();
        targets[i].keypnt.rd = rune_target.keypnt.rd + targetROIs[i].tl();
        targets[i].keypnt.ld = rune_target.keypnt.ld + targetROIs[i].tl();

        targets[i].initKey = true;

        if (targets.size() == 1 && targets[0].initKey) {
            std::vector<cv::Point> out_cnt;
            double max_area = -1.0;
            for (size_t k = 0; k < contours.size(); k++) {
                if (hierarchy[k][3] == -1 && hierarchy[k][2] != -1) {
                    double area = cv::contourArea(contours[k]);
                    if (area > max_area) {
                        max_area = area;
                        out_cnt  = contours[k];
                    }
                }
            }

            if (out_cnt.empty()) {
                targets[i].other = false;
                return;
            }

            cv::Point2f offset = targetROIs[i].tl();
            for (auto& pnt : out_cnt) {
                pnt = cv::Point(
                    static_cast<int>(static_cast<float>(pnt.x) + offset.x),
                    static_cast<int>(static_cast<float>(pnt.y) + offset.y));
            }

            // 计算 4 个中点
            cv::Point2f top_mid   = (targets[i].keypnt.lu + targets[i].keypnt.ru) * 0.5f;
            cv::Point2f bot_mid   = (targets[i].keypnt.rd + targets[i].keypnt.ld) * 0.5f;
            cv::Point2f left_mid  = (targets[i].keypnt.lu + targets[i].keypnt.ld) * 0.5f;
            cv::Point2f right_mid = (targets[i].keypnt.ru + targets[i].keypnt.rd) * 0.5f;

            targets[i].other_point2f.resize(4);

            // 辅助函数：计算射线与轮廓的交点
            auto find_intersection = [&](const cv::Point2f& origin,
                                         const cv::Point2f& dir) -> cv::Point2f {
                const cv::Point2f& ext_p1 = origin;
                cv::Point2f ext_p2        = origin + dir * 1000.0f;

                cv::Point2f best_pt(-1, -1);
                bool found     = false;
                float max_dist = 0.0f;

                for (size_t j = 0; j < out_cnt.size(); j++) {
                    cv::Point2f a(
                        static_cast<float>(out_cnt[j].x), static_cast<float>(out_cnt[j].y));
                    cv::Point2f b(
                        static_cast<float>(out_cnt[(j + 1) % out_cnt.size()].x),
                        static_cast<float>(out_cnt[(j + 1) % out_cnt.size()].y));

                    cv::Point2f d1 = ext_p2 - ext_p1;
                    cv::Point2f d2 = b - a;
                    float cross    = d1.x * d2.y - d1.y * d2.x;
                    if (std::abs(cross) < 1e-10f)
                        continue;

                    cv::Point2f diff = a - ext_p1;
                    float t          = (diff.x * d2.y - diff.y * d2.x) / cross;
                    float u          = (diff.x * d1.y - diff.y * d1.x) / cross;

                    if (u >= 0.0f && u <= 1.0f && t >= 0.0f) {
                        cv::Point2f pt = ext_p1 + t * d1;
                        float dist     = (pt - origin).dot(dir);
                        if (dist > max_dist) {
                            max_dist = dist;
                            best_pt  = pt;
                            found    = true;
                        }
                    }
                }
                if (!found) {
                    targets[0].other = false;
                    return {-1, -1};
                }

                return best_pt;
            };

            targets[i].other = true;

            cv::Point2f dir_up = top_mid - bot_mid;
            float len_up       = std::sqrt(dir_up.dot(dir_up));
            if (len_up > 1e-6f) {
                dir_up /= len_up;
                targets[i].other_point2f[0] = find_intersection(top_mid, dir_up);
            }

            cv::Point2f dir_down = bot_mid - top_mid;
            float len_down       = std::sqrt(dir_down.dot(dir_down));
            if (len_down > 1e-6f) {
                dir_down /= len_down;
                targets[i].other_point2f[1] = find_intersection(bot_mid, dir_down);
            }

            cv::Point2f dir_left = left_mid - right_mid;
            float len_left       = std::sqrt(dir_left.dot(dir_left));
            if (len_left > 1e-6f) {
                dir_left /= len_left;
                targets[i].other_point2f[2] = find_intersection(left_mid, dir_left);
            }

            cv::Point2f dir_right = right_mid - left_mid;
            float len_right       = std::sqrt(dir_right.dot(dir_right));
            if (len_right > 1e-6f) {
                dir_right /= len_right;
                targets[i].other_point2f[3] = find_intersection(right_mid, dir_right);
            }
        }
    }
}

/*通过面积，轮廓面积，长宽比，判断目标是否符合*/
bool findTargetLights(
    const cv::Mat& image, std::vector<Light>& lights, const cv::Rect2f& localRoi,
    const fcs::rune::RuneDetectorConfig& params) {
    lights.clear();
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        if (params.debug_mode) {
            SPDLOG_ERROR("findTargetLights: No contours found");
        }
        return false;
    }
    for (const auto& contour : contours) {
        Light light(contour, localRoi);

        if ((light.roundness >= params.min_roundness && light.roundness <= params.max_roundness)
            == false) {
            continue;
        }

        lights.emplace_back(std::move(light));
    }

    if (lights.empty()) {
        if (params.debug_mode) {
            SPDLOG_ERROR(" findTargetLights: No lights added after filtering");
        }
        return false;
    }

    return true;
}

bool findTarget(
    Target& target, const std::vector<Light>& frames, const fcs::rune::RuneDetectorConfig& params) {

    if (frames.empty()) {

        if (params.debug_mode) {
            SPDLOG_ERROR(" findTarget: No targets passed roundness check");
        }
        return false;
    }
    Light light{
        *std::max_element(frames.begin(), frames.end(), [](const Light& l1, const Light& l2) {
            return l1.roundness < l2.roundness;
        })};

    target.set(light);

    return true;
}

bool RuneDetector::detectAllTargets() {
    std::vector<Light> lights;
    targets.clear();
    bool reverse   = false;
    cv::Mat backup = (targetImg & localMask)(centerRoi);

    if (targetROIs.size() == 1) {
        cv::Rect2f targetRoi = targetROIs[0];
        cv::Mat detect       = (targetImg & localMask)(targetRoi);

    RESTART:
        if (!findTargetLights(detect, lights, targetRoi, params_)) {
            if (!reverse) {
                std::swap(detect, backup);
                std::swap(targetRoi, centerRoi);
                reverse = true;
                goto RESTART;
            } else {
                return false;
            }
        }

        Target t;
        if (!findTarget(t, lights, params_)) {
            if (!reverse) {
                std::swap(detect, backup);
                std::swap(targetRoi, centerRoi);
                reverse = true;
                goto RESTART;
            } else {
                return false;
            }
        }

        targets.push_back(t);
    } else {
        for (const auto& targetRoi : targetROIs) {
            lights.clear();
            cv::Mat detect = (targetImg & localMask)(targetRoi);

            if (findTargetLights(detect, lights, targetRoi, params_) == false) {
                continue;
            }

            Target t;
            if (findTarget(t, lights, params_) == false) {
                continue;
            }

            targets.emplace_back(t);
        }
    }
    if (targets.empty()) {
        if (params_.debug_mode) {
            SPDLOG_ERROR(" detectAllTargets: No targets detected after all checks");
        }
        return false;
    }

    return true;
}

bool RuneDetector::detect(const cv::Mat& frame, fcs::ArmorColor color) {
    image_width_  = frame.cols;
    image_height_ = frame.rows;

    localMask = cv::Mat::zeros(image_height_, image_width_, CV_8U);

    bool reverse = false;

    // Preprocessing
    processPictures(frame, color);

    // Arrow detection
    if (detectAllArrows() == false) {
        arrows.clear();
        targets.clear();
        rcenter.center = cv::Point2f(0, 0);
        status         = Status::ARROW_FAILURE;
        goto FAIL;
    }

    // Set local ROI
    setLocalRoi();

    if (targetROIs.empty()) {
        status = Status::FAILURE;
        goto FAIL;
    }

RESTART:

    // Target detection
    if (detectAllTargets() == false) {
        targets.clear();
        status = Status::ARMOR_FAILURE;
        goto FAIL;
    }

    // Center detection
    if (targetROIs.size() == 1) {
        if (detectCenterR() == false) {
            status         = Status::CENTER_FAILURE;
            rcenter.center = cv::Point2f(0, 0);
            if (reverse == false) {
                std::swap(centerRoi, targetROIs[0]);
                reverse = true;
                goto RESTART;
            } else {
                goto FAIL;
            }
        }
    } else {
        if (detectCenterR() == false) {
            status         = Status::CENTER_FAILURE;
            rcenter.center = cv::Point2f(0, 0);
            goto FAIL;
        }
    }

    // Calculate angles
    for (auto& target : targets) {
        auto vec     = target.center - rcenter.center;
        double angle = std::atan2(vec.y, vec.x);
        target.angle = angle;
    }

    status = Status::SUCCESS;

    return true;
FAIL:
    return false;
}

RuneDetector::RuneDetector(const fcs::rune::RuneDetectorConfig& params)
    : status(Status::FAILURE)
    , params_(params)
    , close_kernel_(cv::getStructuringElement(0, cv::Size(5, 5))) {
    rcenter.center = cv::Point2f(0, 0);

    // Load R template once
    cv::FileStorage fs("config/r_template0.yml", cv::FileStorage::READ);
    cv::Mat R_mat;
    fs["R_contour"] >> R_mat;
    fs.release();

    std::vector<cv::Point> r_template_contour;
    if (!R_mat.empty()) {
        r_template_contour.reserve(R_mat.rows);
        for (int i = 0; i < R_mat.rows; i++) {
            cv::Vec<int, 2> v = R_mat.at<cv::Vec<int, 2>>(i, 0);
            r_template_contour.emplace_back(v[0], v[1]);
        }
        r_template_norm_ = normalizeContour(r_template_contour);
    }
}

inline double normalizeAngle0to2pi(double a) {
    a = std::fmod(a, 2 * M_PI);
    if (a < 0)
        a += 2 * M_PI;
    return a;
}

std::vector<double> angle_diffs = {
    0, 2 * M_PI / 5, 2 * M_PI / 5 * 2, 2 * M_PI / 5 * 3, 2 * M_PI / 5 * 4};

inline cv::Point3f rotateX(const cv::Point3f& p, double roll) {
    double c = std::cos(roll);
    double s = std::sin(roll);
    return {p.x, float(p.y * c - p.z * s), float(p.y * s + p.z * c)};
}

void addOther(
    const cv::Point2f& center, Target& one, const Target& other, std::vector<cv::Point2f>& points2d,
    std::vector<cv::Point3f>& points3d, bool debug_mode = false) {

    auto l1 = center - one.center;
    auto l2 = center - other.center;

    float a1 = std::atan2(l1.y, l1.x);
    float a2 = std::atan2(l2.y, l2.x);

    float d = a1 - a2;
    d       = static_cast<float>(normalizeAngle0to2pi(d));

    int id         = 0;
    double min_err = 1e9;
    for (size_t i = 0; i < angle_diffs.size(); i++) {
        double err = std::abs(angle_diffs[i] - d);
        if (err < min_err) {
            min_err = err;
            id      = static_cast<int>(i);
        }
    }

    if (id < 1) {
        if (debug_mode) {
            std::cerr << "id : " << id << std::endl;
        }
        return;
    }

    points2d.push_back(center);
    points2d.push_back(one.keypnt.lu);
    points2d.push_back(one.keypnt.ru);
    points2d.push_back(one.keypnt.rd);
    points2d.push_back(one.keypnt.ld);
    points2d.push_back(one.center);
    points2d.push_back(other.keypnt.lu);
    points2d.push_back(other.keypnt.ru);
    points2d.push_back(other.keypnt.rd);
    points2d.push_back(other.keypnt.ld);
    points2d.push_back(other.center);
    // ---------- 计算 roll 差 ----------
    double droll = -angle_diffs[id];
    one.droll    = droll;

    points3d.push_back(one.points3d[0]);
    points3d.push_back(one.points3d[1]);
    points3d.push_back(one.points3d[2]);
    points3d.push_back(one.points3d[3]);
    points3d.push_back(one.points3d[4]);
    points3d.push_back(one.points3d[5]);
    // ---------- 旋转后的 3D 点加入 ----------
    points3d.push_back(rotateX(one.points3d[1], droll));
    points3d.push_back(rotateX(one.points3d[2], droll));
    points3d.push_back(rotateX(one.points3d[3], droll));
    points3d.push_back(rotateX(one.points3d[4], droll));
    points3d.push_back(rotateX(one.points3d[5], droll));
}

std::expected<fast_tf::TransformMatrixd<rune, fast_tf::odom>, std::string> RuneDetector::solve(
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double timestamp_sec,
    const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::camera_optical>& odom_T_cam) {

    if (targets.empty()) {
        return std::unexpected("rune_detector_solve_targets_empty");
    }

    if (targets.size() == 1 && !targets[0].initKey) {
        return std::unexpected("Single target but initKey=false");
    }

    if (targets.size() == 2) {
        if (!targets[0].initKey && !targets[1].initKey) {
            return std::unexpected("solve: Both targets have initKey=false");
        }
        if (!targets[0].initKey) {
            std::swap(targets[0], targets[1]);
            targets.pop_back();
        } else if (!targets[1].initKey) {
            targets.pop_back();
        }
    }

    if (has_last_target_ && timestamp_sec - last_timestamp_ > 2) {
        has_last_target_ = false;
    }

    cv::Mat rvec, tvec;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;

    // ── Build 2D-3D correspondences ──
    if (targets.size() == 1) {
        image_points.emplace_back(rcenter.center);
        image_points.emplace_back(targets[0].keypnt.lu);
        image_points.emplace_back(targets[0].keypnt.ru);
        image_points.emplace_back(targets[0].keypnt.rd);
        image_points.emplace_back(targets[0].keypnt.ld);
        image_points.emplace_back(targets[0].center);
        if (targets[0].other) {
            image_points.emplace_back(targets[0].other_point2f[0]);
            image_points.emplace_back(targets[0].other_point2f[1]);
            image_points.emplace_back(targets[0].other_point2f[2]);
            image_points.emplace_back(targets[0].other_point2f[3]);

            object_points = targets[0].small_points3d;
        } else {
            object_points = targets[0].points3d;
        }

    } else if (targets.size() == 2) {
        // 选择 roll 最接近上一帧的靶作为主目标，保证追踪连续性
        if (has_last_target_) {
            double diff0 = std::abs(
                angles::shortest_angular_distance(last_tracked_target_angle_, targets[0].angle));
            double diff1 = std::abs(
                angles::shortest_angular_distance(last_tracked_target_angle_, targets[1].angle));
            if (diff1 < diff0) {
                std::swap(targets[0], targets[1]);
                std::swap(arrows[0], arrows[1]);
            }
        } else {
            if (targets[0].center.y < targets[1].center.y) {
                std::swap(targets[0], targets[1]);
                std::swap(arrows[0], arrows[1]);
            }
        }
        addOther(
            rcenter.center, targets[0], targets[1], image_points, object_points,
            params_.debug_mode);
    }

    if (object_points.size() != image_points.size() || object_points.size() < 4) {
        return std::unexpected("pnp point number invalid");
    }

    // ── Step 1: Undistort to normalized image plane ──
    std::vector<cv::Point2f> norm_points;
    cv::undistortPoints(image_points, norm_points, cameraMatrix, distCoeffs);

    // ── Step 2: Initial PnP (prior-seeded, then fallback) ──
    static const cv::Mat kIdentityK = cv::Mat::eye(3, 3, CV_64F);
    bool pnp_ok                     = false;

    // Try prior-based iterative solve (valid if < 0.5s old)
    if (pose_prior_.timestamp > 0 && (timestamp_sec - pose_prior_.timestamp) < 0.5) {
        rvec   = pose_prior_.rvec.clone();
        tvec   = pose_prior_.tvec.clone();
        pnp_ok = cv::solvePnPRansac(
            object_points, norm_points, kIdentityK, cv::Mat(), rvec, tvec, true,
            cv::SOLVEPNP_ITERATIVE);
    }

    // Fallback: standard method
    if (!pnp_ok) {
        if (targets.size() == 1) {
            pnp_ok = cv::solvePnPRansac(
                object_points, norm_points, kIdentityK, cv::Mat(), rvec, tvec, false,
                cv::SOLVEPNP_IPPE);
        } else {
            pnp_ok = cv::solvePnPRansac(
                object_points, norm_points, kIdentityK, cv::Mat(), rvec, tvec, false,
                cv::SOLVEPNP_SQPNP);
        }
    }

    if (!pnp_ok) {
        return std::unexpected("R点PnP解算失败");
    }

    // Reject degenerate PnP (NaN / Inf)
    {
        const double* rd = rvec.ptr<double>();
        const double* td = tvec.ptr<double>();
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(rd[i]) || !std::isfinite(td[i])) {
                return std::unexpected("R点PnP解算失败 (degenerate)");
            }
        }
    }

    // ── Step 3: Constrained Ceres refinement (roll + yaw + translation, pitch = 0) ──
    // R_cam_rune = R_cam_imu * R_z(yaw) * R_x(roll)  — no pitch
    {
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R_cam_rune;
        cv::cv2eigen(R_cv, R_cam_rune);
        Eigen::Vector3d t_cam_rune(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        const Eigen::Matrix3d R_imu_cam  = odom_T_cam.rotation();
        const Eigen::Matrix3d R_cam_imu  = R_imu_cam.transpose();
        const Eigen::Matrix3d R_imu_rune = R_imu_cam * R_cam_rune;

        // Extract roll and yaw from R_imu_rune = R_z(yaw) * R_x(roll)
        double roll           = std::atan2(R_imu_rune(2, 1), R_imu_rune(2, 2));
        double yaw            = std::atan2(R_imu_rune(1, 0), R_imu_rune(0, 0));
        double translation[3] = {t_cam_rune.x(), t_cam_rune.y(), t_cam_rune.z()};

        // Build Eigen point vectors
        std::vector<Eigen::Vector3d> obj_pts;
        obj_pts.reserve(object_points.size());
        for (const auto& p : object_points) {
            obj_pts.emplace_back(p.x, p.y, p.z);
        }
        std::vector<Eigen::Vector2d> img_pts;
        img_pts.reserve(norm_points.size());
        for (const auto& p : norm_points) {
            img_pts.emplace_back(p.x, p.y);
        }

        // Ceres problem — ownership transferred to Problem via AddResidualBlock
        ceres::Problem problem;
        auto* loss_function = new ceres::HuberLoss(1.0);

        for (size_t i = 0; i < obj_pts.size(); ++i) {
            ceres::CostFunction* cost =
                RuneConstrainedReprojError::Create(obj_pts[i], img_pts[i], R_cam_imu);
            problem.AddResidualBlock(cost, loss_function, &roll, &yaw, translation);
        }

        problem.SetParameterLowerBound(&yaw, 0, -M_PI);
        problem.SetParameterUpperBound(&yaw, 0, M_PI);
        problem.SetParameterLowerBound(translation, 2, 1e-3);

        ceres::Solver::Options options;
        options.linear_solver_type           = ceres::DENSE_QR;
        options.max_num_iterations           = 20;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        const bool t_finite = std::isfinite(translation[0]) && std::isfinite(translation[1])
                           && std::isfinite(translation[2]) && translation[2] > 1e-3;
        if (summary.IsSolutionUsable() && std::isfinite(roll) && std::isfinite(yaw) && t_finite) {
            const Eigen::Matrix3d R_refined =
                R_cam_imu * rune_yaw_rotation(yaw) * rune_roll_rotation(roll);
            cv::Mat R_refined_cv;
            cv::eigen2cv(R_refined, R_refined_cv);
            cv::Rodrigues(R_refined_cv, rvec);
            tvec = (cv::Mat_<double>(3, 1) << translation[0], translation[1], translation[2]);
        }
    }

    // ── Reprojection error (pixel space for debug) ──
    debug_detected_points_ = image_points;
    cv::projectPoints(
        object_points, rvec, tvec, cameraMatrix, distCoeffs, debug_reprojected_points_);

    // double total_error = 0.0;
    // bool is_small_rune = (targets.size() == 1);

    // for (size_t i = 0; i < image_points.size(); ++i) {
    //     double err = cv::norm(image_points[i] - debug_reprojected_points_[i]);
    //     total_error += err;

    //     // 打印每个点的误差（调试模式）
    //     if (params_.debug_mode) {
    //         const char* label = "";
    //         if (is_small_rune) {
    //             switch (i) {
    //             case 0: label = "R标中心"; break;
    //             case 1: label = "LU(左上)"; break;
    //             case 2: label = "RU(左下)"; break;
    //             case 3: label = "RD(右下)"; break;
    //             case 4: label = "LD(右上)"; break;
    //             case 5: label = "靶中心"; break;
    //             case 6: label = "other_top"; break;
    //             case 7: label = "other_bot"; break;
    //             case 8: label = "other_left"; break;
    //             case 9: label = "other_right"; break;
    //             default: label = "?"; break;
    //             }
    //         } else {
    //             switch (i) {
    //             case 0: label = "R标中心"; break;
    //             case 1: label = "LU"; break;
    //             case 2: label = "RU"; break;
    //             case 3: label = "RD"; break;
    //             case 4: label = "LD"; break;
    //             case 5: label = "靶1中心"; break;
    //             case 6: label = "靶2_LU"; break;
    //             case 7: label = "靶2_RU"; break;
    //             case 8: label = "靶2_RD"; break;
    //             case 9: label = "靶2_LD"; break;
    //             case 10: label = "靶2中心"; break;
    //             default: label = "?"; break;
    //             }
    //         }
    //         SPDLOG_ERROR(
    //             "[PnP] [{}] {} err={:.2f}px  det=({:.1f},{:.1f})  rep=({:.1f},{:.1f})", i,
    //             label, err, image_points[i].x, image_points[i].y,
    //             debug_reprojected_points_[i].x, debug_reprojected_points_[i].y);
    //     }
    // }
    // debug_reproj_error_ = total_error / static_cast<double>(image_points.size());
    // if (params_.debug_mode) {
    //     SPDLOG_ERROR("[PnP] 平均误差: {:.2f}px (共{}点)", debug_reproj_error_,
    //     image_points.size());
    // }

    auto pose = fast_tf::TransformMatrixd<rune, fast_tf::camera_optical>::from_pnp(rvec, tvec);

    // Save PnP prior for next frame
    pose_prior_.rvec      = rvec.clone();
    pose_prior_.tvec      = tvec.clone();
    pose_prior_.timestamp = timestamp_sec;

    // Save tracking state
    if (!targets.empty()) {
        last_tracked_target_center_ = targets[0].center;
        last_tracked_target_angle_  = targets[0].angle;
        has_last_target_            = true;
        last_timestamp_             = timestamp_sec;
    }

    return fast_tf::TransformMatrixd<rune, fast_tf::odom>(odom_T_cam.matrix() * pose.matrix());
}

} // namespace fyt::rune
