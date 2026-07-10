// ===========================================================================
// armor.hpp - 装甲板检测与位姿估计（Talos 简化版）
//
// 设计要点：
//   1. 输入：图像中的 4 个角点像素坐标
//   2. PnP：通过 4 个 2D-3D 对应求解目标 3D 位姿
//   3. 输出：3D 位置 (x, y, z) + 朝向角 yaw
//
// 简化处理：
//   - 假设相机内参已知（焦距 fx/fy, 主点 cx/cy）
//   - 使用 IPnP 简化算法（假设 yaw=0 平面正对相机时的投影）
//   - 真实 Talos 使用 OpenCV solvePnP + BA 优化
// ===========================================================================
#pragma once

#include "armor/matrix.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace armor {

// 2D 像素点
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

// 3D 空间点
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 装甲板检测结果（4 角点）
struct ArmorDetection {
    Point2D corners[4];  // 左上、右上、右下、左下
    float confidence = 0.0f;
};

// 目标 3D 位姿
struct TargetPose {
    Point3D position;       // 相机坐标系下的位置（米）
    double yaw = 0.0;       // 偏航角（弧度）
};

// 相机内参（简化）
struct CameraIntrinsics {
    double fx = 1000.0;  // x 焦距（像素）
    double fy = 1000.0;  // y 焦距（像素）
    double cx = 640.0;   // 主点 x（像素）
    double cy = 512.0;   // 主点 y（像素）
};

// ===========================================================================
// ArmorDetector：装甲板检测器
// ===========================================================================
class ArmorDetector {
public:
    explicit ArmorDetector(CameraIntrinsics intrinsics)
        : intrinsics_(intrinsics) {
        // 装甲板物理尺寸（标准小装甲板，单位：米）
        // 以中心为原点，4 个角点的 3D 坐标
        object_points_[0] = {-0.0675,  0.0275, 0.0};  // 左上
        object_points_[1] = { 0.0675,  0.0275, 0.0};  // 右上
        object_points_[2] = { 0.0675, -0.0275, 0.0};  // 右下
        object_points_[3] = {-0.0675, -0.0275, 0.0};  // 左下
    }

    // 检测装甲板：输入图像角点，输出 3D 位姿
    // 简化版 PnP：假设装甲板正对相机（yaw 通过左右不对称估计）
    [[nodiscard]] TargetPose solve_pose(const ArmorDetection& detection) const {
        TargetPose pose;

        // 1. 计算 4 角点的像素中心
        double cx_pix = 0.0, cy_pix = 0.0;
        for (const auto& c : detection.corners) {
            cx_pix += c.x;
            cy_pix += c.y;
        }
        cx_pix /= 4.0;
        cy_pix /= 4.0;

        // 2. 计算装甲板宽度（像素）
        double width_pix = std::hypot(
            detection.corners[1].x - detection.corners[0].x,
            detection.corners[1].y - detection.corners[0].y);
        // 物理宽度 0.135 米
        constexpr double kPhysicalWidth = 0.135;

        // 3. 通过相似三角形求深度 z
        // 焦距 fx, 物理宽度 W, 像素宽度 w
        // z = fx * W / w
        pose.position.z = intrinsics_.fx * kPhysicalWidth / width_pix;

        // 4. 反投影像素中心到 3D 空间
        // x = (cx_pix - cx) * z / fx
        // y = (cy_pix - cy) * z / fy
        pose.position.x = (cx_pix - intrinsics_.cx) * pose.position.z / intrinsics_.fx;
        pose.position.y = (cy_pix - intrinsics_.cy) * pose.position.z / intrinsics_.fy;

        // 5. 估计 yaw：通过左右两半宽度比
        // 简化：当装甲板绕 Y 轴旋转时，左半宽度 vs 右半宽度变化
        double left_width = std::hypot(
            detection.corners[3].x - detection.corners[0].x,
            detection.corners[3].y - detection.corners[0].y);
        double right_width = std::hypot(
            detection.corners[2].x - detection.corners[1].x,
            detection.corners[2].y - detection.corners[1].y);
        double ratio = (left_width - right_width) / (left_width + right_width);
        pose.yaw = std::asin(std::clamp(ratio, -1.0, 1.0));

        return pose;
    }

private:
    CameraIntrinsics intrinsics_;
    Point3D object_points_[4];  // 标准装甲板 3D 模型
};

}  // namespace armor
