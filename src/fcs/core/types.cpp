// 项目核心通用类型头文件，包含ArmorDetection装甲检测结构体、ArmorName/ArmorColor枚举、cls_to_armor_type转换函数声明等
#include "core/types.hpp"
// OpenCV图像处理模块：轮廓包围矩形、轮廓面积计算、点集容器等API
#include <opencv2/imgproc.hpp>

/**
 * @namespace fcs
 * @brief 火控视觉顶层业务命名空间，存放图像检测、跟踪相关数据结构实现
 */
namespace fcs {

/**
 * @brief ArmorDetection 装甲检测结果结构体构造函数实现
 * 构造时一次性完成所有检测属性初始化，并自动计算包围矩形、轮廓面积
 * @param pts std::array<cv::Point2f, 4> 装甲四个角点像素坐标，顺序：左上/右上/右下/左下标准四边形
 * @param armor_name ArmorName 装甲类型编号：One/Two/Three/Outpost/Sentry等
 * @param color ArmorColor 目标阵营颜色：Red/Blue/Neutral/Purple
 * @param conf float 神经网络推理置信度 [0,1]，越高代表检测结果越可靠
 */
ArmorDetection::ArmorDetection(
    std::array<cv::Point2f, 4> pts, ArmorName armor_name, ArmorColor color, float conf)
    // 初始化列表赋值，比构造体内赋值效率更高，嵌入式实时系统优先使用
    : corners(pts)          // 装甲四角像素坐标数组赋值给成员变量
    , name(armor_name)      // 装甲编号类型赋值
    , color(color)          // 目标颜色阵营赋值
    , type(cls_to_armor_type(armor_name)) // 通过装甲编号转换为装甲大类类型（小装甲/大装甲/前哨等）
    , confidence(conf)      // 推理置信度赋值
{
    // 步骤1：将定长数组四角点转为OpenCV标准vector点集，供轮廓计算API使用
    std::vector<cv::Point2f> pts_vec(corners.begin(), corners.end());

    // 步骤2：根据四点轮廓计算最小外接包围矩形（像素坐标，x/y为左上角，width/height宽高）
    rect = cv::boundingRect(pts_vec);

    // 步骤3：计算四边形轮廓的像素面积，强转int存储，用于过滤过小噪点、判断装甲尺寸
    area = static_cast<int>(cv::contourArea(pts_vec));
}

} // namespace fcs