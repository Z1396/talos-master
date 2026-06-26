// 头文件保护，防止同一头文件被多次包含造成重复定义编译错误
#pragma once

/**
 * @brief ArmorMeasurementT 装甲测量类的 PnP 构造辅助工具
 * @brief 头文件使用约束说明：
 * 仅在执行 solvePnP、生成相机光学坐标系测量值的源文件中引入本头文件
 * 将 PnP 相关逻辑从 types.hpp 拆分出来，核心目的：
 * types.hpp 是全局通用基础类型头文件，会被项目大量文件包含；
 * cv::calib3d.hpp（OpenCV 标定/PnP 模块）体积大、编译慢，且仅解算位姿代码才需要；
 * 拆分后避免所有下游编译单元(TU)都依赖 calib3d，大幅缩短全局编译耗时
 */
/// PnP construction helpers for ArmorMeasurementT.
/// Include this only in files that perform solvePnP and create camera-optical measurements.
/// Separated from types.hpp to avoid propagating calib3d.hpp to all downstream TUs.

// 引入项目基础类型定义头文件：包含 ArmorDetection、CameraArmorMeasurement 等业务结构体
#include "core/types.hpp"

// 项目顶层命名空间 fcs，隔离全部业务代码，防止全局命名冲突
namespace fcs {

/**
 * @brief 基于装甲检测结果 + OpenCV solvePnP 输出的旋转/平移向量，构建相机光学坐标系装甲测量数据包
 * @param[in] det 神经网络/图像识别输出的装甲检测结果结构体，包含装甲标签、颜色、置信度等图像侧信息
 * @param[in] rvec OpenCV PnP 输出旋转向量 cv::Mat，数据类型 double，3行1列
 * @param[in] tvec OpenCV PnP 输出平移向量 cv::Mat，数据类型 double，3行1列
 * @param[in] ts 当前帧时间戳，单位纳秒 ns
 * @param[in] dist_to_center 装甲装甲图像中心到画面图像中心点的距离，可选参数默认0.0f
 * @return CameraArmorMeasurement 填充完整的相机坐标系装甲测量结构体
 *
 * @note 位姿坐标系规范（OpenCV 相机光学标准坐标系，本函数输出位姿遵循此规范）：
 * 坐标系原点：相机光心
 * X 轴：画面水平向右
 * Y 轴：画面垂直向下
 * Z 轴：相机镜头向前、指向场景深处（深度轴）
 *
 * @note PnP 位姿数学说明：
 * OpenCV solvePnP 求解结果为【物体坐标系 → 相机光学坐标系】变换（object -> camera）
 * 本函数直接将该原始位姿存入测量包；
 * 该测量包仅适用于相机光学坐标系原始数据，后续如需转换机器人基坐标系/云台坐标系，需要再做坐标变换(TF)
 */
inline CameraArmorMeasurement make_camera_measurement(
    const ArmorDetection& det, const cv::Mat& rvec, const cv::Mat& tvec, uint64_t ts,
    float dist_to_center = 0.0f) {
    // 将 OpenCV 矩阵格式的旋转向量转为 cv::Vec3d 三维向量，方便传入自定义位姿构造接口
    // rvec.at<double>(0/1/2) 依次取出 x/y/z 三轴旋转分量
    const cv::Vec3d rvec_v(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    // 将平移向量矩阵转为三维向量，提取 x/y/z 三轴平移分量（单位：米）
    const cv::Vec3d tvec_v(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    // 实例化相机装甲测量结构体对象
    CameraArmorMeasurement m;

    // 从检测结果拷贝装甲标识名称（如armor_1/armor_2等编号）
    m.name                     = det.name;
    // 拷贝装甲颜色（红/蓝/无）
    m.color                    = det.color;
    // 拷贝装甲类型（大装甲/小装甲/哨兵装甲等）
    m.type                     = det.type;
    // 拷贝神经网络检测置信度 [0,1]，代表识别可信度
    m.confidence               = det.confidence;
    // 赋值装甲图像中心距离画面中心点的距离，用于后续过滤边缘远距离目标
    m.distance_to_image_center = dist_to_center;
    // 赋值当前帧纳秒级时间戳，用于时序对齐、滤波、轨迹平滑
    m.timestamp_ns             = ts;
    // 调用内部 Transform 静态工厂 from_pnp，用旋转、平移向量生成物体到相机的位姿变换矩阵/结构体
    m.transform                = CameraArmorMeasurement::Transform::from_pnp(rvec_v, tvec_v);

    // 返回填充完毕的测量数据包，供后续滤波、决策、云台跟踪模块使用
    return m;
}

} // namespace fcs