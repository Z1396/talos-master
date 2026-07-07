#pragma once

// 装甲/大符相关结构体、枚举定义（LightBlob、LightPair、LdmDetection、ArmorColor、DetectorError）
#include "core/armor_types.hpp"
// LdmDetectorConfig、LdmGeometryConfig 检测器阈值与几何参数配置结构体
#include "ldm_config.hpp"
// 项目自定义基础数据类型别名
#include "types.hpp"

// Eigen 线性代数库基础矩阵、向量
#include <Eigen/Core>
// Eigen 旋转、平移、变换矩阵几何模块
#include <Eigen/Geometry>

// 固定长度数组容器，用于存储固定数量参数/坐标
#include <array>
// C++23 标准预期类型，用于函数返回成功结果或错误枚举，替代异常
#include <expected>
// OpenCV 基础图像、矩阵核心类型 cv::Mat
#include <opencv2/core.hpp>
// 标准字符串，用于日志、错误信息、配置名称
#include <string>
// 动态变长数组容器，存储Blob、灯对、候选网格、检测结果列表
#include <vector>

/**
 * @brief 命名空间：fcs::L2::ldm
 * 功能：隔离激光模块（LDM/大符）检测器全部代码，避免全局命名冲突
 * fcs：项目顶层命名空间；L2：第二层感知模块；ldm：Laser Module 大符识别子模块
 */
namespace fcs::L2::ldm {

/**
 * @brief LDM大符检测器主类
 * 封装完整大符识别流水线：颜色掩码提取灯条Blob → 聚类 → PCA分层配对 → 网格候选筛选 → 输出检测结果
 * 所有图像处理接口对外暴露，内部私有存储全局检测阈值配置
 */
class LdmDetector {
public:
    /**
     * @brief 构造函数（显式构造，禁止隐式类型转换）
     * @param config 检测器全套阈值+几何配置结构体，值拷贝存入成员变量
     * noexcept 保证构造不会抛出异常，提升实时性与稳定性
     */
    explicit LdmDetector(LdmDetectorConfig config) noexcept;

    /**
     * @brief 析构函数，编译器默认生成
     * 无动态内存手动释放需求，使用默认析构
     */
    ~LdmDetector() = default;

    /**
     * @brief 对外检测接口1：使用配置文件预设目标颜色识别大符
     * @param image 输入原始BGR OpenCV图像
     * @return std::expected
     *          成功：std::optional<LdmDetection>
     *              std::nullopt：图像内未检测到合法大符
     *              LdmDetection：完整大符检测结果（灯条、灯对、候选网格、包围盒等）
     *          失败：DetectorError 错误枚举（空图像、参数非法等）
     * const 修饰：该调用不会修改类内部成员config_
     * noexcept：函数无抛异常逻辑
     */
    std::expected<std::optional<LdmDetection>, DetectorError>
        detect(const cv::Mat& image) const noexcept;

    /**
     * @brief 对外检测接口2：手动指定本次识别目标装甲颜色，覆盖配置默认颜色
     * @param image 输入原始BGR OpenCV图像
     * @param color 本次检测目标颜色：Red/Blue/Purple
     * @return std::expected 同上
     * const 修饰：不修改内部配置
     * noexcept：无异常抛出
     */
    std::expected<std::optional<LdmDetection>, DetectorError>
        detect(const cv::Mat& image, ArmorColor color) const noexcept;

private:
    /**
     * @brief 私有成员变量：检测器全局配置
     * 存储全部Blob筛选、灯对配对、候选打分、几何尺寸阈值
     * 构造时传入初始化，全生命周期只读，const检测接口可访问
     */
    LdmDetectorConfig config_;
};

} // namespace fcs::L2::ldm