#pragma once
// 头文件保护指令，防止该头文件被多次include造成重复定义编译错误

// 嵌套命名空间 fcs::rune
// fcs：整个机器人框架顶层命名空间
// rune：能量机关（大符、小符）识别模块专属命名空间，隔离模块代码，避免命名冲突
namespace fcs::rune {

/**
 * @brief 能量机关识别算法全套参数配置结构体
 * 用于RM能量机关（Rune）图像处理阈值、轮廓筛选、ROI区域控制
 * 会通过FCS反射框架 + TOML配置文件自动加载所有参数
 */
struct RuneDetectorConfig {
    // ====================== 调试可视化开关 ======================
    /// 是否绘制识别结果图像（轮廓、框、关键点，用于调试）
    bool draw{false};
    /// L2层级详细调试模式；开启后打印大量中间过程、错误日志
    bool debug_mode{false};
    /// 箭头灯二值化阈值（灰度图阈值分割）
    int arrow_threshold{170};
    /// 目标能量灯二值化阈值
    int target_threshold{130};
    /// R中心灯二值化阈值
    int rcenter_threshold{120};

    // ====================== 流水灯（箭头灯）轮廓筛选参数 ======================
    /// 最小圆度，过滤不规则噪点轮廓
    double min_roundness{0.6};
    /// 最大圆度
    double max_roundness{1.0};
    /// 箭头灯最小轮廓面积
    double min_arrow_light_area{10.0};
    /// 箭头灯最大轮廓面积，过滤超大干扰色块
    double max_arrow_light_area{250.0};
    /// 箭头灯最小长宽比（轮廓外接矩形宽高比）
    double min_arrow_light_aspect_ratio{1.5};
    /// 箭头灯最大长宽比
    double max_arrow_light_aspect_ratio{5.0};

    // ====================== ROI感兴趣区域参数（加速图像处理） ======================
    /// 全局ROI扩张比例，基于上一帧目标范围向外扩大，防止目标跑出画面
    double global_roi_length_ratio{1.5};
    /// 局部ROI距离比例：根据R中心距离动态调整局部搜索范围
    double local_roi_distance_ratio{6.0};
    /// 局部ROI固定宽度
    int local_roi_width{150};

    // ====================== R中心特征点筛选参数 ======================
    /// R中心灯最大长宽比，R中心接近圆形，不能过于细长
    double max_center_aspect_ratio{3.0};

    // ====================== 目标能量装甲灯条参数（待击打目标） ======================
    /// 目标灯条最小面积
    int min_target_light_area{400};
    /// 目标灯条最大面积
    int max_target_light_area{30000};
    /// 目标内层有效轮廓最小面积
    int min_target_light_contour_area{300};
    /// 目标内层有效轮廓最大面积
    int max_target_light_contour_area{10000};
    /// 目标灯条最小长宽比
    double min_target_light_aspect_ratio{0.5};
    /// 目标灯条最大长宽比
    double max_target_light_aspect_ratio{1.5};
    /// 目标ROI区域最小面积阈值
    float min_target_roi_area{100.0f};
};

} // namespace fcs::rune