// 头文件保护，避免多文件重复包含导致编译重定义错误
#pragma once

// 项目全局配置头文件，包含传输枚举 FoxgloveTransport、编译选项、全局宏等
#include "foxglove_config.hpp"

// C++ 标准库
#include <cstdint>     // 定长整型，用于二进制数据存储
#include <optional>    // 可选类型，表达「通道未创建/已创建」状态
#include <string_view> // 只读轻量字符串，存放主题名、编码格式
#include <tuple>       // 元组，用于批量管理所有通道描述符
#include <variant>     // 变体类型，统一承载所有可视化消息
#include <vector>      // 动态数组，JSON 二进制载荷使用

// Foxglove SDK 基础通道 & 内置消息协议头文件
#include <foxglove/channel.hpp>
#include <foxglove/schemas.hpp>

/**
 * @brief 项目可视化模块专属命名空间
 * 基于 Foxglove SDK 二次封装，统一管理**所有可视化通道、消息、元数据定义**
 * 整体设计思想：单一数据源 + 编译期元编程，新增通道仅需添加描述符，其余代码自动推导
 */
namespace fcs::visualization {

// ============================================================================
// 一、通道类型别名
// 对 Foxglove SDK 原生通道类型做别名，统一用 std::optional 包装
// std::optional 语义：通道可能未初始化，取值前需判断是否有效
// ============================================================================
// 类型别名简化写法，统一包装为 std::optional，代表：该通道可选创建、可空
// 3D场景绘制通道：立方体、线条、标记、网格、模型等3D可视化元素
using SceneCh  = std::optional<::foxglove::schemas::SceneUpdateChannel>;
// 压缩图像通道：JPG/PNG 单帧图片，相机画面快照
using ImageCh  = std::optional<::foxglove::schemas::CompressedImageChannel>;
// 压缩视频通道：连续H264/H265视频流，低延迟实时画面
using VideoCh  = std::optional<::foxglove::schemas::CompressedVideoChannel>;
// 原始二进制通用通道，自定义JSON数据包、自定义结构体透传
using JsonCh   = std::optional<::foxglove::RawChannel>;
// TF坐标变换通道：机器人多坐标系转换（base_link、camera、lidar、map）
using TfCh     = std::optional<::foxglove::schemas::FrameTransformsChannel>;
// 相机内参/外参标定通道：fx fy cx cy 畸变系数、相机位姿
using CalibCh  = std::optional<::foxglove::schemas::CameraCalibrationChannel>;
// 日志消息通道：INFO/WARN/ERROR/DEBUG 打印日志，前端日志面板展示
using LogCh    = std::optional<::foxglove::schemas::LogChannel>;

// ============================================================================
// 二、FoxgloveChannels：运行时通道容器
// 程序运行时，**所有已创建的通道实例**全部存放在此结构体中
// 按业务模块分组：3D场景、图像/视频、JSON调试、特殊系统通道
// ============================================================================
struct FoxgloveChannels {
    // -------------------------- 3D 场景类通道（绘制包围盒、轨迹、模型、预测线等） --------------------------
    SceneCh scene_ch;                  // 主场景
    SceneCh ldm_track_scene_ch;        // LDM 目标跟踪场景
    SceneCh track_scene_ch;            // 通用目标跟踪场景
    SceneCh gimbal_scene_ch;           // 云台解算、瞄准线场景
    SceneCh association_scene_ch;       // 数据关联调试场景
    SceneCh mpc_prediction_scene_ch;    // MPC 轨迹预测场景
    SceneCh rune_scene_ch;             // 符识别主场景
    SceneCh rune_ekf_scene_ch;         // 符EKF滤波状态场景
    SceneCh ground_truth_scene_ch;     // 真值对比场景（算法对标使用）

    // -------------------------- 图像/视频类通道（各类图像流、可视化中间图） --------------------------
    ImageCh img_ch;                    // 主相机图像流
    VideoCh video_ch;                  // 视频流（仅用于MCAP离线录制）
    ImageCh calibration_img_ch;         // 相机标定专用图像
    ImageCh binary_img_ch;             // 二值化调试图像
    ImageCh pattern_img_ch;            // 数字/图案识别图像
    ImageCh rune_arrow_img_ch;         // 符箭头检测图像
    ImageCh rune_target_img_ch;        // 符目标区域图像
    ImageCh rune_center_img_ch;       // 符中心定位图像
    ImageCh ekf_heatmap_ch;            // EKF置信度热力图

    // -------------------------- JSON调试通道（基于RawChannel，二进制承载JSON字符串） --------------------------
    JsonCh debug_lights_ch;            // 灯条检测调试信息
    JsonCh debug_armors_ch;            // 装甲板检测调试信息
    JsonCh measurement_ch;             // 滤波观测值
    JsonCh target_ch;                  // 目标状态数据
    JsonCh target_selection_trace_ch;   // 目标选择逻辑轨迹
    JsonCh cmd_gimbal_ch;              // 云台控制指令
    JsonCh resource_ch;                // 系统硬件/资源状态
    JsonCh perf_stats_ch;              // 程序运行性能统计
    JsonCh mpc_traj_ch;                // MPC规划轨迹数据
    JsonCh rune_debug_ch;              // 符算法全量调试数据
    JsonCh pnp_solver_ch;              // PnP位姿解算结果
    JsonCh nn_confidence_ch;           // 神经网络置信度
    JsonCh energy_meter_ch;            // 能量机构状态
    JsonCh ldm_detection_ch;           // LDM模型检测结果
    JsonCh ldm_measurement_ch;         // LDM观测值
    JsonCh ldm_state_ch;               // LDM内部状态
    JsonCh ground_truth_ch;            // 算法真值数据

    // -------------------------- 特殊系统通道（Foxglove标准协议） --------------------------
    TfCh tf_ch;                        // 多坐标系变换树 TF
    CalibCh camera_calib_ch;           // 相机内参、畸变参数
    LogCh log_ch;                      // 系统运行日志
};

// ============================================================================
// 三、通道描述符（核心：单一数据源 Single Source of Truth）
// 设计规则：
// 1. 每一条可视化通道，对应**一个独立的描述符结构体**
// 2. 描述符包含该通道所有元信息，全局只定义一次
// 3. 新增通道流程：仅新增一个描述符 + 可选消息别名，初始化/分发/注册全部自动推导
//
// 描述符固定成员说明：
// - topic:        Foxglove 订阅主题名（客户端/MCAP识别标识）
// - channel_type: 底层SDK通道类型（结构化通道 / Raw原始通道）
// - payload_type: 该通道承载的数据类型
// - member:       指针，指向 FoxgloveChannels 中对应的成员变量（运行时绑定）
// - is_raw:       true=原始二进制通道(RawChannel)，false=Foxglove结构化通道
// - encoding:     仅Raw通道使用，指定二进制编码格式（此处统一为json）
// - transport:    可选，限制通道仅在「WebSocket实时」或「MCAP离线」模式启用
// ============================================================================

// -------------------------- 3D场景通道描述符 --------------------------
// ====================== 第一大类：3D场景标记通道描述符 SceneUpdate ======================
// 所有场景通道共用 Foxglove 标准3D绘制结构 SceneUpdate，用于绘制方框、线条、轨迹、模型、装甲板等3D标记
// 每个struct对应一条独立可视化话题，存放该话题全部静态常量信息
struct scene_def {
    // 话题路径，对应前端Foxglove展示的topic名称
    static constexpr std::string_view topic = "/scene";
    // 该话题对应的通道管理类（之前定义的SceneCh）
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    // 单帧发送的数据载荷结构体（一整批3D绘制图元）
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    // 指针指向 FoxgloveChannels 容器内对应的可选通道成员，用于运行时读写通道
    static constexpr auto member            = &FoxgloveChannels::scene_ch;
    // 是否原始二进制Raw通道：false=Foxglove标准结构化通道，前端自动解析渲染
    static constexpr bool is_raw            = false;
};

// 跟踪目标3D场景话题
struct track_scene_def {
    static constexpr std::string_view topic = "/track/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::track_scene_ch;
    static constexpr bool is_raw            = false;
};

// 激光雷达跟踪场景
struct ldm_track_scene_def {
    static constexpr std::string_view topic = "/ldm/track/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::ldm_track_scene_ch;
    static constexpr bool is_raw            = false;
};

// 云台解算预测场景
struct gimbal_scene_def {
    static constexpr std::string_view topic = "/solver/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::gimbal_scene_ch;
    static constexpr bool is_raw            = false;
};

// 目标匹配关联调试3D场景
struct association_scene_def {
    static constexpr std::string_view topic = "/debug/association_scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::association_scene_ch;
    static constexpr bool is_raw            = false;
};

// MPC模型预测轨迹场景
struct mpc_prediction_scene_def {
    static constexpr std::string_view topic = "/debug/mpc_prediction_scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::mpc_prediction_scene_ch;
    static constexpr bool is_raw            = false;
};

// 能量机关整体绘制场景
struct rune_scene_def {
    static constexpr std::string_view topic = "/rune/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::rune_scene_ch;
    static constexpr bool is_raw            = false;
};

// 能量机关EKF滤波预测场景
struct rune_ekf_scene_def {
    static constexpr std::string_view topic = "/rune/ekf_scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::rune_ekf_scene_ch;
    static constexpr bool is_raw            = false;
};

// 真值对比3D场景（仿真/标定真值）
struct ground_truth_scene_def {
    static constexpr std::string_view topic = "/ground_truth/scene";
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;
    using payload_type                      = ::foxglove::schemas::SceneUpdate;
    static constexpr auto member            = &FoxgloveChannels::ground_truth_scene_ch;
    static constexpr bool is_raw            = false;
};

// -------------------------- 图像通道描述符 压缩单帧图片 CompressedImage --------------------------
struct img_def {
    static constexpr std::string_view topic      = "/image";
    using channel_type                           = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                           = ::foxglove::schemas::CompressedImage;
    static constexpr auto member                 = &FoxgloveChannels::img_ch;
    static constexpr bool is_raw                 = false;
    // 传输模式限制枚举：仅WebSocket实时推流启用，录制MCAP离线包时屏蔽该通道，减少录像体积
    static constexpr FoxgloveTransport transport = FoxgloveTransport::WebSocket;
};

// 相机标定画面
struct calibration_img_def {
    static constexpr std::string_view topic = "/calibration/image";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::calibration_img_ch;
    static constexpr bool is_raw            = false;
};

// 二值化调试图
struct binary_img_def {
    static constexpr std::string_view topic = "/debug/binary_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::binary_img_ch;
    static constexpr bool is_raw            = false;
};

// 数字识别模板匹配图
struct pattern_img_def {
    static constexpr std::string_view topic = "/debug/number_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::pattern_img_ch;
    static constexpr bool is_raw            = false;
};

// 能量机关箭头ROI图
struct rune_arrow_img_def {
    static constexpr std::string_view topic = "/rune/arrow_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::rune_arrow_img_ch;
    static constexpr bool is_raw            = false;
};

// 能量机关目标区域图
struct rune_target_img_def {
    static constexpr std::string_view topic = "/rune/target_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::rune_target_img_ch;
    static constexpr bool is_raw            = false;
};

// 能量机关中心识别图
struct rune_center_img_def {
    static constexpr std::string_view topic = "/rune/center_img";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::rune_center_img_ch;
    static constexpr bool is_raw            = false;
};

// EKF概率热力图
struct ekf_heatmap_def {
    static constexpr std::string_view topic = "/debug/ekf_heatmap";
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;
    using payload_type                      = ::foxglove::schemas::CompressedImage;
    static constexpr auto member            = &FoxgloveChannels::ekf_heatmap_ch;
    static constexpr bool is_raw            = false;
};

// -------------------------- 视频通道描述符（仅MCAP离线录制） --------------------------
struct video_def {
    static constexpr std::string_view topic      = "/image";
    using channel_type                           = ::foxglove::schemas::CompressedVideoChannel;
    using payload_type                           = ::foxglove::schemas::CompressedVideo;
    static constexpr auto member                 = &FoxgloveChannels::video_ch;
    static constexpr bool is_raw                 = false;
    // 传输限制：仅离线MCAP录像开启，实时WebSocket不推送视频流，降低带宽占用
    static constexpr FoxgloveTransport transport = FoxgloveTransport::Mcap;
};

// -------------------------- JSON原始通道描述符 RawChannel 二进制透传 --------------------------
// is_raw=true 代表通用原始二进制通道，不使用Foxglove内置结构化类型，自定义编码格式
// encoding标记二进制内容为json字符串，前端自动解析JSON面板
struct debug_lights_def {
    static constexpr std::string_view topic    = "/debug/lights";
    using channel_type                         = ::foxglove::RawChannel;
    // 载荷为原始字节数组，存放序列化后的JSON字符串
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::debug_lights_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 装甲板检测调试JSON
struct debug_armors_def {
    static constexpr std::string_view topic    = "/debug/armors";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::debug_armors_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 视觉测量原始数据
struct measurement_def {
    static constexpr std::string_view topic    = "/solver/measurement";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::measurement_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 解算目标参数
struct target_def {
    static constexpr std::string_view topic    = "/solver/target";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::target_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 目标选择决策追踪日志
struct target_selection_trace_def {
    static constexpr std::string_view topic    = "/solver/target_selection";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::target_selection_trace_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 云台控制指令
struct cmd_gimbal_def {
    static constexpr std::string_view topic    = "/solver/cmd_gimbal";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::cmd_gimbal_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 硬件资源占用（CPU/内存）
struct resources_def {
    static constexpr std::string_view topic    = "/resources";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::resource_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 性能统计耗时
struct perf_stats_def {
    static constexpr std::string_view topic    = "/perf_stats";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::perf_stats_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// MPC预测轨迹数据
struct mpc_traj_def {
    static constexpr std::string_view topic    = "/mpc/trajectory";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::mpc_traj_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 能量机关全套调试参数
struct rune_debug_def {
    static constexpr std::string_view topic    = "/rune/debug";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::rune_debug_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// PnP位姿解算中间数据
struct pnp_solver_def {
    static constexpr std::string_view topic    = "/debug/pnp_solver";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::pnp_solver_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 神经网络置信度输出
struct nn_confidence_def {
    static constexpr std::string_view topic    = "/debug/nn_confidence";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::nn_confidence_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 能量计状态
struct energy_meter_def {
    static constexpr std::string_view topic    = "/energy_meter/state";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::energy_meter_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 激光雷达原始检测框
struct ldm_detection_def {
    static constexpr std::string_view topic    = "/ldm/detection";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ldm_detection_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 激光雷达测距测量值
struct ldm_measurement_def {
    static constexpr std::string_view topic    = "/ldm/measurement";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ldm_measurement_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 激光雷达滤波状态
struct ldm_state_def {
    static constexpr std::string_view topic    = "/ldm/state";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ldm_state_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// 仿真真值数据
struct ground_truth_def {
    static constexpr std::string_view topic    = "/ground_truth";
    using channel_type                         = ::foxglove::RawChannel;
    using payload_type                         = std::vector<uint8_t>;
    static constexpr auto member               = &FoxgloveChannels::ground_truth_ch;
    static constexpr bool is_raw               = true;
    static constexpr std::string_view encoding = "json";
};

// -------------------------- 特殊系统内置通道描述符 TF/相机标定/日志 --------------------------
// 坐标变换通道，等价ROS2 /tf
struct tf_def {
    static constexpr std::string_view topic = "/tf";
    using channel_type                      = ::foxglove::schemas::FrameTransformsChannel;
    using payload_type                      = ::foxglove::schemas::FrameTransforms;
    static constexpr auto member            = &FoxgloveChannels::tf_ch;
    static constexpr bool is_raw            = false;
};

// 相机内参标定通道，等价ROS2 CameraInfo
struct calib_def {
    static constexpr std::string_view topic = "/camera_info";
    using channel_type                      = ::foxglove::schemas::CameraCalibrationChannel;
    using payload_type                      = ::foxglove::schemas::CameraCalibration;
    static constexpr auto member            = &FoxgloveChannels::camera_calib_ch;
    static constexpr bool is_raw            = false;
};

// 全局日志通道，等价ROS2 /rosout
struct log_def {
    static constexpr std::string_view topic = "/log";
    using channel_type                      = ::foxglove::schemas::LogChannel;
    using payload_type                      = ::foxglove::schemas::Log;
    static constexpr auto member            = &FoxgloveChannels::log_ch;
    static constexpr bool is_raw            = false;
};

// ============================================================================
// 四、通道注册表
// 将所有通道描述符存入 std::tuple 编译期元组容器
// 作用：配合C++17折叠表达式、std::visit、模板遍历，**编译期自动批量处理全部话题**
// 不用手动写几十行重复代码逐个初始化/销毁通道，新增话题只需添加一个xxx_def并加入tuple
// ============================================================================
using ChannelRegistry = std::tuple<
    scene_def, track_scene_def, ldm_track_scene_def, gimbal_scene_def, association_scene_def,
    mpc_prediction_scene_def, rune_scene_def, rune_ekf_scene_def, ground_truth_scene_def, img_def,
    video_def, calibration_img_def, binary_img_def, pattern_img_def, rune_arrow_img_def,
    rune_target_img_def, rune_center_img_def, ekf_heatmap_def, debug_lights_def, debug_armors_def,
    measurement_def, target_def, target_selection_trace_def, cmd_gimbal_def, perf_stats_def,
    resources_def, mpc_traj_def, rune_debug_def, pnp_solver_def, nn_confidence_def,
    energy_meter_def, ldm_detection_def, ldm_measurement_def, ldm_state_def, ground_truth_def,
    tf_def, calib_def, log_def>;

// ============================================================================
// 五、单通道消息包装模板
// 泛型模板，接收任意一个通道描述符xxx_def，自动推导该话题对应的数据载荷
// 统一封装发送数据包结构，业务层只需填充payload字段即可推流
// ============================================================================
template <typename Descriptor>
struct FoxgloveMsg {
    // 从描述符提取该话题对应的消息载荷类型
    typename Descriptor::payload_type payload;
};

// ============================================================================
// 六、消息类型对外别名
// 简化业务代码书写，不用每次写 FoxgloveMsg<scene_def> 冗长模板
// 直接 using SceneMessage 即用，语义清晰、可读性高
// ============================================================================
using SceneMessage                = FoxgloveMsg<scene_def>;
using TrackSceneMessage           = FoxgloveMsg<track_scene_def>;
using LdmTrackSceneMessage        = FoxgloveMsg<ldm_track_scene_def>;
using GimbalSceneMessage          = FoxgloveMsg<gimbal_scene_def>;
using AssociationSceneMessage     = FoxgloveMsg<association_scene_def>;
using MpcPredictionSceneMessage   = FoxgloveMsg<mpc_prediction_scene_def>;
using RuneSceneMessage            = FoxgloveMsg<rune_scene_def>;
using RuneEkfSceneMessage         = FoxgloveMsg<rune_ekf_scene_def>;
using GroundTruthSceneMessage     = FoxgloveMsg<ground_truth_scene_def>;
using ImageMessage                = FoxgloveMsg<img_def>;
using VideoMessage                = FoxgloveMsg<video_def>;
using CalibrationImageMessage     = FoxgloveMsg<calibration_img_def>;
using BinaryImageMessage          = FoxgloveMsg<binary_img_def>;
using PatternImageMessage         = FoxgloveMsg<pattern_img_def>;
using RuneArrowImageMessage       = FoxgloveMsg<rune_arrow_img_def>;
using RuneTargetImageMessage      = FoxgloveMsg<rune_target_img_def>;
using RuneCenterImageMessage      = FoxgloveMsg<rune_center_img_def>;
using EkfHeatmapMessage           = FoxgloveMsg<ekf_heatmap_def>;
using DebugLightsMessage          = FoxgloveMsg<debug_lights_def>;
using DebugArmorsMessage          = FoxgloveMsg<debug_armors_def>;
using MeasurementMessage          = FoxgloveMsg<measurement_def>;
using TargetMessage               = FoxgloveMsg<target_def>;
using TargetSelectionTraceMessage = FoxgloveMsg<target_selection_trace_def>;
using GimbalCmdMessage            = FoxgloveMsg<cmd_gimbal_def>;
using PerfStatsMessage            = FoxgloveMsg<perf_stats_def>;
using ResourceMessage             = FoxgloveMsg<resources_def>;
using MpcTrajectoryMessage        = FoxgloveMsg<mpc_traj_def>;
using RuneDebugMessage            = FoxgloveMsg<rune_debug_def>;
using PnPSolverMessage            = FoxgloveMsg<pnp_solver_def>;
using NNConfidenceMessage         = FoxgloveMsg<nn_confidence_def>;
using EnergyMeterMessage          = FoxgloveMsg<energy_meter_def>;
using LdmDetectionMessage         = FoxgloveMsg<ldm_detection_def>;
using LdmMeasurementMessage       = FoxgloveMsg<ldm_measurement_def>;
using LdmStateMessage             = FoxgloveMsg<ldm_state_def>;
using GroundTruthMessage          = FoxgloveMsg<ground_truth_def>;
using TfMessage                   = FoxgloveMsg<tf_def>;
using LogMessage                  = FoxgloveMsg<log_def>;

// ============================================================================
// 七、载荷分发器 PayloadLogger
// 模板特化：区分「结构化消息」和「原始二进制消息」的写入逻辑
// 统一对外 log 接口，内部自动适配不同通道的 API 差异
// ============================================================================
namespace detail {

/**
 * @brief 通用模板：Foxglove 标准结构化通道
 * 直接调用通道对象的 log(载荷对象) 接口
 */
template <typename Payload>
struct PayloadLogger {
    template <typename Channel>
    static void log(Channel& ch, const Payload& payload) noexcept {
        ch.log(payload);
    }
};

/**
 * @brief 模板特化：RawChannel 二进制载荷（JSON数据）
 * 将 vector<uint8_t> 转为 std::byte 裸指针，调用 RawChannel 字节流接口
 */
template <>
struct PayloadLogger<std::vector<uint8_t>> {
    static void log(::foxglove::RawChannel& ch, const std::vector<uint8_t>& data) noexcept {
        ch.log(reinterpret_cast<const std::byte*>(data.data()), data.size());
    }
};

} // namespace detail

// ============================================================================
// 八、全局统一消息变体 FoxgloveMessage
// 基于 ChannelRegistry 元组，**编译期自动推导**出包含所有消息类型的 std::variant
// 作用：全局消息队列、回调函数只需要接收这一种类型，即可承载所有可视化消息
// 完全类型安全，编译期拦截类型错误
// ============================================================================
namespace detail {
// 推导函数声明：将描述符元组转为对应消息变体
template <typename... Defs>
auto make_message_variant(std::tuple<Defs...>) -> std::variant<FoxgloveMsg<Defs>...>;
} // namespace detail

// 全局唯一消息变体类型，整个可视化模块的消息入口
using FoxgloveMessage = decltype(detail::make_message_variant(ChannelRegistry{}));

// ============================================================================
// 九、工具函数预留区
// 后续可在此扩展：通道初始化、消息发送、批量关闭、过滤函数等通用工具
// ============================================================================

} // namespace fcs::visualization