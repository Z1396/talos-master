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
// 3D场景绘制通道：立方体、线条、标记、网格、模型等3D可视化元素：立方体、线条、包围盒
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
    static constexpr std::string_view topic = "/scene";                     // Foxglove话题名，主3D渲染场景
    // 该话题对应的通道管理类（之前定义的SceneCh）
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel; // Foxglove SceneUpdate通道类型，管理3D图元绘制
    // 单帧发送的数据载荷结构体（一整批3D绘制图元）
    using payload_type                      = ::foxglove::schemas::SceneUpdate;        // 载荷类型：一批3D绘制物体（线、圆、方块等）
    // 指针指向 FoxgloveChannels 容器内对应的可选通道成员，用于运行时读写通道
    static constexpr auto member            = &FoxgloveChannels::scene_ch;             // 编译期成员指针，指向全局通道集合里scene_ch成员
    // 是否原始二进制Raw通道：false=Foxglove标准结构化通道，前端自动解析渲染
    static constexpr bool is_raw            = false;                                   // false：标准schema结构化消息，不是原始二进制透传
};

// 跟踪目标3D场景话题
struct track_scene_def {
    static constexpr std::string_view topic = "/track/scene";                          // Foxglove话题，目标跟踪模块3D可视化场景
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // SceneUpdate通道，用于3D图元渲染
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 载荷：3D场景更新数据包
    static constexpr auto member            = &FoxgloveChannels::track_scene_ch;       // 指向全局通道集合track_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 使用Foxglove标准结构化消息
};

// 激光雷达跟踪场景
struct ldm_track_scene_def {
    static constexpr std::string_view topic = "/ldm/track/scene";                      // Foxglove话题，LDM能量机关跟踪3D可视化
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元更新载荷
    static constexpr auto member            = &FoxgloveChannels::ldm_track_scene_ch;    // 指向全局通道集合ldm_track_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// 云台解算预测场景
struct gimbal_scene_def {
    static constexpr std::string_view topic = "/solver/scene";                          // Foxglove话题，云台解算预测弹道、瞄准点3D绘制
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元数据包
    static constexpr auto member            = &FoxgloveChannels::gimbal_scene_ch;       // 指向全局通道集合gimbal_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// 目标匹配关联调试3D场景
struct association_scene_def {
    static constexpr std::string_view topic = "/debug/association_scene";               // Foxglove话题，目标数据关联匹配调试3D可视化
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元载荷
    static constexpr auto member            = &FoxgloveChannels::association_scene_ch;  // 指向全局通道集合association_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// MPC模型预测轨迹场景
struct mpc_prediction_scene_def {
    static constexpr std::string_view topic = "/debug/mpc_prediction_scene";            // Foxglove话题，MPC预测输出轨迹3D可视化
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元数据包
    static constexpr auto member            = &FoxgloveChannels::mpc_prediction_scene_ch;// 指向全局通道集合mpc_prediction_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// 能量机关整体绘制场景
struct rune_scene_def {
    static constexpr std::string_view topic = "/rune/scene";                           // Foxglove话题，能量机关实体3D绘制场景
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元载荷
    static constexpr auto member            = &FoxgloveChannels::rune_scene_ch;        // 指向全局通道集合rune_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// 能量机关EKF滤波预测场景
struct rune_ekf_scene_def {
    static constexpr std::string_view topic = "/rune/ekf_scene";                        // Foxglove话题，能量机关EKF滤波预测状态3D可视化
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元数据包
    static constexpr auto member            = &FoxgloveChannels::rune_ekf_scene_ch;    // 指向全局通道集合rune_ekf_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// 真值对比3D场景（仿真/标定真值）
struct ground_truth_scene_def {
    static constexpr std::string_view topic = "/ground_truth/scene";                    // Foxglove话题，仿真真值3D可视化，用于对比算法输出
    using channel_type                      = ::foxglove::schemas::SceneUpdateChannel;  // 3D场景更新通道
    using payload_type                      = ::foxglove::schemas::SceneUpdate;         // 3D图元载荷
    static constexpr auto member            = &FoxgloveChannels::ground_truth_scene_ch; // 指向全局通道集合ground_truth_scene_ch成员
    static constexpr bool is_raw            = false;                                    // 标准结构化消息
};

// -------------------------- 图像通道描述符 压缩单帧图片 CompressedImage --------------------------
struct img_def {
    static constexpr std::string_view topic      = "/image";                            // Foxglove话题，相机原始压缩图像
    using channel_type                           = ::foxglove::schemas::CompressedImageChannel; // 压缩图片通道类型
    using payload_type                           = ::foxglove::schemas::CompressedImage;        // 载荷类型：压缩图像消息结构体
    static constexpr auto member                 = &FoxgloveChannels::img_ch;                   // 指向全局通道集合img_ch成员
    static constexpr bool is_raw                 = false;                                        // 使用Foxglove标准图片schema
    // 传输模式限制枚举：仅WebSocket实时推流启用，录制MCAP离线包时屏蔽该通道，减少录像体积
    static constexpr FoxgloveTransport transport = FoxgloveTransport::WebSocket;                 // 仅实时WebSocket推送，MCAP录制不记录此通道
};

// 相机标定画面
struct calibration_img_def {
    static constexpr std::string_view topic = "/calibration/image";                    // Foxglove话题，标定过程图像输出
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::calibration_img_ch;    // 指向全局通道集合calibration_img_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// 二值化调试图
struct binary_img_def {
    static constexpr std::string_view topic = "/debug/binary_img";                     // Foxglove话题，图像二值化中间调试图
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::binary_img_ch;         // 指向全局通道集合binary_img_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// 数字识别模板匹配图
struct pattern_img_def {
    static constexpr std::string_view topic = "/debug/number_img";                     // Foxglove话题，能量机关数字识别调试图像
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::pattern_img_ch;        // 指向全局通道集合pattern_img_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// 能量机关箭头ROI图
struct rune_arrow_img_def {
    static constexpr std::string_view topic = "/rune/arrow_img";                       // Foxglove话题，能量机关箭头ROI裁剪图像
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::rune_arrow_img_ch;    // 指向全局通道集合rune_arrow_img_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// 能量机关目标区域图
struct rune_target_img_def {
    static constexpr std::string_view topic = "/rune/target_img";                      // Foxglove话题，能量机关目标叶片ROI图像
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::rune_target_img_ch;    // 指向全局通道集合rune_target_img_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// 能量机关中心识别图
struct rune_center_img_def {
    static constexpr std::string_view topic = "/rune/center_img";                      // Foxglove话题，能量机关中心点识别调试图
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::rune_center_img_ch;    // 指向全局通道集合rune_center_img_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// EKF概率热力图
struct ekf_heatmap_def {
    static constexpr std::string_view topic = "/debug/ekf_heatmap";                    // Foxglove话题，EKF滤波概率热力图调试图像
    using channel_type                      = ::foxglove::schemas::CompressedImageChannel;// 压缩图片通道
    using payload_type                      = ::foxglove::schemas::CompressedImage;       // 压缩图片载荷
    static constexpr auto member            = &FoxgloveChannels::ekf_heatmap_ch;        // 指向全局通道集合ekf_heatmap_ch成员
    static constexpr bool is_raw            = false;                                     // 标准图片结构化消息
};

// -------------------------- 视频通道描述符（仅MCAP离线录制） --------------------------
struct video_def {
    static constexpr std::string_view topic      = "/image";                            // Foxglove话题，离线录制压缩视频话题
    using channel_type                           = ::foxglove::schemas::CompressedVideoChannel; // 压缩视频通道类型
    using payload_type                           = ::foxglove::schemas::CompressedVideo;        // 载荷：压缩视频帧消息
    static constexpr auto member                 = &FoxgloveChannels::video_ch;                // 指向全局通道集合video_ch成员
    static constexpr bool is_raw                 = false;                                       // 使用Foxglove标准视频schema
    // 传输限制：仅离线MCAP录像开启，实时WebSocket不推送视频流，降低带宽占用
    static constexpr FoxgloveTransport transport = FoxgloveTransport::Mcap;                    // 只写入MCAP录像，实时网页端不推送
};

// -------------------------- JSON原始通道描述符 RawChannel 二进制透传 --------------------------
// is_raw=true 代表通用原始二进制通道，不使用Foxglove内置结构化类型，自定义编码格式
// encoding标记二进制内容为json字符串，前端自动解析JSON面板
struct debug_lights_def {
    static constexpr std::string_view topic    = "/debug/lights";                      // Foxglove话题，灯板状态调试JSON通道
    using channel_type                         = ::foxglove::RawChannel;               // 原始二进制透传通道，无固定schema
    // 载荷为原始字节数组，存放序列化后的JSON字符串
    using payload_type                         = std::vector<uint8_t>;                 // 载荷类型：原始uint8字节buffer，存放json字符串
    static constexpr auto member               = &FoxgloveChannels::debug_lights_ch;   // 指向全局通道集合debug_lights_ch成员
    static constexpr bool is_raw               = true;                                  // true：原始二进制通道，不走标准schema解析
    static constexpr std::string_view encoding = "json";                                // 告诉Foxglove：字节流是json字符串，前端自动渲染json面板
};

// 装甲板检测调试JSON
struct debug_armors_def {
    static constexpr std::string_view topic    = "/debug/armors";                       // Foxglove话题，装甲检测结果调试JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组，存储json文本
    static constexpr auto member = &FoxgloveChannels::debug_armors_ch; // 指向全局通道集合debug_armors_ch成员
    static constexpr bool is_raw               = true;                                 // 启用原始二进制模式
    static constexpr std::string_view encoding = "json";                               // 编码标记：内容为JSON字符串
};

// 视觉测量原始数据
struct measurement_def {
    static constexpr std::string_view topic    = "/solver/measurement";                 // Foxglove话题，PnP视觉测量输出JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::measurement_ch;    // 指向全局通道集合measurement_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // 内容编码为JSON
};

// 解算目标参数
struct target_def {
    static constexpr std::string_view topic    = "/solver/target";                     // Foxglove话题，瞄准解算目标状态JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::target_ch;         // 指向全局通道集合target_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // 内容编码JSON
};

// 目标选择决策追踪日志
struct target_selection_trace_def {
    static constexpr std::string_view topic    = "/solver/target_selection";            // Foxglove话题，目标选择决策追踪日志JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::target_selection_trace_ch; // 指向全局通道集合target_selection_trace_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 云台控制指令
struct cmd_gimbal_def {
    static constexpr std::string_view topic    = "/solver/cmd_gimbal";                 // Foxglove话题，云台输出控制指令JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::cmd_gimbal_ch;     // 指向全局通道集合cmd_gimbal_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 硬件资源占用（CPU/内存）
struct resources_def {
    static constexpr std::string_view topic    = "/resources";                          // Foxglove话题，系统CPU内存硬件资源统计JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::resource_ch;       // 指向全局通道集合resource_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 性能统计耗时
struct perf_stats_def {
    static constexpr std::string_view topic    = "/perf_stats";                         // Foxglove话题，各模块运行耗时性能统计JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::perf_stats_ch;      // 指向全局通道集合perf_stats_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// MPC预测轨迹数据
struct mpc_traj_def {
    static constexpr std::string_view topic    = "/mpc/trajectory";                     // Foxglove话题，MPC预测轨迹点序列JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::mpc_traj_ch;      // 指向全局通道集合mpc_traj_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 能量机关全套调试参数
struct rune_debug_def {
    static constexpr std::string_view topic    = "/rune/debug";                         // Foxglove话题，能量机关全套调试状态JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::rune_debug_ch;    // 指向全局通道集合rune_debug_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// PnP位姿解算中间数据
struct pnp_solver_def {
    static constexpr std::string_view topic    = "/debug/pnp_solver";                   // Foxglove话题，PnP位姿解算中间结果JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::pnp_solver_ch;    // 指向全局通道集合pnp_solver_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 神经网络置信度输出
struct nn_confidence_def {
    static constexpr std::string_view topic    = "/debug/nn_confidence";                // Foxglove话题，神经网络推理置信度输出JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::nn_confidence_ch;  // 指向全局通道集合nn_confidence_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 能量计状态
struct energy_meter_def {
    static constexpr std::string_view topic    = "/energy_meter/state";                 // Foxglove话题，能量机关能量计模块状态JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::energy_meter_ch;   // 指向全局通道集合energy_meter_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 激光雷达原始检测框
struct ldm_detection_def {
    static constexpr std::string_view topic    = "/ldm/detection";                      // Foxglove话题，LDM激光雷达原始检测框JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::ldm_detection_ch;  // 指向全局通道集合ldm_detection_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 激光雷达测距测量值
struct ldm_measurement_def {
    static constexpr std::string_view topic    = "/ldm/measurement";                    // Foxglove话题，LDM激光雷达测距测量数据JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::ldm_measurement_ch;// 指向全局通道集合ldm_measurement_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 激光雷达滤波状态
struct ldm_state_def {
    static constexpr std::string_view topic    = "/ldm/state";                         // Foxglove话题，LDM激光雷达滤波内部状态JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::ldm_state_ch;      // 指向全局通道集合ldm_state_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// 仿真真值数据
struct ground_truth_def {
    static constexpr std::string_view topic    = "/ground_truth";                       // Foxglove话题，仿真环境输出真值数据JSON
    using channel_type                         = ::foxglove::RawChannel;                // 原始二进制透传通道
    using payload_type                         = std::vector<uint8_t>;                  // 载荷：原始字节数组存储json
    static constexpr auto member               = &FoxgloveChannels::ground_truth_ch;   // 指向全局通道集合ground_truth_ch成员
    static constexpr bool is_raw               = true;                                 // 原始二进制通道
    static constexpr std::string_view encoding = "json";                               // JSON编码标记
};

// -------------------------- 特殊系统内置通道描述符 TF/相机标定/日志 --------------------------
// 坐标变换通道，等价ROS2 /tf
struct tf_def {
    static constexpr std::string_view topic = "/tf";                                    // Foxglove话题，坐标系变换TF话题，等价ROS /tf
    using channel_type                      = ::foxglove::schemas::FrameTransformsChannel; // TF坐标变换通道类型
    using payload_type                      = ::foxglove::schemas::FrameTransforms;        // 载荷：多组坐标系变换数据包
    static constexpr auto member            = &FoxgloveChannels::tf_ch;                   // 指向全局通道集合tf_ch成员
    static constexpr bool is_raw            = false;                                      // Foxglove标准TF结构化消息
};

// 相机内参标定通道，等价ROS2 CameraInfo
struct calib_def {
    static constexpr std::string_view topic = "/camera_info";                           // Foxglove话题，相机内参标定信息，等价ROS CameraInfo
    using channel_type                      = ::foxglove::schemas::CameraCalibrationChannel; // 相机标定通道类型
    using payload_type                      = ::foxglove::schemas::CameraCalibration;         // 载荷：相机内参畸变参数结构体
    static constexpr auto member            = &FoxgloveChannels::camera_calib_ch;            // 指向全局通道集合camera_calib_ch成员
    static constexpr bool is_raw            = false;                                          // Foxglove标准相机标定结构化消息
};

// 全局日志通道，等价ROS2 /rosout
struct log_def {
    static constexpr std::string_view topic = "/log";                                   // Foxglove话题，系统日志输出，等价ROS /rosout
    using channel_type                      = ::foxglove::schemas::LogChannel;           // 日志消息通道类型
    using payload_type                      = ::foxglove::schemas::Log;                // 载荷：单条日志消息结构体（等级+文本）
    static constexpr auto member            = &FoxgloveChannels::log_ch;                // 指向全局通道集合log_ch成员
    static constexpr bool is_raw            = false;                                     // Foxglove标准日志结构化消息
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
// Foxglove 可视化消息类型别名定义
// 模板 FoxgloveMsg<T>：基于自定义 protobuf 定义 T，封装成可直接发送给 Foxglove Studio 的消息包装类型
// scene_def / track_scene_def ... 全部是 protobuf 消息结构体，对应每一类调试可视化数据

// ===================== 场景3D可视化消息（Foxglove 3D场景面板） =====================
using SceneMessage                = FoxgloveMsg<scene_def>;                  // 通用3D场景：点、线、立方体、标记等基础调试绘制
using TrackSceneMessage           = FoxgloveMsg<track_scene_def>;            // 目标跟踪3D场景：跟踪框、历史轨迹
using LdmTrackSceneMessage        = FoxgloveMsg<ldm_track_scene_def>;        // LDM能量机关专用跟踪3D可视化
using GimbalSceneMessage          = FoxgloveMsg<gimbal_scene_def>;            // 云台3D可视化：云台坐标系、姿态锥、指向射线
using AssociationSceneMessage     = FoxgloveMsg<association_scene_def>;     // 数据关联可视化：检测与目标匹配连线
using MpcPredictionSceneMessage   = FoxgloveMsg<mpc_prediction_scene_def>;   // MPC预测轨迹3D绘制，预测未来弹丸/机器人运动
using RuneSceneMessage            = FoxgloveMsg<rune_scene_def>;              // 能量机关3D场景：扇叶、机关本体标记
using RuneEkfSceneMessage         = FoxgloveMsg<rune_ekf_scene_def>;         // 能量机关EKF滤波状态3D可视化
using GroundTruthSceneMessage     = FoxgloveMsg<ground_truth_scene_def>;     // 真值3D场景，仿真/标定的真实目标位置

// ===================== 图像类消息（Foxglove Image面板，图像流叠加调试绘制） =====================
using ImageMessage                = FoxgloveMsg<img_def>;                     // 原始相机图像
using VideoMessage                = FoxgloveMsg<video_def>;                   // 视频流编码图像
using CalibrationImageMessage     = FoxgloveMsg<calibration_img_def>;         // 标定专用图像，角点、标定标记绘制
using BinaryImageMessage          = FoxgloveMsg<binary_img_def>;              // 二值化处理之后的掩码图像
using PatternImageMessage         = FoxgloveMsg<pattern_img_def>;             // 模板匹配/图案检测输出图像
using RuneArrowImageMessage       = FoxgloveMsg<rune_arrow_img_def>;          // 能量机关：箭头识别叠加图像
using RuneTargetImageMessage      = FoxgloveMsg<rune_target_img_def>;         // 能量机关：目标扇叶标记叠加图像
using RuneCenterImageMessage      = FoxgloveMsg<rune_center_img_def>;          // 能量机关：中心定位可视化图像
using EkfHeatmapMessage           = FoxgloveMsg<ekf_heatmap_def>;             // EKF置信热力图，概率热图输出

// ===================== 装甲、检测、测量调试消息 =====================
using DebugLightsMessage          = FoxgloveMsg<debug_lights_def>;            // 装甲灯条原始检测结果
using DebugArmorsMessage          = FoxgloveMsg<debug_armors_def>;            // 装甲板检测结果：框、角点、置信度
using MeasurementMessage          = FoxgloveMsg<measurement_def>;             // 测量输出：PnP解算位姿、距离观测值
using TargetMessage               = FoxgloveMsg<target_def>;                    // 最终输出目标：敌方装甲目标状态
using TargetSelectionTraceMessage = FoxgloveMsg<target_selection_trace_def>;  // 目标选择过程追踪日志，调试选靶逻辑

// ===================== 控制、规划、MPC 消息 =====================
using GimbalCmdMessage            = FoxgloveMsg<cmd_gimbal_def>;              // 云台控制指令，yaw/pitch输出指令
using MpcTrajectoryMessage        = FoxgloveMsg<mpc_traj_def>;                // MPC输出完整规划轨迹点序列

// ===================== 能量机关LDM模块专用调试消息 =====================
using RuneDebugMessage            = FoxgloveMsg<rune_debug_def>;               // 能量机关综合调试信息
using LdmDetectionMessage         = FoxgloveMsg<ldm_detection_def>;           // LDM能量机关原始检测输出
using LdmMeasurementMessage       = FoxgloveMsg<ldm_measurement_def>;         // LDM观测值（角度、距离）
using LdmStateMessage             = FoxgloveMsg<ldm_state_def>;               // LDM滤波器内部状态变量输出

// ===================== 算法模块调试消息 =====================
using PnPSolverMessage            = FoxgloveMsg<pnp_solver_def>;               // PnP求解器调试：迭代、残差、位姿结果
using NNConfidenceMessage         = FoxgloveMsg<nn_confidence_def>;            // 神经网络推理置信度输出
using EnergyMeterMessage          = FoxgloveMsg<energy_meter_def>;            // 能量值、弹丸计数等计数值调试

// ===================== 系统、真值、TF、日志、性能统计 =====================
using GroundTruthMessage          = FoxgloveMsg<ground_truth_def>;             // 真值数据，仿真给出真实位姿
using TfMessage                   = FoxgloveMsg<tf_def>;                       // 坐标变换树 TF，各个坐标系转换关系
using LogMessage                  = FoxgloveMsg<log_def>;                      // 文本日志，打印到foxglove控制台
using PerfStatsMessage            = FoxgloveMsg<perf_stats_def>;               // 性能统计：各模块耗时、帧率
using ResourceMessage             = FoxgloveMsg<resources_def>;                // 系统资源：CPU内存占用

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