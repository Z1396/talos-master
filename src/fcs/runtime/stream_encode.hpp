#pragma once
// #pragma once：头文件保护宏，作用等价于 #ifndef / #define / #endif
// 防止同一个头文件被多次#include引发重复定义编译错误，现代C++项目广泛使用

// 内部模块头文件
#include "quanta/stream_encoder.hpp"
// 引入 quanta::EncodeParams 编码器参数类型声明
#include "scheduler/thin.hpp"
// 引入 talos::World、talos::Scheduler（Talos ECS调度器、ECS世界容器类型）

// 标准库头文件
#include <expected>
// C++23标准库：std::expected<T,E>，用于函数优雅返回【成功结果 / 错误信息】
#include <string>
// std::string，承载错误文本信息

namespace fcs::runtime {
// fcs顶层命名空间 -> runtime运行时模块命名空间

/**
 * @brief 注册Quanta视频流媒体整套ECS系统入口函数
 * 【接口语义】：初始化编码器 + 向Talos调度器挂载视频编码系统 + 注册网络发送系统
 * 调用时机：机器人程序启动、流媒体服务初始化阶段调用一次
 *
 * @param world        talos::World&
 *        Talos ECS世界，全局资源容器；会在此World中创建QuantaPacketQueue数据包队列资源
 * @param scheduler    talos::Scheduler&
 *        Talos任务调度器，编码system将注册到此调度器定时执行
 * @param encode_params const quanta::EncodeParams&
 *        编码器配置常量：目标码率、最大分辨率、帧率、编码档次等参数
 * @param src_width    int 原始输入图像初始宽度
 * @param src_height   int 原始输入图像初始高度
 *
 * @return std::expected<void, std::string>
 *      成功：返回 std::expected 包含空值 void
 *      失败：返回 std::unexpected(std::string)，字符串携带失败原因（编码器创建失败等）
 *
 * [[nodiscard]] 属性：强制调用方接收返回值
 * ❌禁止忽略返回值：如果丢弃返回值，编译器给出警告
 * ✅目的：防止上层调用不检测初始化失败，出现编码器无效却继续运行的隐性bug
 */
[[nodiscard]] std::expected<void, std::string> register_quanta_stream_systems(
    talos::World& world,
    talos::Scheduler& scheduler,
    const quanta::EncodeParams& encode_params,
    int src_width,
    int src_height);

} // namespace fcs::runtime