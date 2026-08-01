// 头文件保护指令：防止该头文件被同一编译单元重复包含，替代传统 #ifndef / #define / #endif 写法
#pragma once

// 引入Foxglove可视化服务核心服务类头文件
#include "foxglove_server.hpp"

// 数学库，提供 cos/sin 三角函数，用于球坐标转笛卡尔坐标
#include <cmath>
// 项目自定义消息结构体定义头文件
#include <messages.hpp>
// 项目通用工具函数、辅助模板头文件
#include <utility.hpp>

// 命名空间层级：fcs框架 -> visualization可视化模块 -> detail内部实现细节（对外隐藏）
namespace fcs::visualization::detail {

// ============================================================================
// System Helper Functions - 共享的系统辅助函数
// ============================================================================
//
// 这些函数在多个 foxglove 发布系统中共享使用。
// 放置在独立头文件中，避免重复定义和编译依赖。
// ============================================================================

/**
 * @brief 校验Foxglove服务是否就绪，并且对应通道存在待发送新数据
 * @param server Foxglove服务智能指针
 * @param channel 任意通道对象（支持任意具备 has_new() 成员的通道类型，auto实现泛型）
 * @return bool true=服务就绪且通道有新数据可发送；false=不可发送
 * @note [[nodiscard]] 强制要求调用方接收返回值，禁止丢弃布尔结果，避免逻辑错误
 * @note noexcept 函数不会抛出异常，编译器可优化，嵌入式/实时场景减少开销
 * @note 参数 channel 使用 auto 模板语法(C++14)，泛型适配所有通道类型，不用写模板参数
 */
[[nodiscard]] inline bool
foxglove_ready(const std::shared_ptr<FoxgloveServer>& server, const auto& channel) noexcept 
{
    // 三段校验：
    // 1. server指针非空；2. Foxglove服务已经完成初始化启动；3. 当前通道存在未发送的新数据
    return server && server->is_initialized() && channel.has_new();
}

/**
 * @brief 模板函数：将JSON对象打包成指定消息类型，推入Foxglove发送队列
 * @tparam MessageType 目标消息结构体类型，必须存在 payload 字段用于承载字节流
 * @param server Foxglove服务智能指针
 * @param json_obj nlohmann::json 结构化JSON数据
 * @note noexcept 无异常抛出
 */
template <typename MessageType>
inline void publish_json_message(
    const std::shared_ptr<FoxgloveServer>& server, const nlohmann::json& json_obj) noexcept {
    MessageType msg;
    // json.dump() 将JSON序列化为字符串；json_to_bytes 将字符串转为二进制字节数组存入消息负载
    msg.payload = visualization::json_to_bytes(json_obj.dump());
    // std::move 移动语义，避免消息拷贝，直接转移msg所有权送入发送队列，提升性能
    server->enqueue_message(std::move(msg));
}

/**
 * @brief 球坐标系(距离、偏航、俯仰) 转换为 Foxglove Vector3 笛卡尔直角坐标
 * @param distance 径向距离（球半径）
 * @param yaw 偏航角（水平面旋转，航向角，弧度）
 * @param pitch 俯仰角（竖直方向俯仰，弧度）
 * @return ::foxglove::schemas::Vector3 Foxglove协议定义的三维向量结构
 * 坐标系约定：
 * x：前向、y：横向、z：竖直向上，机器人常用右手坐标系
 */
[[nodiscard]] inline ::foxglove::schemas::Vector3
spherical_to_cartesian(double distance, double yaw, double pitch) noexcept {
    ::foxglove::schemas::Vector3 v;
    // 球坐标转笛卡尔标准公式
    v.x = distance * ::cos(pitch) * ::cos(yaw);
    v.y = distance * ::cos(pitch) * ::sin(yaw);
    v.z = distance * ::sin(pitch);
    return v;
}

/**
 * @brief 重载版本：球坐标转为 Foxglove Point3 点位结构
 * @param distance 径向距离
 * @param yaw 偏航角(rad)
 * @param pitch 俯仰角(rad)
 * @return ::foxglove::schemas::Point3 Foxglove点位结构体，用于标记空间点位
 * @details Vector3 多用于向量、速度；Point3 多用于空间坐标点，协议结构不同，因此做两份重载
 */
[[nodiscard]] inline ::foxglove::schemas::Point3
spherical_to_cartesian_point3(double distance, double yaw, double pitch) noexcept {
    ::foxglove::schemas::Point3 v;
    v.x = distance * ::cos(pitch) * ::cos(yaw);
    v.y = distance * ::cos(pitch) * ::sin(yaw);
    v.z = distance * ::sin(pitch);
    return v;
}

/**
 * @brief 场景实体批量发布模板：仅当存在实体/删除指令时才打包发送，避免空消息占用带宽
 * @tparam SceneMessageType 场景消息模板类型，内置 deletions、entities 两个成员容器
 * @param server Foxglove服务指针
 * @param entities 待新增/更新的3D场景实体（立方体、球体、线条、文本等可视化元素）
 * @param deletions 需要删除的实体列表，默认空；传入 SceneEntityDeletion::ALL 清空整条通道所有实体
 * @note 入参entities/deletions使用std::vector值传递，内部通过std::move转移所有权，减少拷贝
 */
template <typename SceneMessageType>
inline void publish_scene_if_nonempty(
    const std::shared_ptr<FoxgloveServer>& server,
    std::vector<::foxglove::schemas::SceneEntity> entities,
    std::vector<::foxglove::schemas::SceneEntityDeletion> deletions = {}) noexcept {
    // 判定：有实体要绘制 或者 有实体要删除，才组装消息发送
    if (!entities.empty() || !deletions.empty()) {
        SceneMessageType msg;
        // 将删除列表、实体列表移动至消息内部，避免拷贝构造
        msg.payload.deletions = std::move(deletions);
        msg.payload.entities  = std::move(entities);
        // 消息移动入发送队列
        server->enqueue_message(std::move(msg));
    }
}

} // namespace fcs::visualization::detail