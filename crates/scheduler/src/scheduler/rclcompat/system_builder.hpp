#pragma once
// 头文件保护宏，防止重复包含重定义

// 执行策略信息结构体定义
#include "../system/execution_policy.hpp"
// SystemMeta、ChannelMeta、channel_kind 通道类型枚举定义
#include "../system/system_meta.hpp"

// 系统名称字符串
#include <string>
// 运行时类型索引 type_index，用于记录消息/话题类型
#include <typeindex>
// std::move 移动语义
#include <utility>

namespace talos::scheduler::rclcompat {

// ============================================================================
// SystemMetaBuilder：SystemMeta 运行时构建器（流式建造者模式）
// ============================================================================
/**
 * @brief SystemMeta 运行时元数据建造器
 *
 * 用于 RCL 兼容层系统，在运行时动态组装系统元数据，替代编译期从函数签名提取元信息的逻辑。
 * 流式链式调用 API，分步设置系统名、执行策略、读写通道，最后生成完整 SystemMeta。
 *
 * ## 使用示例
 *
 * ```cpp
 * auto meta = SystemMetaBuilder("camera_pub")
 *     .policy(make_policy_info<pool_compute>())
 *     .add_spmc_writer(typeid(ImageFrame), typeid(CameraTag))
 *     .build();
 * ```
 */
class SystemMetaBuilder {
public:
    /**
     * @brief 构造建造器，传入系统唯一名称
     * @param name 系统标识字符串，移动语义存入内部meta
     * noexcept 无异常抛出
     */
    explicit SystemMetaBuilder(std::string name) noexcept { meta_.name = std::move(name); }

    /**
     * @brief 设置系统执行调度策略
     * @param p PolicyInfo 线程池/调度优先级信息
     * @return 当前建造器引用，支持链式连续调用
     */
    SystemMetaBuilder& policy(system::PolicyInfo p) noexcept {
        meta_.policy = p;
        return *this;
    }

    /**
     * @brief 通用底层通道添加模板函数，统一处理所有类型通道
     * @tparam Kind 通道种类枚举 channel_kind：spmc_writer/spmc_reader/spsc_writer/spsc_reader
     * @param type 消息数据类型 type_index
     * @param topic 话题标签类型 type_index
     * @return 当前建造器引用，链式调用
     *
     * 编译期分支：
     * - SPMC 读写通道 → 存入meta_.spmc_channels 数组
     * - SPSC 读写通道 → 存入meta_.spsc_channels 数组
     */
    template <system::channel_kind Kind>
    SystemMetaBuilder&
        add_channel(const std::type_index type, const std::type_index topic) noexcept {
        if constexpr (
            Kind == system::channel_kind::spmc_writer
            || Kind == system::channel_kind::spmc_reader) {
            // 多生产者/多消费者通道，加入spmc列表
            meta_.spmc_channels.push_back(
                system::ChannelMeta{.type = type, .topic = topic, .kind = Kind});
        } else {
            // 单生产者/单消费者通道，加入spsc列表
            meta_.spsc_channels.push_back(
                system::ChannelMeta{.type = type, .topic = topic, .kind = Kind});
        }
        return *this;
    }

    // ========== 便捷封装函数：SPMC 多生产者多消费者通道 ==========
    /**
     * @brief 添加SPMC写通道（发布器）
     * @param type 消息类型type_index
     * @param topic 话题标签type_index
     */
    SystemMetaBuilder&
        add_spmc_writer(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spmc_writer>(type, topic);
    }

    /**
     * @brief 添加SPMC读通道（订阅器）
     */
    SystemMetaBuilder&
        add_spmc_reader(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spmc_reader>(type, topic);
    }

    // ========== 便捷封装函数：SPSC 单生产者单消费者通道 ==========
    /**
     * @brief 添加SPSC写通道
     */
    SystemMetaBuilder&
        add_spsc_writer(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spsc_writer>(type, topic);
    }

    /**
     * @brief 添加SPSC读通道
     */
    SystemMetaBuilder&
        add_spsc_reader(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spsc_reader>(type, topic);
    }

    /**
     * @brief 建造完成：右值版本，移动取出内部SystemMeta，无拷贝
     * 仅临时对象调用 builder.build() 触发此重载
     * @return 完整SystemMeta元数据（移动语义）
     */
    [[nodiscard]] system::SystemMeta build() && noexcept { return std::move(meta_); }

    /**
     * @brief 建造完成：左值版本，拷贝内部meta返回副本
     * 若builder后续仍需复用，调用此重载，不会清空内部数据
     * @return SystemMeta 拷贝副本
     */
    [[nodiscard]] system::SystemMeta build() const& noexcept { return meta_; }

private:
    // 内部存储正在构建的系统元数据
    system::SystemMeta meta_;
};

} // namespace talos::scheduler::rclcompat