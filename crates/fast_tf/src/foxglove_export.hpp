#pragma once
// 头文件保护宏，防止重复包含引发重定义

// 坐标系帧、变换缓存定义
#include "frame.hpp"
// 基础类型：时间戳、约束模板 non_root_frame / root_frame 等
#include "types.hpp"

// Foxglove WebSocket 可视化标准消息结构
#include <foxglove/schemas.hpp>

namespace fast_tf {

/**
 * @brief Foxglove 可视化TF坐标变换导出器
 * 功能：将内部坐标系树变换，序列化为Foxglove标准 FrameTransforms 消息并通过通道输出
 * 区分静态/动态变换：静态变换时间戳置空，动态填充纳秒级时间戳
 * 对外提供批量全量发布、单帧链发布、单变换发布三套接口
 */
class FoxgloveExporter {
public:
    /**
     * @brief 构造函数，绑定Foxglove消息输出通道
     * @param channel Foxglove FrameTransforms 专用通道实例，所有权转移
     */
    explicit FoxgloveExporter(foxglove::schemas::FrameTransformsChannel channel)
        : channel_(std::move(channel)) {}

    /**
     * @brief 发布系统内所有已初始化坐标系变换（推荐主接口）
     * @param system 全局坐标系统容器，存储所有帧变换缓存
     * @param ts 本次发布查询时间戳（纳秒）
     *
     * 逻辑：
     * 1. 遍历全部非根帧变换缓存；
     * 2. 取缓存最新有效变换，无插值结果则使用最后一条历史数据；
     * 3. 区分静态变换（缓存时间区间全0）、动态变换；
     * 4. 统一打包进单条 FrameTransforms 消息批量输出；
     * 5. 静态变换 timestamp 字段设为 std::nullopt，动态填充秒/纳秒时间戳。
     */
    void publish_all(const CoordinateSystem& system, const uint64_t ts) noexcept {
        // 初始化Foxglove批量变换消息容器
        foxglove::schemas::FrameTransforms transforms_msg;

        // 遍历所有坐标系帧缓存，回调处理每一组父子帧变换
        // 遍历所有坐标变换buffer；回调里面每个回调对应一组【父坐标系 → 子坐标系】的变换缓存
        for_each_coordinate_buffer(system, [&](auto frame_tag, const auto& buffer) {
            // 提取当前子帧类型、父祖先帧类型（编译期模板推导）
            // frame_tag 是编译期标签类型，不是运行时字符串
            using Descendant = typename decltype(frame_tag)::type; // 子坐标系（子帧）类型
            using Ancestor   = typename Descendant::ancestor;      // 父坐标系（父帧）类型

            // 读取缓存内最新一条变换记录
            auto latest_result = buffer.latest();
            // 缓存无任何有效数据，直接跳过该帧
            if (!latest_result) {
                return;
            }

            // ============ 时间戳ts：我们想要查询这个时刻的坐标变换 ============
            // 按目标时间戳插值获取变换
            auto tf_result = buffer.lookup(ts, interpolate);
            // 插值失败（时间戳超出缓存区间：ts太早/太晚，不在缓存时间窗口内），降级使用最新一条历史变换
            if (!tf_result) {
                tf_result = latest_result;
            }

            // 获取缓存整体时间区间 [起始时间, 结束时间]
            const auto time_range = buffer.time_range();
            // 判断静态变换：缓存起止时间都为0，代表固定不变的静态TF（比如相机外参、安装偏移，永远不变）
            const bool is_static  = time_range && time_range->first == 0 && time_range->second == 0;

            // 构造单条帧变换消息，Foxglove官方schema结构体 FrameTransform
            foxglove::schemas::FrameTransform frame_transform;

            // 父坐标系ID字符串，来自编译期定义的frame_id常量
            frame_transform.parent_frame_id = std::string(Ancestor::frame_id);
            // 子坐标系ID字符串
            frame_transform.child_frame_id  = std::string(Descendant::frame_id);

            // ========== 静态TF 和动态TF 的时间戳处理（Foxglove规则） ==========
            // 静态变换：时间戳字段置空std::nullopt，Foxglove识别为static_transform，永久生效
            // 动态TF：填入时间戳，单位：秒 + 纳秒（foxglove::schemas::Timestamp 格式）
            if (is_static) {
                frame_transform.timestamp = std::nullopt;
            } else {
                frame_transform.timestamp = foxglove::schemas::Timestamp{
                    static_cast<uint32_t>(ts / 1'000'000'000), // 总秒数，ts是纳秒时间戳
                    static_cast<uint32_t>(ts % 1'000'000'000)  // 剩下的纳秒部分
                };
            }

            // 从插值结果拿平移、四元数旋转
            const auto translation      = tf_result->value.translation();
            const auto quat             = tf_result->value.quaternion();

            // 填进Foxglove消息，注意：Foxglove的坐标结构体是简单的{x,y,z}
            frame_transform.translation = {translation.x(), translation.y(), translation.z()};
            frame_transform.rotation    = {quat.x(), quat.y(), quat.z(), quat.w()};

            // 把这条变换塞到大批量消息 transforms_msg 里面
            // transforms_msg 类型是 FrameTransforms，一次可以携带多条FrameTransform，批量发送
            transforms_msg.transforms.push_back(frame_transform);
        });

        // 通过通道输出完整批量消息，忽略返回错误码
        [[maybe_unused]] auto error = channel_.log(transforms_msg);
    }

    /**
     * @brief 发布从指定子帧一路向上到根坐标系的整条变换链（单条消息打包）
     * @deprecated 废弃接口，推荐统一使用 publish_all()，未来版本移除
     * @tparam Descendant 目标子帧模板约束，必须是非根帧 non_root_frame
     * @param system 全局坐标系统
     * @param ts 查询时间戳（纳秒）
     */
    template <non_root_frame Descendant>
    [[deprecated("Use publish_all() instead - will be removed in future version")]]
    void publish_entire(const CoordinateSystem& system, const uint64_t ts) noexcept {
        foxglove::schemas::FrameTransforms transforms_msg;

        // 递归遍历当前帧所有父级祖先变换，追加到消息
        append_ancestors<Descendant>(system, transforms_msg, ts);

        // 输出整条链变换消息
        [[maybe_unused]] auto error = channel_.log(transforms_msg);
    }

    /**
     * @brief 仅发布单一组父子帧变换 Descendant -> Descendant::ancestor
     * @tparam Descendant 子帧模板约束，非根帧
     * @param system 全局坐标系（仅模板匹配占位，内部不使用）
     * @param transform 待发布的边变换结构体（平移+四元数）
     * @param ts 本次发布时间戳
     * @param transform_ts 变换原始存储时间戳，0代表静态变换
     */
    template <non_root_frame Descendant>
    void publish_single(
        [[maybe_unused]] const CoordinateSystem& system, const EdgeTransform<Descendant>& transform,
        const uint64_t ts, const timestamp_ns_t transform_ts) noexcept {

        foxglove::schemas::FrameTransforms transforms_msg;

        // 原始时间戳为0判定静态变换
        const bool is_static = (transform_ts == 0);
        // 将单条变换追加到消息容器
        append_transform(transforms_msg, transform, ts, is_static);

        // 输出单变换消息
        [[maybe_unused]] auto error = channel_.log(transforms_msg);
    }

private:
    // Foxglove 标准变换消息输出通道句柄
    foxglove::schemas::FrameTransformsChannel channel_;

    /**
     * @brief 递归回溯当前帧所有祖先变换，追加到消息（废弃辅助函数）
     * @deprecated 随 publish_entire 一同移除
     * @tparam Descendant 当前遍历帧类型
     * @param system 坐标系系统
     * @param msg 待填充的批量变换消息
     * @param ts 查询时间戳
     */
    template <frame Descendant>
    [[deprecated("Use publish_all() instead - will be removed in future version")]]
    void append_ancestors(
        const CoordinateSystem& system, foxglove::schemas::FrameTransforms& msg,
        uint64_t ts) noexcept {
        // 根帧无父变换，终止递归
        if constexpr (!root_frame<Descendant>) {
            // 获取当前帧变换缓存
            const auto& buffer = buffer_of<Descendant>(system);
            // 插值查询变换
            if (auto result = buffer.lookup(ts, interpolate); result) {
                // 读取缓存时间区间判断静态/动态
                const auto time_range = buffer.time_range();
                const bool is_static =
                    time_range && time_range->first == 0 && time_range->second == 0;
                // 把当前帧变换写入消息
                append_transform<Descendant>(msg, result->value, ts, is_static);
                // 递归处理父祖先帧
                append_ancestors<typename Descendant::ancestor>(system, msg, ts);
            }
        }
    }

    /**
     * @brief 静态辅助函数：将单组EdgeTransform序列化为Foxglove FrameTransform并加入消息
     * @tparam Descendant 子帧类型
     * @param msg 批量消息容器
     * @param transform 父子帧变换数据
     * @param ts 发布时间戳
     * @param is_static 是否静态变换（控制timestamp字段）
     */
    template <non_root_frame Descendant>
    static void append_transform(
        foxglove::schemas::FrameTransforms& msg, const EdgeTransform<Descendant>& transform,
        const uint64_t ts, bool is_static) noexcept {
        using Ancestor = Descendant::ancestor;

        // 提取平移向量、旋转四元数
        const auto translation = transform.translation();
        const auto quat        = transform.quaternion();

        foxglove::schemas::FrameTransform frame_transform;
        // 父子帧ID赋值
        frame_transform.parent_frame_id = std::string(Ancestor::frame_id);
        frame_transform.child_frame_id  = std::string(Descendant::frame_id);

        // 静态变换清空时间戳，动态拆分纳秒时间戳为秒+纳秒
        if (is_static) {
            frame_transform.timestamp = std::nullopt;
        } else {
            frame_transform.timestamp = foxglove::schemas::Timestamp{
                static_cast<uint32_t>(ts / 1'000'000'000),
                static_cast<uint32_t>(ts % 1'000'000'000)};
        }

        // 填充平移XYZ、旋转四元数XYZW
        frame_transform.translation = {translation.x(), translation.y(), translation.z()};
        frame_transform.rotation    = {quat.x(), quat.y(), quat.z(), quat.w()};

        // 追加到批量消息的变换数组
        msg.transforms.push_back(frame_transform);
    }
};

} // namespace fast_tf