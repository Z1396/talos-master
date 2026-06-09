#pragma once

#include "frame.hpp"
#include "types.hpp"

#include <foxglove/schemas.hpp>

namespace fast_tf {

class FoxgloveExporter {
public:
    explicit FoxgloveExporter(foxglove::schemas::FrameTransformsChannel channel)
        : channel_(std::move(channel)) {}

    /**
     * @brief 发布所有已初始化的坐标系变换
     * @param system 坐标系统
     * @param ts 查询时间戳
     *
     * 遍历所有非 root frame 的 buffer，发布所有有数据的变换。
     * 静态变换（timestamp=0）的 timestamp 字段设为 std::nullopt。
     */
    void publish_all(const CoordinateSystem& system, const uint64_t ts) noexcept {
        foxglove::schemas::FrameTransforms transforms_msg;

        for_each_coordinate_buffer(system, [&](auto frame_tag, const auto& buffer) {
            using Descendant = typename decltype(frame_tag)::type;
            using Ancestor   = typename Descendant::ancestor;

            auto latest_result = buffer.latest();
            if (!latest_result) {
                return;
            }

            auto tf_result = buffer.lookup(ts, interpolate);
            if (!tf_result) {
                tf_result = latest_result;
            }

            const auto time_range = buffer.time_range();
            const bool is_static  = time_range && time_range->first == 0 && time_range->second == 0;

            foxglove::schemas::FrameTransform frame_transform;
            frame_transform.parent_frame_id = std::string(Ancestor::frame_id);
            frame_transform.child_frame_id  = std::string(Descendant::frame_id);

            if (is_static) {
                frame_transform.timestamp = std::nullopt;
            } else {
                frame_transform.timestamp = foxglove::schemas::Timestamp{
                    static_cast<uint32_t>(ts / 1'000'000'000),
                    static_cast<uint32_t>(ts % 1'000'000'000)};
            }

            const auto translation      = tf_result->value.translation();
            const auto quat             = tf_result->value.quaternion();
            frame_transform.translation = {translation.x(), translation.y(), translation.z()};
            frame_transform.rotation    = {quat.x(), quat.y(), quat.z(), quat.w()};

            transforms_msg.transforms.push_back(frame_transform);
        });

        [[maybe_unused]] auto error = channel_.log(transforms_msg);
    }

    /**
     * @brief 发布从 Descendant 到 root 的整条链上的所有变换（单条消息）
     * @deprecated Use publish_all() instead - will be removed in future version
     */
    template <non_root_frame Descendant>
    [[deprecated("Use publish_all() instead - will be removed in future version")]]
    void publish_entire(const CoordinateSystem& system, const uint64_t ts) noexcept {
        foxglove::schemas::FrameTransforms transforms_msg;

        // 直接从 Descendant 开始递归，统一从 system 取
        append_ancestors<Descendant>(system, transforms_msg, ts);

        [[maybe_unused]] auto error = channel_.log(transforms_msg);
    }

    /**
     * @brief 只发布单个变换 Descendant -> Descendant::ancestor
     * @param system 坐标系统
     * @param transform 要发布的变换
     * @param ts 查询/发布时间戳
     * @param transform_ts 变换的原始时间戳（用于判断静态/动态，0表示静态）
     */
    template <non_root_frame Descendant>
    void publish_single(
        [[maybe_unused]] const CoordinateSystem& system, const EdgeTransform<Descendant>& transform,
        const uint64_t ts, const timestamp_ns_t transform_ts) noexcept {

        foxglove::schemas::FrameTransforms transforms_msg;

        // 判断是否为静态变换：timestamp_ns=0 表示静态
        const bool is_static = (transform_ts == 0);
        append_transform(transforms_msg, transform, ts, is_static);

        [[maybe_unused]] auto error = channel_.log(transforms_msg);
    }

private:
    foxglove::schemas::FrameTransformsChannel channel_;

    /**
     * @brief 递归添加祖先链的变换
     * @deprecated Use publish_all() instead - will be removed in future version
     */
    template <frame Descendant>
    [[deprecated("Use publish_all() instead - will be removed in future version")]]
    void append_ancestors(
        const CoordinateSystem& system, foxglove::schemas::FrameTransforms& msg,
        uint64_t ts) noexcept {
        if constexpr (!root_frame<Descendant>) {
            const auto& buffer = buffer_of<Descendant>(system);
            if (auto result = buffer.lookup(ts, interpolate); result) {
                const auto time_range = buffer.time_range();
                const bool is_static =
                    time_range && time_range->first == 0 && time_range->second == 0;
                append_transform<Descendant>(msg, result->value, ts, is_static);
                append_ancestors<typename Descendant::ancestor>(system, msg, ts);
            }
        }
    }

    template <non_root_frame Descendant>
    static void append_transform(
        foxglove::schemas::FrameTransforms& msg, const EdgeTransform<Descendant>& transform,
        const uint64_t ts, bool is_static) noexcept {
        using Ancestor = Descendant::ancestor;

        const auto translation = transform.translation();
        const auto quat        = transform.quaternion();

        foxglove::schemas::FrameTransform frame_transform;
        frame_transform.parent_frame_id = std::string(Ancestor::frame_id);
        frame_transform.child_frame_id  = std::string(Descendant::frame_id);

        // 静态变换的 timestamp 为空（std::nullopt），动态变换才有时间戳
        if (is_static) {
            frame_transform.timestamp = std::nullopt;
        } else {
            frame_transform.timestamp = foxglove::schemas::Timestamp{
                static_cast<uint32_t>(ts / 1'000'000'000),
                static_cast<uint32_t>(ts % 1'000'000'000)};
        }

        frame_transform.translation = {translation.x(), translation.y(), translation.z()};
        frame_transform.rotation    = {quat.x(), quat.y(), quat.z(), quat.w()};

        msg.transforms.push_back(frame_transform);
    }
};

} // namespace fast_tf
