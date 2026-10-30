#pragma once

#include "shm_layout.hpp"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>

namespace ipc::net {

// 按采集帧号缓存 TF 位姿，防止网络跳帧/乱序后把“最新位姿”错配给旧图像。
// 只缓存 T_odom_camera / T_odom_muzzle 实际依赖的 0~3 槽；第 4 槽是底盘兼容通道。
class FrameSynchronizer {
public:
    static constexpr size_t kPoseCount       = 4;                       // 位姿数量
    static constexpr size_t kMaxCachedFrames = 128;                     // 最大缓存帧数
    using PoseBundle                         = std::array<PoseMeta, kPoseCount>; // 位姿包

    // 推收位姿，更新缓存。
    void push_pose(const PoseMeta& pose, const uint8_t index) {
        if (index >= kPoseCount) {
            return;
        }
        {
            std::lock_guard lock(mutex_);
            // 检查 seq 是否已交付
            if (is_consumed_locked(pose.frame_seq)) {
                return; // 已交付帧的迟到 UDP 包不得再次发布。
            }
            frames_[pose.frame_seq].poses[index] = pose;
            prune_locked();  // 清理已交付的帧
        }
        signal_.notify_all();
    }

    // 等待 seq 对应的四路位姿全部到齐。成功返回后即消费该组，同组不会交付两次。
    template <typename Rep, typename Period>
    std::optional<PoseBundle> wait_for_bundle(const uint64_t seq, const std::chrono::duration<Rep, Period> timeout) 
    {
        std::unique_lock lock(mutex_);
        const bool completed = signal_.wait_for(
            lock, timeout, [&] { return is_consumed_locked(seq) || is_ready_locked(seq); });
        if (!completed || is_consumed_locked(seq)) {
            return std::nullopt;
        }

        const auto it = frames_.find(seq);
        if (it == frames_.end() || !is_ready_locked(seq)) {
            return std::nullopt;
        }
        // 构建位姿包
        // 从缓存中移除已交付的帧
        // 更新 last_consumed_seq_ 为当前 seq
        PoseBundle bundle{};
        for (size_t i = 0; i < kPoseCount; ++i) {
            // *it->second.poses[i] 表示获取第 i 个位姿的可选值，确保有值
            bundle[i] = *it->second.poses[i];
        }
        // 从缓存中移除已交付的帧
        frames_.erase(it);
        // 更新 last_consumed_seq_ 为当前 seq
        last_consumed_seq_ = seq;
        return bundle;
    }

    // IMAGE_SYNC 只是“图像已经发出”的提示；TCP 帧头 seq 才是配对依据。
    // 不强制等待该 UDP 包，避免一次 UDP 丢包就永久卡住对应图像。
    void notify_image_sync() { signal_.notify_all(); }

    // 获取当前缓存的帧数。
    size_t cached_frame_count() const {
        std::lock_guard lock(mutex_);
        return frames_.size();
    }

private:
    // 每个采集帧的位姿包。
    struct FramePoses {
        std::array<std::optional<PoseMeta>, kPoseCount> poses{};
    };

    // 检查 seq 是否已交付。
    bool is_consumed_locked(const uint64_t seq) const {
        return last_consumed_seq_.has_value() && seq <= *last_consumed_seq_;
    }

    // 检查 seq 对应的四路位姿是否都已缓存。
    // 未交付帧的迟到 UDP 包不得再次发布。
    // 已交付帧的迟到 UDP 包不得再次发布。
    // 未缓存的位姿不得作为位姿包交付。
    bool is_ready_locked(const uint64_t seq) const {
        const auto it = frames_.find(seq);
        if (it == frames_.end()) {
            return false;
        }
        for (const auto& pose : it->second.poses) {
            if (!pose.has_value()) {
                return false;
            }
        }
        return true;
    }

    // 从缓存中淘汰最老帧，保持最大缓存帧数。
    void prune_locked() {
        // 从缓存中淘汰最老帧，保持最大缓存帧数。
        while (frames_.size() > kMaxCachedFrames) {
            frames_.erase(frames_.begin()); // map 按 seq 排序，优先淘汰最老帧。
        }
    }

    mutable std::mutex mutex_;                  // 保护 frames_ 访问
    std::condition_variable signal_;            // 通知等待者有新帧
    std::map<uint64_t, FramePoses> frames_;     // 按采集帧号缓存 TF 位姿
    std::optional<uint64_t> last_consumed_seq_; // 最后交付的帧号
};

} // namespace ipc::net
