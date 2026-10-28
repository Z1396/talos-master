#include "frame_synchronizer.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

using ipc::net::FrameSynchronizer;
using namespace std::chrono_literals;

void expect(const bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

ipc::PoseMeta make_pose(const uint64_t seq, const float marker) {
    ipc::PoseMeta pose{};
    pose.frame_seq     = seq;
    pose.position[0]   = marker;
    pose.quaternion[0] = 1.0F;
    pose.timestamp_ns  = seq * 1'000'000U;
    return pose;
}

void push_all(FrameSynchronizer& sync, const uint64_t seq, const float base) {
    for (uint8_t i = 0; i < FrameSynchronizer::kPoseCount; ++i) {
        sync.push_pose(make_pose(seq, base + static_cast<float>(i)), i);
    }
}

void out_of_order_and_interleaved_frames_stay_paired() {
    FrameSynchronizer sync;
    // 模拟跳帧：图像只取 seq=40，但 UDP 仍持续送来 39、40、41。
    push_all(sync, 39, 390.0F);
    sync.push_pose(make_pose(41, 410.0F), 0);
    sync.push_pose(make_pose(40, 400.0F), 0);
    sync.push_pose(make_pose(40, 401.0F), 1);
    sync.push_pose(make_pose(41, 411.0F), 1);
    sync.push_pose(make_pose(40, 402.0F), 2);
    sync.push_pose(make_pose(41, 412.0F), 2);
    sync.push_pose(make_pose(40, 403.0F), 3);
    sync.push_pose(make_pose(41, 413.0F), 3);

    const auto bundle = sync.wait_for_bundle(40, 1ms);
    expect(bundle.has_value(), "complete frame must be ready");
    for (size_t i = 0; i < FrameSynchronizer::kPoseCount; ++i) {
        expect((*bundle)[i].frame_seq == 40, "every pose must match image seq");
        expect(
            (*bundle)[i].position[0] == 400.0F + static_cast<float>(i),
            "pose must come from the matching frame, not the latest frame");
    }
}

void waiter_wakes_when_final_pose_arrives() {
    FrameSynchronizer sync;
    std::optional<FrameSynchronizer::PoseBundle> bundle;
    std::thread consumer([&] { bundle = sync.wait_for_bundle(7, 500ms); });
    std::this_thread::sleep_for(10ms);
    push_all(sync, 7, 70.0F);
    consumer.join();
    expect(bundle.has_value(), "waiter must wake after final pose arrives");
    expect((*bundle)[3].frame_seq == 7, "woken waiter received wrong frame");
}

void incomplete_frame_times_out_without_borrowing_neighbors() {
    FrameSynchronizer sync;
    push_all(sync, 8, 80.0F);
    sync.push_pose(make_pose(9, 90.0F), 0);
    const auto begin   = std::chrono::steady_clock::now();
    const auto bundle  = sync.wait_for_bundle(9, 20ms);
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    expect(!bundle.has_value(), "incomplete frame must not fall back to another seq");
    expect(elapsed >= 15ms, "incomplete frame should wait for late UDP packets");
    const auto other = sync.wait_for_bundle(8, 1ms);
    expect(other.has_value(), "timed-out frame must not consume a different frame");
}

void delivered_frame_cannot_be_republished_by_late_packets() {
    FrameSynchronizer sync;
    push_all(sync, 10, 100.0F);
    expect(sync.wait_for_bundle(10, 1ms).has_value(), "initial delivery failed");
    expect(!sync.wait_for_bundle(10, 1ms).has_value(), "same bundle delivered twice");
    push_all(sync, 10, 999.0F);
    expect(!sync.wait_for_bundle(10, 1ms).has_value(), "late packet resurrected old frame");
}

void cache_is_bounded_and_supports_frame_zero() {
    FrameSynchronizer sync;
    push_all(sync, 0, 0.0F);
    expect(sync.wait_for_bundle(0, 1ms).has_value(), "frame zero must be supported");
    for (uint64_t seq = 1; seq <= FrameSynchronizer::kMaxCachedFrames + 20; ++seq) {
        sync.push_pose(make_pose(seq, static_cast<float>(seq)), 0);
    }
    expect(
        sync.cached_frame_count() == FrameSynchronizer::kMaxCachedFrames,
        "pose cache must remain bounded");
}

} // namespace

int main() {
    try {
        out_of_order_and_interleaved_frames_stay_paired();
        waiter_wakes_when_final_pose_arrives();
        incomplete_frame_times_out_without_borrowing_neighbors();
        delivered_frame_cannot_be_republished_by_late_packets();
        cache_is_bounded_and_supports_frame_zero();
        std::cout << "frame_synchronizer_test: all 5 tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "frame_synchronizer_test: " << error.what() << '\n';
        return 1;
    }
}
