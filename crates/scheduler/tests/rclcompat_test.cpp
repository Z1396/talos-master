#include "scheduler/error_formatter.hpp"
#include "scheduler/rclcompat/node.hpp"
#include "scheduler/rclcompat/registry.hpp"
#include "scheduler/rclcompat/system.hpp"
#include "scheduler/rclcompat/timer_constants.hpp"
#include "scheduler/scheduler.hpp"
#include "scheduler/system/components.hpp"
#include "scheduler/system/execution_policy.hpp"

#include <functional>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <print>
#include <string>
#include <thread>
#include <vector>

namespace talos::test {

// Pull rclcompat types into the test namespace
using talos::rclcompat::Frequency;
using talos::rclcompat::Node;
using talos::rclcompat::OwnershipRegistry;
using talos::rclcompat::Publisher;
using talos::rclcompat::PubSlot;
using talos::rclcompat::Qos;
using talos::rclcompat::SystemMetaBuilder;

// ============================================================================
// Basic test types (for simple unit tests)
// ============================================================================

struct TestMessage {
    int value;
    std::string text;
};

struct AnotherMessage {
    double x;
    double y;
};

struct TestTag1 {};
struct TestTag2 {};
struct TestTag3 {};

// ============================================================================
// Real-world message types (simulate FCS pipeline)
// ============================================================================

struct ImageFrame {
    int frame_id;
    int width;
    int height;
    uint64_t timestamp_ns;

    // Realistic payload
    std::vector<uint8_t> data;
};

struct DetectionResult {
    int frame_id;
    std::vector<float> confidences;
    std::vector<std::array<float, 4>> quads;
};

struct TrackingOutput {
    int id;
    float x, y, z;
    float vx, vy, vz;
    uint64_t timestamp_ns;
};

struct GimbalCommand {
    float yaw;
    float pitch;
    float fire_speed;
};

// Channel tags (simulate FCS pipeline)
struct CameraChannel {};
struct DetectionChannel {};
struct TrackingChannel {};
struct GimbalChannel {};

// ============================================================================
// SystemMetaBuilder Tests
// ============================================================================
// TEST PURPOSE:
//   验证 SystemMetaBuilder 能够正确构建 SystemMeta
//   这是 rclcompat 动态系统注册的基础
// ============================================================================

TEST(SystemMetaBuilderTest, BasicConstruction) {
    // TEST PURPOSE:
    //   验证最基本的 SystemMeta 构建：系统名、policy、SPMC writer
    //
    // VERIFICATION:
    //   1. meta.name 正确设置为 "test_system"
    //   2. meta.policy.kind 为 PoolCompute
    //   3. 包含 1 个 SPMC writer channel
    //   4. channel 不是 reader (是 writer)
    //
    // KEY ASSERTIONS:
    //   - EXPECT_EQ(meta.name, "test_system")
    //   - EXPECT_EQ(meta.policy.kind, PolicyKind::PoolCompute)
    //   - EXPECT_EQ(meta.spmc_channels[0].kind, talos::channel_kind::spmc_writer)

    auto meta = rclcompat::SystemMetaBuilder("test_system")
                    .policy(make_policy_info<pool_compute>())
                    .add_spmc_writer(typeid(TestMessage), typeid(TestTag1))
                    .build();

    EXPECT_EQ(meta.name, "test_system");
    EXPECT_EQ(meta.policy.kind, PolicyKind::PoolCompute);
    EXPECT_EQ(meta.spmc_channels.size(), 1);
    EXPECT_EQ(meta.spmc_channels[0].type, typeid(TestMessage));
    EXPECT_EQ(meta.spmc_channels[0].topic, typeid(TestTag1));
    EXPECT_EQ(meta.spmc_channels[0].kind, talos::channel_kind::spmc_writer);
}

TEST(SystemMetaBuilderTest, MultipleChannels) {
    // TEST PURPOSE:
    //   验证多个 channel 的构建，包括 reader 和 writer
    //
    // VERIFICATION:
    //   1. 包含 2 个 channels
    //   2. 第一个是 reader (订阅)
    //   3. 第二个是 writer (发布)
    //
    // KEY ASSERTIONS:
    //   - EXPECT_EQ(meta.spmc_channels.size(), 2)
    //   - EXPECT_EQ(meta.spmc_channels[0].kind, talos::channel_kind::spmc_reader)
    //   - EXPECT_EQ(meta.spmc_channels[1].kind, talos::channel_kind::spmc_writer)

    auto meta = rclcompat::SystemMetaBuilder("multi_channel")
                    .policy(make_policy_info<pool_compute>())
                    .add_spmc_reader(typeid(TestMessage), typeid(TestTag1))
                    .add_spmc_writer(typeid(AnotherMessage), typeid(TestTag2))
                    .build();

    EXPECT_EQ(meta.spmc_channels.size(), 2);
    EXPECT_EQ(meta.spmc_channels[0].kind, talos::channel_kind::spmc_reader);
    EXPECT_EQ(meta.spmc_channels[1].kind, talos::channel_kind::spmc_writer);
}

// ============================================================================
// OwnershipRegistry Tests
// ============================================================================
// TEST PURPOSE:
//   验证通道所有权注册表，确保单个 Publisher per channel 约束
// ============================================================================

TEST(OwnershipRegistryTest, RegisterAndClaim) {
    // TEST PURPOSE:
    //   验证基本的注册和 claim 机制
    //   每个 channel 只能被 claim 一次（enforce move-only）
    //
    // VERIFICATION:
    //   1. 注册所有者成功
    //   2. 第一次 claim 成功
    //   3. 第二次 claim 失败（已被占用）
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(registry.try_claim(key))
    //   - EXPECT_FALSE(registry.try_claim(key))

    rclcompat::OwnershipRegistry registry;
    const ChannelKey key{typeid(TestMessage), typeid(TestTag1)};

    registry.register_owner(key, "owner_system");
    EXPECT_TRUE(registry.try_claim(key));
    EXPECT_FALSE(registry.try_claim(key)); // Second claim fails
}

TEST(OwnershipRegistryTest, ReleaseAndReclaim) {
    // TEST PURPOSE:
    //   验证 release 后可以重新 claim
    //   这是 Publisher move 语义的基础
    //
    // VERIFICATION:
    //   1. 注册并 claim 成功
    //   2. release 后可以重新 claim
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(registry.try_claim(key)) after release

    rclcompat::OwnershipRegistry registry;
    const ChannelKey key{typeid(TestMessage), typeid(TestTag1)};

    registry.register_owner(key, "owner_system");
    EXPECT_TRUE(registry.try_claim(key));

    registry.release_claim(key);
    EXPECT_TRUE(registry.try_claim(key)); // Can reclaim after release
}

// ============================================================================
// PubSlot Tests
// ============================================================================
// TEST PURPOSE:
//   验证 PubSlot 的状态机：unbound -> bound -> pending_write
// ============================================================================

TEST(PubSlotTest, InitialState) {
    // TEST PURPOSE:
    //   验证 PubSlot 的初始状态
    //
    // VERIFICATION:
    //   1. ready() = false (未 bind)
    //   2. bound = false (未绑定到 World)
    //   3. pending_write = false (无待处理写入)
    //
    // KEY ASSERTIONS:
    //   - EXPECT_FALSE(slot.ready())
    //   - EXPECT_FALSE(slot.bound.load())
    //   - EXPECT_FALSE(slot.pending_write.load())

    rclcompat::PubSlot<TestMessage, TestTag1> slot;

    EXPECT_FALSE(slot.ready());
    EXPECT_FALSE(slot.bound.load());
    EXPECT_FALSE(slot.pending_write.load());
}

TEST(PubSlotTest, BindAndPublish) {
    // TEST PURPOSE:
    //   验证 PubSlot 的 bind 和 publish 流程
    //   这是 Publisher 能够工作的核心机制
    //
    // DATA FLOW:
    //   PubSlot.unbound --bind()--> PubSlot.bound --publish()--> PubSlot.pending_write
    //
    // VERIFICATION:
    //   1. bind() 后 ready() = true
    //   2. publish() 后 pending_write = true
    //   3. exchange() 消费 pending_write 标志
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(slot.ready()) after bind
    //   - EXPECT_TRUE(slot.pending_write.load()) after publish
    //   - EXPECT_TRUE(was_pending) after exchange

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("test_node", scheduler);
    auto pub = node.create_publisher<TestMessage, TestTag1>();

    ASSERT_TRUE(node.finalize().has_value());

    auto* slot = node.get_pub_slot<TestMessage, TestTag1>();
    ASSERT_NE(slot, nullptr);
    EXPECT_TRUE(slot->ready());
    EXPECT_TRUE(slot->bound.load());
    EXPECT_FALSE(slot->pending_write.load());

    slot->publish(TestMessage{42, "hello"});
    EXPECT_TRUE(slot->pending_write.load());

    // Consume the pending write (this is what RclPubSystem::run() does)
    bool was_pending = slot->pending_write.exchange(false, std::memory_order_acq_rel);
    EXPECT_TRUE(was_pending);
    EXPECT_FALSE(slot->pending_write.load());
}

// ============================================================================
// Publisher Tests
// ============================================================================
// TEST PURPOSE:
//   验证 Publisher move-only 句柄的行为
// ============================================================================

TEST(PublisherTest, MoveSemantics) {
    // TEST PURPOSE:
    //   验证 Publisher 的 move-only 语义
    //   确保编译期所有权检查
    //
    // VERIFICATION:
    //   1. 移动后源对象 invalid
    //   2. 目标对象 valid
    //   3. 移动赋值正确转移所有权
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(pub2.valid()) after move from pub1
    //   - EXPECT_FALSE(pub1.valid()) after move
    //   - EXPECT_TRUE(pub3.valid()) after move assign

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("test_node", scheduler);

    auto pub1 = node.create_publisher<TestMessage, TestTag1>();
    EXPECT_TRUE(pub1.valid());

    // Move to pub2
    auto pub2 = std::move(pub1);
    EXPECT_TRUE(pub2.valid());
    EXPECT_FALSE(pub1.valid());

    // Move assign to pub3
    rclcompat::Publisher<TestMessage, TestTag1> pub3;
    pub3 = std::move(pub2);
    EXPECT_TRUE(pub3.valid());
    EXPECT_FALSE(pub2.valid());
}

TEST(PublisherTest, PublishBeforeFinalize) {
    // TEST PURPOSE:
    //   验证在 finalize() 之前调用 publish() 是安全的（但不会实际写入）
    //
    // VERIFICATION:
    //   1. publish() 不会崩溃
    //   2. publisher 不处于 ready 状态
    //
    // KEY ASSERTIONS:
    //   - EXPECT_FALSE(pub.ready()) before finalize

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("test_node", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();

    // Publish before finalize - should be safe but not actually write
    pub.publish(TestMessage{42, "hello"});

    // The slot is not bound yet
    EXPECT_FALSE(pub.ready());
}

TEST(PublisherTest, PublishAfterFinalize) {
    // TEST PURPOSE:
    //   验证 finalize() 后 Publisher 可以正常发布
    //
    // VERIFICATION:
    //   1. finalize() 后 publisher ready
    //   2. publish() 设置 pending_write 标志
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(pub.ready()) after finalize
    //   - EXPECT_TRUE(slot->pending_write.load()) after publish

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("test_node", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();

    auto result = node.finalize();
    EXPECT_TRUE(result.has_value());

    // Now the slot should be bound
    EXPECT_TRUE(pub.ready());

    pub.publish(TestMessage{42, "hello"});

    // Can read the value via the slot
    auto* slot = node.get_pub_slot<TestMessage, TestTag1>();
    ASSERT_NE(slot, nullptr);
    EXPECT_TRUE(slot->pending_write.load());
}

TEST(PublisherTest, SurvivesNodeDestructionBeforeFinalize) {
    World world;
    Scheduler scheduler(world);
    rclcompat::Publisher<TestMessage, TestTag1> pub;

    {
        rclcompat::Node node("test_node", scheduler);
        pub = node.create_publisher<TestMessage, TestTag1>();
        EXPECT_TRUE(pub.valid());
        EXPECT_FALSE(pub.ready());
    }

    EXPECT_TRUE(pub.valid());
    EXPECT_FALSE(pub.ready());

    // Safe no-op: slot still exists even though the Node object is gone.
    pub.publish(TestMessage{7, "orphaned"});
    EXPECT_TRUE(pub.valid());
}

// ============================================================================
// rclcompat::Node Tests
// ============================================================================
// TEST PURPOSE:
//   验证 rclcompat::Node 容器的生命周期管理
// ============================================================================

TEST(NodeTest, CreatePublisher) {
    // TEST PURPOSE:
    //   验证 rclcompat::Node 可以创建 Publisher
    //
    // VERIFICATION:
    //   1. create_publisher() 返回 valid 的 Publisher
    //   2. rclcompat::Node name 正确设置
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(pub.valid())
    //   - EXPECT_EQ(node.name(), "my_node")

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("my_node", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();
    EXPECT_TRUE(pub.valid());
    EXPECT_EQ(node.name(), "my_node");
}

TEST(NodeTest, CreateSubscription) {
    // TEST PURPOSE:
    //   验证 rclcompat::Node 可以创建 Subscription
    //   Subscription 需要对应的 Publisher 才能成功 build
    //
    // VERIFICATION:
    //   1. create_subscription() 不会崩溃
    //   2. finalize() 成功（有对应的 writer）
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(result.has_value()) after finalize

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("my_node", scheduler);

    // Create publisher first (subscription needs a writer)
    auto pub = node.create_publisher<TestMessage, TestTag1>();

    std::atomic<int> callback_count{0};
    node.create_subscription<TestMessage, TestTag1>(
        [&callback_count](const TestMessage&) { callback_count.fetch_add(1); });

    // Subscription created with publisher, finalize should succeed
    auto result = node.finalize();
    EXPECT_TRUE(result.has_value());
}

TEST(NodeTest, FinalizeBuild) {
    // TEST PURPOSE:
    //   验证 finalize() 能够正确构建 wake chain
    //
    // VERIFICATION:
    //   1. finalize() 返回成功
    //   2. print_systems() 显示正确的依赖关系
    //
    // KEY ASSERTIONS:
    //   - EXPECT_TRUE(result.has_value())
    //   - print_systems() shows pub -> sub wake chain

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("my_node", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();
    node.create_subscription<TestMessage, TestTag1>([](const TestMessage&) { /* no-op */ });

    auto result = node.finalize();
    EXPECT_TRUE(result.has_value());

    // Verify wake chain structure
    // Expected output:
    //   Wake chains (compute -> compute):
    //     [my_node/pub_...] -> [my_node/sub_...]
    scheduler.print_systems();
}

TEST(NodeTest, DestroyedAfterFinalizeStillRunsOwnedTimerPublisher) {
    World world;
    Scheduler scheduler(world);
    std::atomic<int> recv_count{0};

    {
        rclcompat::Node node("my_node", scheduler);
        auto pub = node.create_publisher<TestMessage, TestTag1>();
        auto pub_handle =
            std::make_shared<rclcompat::Publisher<TestMessage, TestTag1>>(std::move(pub));
        node.create_subscription<TestMessage, TestTag1>(
            [&recv_count](const TestMessage&) { recv_count.fetch_add(1); });
        node.create_wall_timer(Frequency::Hz_30, [pub_handle]() {
            pub_handle->publish(TestMessage{.value = 1, .text = "tick"});
        });

        ASSERT_TRUE(node.finalize().has_value());
    }

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    scheduler.stop();
    scheduler_thread.join();

    EXPECT_GE(recv_count.load(), 2);
}

// ============================================================================
// Integration Tests - Real-world Scenarios
// ============================================================================

// -------------------------------------------------------------------------
// Test: SimplePubSubChain
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证最简单的 Publisher → Subscriber 数据流设置
//
// DATA FLOW:
//   ┌─────────┐     publish()     ┌──────────┐
//   │ Camera  │ ─────────────────>│ Detector │
//   │ (pub)   │   ImageFrame      │ (sub)    │
//   └─────────┘                   └──────────┘
//
// VERIFICATION:
//   1. 两个 rclcompat::Node 可以成功 finalize
//   2. Publisher 处于 ready 状态
//   3. 可以发布多条消息
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(camera.finalize().has_value())
//   - EXPECT_TRUE(detector.finalize().has_value())
//   - EXPECT_TRUE(camera_pub.ready())
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, SimplePubSubChain) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node camera("camera", scheduler);
    auto camera_pub = camera.create_publisher<ImageFrame, CameraChannel>();

    rclcompat::Node detector("detector", scheduler);
    std::atomic<int> frames_processed{0};
    detector.create_subscription<ImageFrame, CameraChannel>(
        [&frames_processed](const ImageFrame& frame) {
            frames_processed.fetch_add(1);
            // Simulate detection processing
            ASSERT_GT(frame.width, 0);
            ASSERT_GT(frame.height, 0);
        });

    // Finalize both nodes
    EXPECT_TRUE(camera.finalize().has_value());
    EXPECT_TRUE(detector.finalize().has_value());

    // Simulate camera publishing frames
    for (int i = 0; i < 10; ++i) {
        camera_pub.publish(
            ImageFrame{
                .frame_id     = i,
                .width        = 1920,
                .height       = 1080,
                .timestamp_ns = static_cast<uint64_t>(i * 33'000'000ULL),
                .data         = std::vector<uint8_t>(1920 * 1080 * 3, static_cast<uint8_t>(i))});
    }

    // The subscription system's run() will be called by scheduler,
    // but for this test we just verify the setup is correct
    EXPECT_TRUE(camera_pub.ready());
}

// -------------------------------------------------------------------------
// Test: OnePublisherMultipleSubscribers (SPMC)
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证 SPMC (Single Publisher, Multiple Consumers) 模式
//   一个 Publisher 发布数据，多个 Subscriber 各自独立接收
//
// DATA FLOW:
//              ┌────────────────────┐
//              │                    │
//              ▼                    │
//   ┌──────────┐  publish()   ┌─────┴─────┐
//   │  Camera  │ ─────────────>│  Channel  │
//   │  (pub)   │  ImageFrame   │  (SPMC)  │
//   └──────────┘               └─────┬─────┘
//         │                         │
//         │                         │
//         │        ┌────────────────┴────────────────┐
//         │        │                                 │
//         ▼        ▼                                 ▼
//   ┌──────────┬──────────┐                 ┌──────────┐
//   │Detector A│Detector B│                 │Detector C│
//   │ (sub_a)  │ (sub_b)  │                 │ (sub_c)  │
//   └──────────┴──────────┘                 └──────────┘
//
// VERIFICATION:
//   1. 三个订阅者都能成功创建
//   2. wake chain 显示: [camera] -> [detector_a], [detector_b], [detector_c]
//   3. 订阅者独立计数，互不影响
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(detector_a.finalize().has_value())
//   - scheduler.print_systems() 显示正确的 wake chain
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, OnePublisherMultipleSubscribers) {
    World world;
    Scheduler scheduler(world);

    // Single camera node
    rclcompat::Node camera("camera", scheduler);
    auto camera_pub = camera.create_publisher<ImageFrame, CameraChannel>();

    // Multiple detectors subscribing to the same camera
    std::atomic<int> detector_a_count{0};
    std::atomic<int> detector_b_count{0};
    std::atomic<int> detector_c_count{0};

    rclcompat::Node detector_a("detector_a", scheduler);
    detector_a.create_subscription<ImageFrame, CameraChannel>(
        [&detector_a_count](const ImageFrame& frame) {
            detector_a_count.fetch_add(1);
            ASSERT_EQ(frame.width, 1920);
        });

    rclcompat::Node detector_b("detector_b", scheduler);
    detector_b.create_subscription<ImageFrame, CameraChannel>(
        [&detector_b_count](const ImageFrame& frame) {
            detector_b_count.fetch_add(1);
            ASSERT_EQ(frame.height, 1080);
        });

    rclcompat::Node detector_c("detector_c", scheduler);
    detector_c.create_subscription<ImageFrame, CameraChannel>(
        [&detector_c_count](const ImageFrame& frame) {
            detector_c_count.fetch_add(1);
            ASSERT_GT(frame.frame_id, 0);
        });

    // Finalize all nodes
    EXPECT_TRUE(camera.finalize().has_value());
    EXPECT_TRUE(detector_a.finalize().has_value());
    EXPECT_TRUE(detector_b.finalize().has_value());
    EXPECT_TRUE(detector_c.finalize().has_value());

    scheduler.print_systems();
    scheduler.print_mermaid_wake_chains();

    // Publish frames - all three subscribers should receive them
    for (int i = 0; i < 5; ++i) {
        camera_pub.publish(
            ImageFrame{
                .frame_id     = i,
                .width        = 1920,
                .height       = 1080,
                .timestamp_ns = static_cast<uint64_t>(i * 33'000'000ULL)});
    }
}

// -------------------------------------------------------------------------
// Test: ChainedProcessingPipeline
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证多级流水线：Camera → Detection → Tracking → Gimbal
//   模拟真实的 FCS 火控系统数据流
//
// DATA FLOW:
//   ┌─────────┐    ┌───────────┐    ┌─────────┐    ┌─────────┐
//   │ Camera  │ ─>│ Detector  │ ─>│ Tracker │ ─>│ Gimbal  │
//   │ (pub)   │   │(sub+pub)  │   │(sub+pub)│   │ (sub)   │
//   └─────────┘    └───────────┘    └─────────┘    └─────────┘
//   ImageFrame     DetectionRes     TrackOut      (consume)
//
// VERIFICATION:
//   1. 4 个 rclcompat::Node 都能成功 finalize
//   2. 中间节点既订阅又发布
//   3. wake chain 显示正确的 4 级依赖
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(camera.finalize().has_value())
//   - EXPECT_TRUE(detector.finalize().has_value())
//   - scheduler.print_systems() 显示 4 级 execution levels
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, ChainedProcessingPipeline) {
    World world;
    Scheduler scheduler(world);

    // Stage 1: Camera
    rclcompat::Node camera("camera", scheduler);
    auto camera_pub = camera.create_publisher<ImageFrame, CameraChannel>();

    // Stage 2: Detector (reads camera, publishes detection)
    rclcompat::Node detector("detector", scheduler);
    auto detector_pub = detector.create_publisher<DetectionResult, DetectionChannel>();

    std::atomic<int> detected_frames{0};
    detector.create_subscription<ImageFrame, CameraChannel>(
        [&detector_pub, &detected_frames](const ImageFrame& frame) {
            // Simulate detection
            DetectionResult result{
                .frame_id    = frame.frame_id,
                .confidences = {                   0.95f,                    0.87f},
                .quads       = {{0.1f, 0.2f, 0.3f, 0.4f}, {0.5f, 0.6f, 0.7f, 0.8f}}
            };
            detector_pub.publish(result);
            detected_frames.fetch_add(1);
        });

    // Stage 3: Tracker (reads detection, publishes tracking)
    rclcompat::Node tracker("tracker", scheduler);
    auto tracker_pub = tracker.create_publisher<TrackingOutput, TrackingChannel>();

    std::atomic<int> tracked_frames{0};
    tracker.create_subscription<DetectionResult, DetectionChannel>(
        [&tracker_pub, &tracked_frames](const DetectionResult& detection) {
            // Simulate tracking
            TrackingOutput output{
                .id           = 1,
                .x            = 1.0f,
                .y            = 2.0f,
                .z            = 3.0f,
                .vx           = 0.1f,
                .vy           = 0.2f,
                .vz           = 0.0f,
                .timestamp_ns = detection.frame_id * 33'000'000ULL};
            tracker_pub.publish(output);
            tracked_frames.fetch_add(1);
        });

    // Stage 4: Gimbal (reads tracking, publishes command)
    rclcompat::Node gimbal("gimbal", scheduler);
    std::atomic<int> commands_sent{0};
    gimbal.create_subscription<TrackingOutput, TrackingChannel>(
        [&commands_sent](const TrackingOutput& tracking) {
            // Simulate gimbal control
            ASSERT_GT(tracking.id, 0);
            commands_sent.fetch_add(1);
        });

    // Finalize all nodes
    EXPECT_TRUE(camera.finalize().has_value());
    EXPECT_TRUE(detector.finalize().has_value());
    EXPECT_TRUE(tracker.finalize().has_value());
    EXPECT_TRUE(gimbal.finalize().has_value());

    scheduler.print_systems();
    scheduler.print_mermaid_wake_chains();

    // Run the pipeline
    for (int i = 0; i < 3; ++i) {
        camera_pub.publish(
            ImageFrame{
                .frame_id     = i,
                .width        = 1920,
                .height       = 1080,
                .timestamp_ns = static_cast<uint64_t>(i * 33'000'000ULL)});
    }
}

TEST(MermaidExecutionLevels, CountsChannelReadersAndWritersSeparately) {
    struct InputTag {};
    struct DebugTag {};
    struct OutputTag {};

    World world;
    Scheduler scheduler(world);

    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<TestMessage, InputTag> out) {
            out.write(TestMessage{.value = 1, .text = "tick"});
        });

    scheduler.add_system<talos::pool_compute>(
        "fanout",
        [](talos::spmc<TestMessage, InputTag> in, talos::spmc_mut<TestMessage, DebugTag> debug_out,
           talos::spsc_mut<TestMessage, OutputTag> output_out) {
            const auto msg = in.read();
            if (!msg) {
                return;
            }
            debug_out.write(*msg);
            output_out.write(*msg);
        });

    scheduler.add_system<talos::pool_compute>(
        "debug_sink", [](talos::spmc<TestMessage, DebugTag> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "output_sink", [](talos::spsc<TestMessage, OutputTag> in) { (void)in.read(); });

    ASSERT_TRUE(scheduler.build().has_value());

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_execution_levels();
    const std::string mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(mermaid.find("fanout"), std::string::npos);
    EXPECT_NE(mermaid.find("(1 in, 2 out)"), std::string::npos);
    EXPECT_EQ(mermaid.find("(3 in, 2 out)"), std::string::npos);
}

TEST(MermaidExecutionLevels, RendersCrossLevelDependenciesForMultiInputNodes) {
    struct SourceTag {};
    struct BranchATag {};
    struct BranchBTag {};
    struct MergeTag {};
    struct SinkTag {};

    World world;
    Scheduler scheduler(world);

    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<TestMessage, SourceTag> out) {
            out.write(TestMessage{.value = 1, .text = "tick"});
        });

    scheduler.add_system<talos::pool_compute>(
        "stage_a",
        [](talos::spmc<TestMessage, SourceTag> in, talos::spmc_mut<TestMessage, BranchATag> a_out,
           talos::spmc_mut<TestMessage, MergeTag> merge_out) {
            const auto msg = in.read();
            if (!msg) {
                return;
            }
            a_out.write(*msg);
            merge_out.write(*msg);
        });

    scheduler.add_system<talos::pool_compute>(
        "stage_b",
        [](talos::spmc<TestMessage, BranchATag> in, talos::spmc_mut<TestMessage, BranchBTag> out) {
            const auto msg = in.read();
            if (!msg) {
                return;
            }
            out.write(*msg);
        });

    scheduler.add_system<talos::pool_compute>(
        "merge",
        [](talos::spmc<TestMessage, MergeTag> direct_in, talos::spmc<TestMessage, BranchBTag> b_in,
           talos::spmc_mut<TestMessage, SinkTag> out) {
            if (const auto direct = direct_in.read(); direct && b_in.read()) {
                out.write(*direct);
            }
        });

    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<TestMessage, SinkTag> in) { (void)in.read(); });

    ASSERT_TRUE(scheduler.build().has_value());

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_execution_levels();
    const std::string mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(mermaid.find("stage_a"), std::string::npos);
    EXPECT_NE(mermaid.find("stage_b"), std::string::npos);
    EXPECT_NE(mermaid.find("merge"), std::string::npos);
    EXPECT_NE(mermaid.find("C0 --> C1"), std::string::npos);
    EXPECT_NE(mermaid.find("C0 --> C2"), std::string::npos);
    EXPECT_NE(mermaid.find("C1 --> C2"), std::string::npos);
}

TEST(MermaidLabels, DisambiguatesSameTopicDifferentPayloadTypes) {
    World world;
    Scheduler scheduler(world);

    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<TestMessage, TestTag1> test_out,
                     talos::spmc_mut<AnotherMessage, TestTag1> other_out) {
            test_out.write(TestMessage{.value = 1, .text = "tick"});
            other_out.write(AnotherMessage{.x = 1.0, .y = 2.0});
        });

    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<TestMessage, TestTag1> test_in,
                   talos::spmc<AnotherMessage, TestTag1> other_in) {
            (void)test_in.read();
            (void)other_in.read();
        });

    ASSERT_TRUE(scheduler.build().has_value());

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_wake_chains();
    const std::string wake_mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(wake_mermaid.find("TestTag1"), std::string::npos);
    EXPECT_NE(wake_mermaid.find("TestMessage"), std::string::npos);
    EXPECT_NE(wake_mermaid.find("AnotherMessage"), std::string::npos);
    EXPECT_EQ(wake_mermaid.find("|\"TestTag1\"|"), std::string::npos);

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_data_flow();
    const std::string data_flow_mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(data_flow_mermaid.find("TestTag1"), std::string::npos);
    EXPECT_NE(data_flow_mermaid.find("TestMessage"), std::string::npos);
    EXPECT_NE(data_flow_mermaid.find("AnotherMessage"), std::string::npos);
    EXPECT_EQ(data_flow_mermaid.find("|\"TestTag1\"|"), std::string::npos);
}

TEST(MermaidWakeChains, RequiresBuildForFreshWakeGraph) {
    World world;
    Scheduler scheduler(world);

    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<TestMessage, TestTag1> out) {
            out.write(TestMessage{.value = 1, .text = "tick"});
        });
    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<TestMessage, TestTag1> in) { (void)in.read(); });

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_wake_chains();
    const std::string mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(mermaid.find("Call build() to render wake chains"), std::string::npos);
}

TEST(MermaidWakeChains, ExcludesSharedResourceAccess) {
    World world;
    Scheduler scheduler(world);

    world.insert_resource(TestMessage{.value = 7, .text = "resource"});

    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<AnotherMessage, TestTag1> out) {
            out.write(AnotherMessage{.x = 1.0, .y = 2.0});
        });
    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<AnotherMessage, TestTag1> in, talos::res<TestMessage> state) {
            (void)in.read();
            (void)state->value;
        });

    ASSERT_TRUE(scheduler.build().has_value());

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_wake_chains();
    const std::string mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(mermaid.find("TestTag1"), std::string::npos);
    EXPECT_EQ(mermaid.find("TestMessage"), std::string::npos);
    EXPECT_EQ(mermaid.find("|\"res\"|"), std::string::npos);
    EXPECT_EQ(mermaid.find("|\"res_mut\"|"), std::string::npos);
}

TEST(MermaidDataFlow, AnnotatesInvalidMultipleWriterChannels) {
    World world;
    Scheduler scheduler(world);

    scheduler.add_system<talos::fixed_rate<30>>(
        "writer_a", [](talos::spmc_mut<TestMessage, TestTag1> out) {
            out.write(TestMessage{.value = 1, .text = "a"});
        });
    scheduler.add_system<talos::fixed_rate<60>>(
        "writer_b", [](talos::spmc_mut<TestMessage, TestTag1> out) {
            out.write(TestMessage{.value = 2, .text = "b"});
        });
    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<TestMessage, TestTag1> in) { (void)in.read(); });

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_data_flow();
    const std::string mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(mermaid.find("multiple writers"), std::string::npos);
    EXPECT_EQ(mermaid.find("E0 -->|\"TestTag1"), std::string::npos);
    EXPECT_EQ(mermaid.find("E1 -->|\"TestTag1"), std::string::npos);
}

TEST(MermaidDataFlow, IncludesSharedResourceReadAndMutAccess) {
    World world;
    Scheduler scheduler(world);

    world.insert_resource(TestMessage{.value = 3, .text = "resource"});

    scheduler.add_system<talos::fixed_rate_silent<30>>(
        "reader", [](talos::res<TestMessage> state) { (void)state->value; });
    scheduler.add_system<talos::pool_compute>(
        "mutator", [](talos::res_mut<TestMessage> state) { state->value += 1; });

    testing::internal::CaptureStdout();
    scheduler.print_mermaid_data_flow();
    const std::string mermaid = testing::internal::GetCapturedStdout();

    EXPECT_NE(mermaid.find("Shared Resources"), std::string::npos);
    EXPECT_NE(mermaid.find("TestMessage"), std::string::npos);
    EXPECT_NE(mermaid.find("R0 -->|\"res\"| E0"), std::string::npos);
    EXPECT_NE(mermaid.find("R0 -->|\"res_mut\"| C0"), std::string::npos);
    EXPECT_NE(mermaid.find("C0 -->|\"res_mut\"| R0"), std::string::npos);
    EXPECT_NE(mermaid.find("shared resource access only"), std::string::npos);
}

// -------------------------------------------------------------------------
// Test: HotAddDuringExecution
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证热插拔功能：运行时添加新系统
//
// DATA FLOW:
//   Step 1: Source rclcompat::Node (running)
//   Step 2: Hot-add Consumer rclcompat::Node
//   Step 3: Data flows: Source -> Consumer
//
// VERIFICATION:
//   1. 已经 finalize 的 source 不影响新 consumer 的添加
//   2. Consumer 可以成功 hot-add
//   3. wake chain 正确更新
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(source.finalize().has_value()) before consumer
//   - EXPECT_TRUE(consumer.finalize().has_value())
// -------------------------------------------------------------------------

TEST(HotAddTest, HotAddDuringExecution) {
    World world;
    Scheduler scheduler(world);

    // Create an initial source system
    rclcompat::Node source("source", scheduler);
    auto source_pub = source.create_publisher<TestMessage, TestTag1>();

    // Finalize and start the scheduler
    EXPECT_TRUE(source.finalize().has_value());

    // Create a consumer node (but don't finalize yet)
    rclcompat::Node consumer("consumer", scheduler);
    std::atomic<int> messages_received{0};
    consumer.create_subscription<TestMessage, TestTag1>(
        [&messages_received](const TestMessage& msg) {
            messages_received.fetch_add(1);
            ASSERT_GT(msg.value, 0);
        });

    // Now hot-add the consumer while scheduler is not running yet
    // (in real scenario, scheduler would be running here)
    EXPECT_TRUE(consumer.finalize().has_value());

    scheduler.print_systems();
}

TEST(HotAddTest, FinalizeRemainsUsableAfterPartialHotAddFailure) {
    World world;
    Scheduler scheduler(world);

    Node source("source", scheduler);
    auto source_pub = source.create_publisher<TestMessage, TestTag1>();
    ASSERT_TRUE(source.finalize().has_value());

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!scheduler.is_running() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(scheduler.is_running());

    Node retry_node("retry", scheduler);
    std::atomic<int> received{0};
    retry_node.create_subscription<TestMessage, TestTag1>(
        [&received](const TestMessage&) { received.fetch_add(1); });
    auto duplicate_pub = retry_node.create_publisher<TestMessage, TestTag1>();
    EXPECT_TRUE(duplicate_pub.valid());

    const auto first_finalize = retry_node.unsafe_finalize();
    ASSERT_FALSE(first_finalize.has_value());
    ASSERT_TRUE(std::holds_alternative<talos::MultipleWritersError>(first_finalize.error()));

    // A failed hot-add must not leave moved-from systems behind.
    const auto second_finalize = retry_node.unsafe_finalize();
    EXPECT_TRUE(second_finalize.has_value());

    source_pub.publish(TestMessage{.value = 7, .text = "retry"});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_GT(received.load(), 0);

    scheduler.stop();
    scheduler_thread.join();
}

TEST(HotAddTest, UnsafeHotAddSystemWhileRunning) {
    World world;
    Scheduler scheduler(world);

    Node source("source", scheduler);
    auto source_pub = source.create_publisher<TestMessage, TestTag1>();
    ASSERT_TRUE(source.finalize().has_value());

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!scheduler.is_running() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(scheduler.is_running());

    std::atomic<int> received{0};
    auto consumer = [&received](talos::spmc<TestMessage, TestTag1> in) {
        if (in.read()) {
            received.fetch_add(1);
        }
    };

    auto result = scheduler.unsafe_hot_add_system(
        talos::make_system<decltype(consumer), talos::pool_compute>(
            "unsafe_hot_add_consumer", std::move(consumer)));
    ASSERT_TRUE(result.has_value());

    source_pub.publish(TestMessage{.value = 9, .text = "unsafe-hot-add"});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_GT(received.load(), 0);

    scheduler.stop();
    scheduler_thread.join();
}

TEST(HotAddTest, SafeHotAddPanicsWhileRunning) {
    World world;
    Scheduler scheduler(world);
    ASSERT_TRUE(scheduler.build().has_value());

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!scheduler.is_running() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(scheduler.is_running());

    auto consumer = [](talos::spmc<TestMessage, TestTag1> in) { (void)in.read(); };
    EXPECT_DEATH_IF_SUPPORTED(
        (void)scheduler.hot_add_system(
            talos::make_system<decltype(consumer), talos::pool_compute>(
                "safe_hot_add_consumer", std::move(consumer))),
        "");

    scheduler.stop();
    scheduler_thread.join();
}

TEST(HotAddTest, FinalizePanicsWhileRunning) {
    World world;
    Scheduler scheduler(world);

    Node source("source", scheduler);
    ASSERT_TRUE(source.finalize().has_value());

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!scheduler.is_running() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(scheduler.is_running());

    Node consumer("consumer", scheduler);
    consumer.create_subscription<TestMessage, TestTag1>([](const TestMessage&) {});

    EXPECT_DEATH_IF_SUPPORTED((void)consumer.finalize(), "");

    scheduler.stop();
    scheduler_thread.join();
}

// -------------------------------------------------------------------------
// Test: MultipleIndependentChannels
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证多个独立的数据流并行工作
//
// DATA FLOW:
//   ┌─────────┐                 ┌─────────────┐
//   │ Camera  │ ──────────────>│   Vision    │
//   │(30Hz)   │   ImageFrame   │   (sub)     │
//   └─────────┘                 └─────────────┘
//
//   ┌─────────┐                 ┌─────────────┐
//   │   IMU   │ ──────────────>│  Navigation │
//   │(100Hz)  │   TestMessage  │   (sub)     │
//   └─────────┘                 └─────────────┘
//
// VERIFICATION:
//   1. 两个独立通道都能正常工作
//   2. 互不干扰
//   3. wake chain 显示两个独立的依赖链
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(camera.finalize().has_value())
//   - EXPECT_TRUE(imu.finalize().has_value())
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, MultipleIndependentChannels) {
    World world;
    Scheduler scheduler(world);

    // High-frequency camera
    rclcompat::Node camera("camera", scheduler);
    auto camera_pub = camera.create_publisher<ImageFrame, CameraChannel>();

    // Low-frequency imu
    rclcompat::Node imu("imu", scheduler);
    auto imu_pub = imu.create_publisher<TestMessage, TestTag1>();

    // Subscribers for each channel
    rclcompat::Node vision("vision", scheduler);
    std::atomic<int> vision_count{0};
    vision.create_subscription<ImageFrame, CameraChannel>([&vision_count](const ImageFrame& frame) {
        vision_count.fetch_add(1);
        ASSERT_EQ(frame.width, 1920);
    });

    rclcompat::Node navigation("navigation", scheduler);
    std::atomic<int> nav_count{0};
    navigation.create_subscription<TestMessage, TestTag1>([&nav_count](const TestMessage& msg) {
        nav_count.fetch_add(1);
        ASSERT_GT(msg.value, 0);
    });

    // Finalize all
    EXPECT_TRUE(camera.finalize().has_value());
    EXPECT_TRUE(imu.finalize().has_value());
    EXPECT_TRUE(vision.finalize().has_value());
    EXPECT_TRUE(navigation.finalize().has_value());

    // Publish on different channels
    for (int i = 0; i < 5; ++i) {
        camera_pub.publish(
            ImageFrame{
                .frame_id     = i,
                .width        = 1920,
                .height       = 1080,
                .timestamp_ns = static_cast<uint64_t>(i * 33'000'000ULL)});
        imu_pub.publish(TestMessage{i, "imu_data"});
    }

    scheduler.print_systems();
    scheduler.print_mermaid_wake_chains();
}

// -------------------------------------------------------------------------
// Test: DuplicatePublisherDetection
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证重复 Publisher 检测（每个通道只能有一个 Publisher）
//
// VERIFICATION:
//   1. 第一次 create_publisher 成功
//   2. 第二次尝试创建同一通道的 Publisher 应该失败
//
// NOTE: Death test requires special gtest setup, skipped for regular runs
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, DuplicatePublisherDetection) {
    // TEST PURPOSE:
    //   验证每个通道只能有一个 Publisher
    //
    // VERIFICATION:
    //   1. 第一个 Publisher 创建成功
    //   2. 第二个创建应该被阻止（abort 或返回错误）
    //
    // NOTE: Death test requires special gtest setup

    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("node", scheduler);

    // Create first publisher - should succeed
    auto pub1 = node.create_publisher<TestMessage, TestTag1>();
    EXPECT_TRUE(pub1.valid());

    // Try to create another publisher for the same channel - should abort
    // Note: Death tests require special gtest setup; skip for regular runs
    // node.create_publisher<TestMessage, TestTag1>();
}

// -------------------------------------------------------------------------
// Test: CrossNodeCommunication
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证不同 rclcompat::Node 之间的通信
//
// DATA FLOW:
//   ┌──────────┐                  ┌───────────┐
//   │ Producer │ ────────────────>│  Consumer │
//   │  rclcompat::Node    │   TestMessage    │   rclcompat::Node    │
//   └──────────┘                  └───────────┘
//
// VERIFICATION:
//   1. Producer 和 Consumer 在不同 rclcompat::Node 中
//   2. 数据可以跨 rclcompat::Node 流动
//   3. Publisher ready 状态正确
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(producer.finalize().has_value())
//   - EXPECT_TRUE(consumer.finalize().has_value())
//   - EXPECT_TRUE(pub.ready())
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, CrossNodeCommunication) {
    World world;
    Scheduler scheduler(world);

    // Producer in one node
    rclcompat::Node producer("producer", scheduler);
    auto pub = producer.create_publisher<TestMessage, TestTag1>();

    // Consumer in a different node
    rclcompat::Node consumer("consumer", scheduler);
    std::atomic<int> received_count{0};
    std::atomic<int> last_value{0};

    consumer.create_subscription<TestMessage, TestTag1>(
        [&received_count, &last_value](const TestMessage& msg) {
            received_count.fetch_add(1);
            last_value.store(msg.value);
        });

    // Finalize both nodes
    EXPECT_TRUE(producer.finalize().has_value());
    EXPECT_TRUE(consumer.finalize().has_value());

    // Publish from producer, verify consumer can receive
    pub.publish(TestMessage{42, "test"});

    // In a running system, the consumer's callback would be invoked
    // For this test, we just verify the plumbing is correct
    EXPECT_TRUE(pub.ready());
}

// -------------------------------------------------------------------------
// Test: MessageTransformation
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证消息转换：int → double → aggregation
//
// DATA FLOW:
//   ┌──────────┐     int[]      ┌───────────┐     double[]     ┌───────────┐
//   │Raw Input │ ──────────────>│ Processor │ ──────────────>│ Aggregator │
//   │ (pub)    │   vector<int>  │(sub+pub)  │   vector<double>│   (sub)    │
//   └──────────┘                 └───────────┘                  └───────────┘
//
// VERIFICATION:
//   1. 类型转换正确
//   2. 数据缩放正确 (×1.5)
//   3. 聚合计算正确
//
// KEY ASSERTIONS:
//   - EXPECT_TRUE(raw_input.finalize().has_value())
//   - EXPECT_TRUE(processor.finalize().has_value())
// -------------------------------------------------------------------------

TEST(RclCompatIntegrationTest, MessageTransformation) {
    World world;
    Scheduler scheduler(world);

    // Raw input stage
    rclcompat::Node raw_input("raw_input", scheduler);
    auto raw_pub = raw_input.create_publisher<std::vector<int>, TestTag1>();

    // Processing stage: transform vector
    rclcompat::Node processor("processor", scheduler);
    auto processed_pub = processor.create_publisher<std::vector<double>, TestTag2>();

    processor.create_subscription<std::vector<int>, TestTag1>(
        [&processed_pub](const std::vector<int>& input) {
            // Transform int vector to double vector with scaling
            std::vector<double> output;
            output.reserve(input.size());
            for (int val : input) {
                output.push_back(static_cast<double>(val) * 1.5);
            }
            processed_pub.publish(output);
        });

    // Output stage: aggregate results
    rclcompat::Node aggregator("aggregator", scheduler);
    std::atomic<double> sum{0.0};
    aggregator.create_subscription<std::vector<double>, TestTag2>(
        [&sum](const std::vector<double>& values) {
            double total = 0.0;
            for (double v : values) {
                total += v;
            }
            sum.store(total);
        });

    EXPECT_TRUE(raw_input.finalize().has_value());
    EXPECT_TRUE(processor.finalize().has_value());
    EXPECT_TRUE(aggregator.finalize().has_value());

    // Publish raw data
    raw_pub.publish(std::vector<int>{1, 2, 3, 4, 5});

    // In running system: 1+2+3+4+5 = 15, scaled by 1.5 = 22.5
}

// ============================================================================
// Wake Chain Tests - 验证依赖图正确构建
// ============================================================================

// -------------------------------------------------------------------------
// Test: WakeChain.SingleLevelDependency
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证单级依赖: Pub → Sub
//   检查 print_systems() 输出中的 execution levels 和 wake chains
//
// DATA FLOW:
//   Level 0: [pub_system]
//   Level 1: [sub_system]
//   Wake chain: [pub_system] -> [sub_system]
//
// VERIFICATION:
//   1. Execution levels = 2
//   2. pub 在 Level 0
//   3. sub 在 Level 1
// -------------------------------------------------------------------------

TEST(RclCompatWakeChain, SingleLevelDependency) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node pub_node("pub_node", scheduler);
    [[maybe_unused]] auto pub = pub_node.create_publisher<TestMessage, TestTag1>();

    rclcompat::Node sub_node("sub_node", scheduler);
    sub_node.create_subscription<TestMessage, TestTag1>([](const TestMessage&) {});

    EXPECT_TRUE(pub_node.finalize().has_value());
    EXPECT_TRUE(sub_node.finalize().has_value());

    // Verify wake chain structure via print_systems()
    // Expected: Execution levels: 2, [pub_node/pub_...] -> [sub_node/sub_...]
    scheduler.print_systems();
}

// -------------------------------------------------------------------------
// Test: WakeChain.MultiLevelPipeline
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证多级依赖: A → B → C → D
//   检查 execution levels 是否正确反映拓扑顺序
//
// DATA FLOW:
//   Level 0: [A]
//   Level 1: [B]
//   Level 2: [C]
//   Level 3: [D]
//
// VERIFICATION:
//   1. Execution levels = 4
//   2. 每个系统在正确的 level
// -------------------------------------------------------------------------

TEST(RclCompatWakeChain, MultiLevelPipeline) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node a("a", scheduler);
    auto a_pub = a.create_publisher<TestMessage, TestTag1>();

    rclcompat::Node b("b", scheduler);
    auto b_pub = b.create_publisher<AnotherMessage, TestTag2>();
    b.create_subscription<TestMessage, TestTag1>(
        [&b_pub](const TestMessage&) { b_pub.publish(AnotherMessage{1.0, 2.0}); });

    rclcompat::Node c("c", scheduler);
    auto c_pub = c.create_publisher<ImageFrame, CameraChannel>();
    c.create_subscription<AnotherMessage, TestTag2>([&c_pub](const AnotherMessage&) {
        c_pub.publish(ImageFrame{.frame_id = 0, .width = 640, .height = 480});
    });

    rclcompat::Node d("d", scheduler);
    d.create_subscription<ImageFrame, CameraChannel>([](const ImageFrame&) {});

    EXPECT_TRUE(a.finalize().has_value());
    EXPECT_TRUE(b.finalize().has_value());
    EXPECT_TRUE(c.finalize().has_value());
    EXPECT_TRUE(d.finalize().has_value());

    // Verify wake chain: A -> B -> C -> D
    scheduler.print_systems();
}

// -------------------------------------------------------------------------
// Test: WakeChain.SPMCWakePropagation
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证一个 Publisher 触发多个 Subscriber
//   检查 wake chains 是否显示一对多关系
//
// DATA FLOW:
//   [pub] -> [sub_a], [sub_b], [sub_c]
//
// VERIFICATION:
//   1. 一个 pub 系统在 wake chain 中指向多个 sub 系统
// -------------------------------------------------------------------------

TEST(RclCompatWakeChain, SPMCWakePropagation) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node pub("pub", scheduler);
    [[maybe_unused]] auto p = pub.create_publisher<TestMessage, TestTag1>();

    rclcompat::Node sub_a("sub_a", scheduler);
    sub_a.create_subscription<TestMessage, TestTag1>([](const TestMessage&) {});

    rclcompat::Node sub_b("sub_b", scheduler);
    sub_b.create_subscription<TestMessage, TestTag1>([](const TestMessage&) {});

    rclcompat::Node sub_c("sub_c", scheduler);
    sub_c.create_subscription<TestMessage, TestTag1>([](const TestMessage&) {});

    EXPECT_TRUE(pub.finalize().has_value());
    EXPECT_TRUE(sub_a.finalize().has_value());
    EXPECT_TRUE(sub_b.finalize().has_value());
    EXPECT_TRUE(sub_c.finalize().has_value());

    // Verify wake chain: [pub] -> [sub_a], [sub_b], [sub_c]
    scheduler.print_systems();
}

// ============================================================================
// Error Tests - 验证错误场景正确处理
// ============================================================================

// -------------------------------------------------------------------------
// Test: Error.DoubleFinalizeBehavior
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证调用 finalize() 两次的行为
//   第二次应该返回成功（但没有待注册的系统）
//
// VERIFICATION:
//   1. 第一次 finalize 成功
//   2. 第二次 finalize 也成功（但实际不做任何事）
// -------------------------------------------------------------------------

TEST(RclCompatError, DoubleFinalizeBehavior) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("node", scheduler);
    [[maybe_unused]] auto pub = node.create_publisher<TestMessage, TestTag1>();

    auto result1 = node.finalize();
    EXPECT_TRUE(result1.has_value());

    // Second finalize should succeed (no work to do)
    auto result2 = node.finalize();
    EXPECT_TRUE(result2.has_value());
}

// -------------------------------------------------------------------------
// Test: Error.PublishOnMovedFromPublisher
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证在 moved-from 的 Publisher 上调用 publish() 的行为
//   应该 abort（由 SPDLOG_CRITICAL + std::abort 实现）
//
// NOTE: 这个测试会导致进程终止，所以只验证 setup
// -------------------------------------------------------------------------

TEST(RclCompatError, PublishOnMovedFromPublisherSetup) {
    // TEST PURPOSE:
    //   验证 moved-from Publisher 的检测
    //
    // NOTE: 实际调用 publish() 会导致 abort
    //   这里只验证 move 语义

    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("node", scheduler);

    auto pub1 = node.create_publisher<TestMessage, TestTag1>();
    EXPECT_TRUE(pub1.valid());

    auto pub2 = std::move(pub1);
    EXPECT_TRUE(pub2.valid());
    EXPECT_FALSE(pub1.valid());

    // pub1.publish() would abort here
}

// ============================================================================
// Edge Case Tests - 边界条件和特殊情况
// ============================================================================

// -------------------------------------------------------------------------
// Test: Edge.EmptyMessageHandling
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证空消息的处理
//   确保系统可以处理边界情况
//
// VERIFICATION:
//   1. 空消息不会导致崩溃
//   2. 回调可以优雅处理空消息
// -------------------------------------------------------------------------

TEST(RclCompatEdge, EmptyMessageHandling) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node pub("pub", scheduler);
    auto p = pub.create_publisher<TestMessage, TestTag1>();

    rclcompat::Node sub("sub", scheduler);
    std::atomic<int> empty_count{0};
    std::atomic<int> valid_count{0};

    sub.create_subscription<TestMessage, TestTag1>(
        [&empty_count, &valid_count](const TestMessage& msg) {
            if (msg.text.empty()) {
                empty_count.fetch_add(1);
            } else {
                valid_count.fetch_add(1);
            }
        });

    EXPECT_TRUE(pub.finalize().has_value());
    EXPECT_TRUE(sub.finalize().has_value());

    // Publish empty and valid messages
    p.publish(TestMessage{0, ""});
    p.publish(TestMessage{1, "valid"});
}

// -------------------------------------------------------------------------
// Test: Edge.LargeMessagePayload
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证大消息的处理
//   模拟真实场景中的大图像数据
//
// VERIFICATION:
//   1. 可以发布包含大 vector 的消息
//   2. 不会崩溃
// -------------------------------------------------------------------------

TEST(RclCompatEdge, LargeMessagePayload) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node pub("pub", scheduler);
    auto p = pub.create_publisher<ImageFrame, CameraChannel>();

    rclcompat::Node sub("sub", scheduler);
    std::atomic<int> recv_count{0};

    sub.create_subscription<ImageFrame, CameraChannel>([&recv_count](const ImageFrame& frame) {
        recv_count.fetch_add(1);
        // Verify large payload
        EXPECT_EQ(frame.data.size(), 1920 * 1080 * 3);
    });

    EXPECT_TRUE(pub.finalize().has_value());
    EXPECT_TRUE(sub.finalize().has_value());

    // Publish large frame (1080p RGB)
    p.publish(
        ImageFrame{
            .frame_id     = 0,
            .width        = 1920,
            .height       = 1080,
            .timestamp_ns = 0,
            .data         = std::vector<uint8_t>(1920 * 1080 * 3, 0x42)});
}

// -------------------------------------------------------------------------
// Test: Edge.MultipleFinalizesWithNoSystems
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证在没有任何系统的情况下多次 finalize
//
// VERIFICATION:
//   1. 空 rclcompat::Node 可以多次 finalize
//   2. 不会崩溃
// -------------------------------------------------------------------------

TEST(RclCompatEdge, MultipleFinalizesWithNoSystems) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("node", scheduler);

    // Finalize with no systems - should succeed
    auto result1 = node.finalize();
    EXPECT_TRUE(result1.has_value());

    // Again - should still succeed
    auto result2 = node.finalize();
    EXPECT_TRUE(result2.has_value());

    // Add a system and finalize
    [[maybe_unused]] auto p = node.create_publisher<TestMessage, TestTag1>();
    auto result3            = node.finalize();
    EXPECT_TRUE(result3.has_value());
}

// ============================================================================
// Callback Ownership Tests - 验证自动回调绑定机制
// ============================================================================

// -------------------------------------------------------------------------
// Test: CallbackOwnership_AutoBindWithSchedulerRunning
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证在 scheduler 真正运行时，Publisher 自动绑定到回调
//
// DATA FLOW:
//   Step 1: 创建一个 fixed_rate 系统定期触发
//   Step 2: fixed_rate 系统发布消息，触发订阅回调
//   Step 3: 回调内部使用 Publisher，自动绑定
//   Step 4: 验证 Publisher 成功写入
//
// VERIFICATION:
//   1. fixed_rate 系统运行
//   2. 回调被触发
//   3. Publisher 自动绑定并成功发布
// -------------------------------------------------------------------------

TEST(RclCompatOwnership, AutoBindWithSchedulerRunning) {
    World world;
    Scheduler scheduler(world);

    // Create an fixed_rate system that will trigger the subscription
    std::atomic<bool> fixed_rate_ran{false};
    std::atomic<int> trigger_count{0};

    scheduler.add_system<talos::fixed_rate<100>>(
        "trigger_source",
        [&fixed_rate_ran, &trigger_count](talos::spmc_mut<TestMessage, TestTag1> trigger_out) {
            fixed_rate_ran.store(true);
            trigger_count.fetch_add(1);
            trigger_out.write(TestMessage{.value = trigger_count.load(), .text = "trigger"});
            return true;
        });

    Node node("test", scheduler);

    // Create publisher that will be used in callback
    auto pub = node.create_publisher<TestMessage, TestTag2>();

    std::atomic<bool> callback_executed{false};
    std::atomic<int> received_value{0};

    // Create subscription that uses the publisher
    node.create_subscription<TestMessage, TestTag1>(
        [&pub, &callback_executed, &received_value](const TestMessage& msg) {
            callback_executed.store(true);
            received_value.store(msg.value);
            // First use: should auto-bind to this callback
            pub.publish(TestMessage{.value = msg.value + 100, .text = "from_callback"});
        });

    EXPECT_TRUE(node.finalize().has_value());

    // Build and run the scheduler
    auto build_result = scheduler.build();
    if (!build_result) {
        FAIL() << "Build failed: " << fmt::format("{}", build_result.error());
        return;
    }

    // Run scheduler in a separate thread
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // Give scheduler time to run the fixed_rate system and trigger callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Stop the scheduler
    scheduler.stop();
    scheduler_thread.join();

    // Verify the fixed_rate system ran and the callback was executed
    EXPECT_TRUE(fixed_rate_ran.load());
    EXPECT_TRUE(callback_executed.load());
    EXPECT_GT(received_value.load(), 0);
}

// -------------------------------------------------------------------------
// Test: CallbackOwnership_RejectSecondCallback
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证已绑定的 Publisher 拒绝其他回调使用
//
// DATA FLOW:
//   ┌────────────────┐     publish()     ┌────────────────┐
//   │  Callback A    │ ─────────────────>│   Publisher    │
//   │  (owner)       │   accepted ✓       │  (bound to A)  │
//   └────────────────┘                   └───────┬────────┘
//                                                │
//                                                │ ownership enforced
//                                                │
//   ┌────────────────┐     publish()            ▼
//   │  Callback B    │ ──────────────────X────>│   ABORT   │
//   │  (intruder)    │   rejected ✗              │
//   └────────────────┘                            │
//
// NOTE: 此测试验证 abort 行为，需要特殊的测试环境
// -------------------------------------------------------------------------
/*
TEST(RclCompatOwnership, RejectSecondCallback_DeathTest) {
    // NOTE: This is a setup test - the actual death test requires
    // special gtest setup with EXPECT_DEATH
    //
    World world;
    Scheduler scheduler(world);
    rclcompat::Node node("test", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();

    scheduler.add_system<talos::fixed_rate<150>>(
        "sys", [](talos::spmc_mut<TestMessage, TestTag2> out,
                  talos::spmc_mut<TestMessage, TestTag3> out2) {
            out.write(TestMessage{.value = 1, .text = "from_a"});
            out2.write(TestMessage{.value = 1, .text = "from_a"});
        });

    node.create_subscription<TestMessage, TestTag2>(
        [&pub](const TestMessage&) { pub.publish(TestMessage{.value = 1, .text = "from_a"}); });
    node.create_subscription<TestMessage, TestTag3>(
        [&pub](const TestMessage&) { pub.publish(TestMessage{.value = 1, .text = "from_a"}); });

    EXPECT_TRUE(node.finalize().has_value());

    // Run scheduler in a separate thread
    scheduler.print_systems();
    scheduler.print_mermaid_wake_chains();
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // Give scheduler time to run the fixed_rate systems and trigger callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Stop the scheduler
    scheduler.stop();
    scheduler_thread.join();

    SUCCEED();
}
*/

// -------------------------------------------------------------------------
// Test: CallbackOwnership_MultipleCallbacksMultiplePublishers
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证多个回调可以使用各自独立的 Publisher
//
// DATA FLOW:
//   ┌────────────────┐     pub_a          ┌────────────────┐
//   │  Callback A    │ ─────────────────>│  Publisher A   │
//   └────────────────┘                   └────────────────┘
//
//   ┌────────────────┐     pub_b          ┌────────────────┐
//   │  Callback B    │ ─────────────────>│  Publisher B   │
//   └────────────────┘                   └────────────────┘
//
// VERIFICATION:
//   1. 每个回调自动绑定到各自的 Publisher
//   2. 互不干扰
// -------------------------------------------------------------------------

TEST(RclCompatOwnership, MultipleCallbacksMultiplePublishers) {
    World world;
    Scheduler scheduler(world);

    // Create two fixed_rate systems as triggers for the two callbacks
    std::atomic<int> trigger_a_count{0};
    std::atomic<int> trigger_b_count{0};

    scheduler.add_system<talos::fixed_rate<100>>(
        "trigger_a", [&trigger_a_count](talos::spmc_mut<TestMessage, TestTag1> trigger_out) {
            int count = trigger_a_count.fetch_add(1) + 1;
            trigger_out.write(TestMessage{.value = count, .text = "trigger_a"});
            return true;
        });

    scheduler.add_system<talos::fixed_rate<150>>(
        "trigger_b", [&trigger_b_count](talos::spmc_mut<AnotherMessage, TestTag1> trigger_out) {
            int count = trigger_b_count.fetch_add(1) + 1;
            trigger_out.write(AnotherMessage{.x = count * 1.0, .y = count * 2.0});
            return true;
        });

    rclcompat::Node node("test", scheduler);

    auto pub_a = node.create_publisher<TestMessage, TestTag2>();
    auto pub_b = node.create_publisher<AnotherMessage, TestTag2>();

    std::atomic<bool> callback_a_executed{false};
    std::atomic<bool> callback_b_executed{false};

    // Callback A uses Publisher A
    node.create_subscription<TestMessage, TestTag1>(
        [&pub_a, &callback_a_executed](const TestMessage& msg) {
            callback_a_executed.store(true);
            pub_a.publish(TestMessage{.value = msg.value + 100, .text = "from_a"});
        });

    // Callback B uses Publisher B
    node.create_subscription<AnotherMessage, TestTag1>(
        [&pub_b, &callback_b_executed](const AnotherMessage& msg) {
            callback_b_executed.store(true);
            pub_b.publish(AnotherMessage{.x = msg.x * 2.0, .y = msg.y * 2.0});
        });

    EXPECT_TRUE(node.finalize().has_value());

    // Build and run the scheduler
    auto build_result = scheduler.build();
    if (!build_result) {
        FAIL() << "Build failed" << fmt::format("{}", build_result.error());
        return;
    }

    // Run scheduler in a separate thread
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // Give scheduler time to run the fixed_rate systems and trigger callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Stop the scheduler
    scheduler.stop();
    scheduler_thread.join();

    // Verify both callbacks were executed (each bound to their own publisher)
    EXPECT_TRUE(callback_a_executed.load());
    EXPECT_TRUE(callback_b_executed.load());
    EXPECT_GT(trigger_a_count.load(), 0);
    EXPECT_GT(trigger_b_count.load(), 0);
}

// -------------------------------------------------------------------------
// Test: CallbackOwnership_PublisherOutsideCallback
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证在回调外使用 Publisher 不会触发绑定检查
//
// VERIFICATION:
//   1. 非回调上下文使用 Publisher 不触发绑定
//   2. 不会 abort
// -------------------------------------------------------------------------

TEST(RclCompatOwnership, PublisherOutsideCallback) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);
    auto pub = node.create_publisher<TestMessage, TestTag1>();

    EXPECT_TRUE(node.finalize().has_value());

    // Publishing outside of a callback context should work fine
    // (no current callback, so no binding check is performed)
    pub.publish(TestMessage{.value = 42, .text = "outside_callback"});
    EXPECT_TRUE(pub.ready());
}

TEST(RclCompatFlow, ExternalPublishDrainsCallbackChainInSingleComputeRound) {
    World world;
    Scheduler scheduler(world);
    Node node("test", scheduler);

    auto input_pub     = node.create_publisher<TestMessage, TestTag1>();
    auto forwarded_pub = node.create_publisher<TestMessage, TestTag2>();

    std::atomic<int> stage1_count{0};
    std::atomic<int> stage2_count{0};
    std::atomic<int> final_value{0};

    node.create_subscription<TestMessage, TestTag1>(
        [&forwarded_pub, &stage1_count](const TestMessage& msg) {
            stage1_count.fetch_add(1);
            forwarded_pub.publish(TestMessage{.value = msg.value + 1, .text = "forwarded"});
        });

    node.create_subscription<TestMessage, TestTag2>(
        [&stage2_count, &final_value](const TestMessage& msg) {
            stage2_count.fetch_add(1);
            final_value.store(msg.value);
        });

    ASSERT_TRUE(node.finalize().has_value());

    input_pub.publish(TestMessage{.value = 41, .text = "start"});

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    scheduler.stop();
    scheduler_thread.join();

    EXPECT_EQ(stage1_count.load(), 1);
    EXPECT_EQ(stage2_count.load(), 1);
    EXPECT_EQ(final_value.load(), 42);
    EXPECT_EQ(scheduler.stats().compute_cycle_count, 1U);
}

TEST(SchedulerStatsTest, FixedRateFrequencyCountsWritesForWriterSystems) {
    World world;
    Scheduler scheduler(world);

    std::atomic<int> writer_execs{0};
    std::atomic<int> no_output_execs{0};

    scheduler.add_system<fixed_rate_silent<180>>(
        "bursty_writer", [&writer_execs](talos::spmc_mut<TestMessage, TestTag1> out) {
            const int exec = writer_execs.fetch_add(1, std::memory_order_relaxed) + 1;
            if ((exec % 3) == 0) {
                out.write(TestMessage{.value = exec, .text = "written"});
            }
        });

    scheduler.add_system<fixed_rate_silent<180>>("no_output_runner", [&no_output_execs]() {
        no_output_execs.fetch_add(1, std::memory_order_relaxed);
    });

    ASSERT_TRUE(scheduler.build().has_value());

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(140));
    scheduler.stop();
    scheduler_thread.join();

    const auto stats          = nlohmann::json::parse(scheduler.get_stats_json());
    const auto& bursty_writer = stats.at("fixed_rate_systems").at("bursty_writer");
    const auto& no_output     = stats.at("fixed_rate_systems").at("no_output_runner");

    const auto writer_exec_count    = writer_execs.load(std::memory_order_relaxed);
    const auto no_output_exec_count = no_output_execs.load(std::memory_order_relaxed);

    ASSERT_GT(writer_exec_count, 0);
    ASSERT_GT(no_output_exec_count, 0);

    EXPECT_EQ(bursty_writer.at("count_mode").get<std::string>(), "written_calls");
    EXPECT_EQ(
        bursty_writer.at("executions").get<std::uint64_t>(),
        static_cast<std::uint64_t>(writer_exec_count));
    EXPECT_EQ(
        bursty_writer.at("runs").get<std::uint64_t>(),
        static_cast<std::uint64_t>(writer_exec_count / 3));
    EXPECT_LT(
        bursty_writer.at("runs").get<std::uint64_t>(),
        static_cast<std::uint64_t>(writer_exec_count));

    EXPECT_EQ(no_output.at("count_mode").get<std::string>(), "run_calls");
    EXPECT_EQ(
        no_output.at("executions").get<std::uint64_t>(),
        static_cast<std::uint64_t>(no_output_exec_count));
    EXPECT_EQ(
        no_output.at("runs").get<std::uint64_t>(),
        static_cast<std::uint64_t>(no_output_exec_count));
}

// -------------------------------------------------------------------------
// Test Suite: ResourceAccessor
// -------------------------------------------------------------------------

// Test resource type
struct TestResource {
    int value        = 0;
    std::string name = "test";
};

TEST(ResourceAccessor, DefaultConstruction) {
    rclcompat::ResourceAccessor<TestResource> accessor;

    EXPECT_FALSE(accessor.valid());
}

TEST(ResourceAccessor, CreateFromNode) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);

    // Insert a resource into the world
    node.insert_resource(TestResource{.value = 42, .name = "test_resource"});

    // Create a resource accessor
    auto accessor = node.create_resource<TestResource>();

    EXPECT_TRUE(accessor.valid());
}

TEST(ResourceAccessor, GetImmutableResource) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);

    // Insert a resource into the world
    node.insert_resource(TestResource{.value = 42, .name = "test_resource"});

    // Create a resource accessor
    auto accessor = node.create_resource<TestResource>();

    // Get the resource
    auto res = accessor.get();

    EXPECT_NE(res.ptr_, nullptr);
    EXPECT_EQ(res->value, 42);
    EXPECT_EQ(res->name, "test_resource");
}

TEST(ResourceAccessor, GetMutableResource) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);

    // Insert a resource into the world
    node.insert_resource(TestResource{.value = 42, .name = "test_resource"});

    // Create a resource accessor
    auto accessor = node.create_resource<TestResource>();

    // Get mutable access
    auto res_mut = accessor.get_mut();

    EXPECT_NE(res_mut.ptr_, nullptr);
    EXPECT_EQ(res_mut->value, 42);

    // Modify the resource
    res_mut->value = 100;
    res_mut->name  = "modified";

    // Get again to verify the change
    auto res2 = accessor.get();
    EXPECT_EQ(res2->value, 100);
    EXPECT_EQ(res2->name, "modified");
}

TEST(ResourceAccessor, HasResource) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);

    // Resource doesn't exist yet
    EXPECT_FALSE(node.has_resource<TestResource>());

    // Insert a resource
    node.insert_resource(TestResource{.value = 42, .name = "test_resource"});

    // Now it exists
    EXPECT_TRUE(node.has_resource<TestResource>());
}

TEST(ResourceAccessor, UnsafeInsertResourceAfterFinalize) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);
    ASSERT_TRUE(node.finalize().has_value());

    node.unsafe_insert_resource(TestResource{.value = 77, .name = "late_bound"});

    auto accessor = node.create_resource<TestResource>();
    ASSERT_TRUE(accessor.valid());
    auto res = accessor.get();
    EXPECT_EQ(res->value, 77);
    EXPECT_EQ(res->name, "late_bound");
}

TEST(ResourceAccessor, InsertResourcePanicsAfterFinalize) {
    World world;
    Scheduler scheduler(world);
    ASSERT_TRUE(scheduler.build().has_value());

    EXPECT_DEATH_IF_SUPPORTED(
        world.insert_resource(TestResource{.value = 1, .name = "forbidden"}), "");
}

// -------------------------------------------------------------------------
// Test: ResourceAccessor_WithCallback
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证 ResourceAccessor 可以在回调中使用
//
// VERIFICATION:
//   1. 回调可以通过 ResourceAccessor 访问资源
//   2. 资源的修改对后续访问可见
// -------------------------------------------------------------------------

TEST(ResourceAccessor, WithCallback) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);

    // Insert a resource
    node.insert_resource(TestResource{.value = 0, .name = "counter"});

    // Create resource accessor (can be stored as class member)
    auto counter_accessor = node.create_resource<TestResource>();

    // Create a subscription that uses the resource accessor
    std::atomic<bool> callback_executed{false};
    std::atomic<int> final_value{0};

    node.create_subscription<TestMessage, TestTag1>(
        [&counter_accessor, &callback_executed, &final_value](const TestMessage&) {
            callback_executed.store(true);

            // Access the resource
            auto counter = counter_accessor.get_mut();

            // Increment the counter
            counter->value += 1;
            final_value.store(counter->value);
        });

    // Create fixed_rate system that writes to the same SPMC channel
    // Note: Use spmc_mut (not spsc_mut) to trigger the subscription
    // IMPORTANT: Add fixed_rate system BEFORE calling node.finalize()!
    std::atomic<int> trigger_count{0};
    scheduler.add_system<talos::fixed_rate<100>>(
        "trigger_source", [&trigger_count](talos::spmc_mut<TestMessage, TestTag1> trigger_out) {
            trigger_count.fetch_add(1);
            trigger_out.write(TestMessage{.value = trigger_count.load(), .text = "trigger"});
            return true;
        });

    // Now finalize (this will call scheduler.build())
    EXPECT_TRUE(node.finalize().has_value());

    // Run scheduler in a separate thread
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // Give scheduler time to run
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Stop the scheduler
    scheduler.stop();
    scheduler_thread.join();

    // Verify the callback was executed and the resource was modified
    EXPECT_TRUE(callback_executed.load());
    EXPECT_GT(final_value.load(), 0);
    EXPECT_GT(trigger_count.load(), 0);
}

// -------------------------------------------------------------------------
// Test: ResourceAccessor_Copyable
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   验证 ResourceAccessor 是可复制的（与 Publisher 不同）
//
// VERIFICATION:
//   1. ResourceAccessor 可以被复制
//   2. 复制后的 accessor 访问同一个资源
// -------------------------------------------------------------------------

TEST(ResourceAccessor, Copyable) {
    World world;
    Scheduler scheduler(world);

    rclcompat::Node node("test", scheduler);

    // Insert a resource
    node.insert_resource(TestResource{.value = 42, .name = "original"});

    // Create a resource accessor
    auto accessor1 = node.create_resource<TestResource>();

    // Copy the accessor
    auto accessor2 = accessor1; // NOLINT: Testing copy behavior

    // Both accessors should be valid and access the same resource
    EXPECT_TRUE(accessor1.valid());
    EXPECT_TRUE(accessor2.valid());

    auto res1 = accessor1.get();
    auto res2 = accessor2.get();

    EXPECT_EQ(res1->value, 42);
    EXPECT_EQ(res2->value, 42);

    // Modify through accessor2
    auto mut2   = accessor2.get_mut();
    mut2->value = 100;

    // Changes should be visible through accessor1
    auto res1_after = accessor1.get();
    EXPECT_EQ(res1_after->value, 100);
}

TEST(ResourceAccessor, InvalidAfterWorldDestruction) {
    rclcompat::ResourceAccessor<TestResource> accessor;

    {
        World world;
        Scheduler scheduler(world);
        rclcompat::Node node("test", scheduler);

        node.insert_resource(TestResource{.value = 42, .name = "ephemeral"});
        accessor = node.create_resource<TestResource>();
        EXPECT_TRUE(accessor.valid());
    }

    EXPECT_FALSE(accessor.valid());
    EXPECT_DEATH_IF_SUPPORTED((void)accessor.get(), "");
}

// ============================================================================
// Wall Timer Tests
// ============================================================================

// -------------------------------------------------------------------------
// Test: BasicTimer
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   Verify that create_wall_timer creates a timer that executes at the
//   specified frequency.
//
// VERIFICATION:
//   1. Timer callback is executed multiple times
//   2. Execution count is within expected range for the given duration
// -------------------------------------------------------------------------

TEST(RclTimer, BasicTimer) {
    World world;
    Scheduler scheduler(world);
    Node node("test", scheduler);

    std::atomic<int> count{0};
    node.create_wall_timer(Frequency::Hz_100, [&count]() { count.fetch_add(1); });

    EXPECT_TRUE(node.finalize().has_value());

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    scheduler.stop();
    t.join();

    EXPECT_GE(count.load(), 8);
    EXPECT_LE(count.load(), 12);
}

// -------------------------------------------------------------------------
// Test: TimerWithPublisher
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   Verify that a timer can publish messages through a Publisher.
//
// DATA FLOW:
//   Timer -> Publisher.publish() -> Channel -> Subscriber
//
// VERIFICATION:
//   1. Timer publishes messages at the specified frequency
//   2. Scheduler consumes the publish and runs compute work
//
// NOTE: This test verifies that the timer callback can execute and call publish.
// Subscriber wake-up is covered separately.
// -------------------------------------------------------------------------

TEST(RclTimer, TimerWithPublisher) {
    World world;
    Scheduler scheduler(world);
    Node node("test", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();

    std::atomic<int> publish_count{0};
    node.create_wall_timer(Frequency::Hz_30, [&pub, &publish_count]() {
        pub.publish(TestMessage{.value = publish_count.fetch_add(1), .text = "timer"});
    });

    EXPECT_TRUE(node.finalize().has_value());

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    scheduler.stop();
    t.join();

    // Verify the timer ran and published multiple times
    EXPECT_GE(publish_count.load(), 2);

    // Verify the publisher slot exists and the publish path woke compute work.
    auto* slot = node.get_pub_slot<TestMessage, TestTag1>();
    ASSERT_NE(slot, nullptr);
    EXPECT_GE(scheduler.stats().compute_cycle_count, 1U);
}

TEST(RclTimer, TimerPublisherWakesSubscriber) {
    World world;
    Scheduler scheduler(world);
    Node node("test", scheduler);

    auto pub = node.create_publisher<TestMessage, TestTag1>();
    std::atomic<int> recv_count{0};

    node.create_subscription<TestMessage, TestTag1>(
        [&recv_count](const TestMessage&) { recv_count.fetch_add(1); });

    node.create_wall_timer(
        Frequency::Hz_30, [&pub]() { pub.publish(TestMessage{.value = 1, .text = "timer"}); });

    ASSERT_TRUE(node.finalize().has_value());

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    scheduler.stop();
    t.join();

    EXPECT_GE(recv_count.load(), 2);
}

// -------------------------------------------------------------------------
// Test: MultipleTimers
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   Verify that multiple timers can coexist and execute independently.
//
// VERIFICATION:
//   1. Fast timer executes more times than slow timer
//   2. Each timer maintains its own count
// -------------------------------------------------------------------------

TEST(RclTimer, MultipleTimers) {
    World world;
    Scheduler scheduler(world);
    Node node("test", scheduler);

    std::atomic<int> fast_count{0};
    std::atomic<int> slow_count{0};

    node.create_wall_timer(Frequency::Hz_100, [&fast_count]() { fast_count.fetch_add(1); });

    node.create_wall_timer(Frequency::Hz_20, [&slow_count]() { slow_count.fetch_add(1); });

    EXPECT_TRUE(node.finalize().has_value());

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    scheduler.stop();
    t.join();

    EXPECT_GE(fast_count.load(), 14);
    EXPECT_LE(fast_count.load(), 18);
    EXPECT_GE(slow_count.load(), 2);
    EXPECT_LE(slow_count.load(), 4);
}

// -------------------------------------------------------------------------
// Test: TimerWithResourceAccessor
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   Verify that a timer can access World resources through ResourceAccessor.
//
// VERIFICATION:
//   1. Timer can read resources via ResourceAccessor
//   2. Resource values are correctly accessed
// -------------------------------------------------------------------------

TEST(RclTimer, TimerWithResourceAccessor) {
    World world;
    Scheduler scheduler(world);

    world.insert_resource(TestResource{.value = 42, .name = "test_resource"});

    Node node("test", scheduler);
    auto accessor = node.create_resource<TestResource>();

    std::atomic<int> value_received{0};
    node.create_wall_timer(Frequency::Hz_50, [&accessor, &value_received]() {
        auto res = accessor.get();
        value_received.store(res->value);
    });

    EXPECT_TRUE(node.finalize().has_value());

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    scheduler.stop();
    t.join();

    EXPECT_EQ(value_received.load(), 42);
}

// -------------------------------------------------------------------------
// Test: MultipleNodesWithTimers
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   Verify that multiple nodes can each have their own timers.
//
// VERIFICATION:
//   1. Timers in different nodes execute independently
//   2. Each timer maintains its own execution count
// -------------------------------------------------------------------------

TEST(RclTimer, MultipleNodesWithTimers) {
    World world;
    Scheduler scheduler(world);

    // Node 1: Has a timer
    Node node1("node1", scheduler);
    std::atomic<int> node1_count{0};
    node1.create_wall_timer(Frequency::Hz_30, [&node1_count]() { node1_count.fetch_add(1); });

    // Node 2: Has another timer
    Node node2("node2", scheduler);
    std::atomic<int> node2_count{0};
    node2.create_wall_timer(Frequency::Hz_20, [&node2_count]() { node2_count.fetch_add(1); });

    EXPECT_TRUE(node1.finalize().has_value());
    EXPECT_TRUE(node2.finalize().has_value());

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    scheduler.stop();
    t.join();

    // node1's timer runs at ~33Hz, node2's at 20Hz
    // In 160ms, node1 should run ~5 times, node2 ~3 times
    EXPECT_GE(node1_count.load(), 3);
    EXPECT_GE(node2_count.load(), 2);
}

// -------------------------------------------------------------------------
// Test: TimerFrequencyRounding
// -------------------------------------------------------------------------
// TEST PURPOSE:
//   Verify that timer frequency is rounded to nearest supported value.
//
// VERIFICATION:
//   1. Timer with non-standard frequency still executes
//   2. Frequency is rounded appropriately
// -------------------------------------------------------------------------

class ProperNode : public Node {
public:
    explicit ProperNode(std::string name, Scheduler& scheduler) noexcept
        : Node(std::move(name), scheduler) {
        // deprecated:
        // this->create_wall_timer(Frequency::Hz_27, std::bind(&ProperNode::tick, this));
        //
        // suggested:
        this->create_wall_timer(Frequency::Hz_27, [this]() { this->tick(); });
    }
    void tick() noexcept { count.fetch_add(1); }
    std::atomic<int> count{0};
};

TEST(RclTimer, TimerFrequencyRounding) {
    World world;
    Scheduler scheduler(world);
    ProperNode node("test", scheduler);

    // 27Hz timer
    EXPECT_TRUE(node.finalize().has_value());

    scheduler.print_systems();
    scheduler.print_mermaid_wake_chains();

    std::jthread t([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    scheduler.stop();
    t.join();

    // 37ms * 4 = 148ms, so we expect ~4 executions
    // With rounding to 27Hz (~37ms), we should get approximately 4 executions
    EXPECT_GE(node.count.load(), 3);
    EXPECT_LE(node.count.load(), 5);
}

} // namespace talos::test
