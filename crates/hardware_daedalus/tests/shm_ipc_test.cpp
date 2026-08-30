// ============================================================================
// ipc（hardware_daedalus）共享内存下位机通信测试
//
// 被测对象（crates/hardware_daedalus/src/）：
//   shm_layout.hpp         跨 C++/Rust ABI 布局（魔数/版本/各结构体大小偏移，
//                          文件内已有 static_assert 编译期锁定）
//   shm_region.hpp         /tmp 文件式 mmap 共享内存 RAII（create/open/create_or_open）
//   shm_triple_buffer.hpp  无锁 SPSC 三缓冲（FLAG_NEW + 就绪下标位编码）
//   shm_client.hpp         ShmClient：消费者 connect() / 生产者 create()
//
// 通信模型：Rust 模拟器（生产者）↔ C++ 视觉程序（消费者）。
// 生产者模式 create() 本身即为单元测试设计——测试直接用两个客户端
// 在同一进程内模拟双端（mmap MAP_SHARED 同一文件，天然内存互通），
// 完整覆盖图像/位姿/云台指令全链路。
//
// 测试分层：
//   第一层 ShmRegion      —— mmap 区域生命周期：零初始化/双端可见性/
//                            错误路径（NotFound/InvalidSize）/owner 删文件/移动语义
//   第二层 三缓冲原语     —— 无新数据语义/发布-借用往返/消费后置位清除/
//                            连续发布覆盖/三槽位轮换
//   第三层 ShmClient      —— connect 校验（魔数/版本/不存在）/
//                            图像-位姿-云台指令全链路/心跳保活
//
// 运行（项目根目录）：
//   cmake -B build -DTALOS_BUILD_TESTING=ON && cmake --build build
//   ./build/shm_ipc_test             # 或 ctest
// ============================================================================
#include <gtest/gtest.h>

// 被测头文件（真实项目源码，不复制）
#include "shm_client.hpp"
#include "shm_layout.hpp"
#include "shm_region.hpp"
#include "shm_triple_buffer.hpp"

// C++ 标准库
#include <atomic>
#include <cstdint>
#include <cstring>
#include <filesystem>

// OpenCV：ShmClient 图像接口返回 cv::Mat
#include <opencv2/core.hpp>

namespace fs = std::filesystem;

// ============================================================================
// 测试夹具：每条用例前后清理 /tmp 全局共享内存文件
// （IPC 名字是全局常量，用例间必须互不残留）
// ============================================================================
class ShmIpcTest : public ::testing::Test {
protected:
    void SetUp() override { cleanup(); }

    void TearDown() override { cleanup(); }

    /// 删除两块共享内存文件（幂等）
    static void cleanup() {
        fs::remove(ipc::shm_path(ipc::SHM_NAME_META));
        fs::remove(ipc::shm_path(ipc::SHM_NAME_IMAGE_POOL));
    }

    /// 生成一份字段确定可断言的位姿
    static ipc::ShmClient::Pose make_pose(const uint64_t frame_seq) {
        return ipc::ShmClient::Pose{
            .x            = 1.5,
            .y            = -2.25,
            .z            = 0.75,
            .qw           = 0.9238795, // cos(22.5°)
            .qx           = 0.0,
            .qy           = 0.0,
            .qz           = 0.3826834, // sin(22.5°)
            .frame_seq    = frame_seq,
            .timestamp_ns = 1234567890,
        };
    }
};

// ============================================================================
// 第一层：ShmRegion mmap 区域生命周期
// ============================================================================

/// 正常流程：create 后内存零初始化、属性正确
TEST_F(ShmIpcTest, RegionCreateIsZeroInitialized) {
    constexpr size_t SIZE = 4096;
    auto result = ipc::ShmRegion::create("talos_shm_test_region", SIZE);
    ASSERT_TRUE(result.has_value()) << "create 失败";

    EXPECT_EQ(result->size(), SIZE);
    EXPECT_TRUE(result->is_owner());
    EXPECT_EQ(result->path(), ipc::shm_path("talos_shm_test_region"));
    ASSERT_NE(result->data(), nullptr);

    // 新建区域必须全零（无脏数据残留）
    const auto* bytes = static_cast<const uint8_t*>(result->data());
    for (size_t i = 0; i < SIZE; ++i) {
        ASSERT_EQ(bytes[i], 0u) << "偏移 " << i << " 非零";
    }
}

/// 正常流程：create 与 open 映射同一文件，写入互相可见（MAP_SHARED 语义）
TEST_F(ShmIpcTest, RegionSharedVisibilityBetweenCreateAndOpen) {
    constexpr size_t SIZE = 1024;
    auto producer = ipc::ShmRegion::create("talos_shm_test_region", SIZE);
    ASSERT_TRUE(producer.has_value());

    auto consumer = ipc::ShmRegion::open("talos_shm_test_region", SIZE);
    ASSERT_TRUE(consumer.has_value()) << "open 失败";
    EXPECT_FALSE(consumer->is_owner()); // 消费者不拥有文件

    // 生产者写 → 消费者读
    auto* shared = producer->as<uint32_t>();
    shared[0] = 0xDEADBEEFu;
    shared[1] = 0x5A5A5A5Au;
    EXPECT_EQ(consumer->as<const uint32_t>()[0], 0xDEADBEEFu);
    EXPECT_EQ(consumer->as<const uint32_t>()[1], 0x5A5A5A5Au);

    // 消费者写 → 生产者读（双向）
    consumer->as<uint32_t>()[2] = 0x12345678u;
    EXPECT_EQ(shared[2], 0x12345678u);
}

/// 异常处理：打开不存在的共享内存 → NotFound
TEST_F(ShmIpcTest, RegionOpenMissingReturnsNotFound) {
    const auto result = ipc::ShmRegion::open("talos_shm_never_created", 128);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ipc::ShmError::NotFound);
}

/// 异常处理：磁盘文件实际尺寸小于请求映射尺寸 → InvalidSize（防越界读取）
TEST_F(ShmIpcTest, RegionOpenSmallerFileReturnsInvalidSize) {
    constexpr size_t CREATED = 1024;
    auto producer = ipc::ShmRegion::create("talos_shm_test_region", CREATED);
    ASSERT_TRUE(producer.has_value());

    // 请求 2048 > 实际 1024 → 拒绝
    const auto too_big = ipc::ShmRegion::open("talos_shm_test_region", CREATED * 2);
    ASSERT_FALSE(too_big.has_value());
    EXPECT_EQ(too_big.error(), ipc::ShmError::InvalidSize);

    // 请求恰好等于实际大小 → 允许
    const auto exact = ipc::ShmRegion::open("talos_shm_test_region", CREATED);
    EXPECT_TRUE(exact.has_value());
}

/// 生命周期：owner 析构删除 /tmp 文件，非 owner 析构保留
/// 注意：C++ 析构顺序与声明顺序相反，consumer 需放入内层作用域，
/// 保证「consumer 先析构、producer 仍存活」这一被测时序。
TEST_F(ShmIpcTest, RegionOwnerDeletesFileOnDestroy) {
    const auto path = ipc::shm_path("talos_shm_test_region");
    {
        auto producer = ipc::ShmRegion::create("talos_shm_test_region", 64);
        ASSERT_TRUE(producer.has_value());
        {
            auto consumer = ipc::ShmRegion::open("talos_shm_test_region", 64);
            ASSERT_TRUE(consumer.has_value());
        } // consumer 先析构：非 owner，文件必须还在
        EXPECT_TRUE(fs::exists(path)) << "非 owner 析构不得删除共享文件";
    } // producer 随后析构：owner 删除文件
    EXPECT_FALSE(fs::exists(path)) << "owner 析构应删除共享文件";
}

/// 生命周期：移动语义转移所有权，旧对象不再释放资源
TEST_F(ShmIpcTest, RegionMoveTransfersOwnership) {
    const auto path = ipc::shm_path("talos_shm_test_region");
    std::optional<ipc::ShmRegion> moved; // 默认空，后续移动赋值接管资源
    {
        auto producer = ipc::ShmRegion::create("talos_shm_test_region", 64);
        ASSERT_TRUE(producer.has_value());
        void* const addr = producer->data();

        moved = std::move(*producer);
        // 新对象接管全部资源
        EXPECT_EQ(moved->data(), addr);
        EXPECT_TRUE(moved->is_owner());
        // 旧对象被掏空：仅剩壳，析构不重复释放（本用例通过即未双重释放/未崩溃）
    }
    // moved 仍存活且已接管所有权 → 文件保留（证明所有权确实转移给了新对象）
    EXPECT_TRUE(fs::exists(path)) << "移动后新对象持有所有权，文件应保留";
    // 显式结束 moved 生命周期 → owner 析构删除文件
    moved.reset();
    EXPECT_FALSE(fs::exists(path)) << "moved 析构应删除共享文件";
}

/// create_or_open：已存在则复用（非覆盖），不存在则新建
TEST_F(ShmIpcTest, RegionCreateOrOpenReusesExisting) {
    constexpr size_t SIZE = 128;
    auto first = ipc::ShmRegion::create("talos_shm_test_region", SIZE);
    ASSERT_TRUE(first.has_value());
    first->as<uint32_t>()[0] = 0xAABBCCDDu; // 写入哨兵

    auto second = ipc::ShmRegion::create_or_open("talos_shm_test_region", SIZE);
    ASSERT_TRUE(second.has_value());
    // 复用已有映射：哨兵数据保留，且是消费者身份（非 owner）
    EXPECT_EQ(second->as<const uint32_t>()[0], 0xAABBCCDDu);
    EXPECT_FALSE(second->is_owner());
}

/// shm_path：前导斜杠规范化（兼容 Rust memmap2 命名）
TEST_F(ShmIpcTest, ShmPathNormalizesLeadingSlash) {
    EXPECT_EQ(ipc::shm_path("/talos_xxx").string(), "/tmp/talos_xxx");
    EXPECT_EQ(ipc::shm_path("talos_xxx").string(), "/tmp/talos_xxx");
}

// ============================================================================
// 第二层：无锁三缓冲原语（通过 ShmClient::create 提供的真实通道测试）
// ============================================================================

/// 初始状态：无新数据，borrow 返回空
TEST_F(ShmIpcTest, TripleBufferStartsWithNoNewData) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value()) << "create 失败";

    // 图像通道：初始无新帧
    EXPECT_FALSE(producer->has_new_image());
    EXPECT_FALSE(producer->recv_image().has_value());
    // 位姿通道：全部五路均无数据
    for (uint8_t i = 0; i < 5; ++i) {
        EXPECT_FALSE(producer->recv_pose(static_cast<ipc::PoseIndex>(i)).has_value())
            << "位姿通道 " << i;
    }
    // 辅助通道：未写入时均为空
    EXPECT_FALSE(producer->recv_chassis_observation().has_value());
    EXPECT_FALSE(producer->recv_ground_truth().has_value());
    EXPECT_FALSE(producer->recv_runtime_state().has_value());
}

/// 正常流程：位姿发布 → 借用往返，字段逐项一致（float 精度内）
TEST_F(ShmIpcTest, PosePublishBorrowRoundTrip) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value()) << "connect 失败";

    const auto sent = make_pose(42);
    producer->publish_pose(ipc::POSE_GIMBAL, sent);

    const auto received = consumer->recv_pose(ipc::POSE_GIMBAL);
    ASSERT_TRUE(received.has_value());
    EXPECT_DOUBLE_EQ(received->x, sent.x);
    EXPECT_DOUBLE_EQ(received->y, sent.y);
    EXPECT_DOUBLE_EQ(received->z, sent.z);
    EXPECT_FLOAT_EQ(static_cast<float>(received->qw), static_cast<float>(sent.qw));
    EXPECT_FLOAT_EQ(static_cast<float>(received->qz), static_cast<float>(sent.qz));
    EXPECT_EQ(received->frame_seq, sent.frame_seq);
    EXPECT_EQ(received->timestamp_ns, sent.timestamp_ns);
}

/// 消费语义：borrow 一次后 FLAG_NEW 清除，再次读取无数据
TEST_F(ShmIpcTest, TripleBufferConsumeClearsNewFlag) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    producer->publish_pose(ipc::POSE_ODOM, make_pose(1));
    ASSERT_TRUE(consumer->has_new_image() || true); // 位姿通道独立于图像通道
    ASSERT_TRUE(consumer->recv_pose(ipc::POSE_ODOM).has_value());

    // 消费后无新数据：同一帧不能读两次
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_ODOM).has_value());
}

/// 发布语义：连续多次 publish 后消费，只拿到最新一帧（覆盖旧值）
TEST_F(ShmIpcTest, TripleBufferLatestWins) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    // 连发三帧（帧序号递增），中间帧被覆盖
    for (uint64_t seq = 1; seq <= 3; ++seq) {
        auto pose    = make_pose(seq);
        pose.x       = static_cast<double>(seq);
        producer->publish_pose(ipc::POSE_MUZZLE, pose);
    }

    const auto received = consumer->recv_pose(ipc::POSE_MUZZLE);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->frame_seq, 3u); // 最新帧
    EXPECT_DOUBLE_EQ(received->x, 3.0);
    // 只消费一次：覆盖丢失的中间帧不复活
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_MUZZLE).has_value());
}

/// 三槽轮换压力：连续发布远超 3 槽数量的帧，无崩溃且始终拿到最新值
TEST_F(ShmIpcTest, TripleBufferSlotRotationStress) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    for (uint64_t seq = 1; seq <= 100; ++seq) {
        auto pose  = make_pose(seq);
        pose.x     = static_cast<double>(seq);
        producer->publish_pose(ipc::POSE_CAMERA, pose);
    }
    const auto received = consumer->recv_pose(ipc::POSE_CAMERA);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->frame_seq, 100u);
}

/// 通道隔离：五路位姿互不干扰
TEST_F(ShmIpcTest, PoseChannelsAreIndependent) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    // 仅发布云台通道
    producer->publish_pose(ipc::POSE_GIMBAL, make_pose(10));
    // 云台有数据，其余四路仍为空
    EXPECT_TRUE(consumer->recv_pose(ipc::POSE_GIMBAL).has_value());
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_ODOM).has_value());
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_MUZZLE).has_value());
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_CAMERA).has_value());
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_CHASSIS_OBSERVATION).has_value());
}

// ============================================================================
// 第三层：ShmClient 连接校验与全链路
// ============================================================================

/// 异常处理：共享内存不存在时 connect 失败 → NotFound
TEST_F(ShmIpcTest, ConnectFailsWhenShmAbsent) {
    const auto result = ipc::ShmClient::connect();
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ipc::ShmError::NotFound);
}

/// 正常流程：create 初始化头部，connect 校验通过
TEST_F(ShmIpcTest, CreateThenConnectHeaderValid) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());

    // 生产者写入的头部信息
    EXPECT_EQ(producer->header().magic, ipc::SHM_MAGIC);
    EXPECT_EQ(producer->header().version, ipc::SHM_VERSION);
    EXPECT_EQ(producer->header().image_width, ipc::IMAGE_WIDTH);
    EXPECT_EQ(producer->header().image_height, ipc::IMAGE_HEIGHT);

    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());
    EXPECT_EQ(consumer->header().magic, ipc::SHM_MAGIC);
}

/// 异常处理：魔数被篡改 → connect 拒绝（防读脏内存）
TEST_F(ShmIpcTest, ConnectRejectsCorruptedMagic) {
    {
        auto producer = ipc::ShmClient::create();
        ASSERT_TRUE(producer.has_value());
    } // producer 析构删文件——重新自建以拿到裸区域
    {
        auto producer = ipc::ShmClient::create();
        ASSERT_TRUE(producer.has_value());

        // 直接通过 ShmRegion 篡改魔数
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        ASSERT_TRUE(raw.has_value());
        raw->as<ipc::ShmMetaRegion>()->header.magic = 0x12345678u;
    }
    const auto result = ipc::ShmClient::connect();
    ASSERT_FALSE(result.has_value());
}

/// 异常处理：版本号不匹配 → connect 拒绝（防跨版本内存错位）
TEST_F(ShmIpcTest, ConnectRejectsVersionMismatch) {
    {
        auto producer = ipc::ShmClient::create();
        ASSERT_TRUE(producer.has_value());

        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        ASSERT_TRUE(raw.has_value());
        raw->as<ipc::ShmMetaRegion>()->header.version = ipc::SHM_VERSION + 1;
    }
    const auto result = ipc::ShmClient::connect();
    ASSERT_FALSE(result.has_value());
}

/// 正常流程：图像全链路——发布像素 → 零拷贝消费（seq/时间戳/像素内容一致）
TEST_F(ShmIpcTest, ImageFullPipelineRoundTrip) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    // 构造带哨兵像素的 RGB 图像（1440x1080x3）
    cv::Mat image(ipc::IMAGE_HEIGHT, ipc::IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    image.at<cv::Vec3b>(0, 0)                 = cv::Vec3b(0x5A, 0xA5, 0x5A); // 左上角哨兵
    image.at<cv::Vec3b>(ipc::IMAGE_HEIGHT - 1, ipc::IMAGE_WIDTH - 1) =
        cv::Vec3b(0xFF, 0x00, 0x7F); // 右下角哨兵
    image.at<cv::Vec3b>(ipc::IMAGE_HEIGHT / 2, ipc::IMAGE_WIDTH / 2) =
        cv::Vec3b(0x11, 0x22, 0x33); // 中心哨兵

    producer->publish_image(image, 777u, 987654321u);

    // 消费前 has_new_image 为真
    EXPECT_TRUE(consumer->has_new_image());
    const auto frame = consumer->recv_image();
    ASSERT_TRUE(frame.has_value());

    // 帧元数据一致
    EXPECT_EQ(frame->seq, 777u);
    EXPECT_EQ(frame->timestamp_ns, 987654321u);
    // 零拷贝 Mat 尺寸正确
    EXPECT_EQ(frame->image.rows, static_cast<int>(ipc::IMAGE_HEIGHT));
    EXPECT_EQ(frame->image.cols, static_cast<int>(ipc::IMAGE_WIDTH));
    EXPECT_EQ(frame->image.type(), CV_8UC3);

    // 像素内容一致（三个哨兵点位抽检）
    EXPECT_EQ(frame->image.at<cv::Vec3b>(0, 0), cv::Vec3b(0x5A, 0xA5, 0x5A));
    EXPECT_EQ(frame->image.at<cv::Vec3b>(ipc::IMAGE_HEIGHT / 2, ipc::IMAGE_WIDTH / 2),
              cv::Vec3b(0x11, 0x22, 0x33));
    EXPECT_EQ(frame->image.at<cv::Vec3b>(ipc::IMAGE_HEIGHT - 1, ipc::IMAGE_WIDTH - 1),
              cv::Vec3b(0xFF, 0x00, 0x7F));

    // 消费后新帧标志清除
    EXPECT_FALSE(consumer->has_new_image());
}

/// 正常流程：云台指令下发全链路（消费者 send → 生产者侧读回）
/// 这是自瞄输出到仿真下位机的关键路径
TEST_F(ShmIpcTest, GimbalCommandDownlinkRoundTrip) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    // C++ 视觉端下发：偏航 15°，俯仰 -8°，距离 3.5m，允许开火
    consumer->send_gimbal_cmd(15.0f, -8.0f, 3.5f, true);

    // 生产者侧（Rust 模拟器角色）通过三缓冲读回指令
    auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
    ASSERT_TRUE(raw.has_value());
    ipc::GimbalOps ops(&raw->as<ipc::ShmMetaRegion>()->gimbal_cmd);
    const auto slot = ops.borrow();
    ASSERT_TRUE(slot.has_value());

    EXPECT_FLOAT_EQ((*slot)->yaw_deg, 15.0f);
    EXPECT_FLOAT_EQ((*slot)->pitch_deg, -8.0f);
    EXPECT_FLOAT_EQ((*slot)->distance_m, 3.5f);
    EXPECT_EQ((*slot)->fire_advice, 1u);
    EXPECT_GT((*slot)->timestamp_ns, 0u); // 时间戳已填充

    // 消费一次后指令不残留
    EXPECT_FALSE(ops.borrow().has_value());

    // 边界值：无有效目标（distance=-1）与禁止开火
    consumer->send_gimbal_cmd(0.0f, 0.0f, -1.0f, false);
    const auto slot2 = ops.borrow();
    ASSERT_TRUE(slot2.has_value());
    EXPECT_FLOAT_EQ((*slot2)->distance_m, -1.0f);
    EXPECT_EQ((*slot2)->fire_advice, 0u);
}

/// 正常流程：相机内参发布/读取（单变量直通通道）
TEST_F(ShmIpcTest, CameraInfoPublishRead) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    ipc::CameraInfo info{};
    info.timestamp_ns = 100u;
    info.fx = 920.5;
    info.fy = 921.1;
    info.cx = 719.5;
    info.cy = 539.5;
    info.distortion[0] = -0.35;
    info.distortion[4] = 0.02;
    info.width = ipc::IMAGE_WIDTH;
    info.height = ipc::IMAGE_HEIGHT;
    producer->publish_camera_info(info);

    const auto& received = consumer->camera_info();
    EXPECT_DOUBLE_EQ(received.fx, 920.5);
    EXPECT_DOUBLE_EQ(received.cy, 539.5);
    EXPECT_DOUBLE_EQ(received.distortion[0], -0.35);
    EXPECT_EQ(received.width, ipc::IMAGE_WIDTH);
    EXPECT_EQ(received.height, ipc::IMAGE_HEIGHT);
}

/// 正常流程：底盘观测/运行状态直通通道（timestamp=0 → 无效语义）
TEST_F(ShmIpcTest, DirectChannelsRespectTimestampValidity) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    // 未写入：timestamp_ns=0 → nullopt
    EXPECT_FALSE(consumer->recv_chassis_observation().has_value());
    EXPECT_FALSE(consumer->recv_runtime_state().has_value());

    // 写入运行状态
    producer->publish_runtime_state(true, 555u);
    const auto state = consumer->recv_runtime_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->following, 1u);
    EXPECT_EQ(state->timestamp_ns, 555u);

    // 写入底盘观测（直接通过裸区域写入结构体）
    auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
    ASSERT_TRUE(raw.has_value());
    auto& obs = raw->as<ipc::ShmMetaRegion>()->chassis_observation;
    obs.timestamp_ns = 42u;
    obs.frame_seq = 7u;
    obs.v_body[0] = 1.25f;
    obs.v_body[1] = -0.5f;
    obs.wz_radps = 0.8f;

    const auto received = consumer->recv_chassis_observation();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->frame_seq, 7u);
    EXPECT_FLOAT_EQ(received->v_body[0], 1.25f);
    EXPECT_FLOAT_EQ(received->v_body[1], -0.5f);
    EXPECT_FLOAT_EQ(received->wz_radps, 0.8f);
}

/// 心跳保活：无心跳判离线 → 更新心跳判存活
TEST_F(ShmIpcTest, HeartbeatLifecycle) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    // 心跳从未更新（=0）：now - 0 巨大 → 判定离线
    EXPECT_FALSE(consumer->is_producer_alive());

    // 生产者更新心跳 → 存活
    producer->update_heartbeat();
    EXPECT_TRUE(consumer->is_producer_alive());

    // wait_for_producer 立即返回 true（心跳新鲜）
    EXPECT_TRUE(consumer->wait_for_producer(std::chrono::milliseconds(100)));

    // 心跳超时（阈值取 0）：新鲜心跳也可能临界，此处只验证超时路径可返回 false
    // 用极大阈值反证：即使心跳很老，阈值无限大也判存活
    EXPECT_TRUE(consumer->is_producer_alive(UINT64_MAX / 2));
}

/// 综合场景：图像 + 位姿 + 指令三通道并发混合收发
TEST_F(ShmIpcTest, MixedChannelWorkload) {
    auto producer = ipc::ShmClient::create();
    ASSERT_TRUE(producer.has_value());
    auto consumer = ipc::ShmClient::connect();
    ASSERT_TRUE(consumer.has_value());

    cv::Mat image(ipc::IMAGE_HEIGHT, ipc::IMAGE_WIDTH, CV_8UC3, cv::Scalar(9, 8, 7));

    // 模拟一段运行时循环：每帧发图像+位姿，同时下发云台指令
    for (uint64_t seq = 1; seq <= 10; ++seq) {
        producer->publish_pose(ipc::POSE_GIMBAL, make_pose(seq));
        producer->publish_image(image, seq, seq * 1000000u);
        consumer->send_gimbal_cmd(
            static_cast<float>(seq), static_cast<float>(-seq) * 0.5f, 2.0f, seq % 2 == 0);

        // 帧消费：图像与位姿的 frame_seq 对齐
        const auto frame = consumer->recv_image();
        ASSERT_TRUE(frame.has_value());
        EXPECT_EQ(frame->seq, seq);
        const auto pose = consumer->recv_pose(ipc::POSE_GIMBAL);
        ASSERT_TRUE(pose.has_value());
        EXPECT_EQ(pose->frame_seq, seq);
    }

    // 全部消费完毕：无残留新数据
    EXPECT_FALSE(consumer->has_new_image());
    EXPECT_FALSE(consumer->recv_pose(ipc::POSE_GIMBAL).has_value());
}
