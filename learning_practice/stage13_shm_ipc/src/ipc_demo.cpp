// ============================================================================
// stage13：共享内存 IPC 单进程三层功能自测
//
// 被测对象（全部为项目真实源码，零复制）：
//   第一层 shm_region.hpp        ShmRegion：/tmp 文件式 mmap RAII
//   第二层 shm_triple_buffer.hpp TripleBufferOps：SPSC 无锁三缓冲原语
//   第三层 shm_client.hpp        ShmClient：create/connect + 图像/位姿/指令全链路
//
// 测试清单：
//   [1] ShmRegion：create 零初始化 / 双映射可见 / NotFound / InvalidSize
//       / owner 析构删文件 / 非 owner 保留 / move 转移所有权
//   [2] 三缓冲原语：初始无数据 / publish→borrow 往返 / FLAG_NEW 清除
//       / latest-wins / 三槽轮换压力
//   [3] ShmClient：connect 魔数·版本校验 / publish_image→recv_image 零拷贝
//       / publish_pose 五路隔离 / send_gimbal_cmd→GimbalOps 读回 / 心跳保活
//
// 运行：./ipc_demo —— 全部断言通过退出码 0，/tmp/talos_ipc_* 无残留
// ============================================================================

#include "shm_client.hpp"
#include "shm_layout.hpp"
#include "shm_region.hpp"
#include "shm_triple_buffer.hpp"

// C++ 标准库
#include <cassert>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>

// fmt：格式化输出
#include <fmt/core.h>

// OpenCV：构造测试图像矩阵
#include <opencv2/core.hpp>

namespace fs = std::filesystem;

// ============================================================================
// 测试夹具：ShmClient 使用固定的全局名字（/tmp/talos_ipc_meta 等），
// 每个用例前后必须清理残留文件，避免上一次异常退出留下的旧内存污染
// ============================================================================
static void cleanup_shm_files() {
    fs::remove(ipc::shm_path(ipc::SHM_NAME_META));
    fs::remove(ipc::shm_path(ipc::SHM_NAME_IMAGE_POOL));
}

/// 用例开始前打印标题并清理现场
static void test_begin(const char* name) {
    cleanup_shm_files();
    fmt::print("\n---- [{}] ----\n", name);
}

/// 用例结束：断言共享文件已被 owner 清理干净（防泄漏检查）
static void test_end_no_leak() {
    assert(!fs::exists(ipc::shm_path(ipc::SHM_NAME_META)));
    assert(!fs::exists(ipc::shm_path(ipc::SHM_NAME_IMAGE_POOL)));
    fmt::print("  [OK] /tmp 共享文件已清理，无残留\n");
}

// ============================================================================
// 第一层：ShmRegion 文件式 mmap RAII
// ============================================================================
static void test_shm_region() {
    test_begin("1-ShmRegion 基础生命周期");

    // --- create：新建映射，内存应全部零初始化 ---
    auto created = ipc::ShmRegion::create("stage13_demo", 128);
    assert(created.has_value());
    assert(created->size() == 128);
    assert(created->is_owner()); // 创建者持有所有权
    const auto* bytes = static_cast<const uint8_t*>(created->data());
    for (size_t i = 0; i < 128; ++i) {
        assert(bytes[i] == 0); // memset 清零，无脏数据
    }
    fmt::print("  [OK] create 零初始化 + owner 标记\n");

    // --- 同进程双映射：MAP_SHARED 写入互相可见（跨进程语义的基础）---
    auto opened = ipc::ShmRegion::open("stage13_demo", 128);
    assert(opened.has_value());
    assert(!opened->is_owner()); // open 模式：消费者，析构不删文件
    static_cast<uint8_t*>(created->data())[0] = 0xAB;
    assert(static_cast<const uint8_t*>(opened->data())[0] == 0xAB);
    static_cast<uint8_t*>(opened->data())[127] = 0xCD;
    assert(static_cast<const uint8_t*>(created->data())[127] == 0xCD);
    fmt::print("  [OK] 双映射 MAP_SHARED 写入互相可见\n");

    // --- 错误路径：NotFound（文件不存在）---
    const auto missing = ipc::ShmRegion::open("stage13_never_created", 64);
    assert(!missing.has_value());
    assert(missing.error() == ipc::ShmError::NotFound);
    fmt::print("  [OK] open 不存在文件 → {}\n", missing.error());

    // --- 错误路径：InvalidSize（磁盘文件小于请求映射大小）---
    const auto too_big = ipc::ShmRegion::open("stage13_demo", 256);
    assert(!too_big.has_value());
    assert(too_big.error() == ipc::ShmError::InvalidSize);
    fmt::print("  [OK] open 尺寸不足 → {}\n", too_big.error());

    // --- 非 owner 析构不删文件，owner 析构才删 ---
    // 注意：GCC 的 std::expected 无 reset()，用 unexpected 赋值销毁内含对象
    opened = std::unexpected(ipc::ShmError::OpenFailed); // 消费者先析构
    assert(fs::exists(created->path())); // 文件必须还在
    created = std::unexpected(ipc::ShmError::OpenFailed); // 创建者析构 → 删除文件
    assert(!fs::exists(ipc::shm_path("stage13_demo")));
    fmt::print("  [OK] 非 owner 析构保留文件 / owner 析构删除文件\n");

    // --- move 语义：所有权转移，旧对象不再持有资源 ---
    auto src = ipc::ShmRegion::create("stage13_move", 64);
    assert(src.has_value());
    const void* addr = src->data();
    auto dst = std::move(src); // 移动后：dst 接管 owner，src 变空壳
    assert(dst->is_owner());
    assert(dst->data() == addr);
    dst = std::unexpected(ipc::ShmError::OpenFailed); // 新 owner 析构 → 删文件
    assert(!fs::exists(ipc::shm_path("stage13_move")));
    fmt::print("  [OK] move 转移所有权并正确清理\n");
}

// ============================================================================
// 第二层：TripleBufferOps 无锁三缓冲原语（栈上结构，不涉及共享内存文件）
// ============================================================================
static void test_triple_buffer() {
    test_begin("2-TripleBuffer 无锁三缓冲原语");

    ipc::ImageTripleBuffer buf{}; // state=1, write_idx=0, read_idx=2
    ipc::ImageOps       ops(&buf);

    // --- 初始无新数据：has_new_data=false，borrow 返回 nullopt ---
    assert(!ops.has_new_data());
    assert(!ops.borrow().has_value());
    fmt::print("  [OK] 初始状态无新数据，borrow 返回 nullopt\n");

    // --- publish→borrow 往返 ---
    ops.borrow_mut().seq = 42; // borrow_mut 拿可写槽写入
    assert(!ops.has_new_data()); // 写入未发布，消费者不可见
    ops.publish();              // 发布：FLAG_NEW 置位 + 槽位交换
    assert(ops.has_new_data());
    const auto got = ops.borrow();
    assert(got.has_value());
    assert((*got)->seq == 42);
    fmt::print("  [OK] publish→borrow 往返数据一致\n");

    // --- 消费后 FLAG_NEW 清除：同一帧不会被读两次 ---
    assert(!ops.has_new_data());
    assert(!ops.borrow().has_value());
    assert(ops.current().seq == 42); // current 仍可读上次消费的槽
    fmt::print("  [OK] 消费后 FLAG_NEW 清除（同帧不重复消费）\n");

    // --- latest-wins：连发两帧不消费，borrow 只取最新 ---
    ops.borrow_mut().seq = 100;
    ops.publish();
    ops.borrow_mut().seq = 101; // 第二帧覆盖式发布
    ops.publish();
    const auto latest = ops.borrow();
    assert(latest.has_value());
    assert((*latest)->seq == 101); // 旧帧 100 被丢弃
    fmt::print("  [OK] latest-wins：连发两帧只保留最新\n");

    // --- 三槽轮换压力：10000 次发布/消费，槽位循环使用不出错 ---
    for (uint64_t i = 0; i < 10000; ++i) {
        ops.borrow_mut().seq = i;
        ops.publish();
        const auto s = ops.borrow();
        assert(s.has_value());
        assert((*s)->seq == i); // 每帧严格一一对应 → 槽位轮换正确
    }
    fmt::print("  [OK] 三槽轮换压力 10000 次发布/消费全对\n");
}

// ============================================================================
// 第三层：ShmClient 全链路（create 生产者视角 + 原始映射篡改模拟异常）
// ============================================================================
static void test_shm_client() {
    test_begin("3-ShmClient 全链路");

    // --- create：建立两块共享内存并初始化头部 ---
    auto producer = ipc::ShmClient::create();
    assert(producer.has_value());
    assert(producer->header().magic == ipc::SHM_MAGIC);
    assert(producer->header().version == ipc::SHM_VERSION);
    fmt::print("  [OK] create 初始化头部 magic={:#x} version={}\n",
               ipc::SHM_MAGIC, ipc::SHM_VERSION);

    // --- connect 正常连接 ---
    auto consumer = ipc::ShmClient::connect();
    assert(consumer.has_value());
    fmt::print("  [OK] connect 正常连接\n");

    // --- connect 拒绝：篡改魔数（模拟残留垃圾文件）---
    {
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        assert(raw.has_value());
        raw->as<ipc::ShmMetaRegion>()->header.magic = 0xDEADBEEF;
    }
    auto bad_magic = ipc::ShmClient::connect();
    assert(!bad_magic.has_value());
    fmt::print("  [OK] connect 魔数校验拒绝 → {}\n", bad_magic.error());

    // --- connect 拒绝：版本不匹配（模拟 Rust/C++ 两端布局不一致）---
    {
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        assert(raw.has_value());
        auto* meta = raw->as<ipc::ShmMetaRegion>();
        meta->header.magic   = ipc::SHM_MAGIC; // 恢复魔数
        meta->header.version = ipc::SHM_VERSION + 1; // 篡改版本
    }
    auto bad_ver = ipc::ShmClient::connect();
    assert(!bad_ver.has_value());
    fmt::print("  [OK] connect 版本校验拒绝 → {}\n", bad_ver.error());
    {
        // 恢复正确版本，供后续用例继续使用
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        raw->as<ipc::ShmMetaRegion>()->header.version = ipc::SHM_VERSION;
    }

    // --- publish_image→recv_image：零拷贝像素抽检 ---
    // 注意：publish_image 固定 memcpy IMAGE_SIZE 字节，测试图必须全尺寸 1440x1080x3
    cv::Mat img(ipc::IMAGE_HEIGHT, ipc::IMAGE_WIDTH, CV_8UC3);
    for (int r = 0; r < img.rows; ++r) {
        img.row(r) = cv::Scalar(r & 0xFF, (r >> 1) & 0xFF, (r >> 2) & 0xFF); // 行渐变
    }
    img.at<cv::Vec3b>(0, 0)                 = {0x11, 0x22, 0x33}; // 左上哨兵
    img.at<cv::Vec3b>(ipc::IMAGE_HEIGHT - 1, ipc::IMAGE_WIDTH - 1) = {0xAA, 0xBB, 0xCC}; // 右下哨兵
    producer->publish_image(img, 7, 123456789ULL);
    const auto frame = consumer->recv_image();
    assert(frame.has_value());
    assert(frame->seq == 7);
    assert(frame->timestamp_ns == 123456789ULL);
    assert(frame->image.rows == static_cast<int>(ipc::IMAGE_HEIGHT));
    assert(frame->image.cols == static_cast<int>(ipc::IMAGE_WIDTH));
    assert(frame->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(0x11, 0x22, 0x33));
    assert(frame->image.at<cv::Vec3b>(ipc::IMAGE_HEIGHT - 1, ipc::IMAGE_WIDTH - 1)
           == cv::Vec3b(0xAA, 0xBB, 0xCC));
    assert(!consumer->recv_image().has_value()); // 消费后无新帧
    fmt::print("  [OK] publish_image→recv_image 像素抽检一致（seq 递增）\n");

    // --- 零拷贝验证：通过第三个映射直接改写共享内存像素，
    //     consumer 已持有的 Mat 无需重新 recv 即可看到变化 ---
    //     （不能跨映射比较指针：同一文件两次 mmap 得到不同虚拟地址）
    {
        auto pool = ipc::ShmRegion::open(ipc::SHM_NAME_IMAGE_POOL, ipc::IMAGE_POOL_SIZE);
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        assert(pool.has_value() && raw.has_value());
        // current()：consumer 刚消费的槽（read_idx 已写入共享内存，跨映射可见）
        ipc::ImageOps ops(&raw->as<ipc::ShmMetaRegion>()->image);
        const auto buffer_id = ops.current().buffer_id;
        // 直接改写图像池该槽首像素 → 若 Mat 是零拷贝，立即读到新值
        auto* pool_bytes = static_cast<uint8_t*>(pool->data());
        pool_bytes[buffer_id * ipc::IMAGE_SIZE] = 0xEE;
        assert(frame->image.at<cv::Vec3b>(0, 0)[0] == 0xEE);
        fmt::print("  [OK] 零拷贝：直接改写共享内存，已持有的 cv::Mat 立即可见\n");
    }

    // --- publish_pose 五路通道隔离 ---
    for (uint8_t ch = 0; ch < 5; ++ch) {
        const ipc::ShmClient::Pose pose{
            .x = 1.0 * ch, .y = 2.0 * ch, .z = 3.0 * ch,
            .qw = 1.0, .qx = 0.1 * ch, .qy = 0.2 * ch, .qz = 0.3 * ch,
            .frame_seq = 100U + ch, .timestamp_ns = 1000U + ch,
        };
        producer->publish_pose(static_cast<ipc::PoseIndex>(ch), pose);
    }
    for (uint8_t ch = 0; ch < 5; ++ch) {
        const auto pose = consumer->recv_pose(static_cast<ipc::PoseIndex>(ch));
        assert(pose.has_value());
        assert(pose->x == 1.0 * ch); // 每路数据互不串扰
        assert(pose->frame_seq == 100U + ch);
    }
    fmt::print("  [OK] 五路位姿通道隔离（GIMBAL/ODOM/MUZZLE/CAMERA/CHASSIS_OBS）\n");

    // --- send_gimbal_cmd → GimbalOps 读回（消费者写通道）---
    consumer->send_gimbal_cmd(15.0f, -8.0f, 3.5f, true);
    {
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        assert(raw.has_value());
        ipc::GimbalOps ops(&raw->as<ipc::ShmMetaRegion>()->gimbal_cmd);
        const auto cmd = ops.borrow();
        assert(cmd.has_value());
        assert((*cmd)->yaw_deg == 15.0f);
        assert((*cmd)->pitch_deg == -8.0f);
        assert((*cmd)->distance_m == 3.5f);
        assert((*cmd)->fire_advice == 1);
    }
    fmt::print("  [OK] send_gimbal_cmd→GimbalOps 读回字段一致\n");

    // --- 心跳保活 ---
    producer->update_heartbeat();
    assert(consumer->is_producer_alive()); // 心跳新鲜 → 存活
    assert(consumer->wait_for_producer(std::chrono::milliseconds(100))); // 立即返回
    {
        // 手动清零心跳（模拟生产者崩溃后残留的旧内存）→ 判定死亡
        auto raw = ipc::ShmRegion::open(ipc::SHM_NAME_META, sizeof(ipc::ShmMetaRegion));
        raw->as<ipc::ShmMetaRegion>()->header.heartbeat_ns = 0;
    }
    assert(!consumer->is_producer_alive());
    fmt::print("  [OK] 心跳保活：新鲜→存活，清零→死亡\n");

    // --- 析构清理：producer（owner）析构删除两块共享文件 ---
    producer = std::unexpected(ipc::ShmError::OpenFailed); // 先销毁 owner → 删文件
    consumer = std::unexpected(ipc::ShmError::OpenFailed);
}

// ============================================================================
// main：三层依次执行，全部断言通过输出总结
// ============================================================================
int main() {
    fmt::print("==== stage13 共享内存 IPC 单进程三层自测 ====\n");

    test_shm_region();     // 第一层：RAII 映射
    test_triple_buffer();  // 第二层：无锁三缓冲
    test_shm_client();     // 第三层：客户端全链路（含清理检查）

    test_end_no_leak();
    fmt::print("\n==== 全部通过：ShmRegion + TripleBuffer + ShmClient ====\n");
    return 0;
}
