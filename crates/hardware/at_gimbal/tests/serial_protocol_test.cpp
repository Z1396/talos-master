// ============================================================================
// talos_gimbal 下位机串口通信协议测试
//
// 被测对象（crates/hardware/at_gimbal/src/talos_gimbal/）：
//   packet.hpp    协议帧定义（SoF=0x5A ... EoF=0xA5，#pragma pack(1)）
//   stm32.hpp     Stm32Parser：按帧 ID 分发的静态解析器（0x01 IMU / 0x03 能力+简易IMU）
//   serial.hpp    SerialImpl：termios 串口 + try_parse_frame 帧同步状态机
//   mcu_device.hpp McuDeviceHandle：variant 统一设备句柄
//
// 测试策略（无真实 STM32 下位机，三层全覆盖）：
//   第一层 帧布局      —— 编译期 static_assert 锁定二进制协议内存布局
//                        （大小/偏移/字节序），防止字段增删悄悄破坏线协议
//   第二层 解析层      —— 直接向 Stm32Parser::parse 喂字节流，
//                        覆盖正常分发/短包/坏帧头/未知ID/截断帧
//   第三层 串口集成    —— 用 POSIX 伪终端（PTY）对模拟下位机：
//                        master 端扮演 STM32，slave 端被 SerialImpl 当真实
//                        /dev/ttyS4 打开，真实走 termios/poll/read/write 路径，
//                        覆盖收发回环、乱码重同步、半帧拼接、坏帧尾恢复
//
// 运行（项目根目录）：
//   cmake -B build -DTALOS_BUILD_TESTING=ON && cmake --build build
//   ./build/serial_protocol_test        # 或 ctest
// ============================================================================
#include <gtest/gtest.h>

// talos_gimbal 串口通信协议（被测头文件）
#include "talos_gimbal/mcu_device.hpp"
#include "talos_gimbal/packet.hpp"
#include "talos_gimbal/serial.hpp"
#include "talos_gimbal/stm32.hpp"

// C++ 标准库
#include <cstddef>     // std::byte
#include <cstdint>     // 固定宽度整型
#include <cstring>     // memcmp/memcpy
#include <string>      // 错误信息
#include <type_traits> // std::endian
#include <vector>      // 字节流容器

// POSIX 系统调用
#include <fcntl.h>    // open/O_RDWR
#include <poll.h>     // poll
#include <pty.h>      // posix_openpt/grantpt/unlockpt/ptsname
#include <unistd.h>   // read/write/close

namespace tg = talos_gimbal;

// ============================================================================
// 第一层：帧二进制布局测试（编译期协议锁定）
// ============================================================================
// 协议在 STM32（ARM 小端）与上位机（x86_64 小端）之间传输，两端必须同字节序
static_assert(std::endian::native == std::endian::little, "协议要求小端平台");

// 帧头：sof + len + id = 3 字节，pack(1) 不允许有填充
static_assert(sizeof(tg::HeaderFrame) == 3);
static_assert(offsetof(tg::HeaderFrame, sof) == 0);
static_assert(offsetof(tg::HeaderFrame, len) == 1);
static_assert(offsetof(tg::HeaderFrame, id) == 2);

// 帧起始/结束魔数固定值（改动即破坏与存量下位机的兼容性）
static_assert(tg::HeaderFrame::SoF() == 0x5A);
static_assert(tg::HeaderFrame::EoF() == 0xA5);

// 帧ID 0x01 完整IMU帧：3 + 4 + (Color 1 + 7*float 28) + 1 = 37
static_assert(sizeof(tg::ReceiveImuData) == 37);
static_assert(offsetof(tg::ReceiveImuData, time_stamp) == 3);
static_assert(offsetof(tg::ReceiveImuData, data) == 7);
static_assert(offsetof(tg::ReceiveImuData, eof) == 36);

// 帧ID 0x03 能力开关帧：3 + 4 + 3 + 1 = 11
static_assert(sizeof(tg::ReceiveCapabilitiesData) == 11);
static_assert(offsetof(tg::ReceiveCapabilitiesData, eof) == 10);

// 简易IMU帧：3 + 4 + (Color 1 + 3*float 12) + 1 = 21
static_assert(sizeof(tg::ReceiveSimpleImuData) == 21);
static_assert(offsetof(tg::ReceiveSimpleImuData, eof) == 20);

// Quanta 图传帧：3 + 4 + (2 + 298) + 1 = 308
static_assert(sizeof(tg::SendQuantaData) == 308);

// 视觉预测帧（发送方向，无时间戳）：3 + (bool 1 + 7*float 28) + 1 = 33
static_assert(sizeof(tg::SendVisionData) == 33);

// 极简视觉帧：3 + 4 + 1 = 8
static_assert(sizeof(tg::SendSimpleVisionData) == 8);

TEST(FrameLayout, Constants) {
    // 运行时再验证一次魔数（编译期断言之外的保险）
    EXPECT_EQ(tg::HeaderFrame::SoF(), 0x5A);
    EXPECT_EQ(tg::HeaderFrame::EoF(), 0xA5);
}

// ============================================================================
// 测试工具：构造合法协议帧
// ============================================================================

/// 帧头 len 字段的语义：载荷字节数（不含帧头3 + 时间戳4 + 帧尾1）
/// 与 SerialImpl::try_parse_frame 的 frame_size = 3 + 4 + len + 1 公式对齐
template <typename Frame>
constexpr uint8_t payload_len() {
    return sizeof(Frame) - sizeof(tg::HeaderFrame) - sizeof(uint32_t) - 1;
}

/// 构造一份字段确定可断言的完整 IMU 帧
static tg::ReceiveImuData make_imu_frame(const uint32_t stamp, const float yaw) {
    tg::ReceiveImuData frame{};
    frame.header.sof = tg::HeaderFrame::SoF();
    frame.header.len = payload_len<tg::ReceiveImuData>();
    frame.header.id  = 0x01;
    frame.time_stamp = stamp;
    frame.data.self_color   = tg::Color::Blue;
    frame.data.bullet_speed = 15.5f;
    frame.data.yaw           = yaw;
    frame.data.pitch         = 0.02f;
    frame.data.roll          = -0.03f;
    frame.data.yaw_vel       = 1.25f;
    frame.data.pitch_vel     = -2.5f;
    frame.data.roll_vel      = 0.5f;
    frame.eof                = tg::HeaderFrame::EoF();
    return frame;
}

/// 任意结构体 → 字节流（两端同为小端 + pack(1)，bit 级一致）
template <typename T>
static std::vector<uint8_t> to_bytes(const T& value) {
    std::vector<uint8_t> bytes(sizeof(T));
    std::memcpy(bytes.data(), &value, sizeof(T));
    return bytes;
}

/// 字节流 → span<std::byte>（Stm32Parser::parse 的入参类型）
static std::span<const std::byte> as_span(const std::vector<uint8_t>& bytes) {
    return {reinterpret_cast<const std::byte*>(bytes.data()), bytes.size()};
}

// ============================================================================
// 测试工具：PTY 伪终端对（模拟下位机串口）
// ============================================================================
// master 端 = 测试代码（扮演 STM32），slave 端 = SerialImpl 连接的"串口设备"。
// 数据链路完整经过内核 termios 分层，是无需硬件的最真实集成测试。
class PtyPair {
public:
    PtyPair() {
        // 打开 PTY master：读写 | 不作为控制终端
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        // 分配 slave 权限并解锁，获取 slave 路径（/dev/pts/N）
        if (::grantpt(master_) == 0 && ::unlockpt(master_) == 0) {
            slave_path_ = ::ptsname(master_);
        }
    }

    ~PtyPair() {
        if (master_ >= 0) {
            // master 关闭会向 slave 发送 EOF/HUP
            ::close(master_);
        }
    }

    // 禁止拷贝/移动：持有裸 fd，RAII 管理
    PtyPair(const PtyPair&)            = delete;
    PtyPair& operator=(const PtyPair&) = delete;

    /// slave 设备路径（传给 SerialImpl::connect 的 device_path）
    [[nodiscard]] const std::string& slave_path() const { return slave_path_; }

    /// 下位机 → 上位机：向 master 写数据（模拟 STM32 发帧）
    bool write_master(const void* data, const size_t size) {
        return ::write(master_, data, size) == static_cast<ssize_t>(size);
    }

    /// 上位机 → 下位机：从 master 读数据（接收上位机下发指令），带超时
    std::vector<uint8_t> read_master(const size_t size, const int timeout_ms = 500) {
        std::vector<uint8_t> buffer(size);
        size_t got = 0;
        while (got < size) {
            struct pollfd pfd{.fd = master_, .events = POLLIN, .revents = 0};
            if (::poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN)) {
                break; // 超时：返回已读部分
            }
            const auto n = ::read(master_, buffer.data() + got, size - got);
            if (n <= 0) {
                break;
            }
            got += static_cast<size_t>(n);
        }
        buffer.resize(got);
        return buffer;
    }

private:
    int         master_    = -1;
    std::string slave_path_;
};

// ============================================================================
// 第二层：Stm32Parser 解析层测试（纯逻辑，无 IO）
// ============================================================================
class Stm32ParserTest : public ::testing::Test {
protected:
    // 解析器通过静态指针绑定输出存储，绑定到 fixture 成员
    void SetUp() override {
        tg::Stm32Parser::latest_imu          = &imu_;
        tg::Stm32Parser::latest_capabilities = &caps_;
    }

    void TearDown() override {
        tg::Stm32Parser::latest_imu          = nullptr;
        tg::Stm32Parser::latest_capabilities = nullptr;
    }

    tg::ReceiveImuData          imu_{}; // 输出槽：清零初始化
    tg::ReceiveCapabilitiesData caps_{};
};

/// 正常流程：合法 IMU 帧（id=0x01）逐字段完整解析
TEST_F(Stm32ParserTest, ParsesValidImuFrame) {
    const auto frame = make_imu_frame(123456u, 1.5f);
    tg::Stm32Parser::parse(as_span(to_bytes(frame)));

    // 帧头 + 时间戳 + 全部姿态字段 bit 级一致
    EXPECT_EQ(memcmp(&imu_.header, &frame.header, sizeof(frame.header)), 0);
    EXPECT_EQ(imu_.time_stamp, 123456u);
    EXPECT_EQ(imu_.data.self_color, tg::Color::Blue);
    EXPECT_FLOAT_EQ(imu_.data.bullet_speed, 15.5f);
    EXPECT_FLOAT_EQ(imu_.data.yaw, 1.5f);
    EXPECT_FLOAT_EQ(imu_.data.pitch, 0.02f);
    EXPECT_FLOAT_EQ(imu_.data.roll, -0.03f);
    EXPECT_FLOAT_EQ(imu_.data.yaw_vel, 1.25f);
    EXPECT_FLOAT_EQ(imu_.data.pitch_vel, -2.5f);
    EXPECT_FLOAT_EQ(imu_.data.roll_vel, 0.5f);
    EXPECT_EQ(imu_.eof, tg::HeaderFrame::EoF());
}

/// 正常流程：能力开关帧（id=0x03，size==11）只填充 capabilities
TEST_F(Stm32ParserTest, ParsesValidCapabilitiesFrame) {
    tg::ReceiveCapabilitiesData frame{};
    frame.header.sof = tg::HeaderFrame::SoF();
    frame.header.len = payload_len<tg::ReceiveCapabilitiesData>();
    frame.header.id  = 0x03;
    frame.time_stamp = 42u;
    frame.data.following = 1;
    frame.data.power_rune = 0;
    frame.data.quanta     = 1;
    frame.eof             = tg::HeaderFrame::EoF();

    tg::Stm32Parser::parse(as_span(to_bytes(frame)));

    EXPECT_EQ(caps_.time_stamp, 42u);
    EXPECT_EQ(caps_.data.following, 1);
    EXPECT_EQ(caps_.data.power_rune, 0);
    EXPECT_EQ(caps_.data.quanta, 1);
    // 能力帧（11 字节）不满足简易IMU分支的 size>=21，不得污染 imu_ 输出槽
    EXPECT_EQ(imu_.time_stamp, 0u);
}

/// 正常流程：简易IMU帧（id=0x03，size==21）合并进完整IMU结构
/// 缺失字段（弹速/三轴角速度）按协议约定清零
TEST_F(Stm32ParserTest, MergesSimpleImuIntoLatestImu) {
    // 先灌一份完整 IMU，验证简易帧会覆盖姿态、清零速度
    const auto full = make_imu_frame(1u, 0.1f);
    tg::Stm32Parser::parse(as_span(to_bytes(full)));
    ASSERT_FLOAT_EQ(imu_.data.yaw_vel, 1.25f);

    tg::ReceiveSimpleImuData simple{};
    simple.header.sof = tg::HeaderFrame::SoF();
    simple.header.len = payload_len<tg::ReceiveSimpleImuData>();
    simple.header.id  = 0x03;
    simple.time_stamp = 999u;
    simple.data.self_color = tg::Color::Red;
    simple.data.yaw        = 0.5f;
    simple.data.pitch      = -0.25f;
    simple.data.roll       = 0.125f;
    simple.eof             = tg::HeaderFrame::EoF();

    tg::Stm32Parser::parse(as_span(to_bytes(simple)));

    EXPECT_EQ(imu_.time_stamp, 999u);
    EXPECT_EQ(imu_.data.self_color, tg::Color::Red);
    EXPECT_FLOAT_EQ(imu_.data.yaw, 0.5f);
    EXPECT_FLOAT_EQ(imu_.data.pitch, -0.25f);
    EXPECT_FLOAT_EQ(imu_.data.roll, 0.125f);
    // 简易帧缺失字段被显式清零（不是保留旧值）
    EXPECT_FLOAT_EQ(imu_.data.bullet_speed, 0.0f);
    EXPECT_FLOAT_EQ(imu_.data.yaw_vel, 0.0f);
    EXPECT_FLOAT_EQ(imu_.data.pitch_vel, 0.0f);
    EXPECT_FLOAT_EQ(imu_.data.roll_vel, 0.0f);
}

/// 异常处理：小于帧头的短包直接丢弃，不越界、无副作用
TEST_F(Stm32ParserTest, DropsTooSmallFrame) {
    const std::vector<uint8_t> garbage{0x5A, 0x00}; // 2 字节 < sizeof(HeaderFrame)
    tg::Stm32Parser::parse(as_span(garbage));

    EXPECT_EQ(imu_.time_stamp, 0u);
    EXPECT_EQ(caps_.time_stamp, 0u);
}

/// 异常处理：帧头魔数错误整帧丢弃
TEST_F(Stm32ParserTest, DropsInvalidSoF) {
    auto       frame = make_imu_frame(77u, 0.7f);
    const auto bytes = to_bytes(frame);
    auto       bad   = bytes;
    bad[0]            = 0x00; // 篡改 SoF

    tg::Stm32Parser::parse(as_span(bad));
    EXPECT_EQ(imu_.time_stamp, 0u);

    // 对照：合法帧正常解析
    tg::Stm32Parser::parse(as_span(bytes));
    EXPECT_EQ(imu_.time_stamp, 77u);
}

/// 异常处理：未知帧 ID 静默忽略（前向兼容：下位机新增帧不崩溃）
TEST_F(Stm32ParserTest, IgnoresUnknownFrameId) {
    auto frame = make_imu_frame(88u, 0.7f);
    frame.header.id = 0x42; // 未定义的帧ID

    tg::Stm32Parser::parse(as_span(to_bytes(frame)));
    EXPECT_EQ(imu_.time_stamp, 0u);
    EXPECT_EQ(caps_.time_stamp, 0u);
}

/// 边界条件：IMU 帧长度不足 sizeof(ReceiveImuData) 时不更新（截断保护）
TEST_F(Stm32ParserTest, KeepsOldValueOnTruncatedImuFrame) {
    // 先解析一帧合法数据
    tg::Stm32Parser::parse(as_span(to_bytes(make_imu_frame(100u, 0.3f))));
    ASSERT_EQ(imu_.time_stamp, 100u);

    // 再喂入截断帧（长度 -1 不满足 >= sizeof 分支）
    auto bytes = to_bytes(make_imu_frame(200u, 0.9f));
    bytes.resize(bytes.size() - 1);
    tg::Stm32Parser::parse(as_span(bytes));

    // 旧值保留，未被截断数据污染
    EXPECT_EQ(imu_.time_stamp, 100u);
}

/// 边界条件：载荷中包含 0x5A/0xA5 魔数字节不受影响（长度由 len 字段决定）
TEST_F(Stm32ParserTest, ToleratesMagicBytesInsidePayload) {
    auto frame = make_imu_frame(555u, 0.25f);
    // 弹速字段位模式注入 0x5A5A5A5A（位模式含 SoF 魔数的合法规格化 float）
    uint32_t magic = 0x5A5A5A5Au;
    std::memcpy(&frame.data.bullet_speed, &magic, sizeof(magic));
    float expected = 0.0f;
    std::memcpy(&expected, &magic, sizeof(expected)); // 同一位模式的期望 float

    tg::Stm32Parser::parse(as_span(to_bytes(frame)));

    EXPECT_EQ(imu_.time_stamp, 555u);
    EXPECT_FLOAT_EQ(imu_.data.bullet_speed, expected);
}

// ============================================================================
// 第三层：SerialImpl 串口集成测试（PTY 模拟下位机）
// ============================================================================
class SerialImplTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_FALSE(pty_.slave_path().empty()) << "PTY 创建失败";
        tg::Stm32Parser::latest_imu          = &imu_;
        tg::Stm32Parser::latest_capabilities = &caps_;
    }

    void TearDown() override {
        tg::Stm32Parser::latest_imu          = nullptr;
        tg::Stm32Parser::latest_capabilities = nullptr;
    }

    PtyPair                    pty_; // master=测试(下位机侧)，slave=被测串口
    tg::ReceiveImuData          imu_{};
    tg::ReceiveCapabilitiesData caps_{};
};

/// 异常处理：连接不存在的设备路径 → expected 错误携带路径与 errno 描述
TEST_F(SerialImplTest, ConnectFailureReturnsError) {
    tg::SerialDevice device;
    const auto result = device.connect("/dev/does-not-exist-talos-test", 115200);

    ASSERT_FALSE(result.has_value());
    // 错误信息包含设备路径与 "open serial device" 前缀，便于现场排障
    EXPECT_NE(result.error().find("open serial device"), std::string::npos);
    EXPECT_NE(result.error().find("/dev/does-not-exist-talos-test"), std::string::npos);
    EXPECT_FALSE(device.is_connected());
}

/// 正常流程：PTY 上连接成功 + 断开状态翻转
TEST_F(SerialImplTest, ConnectAndDisconnect) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());
    EXPECT_TRUE(device.is_connected());

    device.disconnect();
    EXPECT_FALSE(device.is_connected());

    // 断开后收发均为安全空操作/错误，不崩溃
    device.handle_events();
    const uint8_t byte = 0x00;
    EXPECT_FALSE(device.send_sync(&byte, 1).has_value());
}

/// 异常处理：未连接时 send_sync → "device not connected"
TEST_F(SerialImplTest, SendSyncFailsWhenNotConnected) {
    tg::SerialDevice device;
    const uint8_t    data[] = {0x5A, 0x01, 0x01};

    const auto result = device.send_sync(data, sizeof(data));
    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("device not connected"), std::string::npos);
}

/// 正常流程：上位机 → 下位机发送回环（send_sync → PTY master 收到相同字节）
TEST_F(SerialImplTest, SendSyncRoundTrip) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 构造视觉预测指令帧（上位机 → MCU 的业务载荷）
    tg::SendVisionData vision{};
    vision.header.sof = tg::HeaderFrame::SoF();
    vision.header.id  = 0x02;
    vision.data.fire_advice = true;
    vision.data.target_yaw   = 1.234f;
    vision.data.target_pitch = -0.567f;
    vision.data.distance     = 6.28f;

    const auto sent = to_bytes(vision);
    ASSERT_TRUE(device.send_sync(sent.data(), sent.size()).has_value());

    // 下位机侧（PTY master）收到的字节与发送端逐字节一致
    const auto received = pty_.read_master(sent.size());
    ASSERT_EQ(received.size(), sent.size());
    EXPECT_EQ(memcmp(received.data(), sent.data(), sent.size()), 0);

    // 边界：零长度发送是合法空操作
    EXPECT_TRUE(device.send_sync(nullptr, 0).has_value());
}

/// 正常流程：下位机 → 上位机接收分发（master 写帧 → handle_events 解析）
TEST_F(SerialImplTest, ReceiveDispatchesImuFrame) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    const auto frame = make_imu_frame(4242u, 0.75f);
    const auto bytes = to_bytes(frame);
    ASSERT_TRUE(pty_.write_master(bytes.data(), bytes.size()));

    device.handle_events();

    EXPECT_EQ(imu_.time_stamp, 4242u);
    EXPECT_FLOAT_EQ(imu_.data.yaw, 0.75f);
    EXPECT_EQ(imu_.eof, tg::HeaderFrame::EoF());
}

/// 异常恢复：帧前乱码被丢弃，后续帧正常重同步解析
TEST_F(SerialImplTest, ResyncAfterLeadingGarbage) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    const auto frame = make_imu_frame(111u, 0.1f);
    const auto bytes = to_bytes(frame);

    // 纯垃圾（无 SoF）→ 整段清空
    const uint8_t garbage[] = {0x11, 0x22, 0x33, 0x44};
    ASSERT_TRUE(pty_.write_master(garbage, sizeof(garbage)));
    // 乱码 + 合法帧连写 → 乱码丢弃后帧正常解析
    std::vector<uint8_t> stream(garbage, garbage + sizeof(garbage));
    stream.insert(stream.end(), bytes.begin(), bytes.end());
    ASSERT_TRUE(pty_.write_master(stream.data(), stream.size()));

    device.handle_events();

    EXPECT_EQ(imu_.time_stamp, 111u);
}

/// 边界条件：帧尾字节错误 → 该帧丢弃，后续帧恢复解析
TEST_F(SerialImplTest, RecoversAfterBadEof) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    auto       bad    = to_bytes(make_imu_frame(66u, 0.1f));
    bad.back()        = 0x00; // 篡改 EoF
    const auto good   = to_bytes(make_imu_frame(77u, 0.2f));

    std::vector<uint8_t> stream = bad;
    stream.insert(stream.end(), good.begin(), good.end());
    ASSERT_TRUE(pty_.write_master(stream.data(), stream.size()));

    device.handle_events();

    // 坏帧丢弃，好帧照常解析
    EXPECT_EQ(imu_.time_stamp, 77u);
    EXPECT_EQ(imu_.eof, tg::HeaderFrame::EoF());
}

/// 边界条件：帧分多次到达（半帧 → handle_events 无解析 → 补齐后解析成功）
TEST_F(SerialImplTest, HandlesSplitFrameArrival) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    const auto bytes = to_bytes(make_imu_frame(888u, 0.15f));

    // 前半帧到达：长度不足，缓存等待，不误解析
    const size_t half = bytes.size() / 2;
    ASSERT_TRUE(pty_.write_master(bytes.data(), half));
    device.handle_events();
    EXPECT_EQ(imu_.time_stamp, 0u);

    // 后半帧补齐：拼成完整帧解析成功
    ASSERT_TRUE(pty_.write_master(bytes.data() + half, bytes.size() - half));
    device.handle_events();
    EXPECT_EQ(imu_.time_stamp, 888u);
}

/// 正常流程：一次读取包含多帧，循环解析全部消费
TEST_F(SerialImplTest, ParsesMultipleFramesInOneRead) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 两帧 IMU + 一帧能力开关混合写入（时间戳递增，验证最终值为最后一帧）
    const auto f1 = to_bytes(make_imu_frame(1u, 0.05f));
    const auto f2 = to_bytes(make_imu_frame(2u, 0.10f));

    tg::ReceiveCapabilitiesData caps_frame{};
    caps_frame.header.sof = tg::HeaderFrame::SoF();
    caps_frame.header.len = payload_len<tg::ReceiveCapabilitiesData>();
    caps_frame.header.id  = 0x03;
    caps_frame.time_stamp = 3u;
    caps_frame.data.following = 1;
    caps_frame.data.quanta     = 1;
    caps_frame.eof             = tg::HeaderFrame::EoF();
    const auto f3 = to_bytes(caps_frame);

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    stream.insert(stream.end(), f3.begin(), f3.end());
    ASSERT_TRUE(pty_.write_master(stream.data(), stream.size()));

    device.handle_events();

    // 两帧 IMU 顺序解析，保留最后一帧
    EXPECT_EQ(imu_.time_stamp, 2u);
    EXPECT_FLOAT_EQ(imu_.data.yaw, 0.10f);
    // 能力帧同时解析成功
    EXPECT_EQ(caps_.time_stamp, 3u);
    EXPECT_EQ(caps_.data.following, 1);
    EXPECT_EQ(caps_.data.quanta, 1);
}

/// 边界条件：len 字段声明超过实际到达数据 → 等待补齐，不越界
TEST_F(SerialImplTest, WaitsForDeclaredFrameLength) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    auto frame = make_imu_frame(999u, 0.2f);
    // len 声明比真实载荷大：串口层会按更长帧等待，不解析不越界
    frame.header.len = payload_len<tg::ReceiveImuData>() + 10;
    const auto bytes = to_bytes(frame);

    ASSERT_TRUE(pty_.write_master(bytes.data(), bytes.size()));
    device.handle_events();
    // 帧不完整：既不解析，也不崩溃
    EXPECT_EQ(imu_.time_stamp, 0u);

    // 补齐多声明的 10 字节 + EoF 后仍未对齐（协议帧尾错位）→ 仍安全
    std::vector<uint8_t> extra(10, 0x00);
    ASSERT_TRUE(pty_.write_master(extra.data(), extra.size()));
    device.handle_events();
    // 无崩溃即为通过（EoF 校验失败按坏帧丢弃）
    SUCCEED();
}

/// 正常流程：长时间运行场景下缓存不无限增长（每帧消费后缓存清空）
TEST_F(SerialImplTest, RxBufferDrainsAfterParse) {
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 连续写入多帧，全部被消费
    for (uint32_t stamp = 0; stamp < 20; ++stamp) {
        const auto bytes = to_bytes(make_imu_frame(stamp, 0.01f));
        ASSERT_TRUE(pty_.write_master(bytes.data(), bytes.size()));
        device.handle_events();
        ASSERT_EQ(imu_.time_stamp, stamp);
    }
    EXPECT_EQ(imu_.time_stamp, 19u);
}

// ============================================================================
// 第四层：McuDeviceHandle 统一设备句柄测试
// ============================================================================
/// 异常处理：create_serial 失败 → 错误信息含路径与波特率上下文
TEST(McuDeviceHandleTest, CreateSerialFailureIncludesContext) {
    const auto result = tg::McuDeviceHandle::create_serial("/dev/does-not-exist-talos-test", 115200);

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("connect serial mcu"), std::string::npos);
    EXPECT_NE(result.error().find("baud_rate=115200"), std::string::npos);
}

/// 正常流程：create_serial 成功 → is_serial / is_connected / 收发回环
TEST(McuDeviceHandleTest, CreateSerialOverPty) {
    PtyPair pty;
    ASSERT_FALSE(pty.slave_path().empty());

    auto result = tg::McuDeviceHandle::create_serial(pty.slave_path(), 115200);
    ASSERT_TRUE(result.has_value()) << result.error();
    auto& handle = *result;

    // variant 分发：确认为串口设备且已连接
    EXPECT_TRUE(handle.is_serial());
    EXPECT_TRUE(handle.is_connected());

    // 句柄统一 send_sync 下发指令，PTY master 侧收到
    tg::SendSimpleVisionData simple{};
    simple.header.sof = tg::HeaderFrame::SoF();
    simple.header.id  = 0x04;
    simple.data.target_yaw = -0.5f;
    const auto bytes = to_bytes(simple);

    ASSERT_TRUE(handle.send_sync(bytes.data(), bytes.size()).has_value());
    const auto received = pty.read_master(bytes.size());
    ASSERT_EQ(received.size(), bytes.size());
    EXPECT_EQ(memcmp(received.data(), bytes.data(), bytes.size()), 0);

    // 句柄统一 handle_events 收帧解析
    tg::ReceiveImuData imu{};
    tg::Stm32Parser::latest_imu = &imu;
    const auto frame = to_bytes(make_imu_frame(31337u, 0.42f));
    ASSERT_TRUE(pty.write_master(frame.data(), frame.size()));
    handle.handle_events();
    tg::Stm32Parser::latest_imu = nullptr;

    EXPECT_EQ(imu.time_stamp, 31337u);
    EXPECT_FLOAT_EQ(imu.data.yaw, 0.42f);
}

/// 生命周期：句柄可移动转移所有权（variant + unique_ptr 移动语义）
TEST(McuDeviceHandleTest, HandleIsMovable) {
    PtyPair pty;
    ASSERT_FALSE(pty.slave_path().empty());

    auto result = tg::McuDeviceHandle::create_serial(pty.slave_path(), 115200);
    ASSERT_TRUE(result.has_value());

    // 移动构造：新句柄接管连接，旧句柄失效
    tg::McuDeviceHandle moved = std::move(*result);
    EXPECT_TRUE(moved.is_connected());
    EXPECT_TRUE(moved.is_serial());
}
