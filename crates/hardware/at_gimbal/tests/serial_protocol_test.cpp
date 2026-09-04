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
#include "talos_gimbal/stm32.hpp"

// C++ 标准库
#include <bit>     // std::endian 字节序检测
#include <cstddef> // std::byte
#include <cstdint> // 固定宽度整型
#include <cstring> // memcmp/memcpy
#include <string>  // 错误信息
#include <vector>  // 字节流容器

// POSIX 系统调用
#include <fcntl.h>  // open/O_RDWR
#include <poll.h>   // poll
#include <pty.h>    // posix_openpt/grantpt/unlockpt/ptsname
#include <unistd.h> // read/write/close

namespace tg = talos_gimbal;

// ============================================================================
// 第一层：帧二进制布局测试（编译期协议锁定）
// ============================================================================
// 协议在 STM32（ARM 小端）与上位机（x86_64 小端）之间传输，两端必须同字节序
static_assert(std::endian::native == std::endian::little, "协议要求小端平台");

// 帧头：sof + len + id = 3 字节，pack(1) 不允许有填充
/*如果断言失败会发生什么？
error: static assertion failed
note: the comparison reduces to '4 == 3'
编译立即停止，防止生成与 STM32 下位机不兼容的可执行文件。*/
static_assert(sizeof(tg::HeaderFrame) == 3);
/*# offsetof
#include <cstddef>
**`offsetof(T, member)`**：得到结构体成员相对于结构体起始地址的**字节偏移量（size_t）**。
> 是宏，不是函数；编译期可以求值（普通标准布局类型）。
struct S {
    char a;     // 0
    int  b;     // 4  (因为对齐，a占1，填充3字节)
    short c;    // 8
};
// 获取成员偏移
size_t off_a = offsetof(S, a); // 0
size_t off_b = offsetof(S, b); // 4
size_t off_c = offsetof(S, c); // 8
内存布局：
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| a | pad | pad | pad | b(0‑3) | b | b | b | c | c |
`offsetof(S,b)` = 4：从 `S*` 起始地址，加 4 字节才到 `b`。*/
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
    frame.header.sof        = tg::HeaderFrame::SoF();
    frame.header.len        = payload_len<tg::ReceiveImuData>();
    frame.header.id         = 0x01;
    frame.time_stamp        = stamp;
    frame.data.self_color   = tg::Color::Blue;
    frame.data.bullet_speed = 15.5f;
    frame.data.yaw          = yaw;
    frame.data.pitch        = 0.02f;
    frame.data.roll         = -0.03f;
    frame.data.yaw_vel      = 1.25f;
    frame.data.pitch_vel    = -2.5f;
    frame.data.roll_vel     = 0.5f;
    frame.eof               = tg::HeaderFrame::EoF();
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
/*std::span<const std::byte>	C++20 连续内存视图，只读字节序列
reinterpret_cast<const std::byte*>	将 const uint8_t* 强制转为 const std::byte*
bytes.data()	获取 vector 内部数组首地址
bytes.size()	元素个数（字节数）*/
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
        /*- 打开一个 PTY master 伪终端。
        - `O_RDWR`：可读可写。
        - `O_NOCTTY`：**不要把这个 PTY 设置成当前进程的控制终端**（非常关键，否则信号行为会错乱）。
        - 返回值：成功返回 master fd；失败返回 `-1`。
        > ⚠️ 代码这里没有判断 `master_ < 0` 的错误处理，生产要补；测试环境一般没问题。*/
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        /*2. **`grantpt(int fd)`**
        int grantpt(int fd);
        设置 PTY‑slave 设备文件权限、属主。必须调用，否则 slave 打不开。返回 0 成功，‑1 失败。

        3. **`unlockpt(int fd)`**
        int unlockpt(int fd);
        解锁 slave 设备。**在使用 slave 之前必须解锁**。grantpt 之后一定要 unlockpt。返回 0 成功。

        4. **`ptsname(int fd)`**
        char* ptsname(int fd);
        返回 master 对应的 slave 设备路径字符串，例如 `"/dev/pts/5"`。
        > 返回的指针是库内部静态缓冲区，**必须立刻拷贝出来**，所以代码存入 `slave_path_`。
        > 调用顺序硬性规定：
        > `posix_openpt()` → `grantpt()` → `unlockpt()` → `ptsname()`，顺序不能乱。*/
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
    bool write_master(const void* data, const size_t size) const {
        return ::write(master_, data, size) == static_cast<ssize_t>(size);
    }

    /// 上位机 → 下位机：从 master 读数据（接收上位机下发指令），带超时
    std::vector<uint8_t> read_master(const size_t size, const int timeout_ms = 500) {
        // 预分配一块缓冲区，最大准备接收 size 字节；vector初始全部填0
        std::vector<uint8_t> buffer(size);

        // got：记录已经成功读到多少字节，初始0
        size_t got = 0;

        // 循环：还没有读到期望的size字节就继续循环
        while (got < size)
        {
            // 定义pollfd结构体，用来告诉poll要监听哪个fd、关心什么事件
            // fd：要监听的文件描述符，这里是pty master的fd
            // events：我们关心的事件，POLLIN = 有数据可读
            // revents：输出参数，poll系统调用返回后，由内核填充实际发生的事件
            struct pollfd pfd{.fd = master_, .events = POLLIN, .revents = 0};

            // poll(fds数组, fds数量, 超时毫秒)
            // 返回值：>0 有就绪事件；0 超时；-1 系统调用出错
            // 条件判断：poll<=0 代表超时/出错；或者revents里面没有POLLIN（没有可读数据）
            if (::poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN))
            {
                break; // 超时 / 出错 / 没有可读数据，直接跳出读循环，返回目前已经读到的部分
            }

            // poll确认有数据可读，调用read系统调用读取字节
            // buffer.data() + got：写到buffer里面“已经读到位置”的后面，追加写入
            // size - got：剩余还想要读的字节数（缓冲区剩余空间）
            // 返回n：实际读到的字节数；n>0正常读到；n=0代表EOF对端挂断；n=-1出错
            const auto n = ::read(master_, buffer.data() + got, size - got);

            // n <= 0：EOF或者系统调用出错，直接退出循环，不再继续读
            if (n <= 0)
            {
                break;
            }

            // 累加已经读到的字节数量，n是ssize_t有符号，转成无符号size_t存入got
            got += static_cast<size_t>(n);
        }

        // buffer原本分配了size大小；现在resize裁剪为实际读到got字节。
        // 如果什么都没读到 got=0，返回空vector。
        buffer.resize(got);

        // 返回读到的数据（可能不足size字节，超时就返回已读到的片段）
        return buffer;
    }


private:
    int master_ = -1;
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

    tg::ReceiveImuData imu_{}; // 输出槽：清零初始化
    tg::ReceiveCapabilitiesData caps_{};
};

// ============================================================================
// 测试套件：Stm32ParserTest
// 测试目标：Stm32Parser 静态解析器的帧解析逻辑
// 测试策略：直接向解析器喂入字节流，验证解析结果
// 测试环境：无真实硬件，纯内存操作
// ============================================================================

// ----------------------------------------------------------------------------
// 测试用例 1：正常流程 - 完整 IMU 帧（ID=0x01）逐字段解析
// 验证解析器能正确解析完整 IMU 帧的所有字段
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, ParsesValidImuFrame) {
    // 1. 构造一个完整的 IMU 帧
    //    make_imu_frame() 创建 ReceiveImuData 结构体，填充：
    //    - 帧头：SoF=0x5A, len=30, id=0x01
    //    - 时间戳：123456
    //    - 颜色：Blue
    //    - 弹速：15.5 m/s
    //    - 姿态：yaw=1.5, pitch=0.02, roll=-0.03
    //    - 角速度：yaw_vel=1.25, pitch_vel=-2.5, roll_vel=0.5
    //    - 帧尾：EoF=0xA5
    const auto frame = make_imu_frame(123456u, 1.5f);
    
    // 2. 序列化 + 解析
    //    to_bytes(frame)    ：结构体 → vector<uint8_t> 字节流
    //    as_span(...)       ：vector<uint8_t> → span<const std::byte>
    //    Stm32Parser::parse ：解析字节流，结果存入 imu_（通过全局指针）
    tg::Stm32Parser::parse(as_span(to_bytes(frame)));

    // 3. 验证：帧头完全一致
    //    memcmp 比较 imu_.header 和 frame.header 的内存
    //    返回 0 表示完全相同（3 字节：sof, len, id）
    EXPECT_EQ(memcmp(&imu_.header, &frame.header, sizeof(frame.header)), 0);
    
    // 4. 验证：时间戳正确
    EXPECT_EQ(imu_.time_stamp, 123456u);
    
    // 5. 验证：颜色正确（Blue = 1）
    EXPECT_EQ(imu_.data.self_color, tg::Color::Blue);
    
    // 6. 验证：弹速正确（使用 EXPECT_FLOAT_EQ 处理浮点误差）
    EXPECT_FLOAT_EQ(imu_.data.bullet_speed, 15.5f);
    
    // 7. 验证：偏航角正确
    EXPECT_FLOAT_EQ(imu_.data.yaw, 1.5f);
    
    // 8. 验证：俯仰角正确
    EXPECT_FLOAT_EQ(imu_.data.pitch, 0.02f);
    
    // 9. 验证：横滚角正确
    EXPECT_FLOAT_EQ(imu_.data.roll, -0.03f);
    
    // 10. 验证：偏航角速度正确
    EXPECT_FLOAT_EQ(imu_.data.yaw_vel, 1.25f);
    
    // 11. 验证：俯仰角速度正确
    EXPECT_FLOAT_EQ(imu_.data.pitch_vel, -2.5f);
    
    // 12. 验证：横滚角速度正确
    EXPECT_FLOAT_EQ(imu_.data.roll_vel, 0.5f);
    
    // 13. 验证：帧尾魔数正确
    EXPECT_EQ(imu_.eof, tg::HeaderFrame::EoF());
}


// ----------------------------------------------------------------------------
// 测试用例 2：正常流程 - 能力开关帧（ID=0x03，size=11）
// 验证解析器能正确解析能力开关帧，且不污染 IMU 输出槽
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, ParsesValidCapabilitiesFrame) {
    // 1. 构造能力开关帧
    //    帧 ID 0x03，总大小 11 字节
    //    包含：帧头(3) + 时间戳(4) + 能力标志(3) + 帧尾(1)
    tg::ReceiveCapabilitiesData frame{};
    frame.header.sof      = tg::HeaderFrame::SoF();  // 0x5A
    frame.header.len      = payload_len<tg::ReceiveCapabilitiesData>();  // 4 字节载荷
    frame.header.id       = 0x03;                    // 能力帧 ID
    frame.time_stamp      = 42u;                     // 时间戳
    frame.data.following  = 1;                       // 跟随模式：开启
    frame.data.power_rune = 0;                       // 能量机关：关闭
    frame.data.quanta     = 1;                       // 图传模式：开启
    frame.eof             = tg::HeaderFrame::EoF();  // 0xA5

    // 2. 序列化并解析
    tg::Stm32Parser::parse(as_span(to_bytes(frame)));

    // 3. 验证：能力帧的输出槽 caps_ 被正确填充
    EXPECT_EQ(caps_.time_stamp, 42u);
    EXPECT_EQ(caps_.data.following, 1);
    EXPECT_EQ(caps_.data.power_rune, 0);
    EXPECT_EQ(caps_.data.quanta, 1);
    
    // 4. 验证：能力帧不会污染 IMU 输出槽 imu_
    //    因为能力帧只有 11 字节，不满足简易 IMU 的 size>=21 条件
    //    所以 imu_ 应保持初始值 0
    EXPECT_EQ(imu_.time_stamp, 0u);
}


// ----------------------------------------------------------------------------
// 测试用例 3：正常流程 - 简易 IMU 帧合并到完整 IMU 结构
// 验证 ID=0x03 且 size=21 的帧被识别为简易 IMU，合并到 imu_
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, MergesSimpleImuIntoLatestImu) {
    // 1. 先解析一帧完整 IMU，让 imu_ 有旧数据
    //    时间戳=1，偏航角=0.1，角速度=1.25
    const auto full = make_imu_frame(1u, 0.1f);
    tg::Stm32Parser::parse(as_span(to_bytes(full)));
    
    // 2. 确认 imu_ 有旧数据（偏航角速度 = 1.25）
    //    ASSERT_FLOAT_EQ 使用 ASSERT，如果失败立即终止测试
    ASSERT_FLOAT_EQ(imu_.data.yaw_vel, 1.25f);

    // 3. 构造简易 IMU 帧（ID=0x03，size=21）
    //    简易 IMU 只包含：时间戳 + 颜色 + yaw/pitch/roll
    //    不包含：弹速、三轴角速度
    tg::ReceiveSimpleImuData simple{};
    simple.header.sof      = tg::HeaderFrame::SoF();
    simple.header.len      = payload_len<tg::ReceiveSimpleImuData>();  // 12 字节载荷
    simple.header.id       = 0x03;                     // 同一 ID：0x03
    simple.time_stamp      = 999u;                     // 新时间戳
    simple.data.self_color = tg::Color::Red;           // 红方
    simple.data.yaw        = 0.5f;                     // 新偏航角
    simple.data.pitch      = -0.25f;                   // 新俯仰角
    simple.data.roll       = 0.125f;                   // 新横滚角
    simple.eof             = tg::HeaderFrame::EoF();

    // 4. 解析简易 IMU 帧
    //    解析器检测到 ID=0x03 且 size=21，走简易 IMU 分支
    //    将简易帧数据合并到 imu_ 结构体
    tg::Stm32Parser::parse(as_span(to_bytes(simple)));

    // 5. 验证：时间戳被更新
    EXPECT_EQ(imu_.time_stamp, 999u);
    
    // 6. 验证：颜色被更新为 Red
    EXPECT_EQ(imu_.data.self_color, tg::Color::Red);
    
    // 7. 验证：姿态角被更新
    EXPECT_FLOAT_EQ(imu_.data.yaw, 0.5f);
    EXPECT_FLOAT_EQ(imu_.data.pitch, -0.25f);
    EXPECT_FLOAT_EQ(imu_.data.roll, 0.125f);
    
    // 8. 验证：简易帧缺失的字段被清零，而不是保留旧值
    //    这是协议设计的关键行为：未收到的字段 = 0
    EXPECT_FLOAT_EQ(imu_.data.bullet_speed, 0.0f);
    EXPECT_FLOAT_EQ(imu_.data.yaw_vel, 0.0f);
    EXPECT_FLOAT_EQ(imu_.data.pitch_vel, 0.0f);
    EXPECT_FLOAT_EQ(imu_.data.roll_vel, 0.0f);
}


// ----------------------------------------------------------------------------
// 测试用例 4：异常处理 - 小于帧头的短包直接丢弃
// 验证解析器不会因为短包而越界访问
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, DropsTooSmallFrame) {
    // 1. 准备一个只有 2 字节的数据（正常帧头需要 3 字节）
    const std::vector<uint8_t> garbage{0x5A, 0x00};
    
    // 2. 尝试解析
    tg::Stm32Parser::parse(as_span(garbage));

    // 3. 验证：两个输出槽都保持初始值 0
    //    解析器检测到数据 < sizeof(HeaderFrame)，直接返回
    EXPECT_EQ(imu_.time_stamp, 0u);
    EXPECT_EQ(caps_.time_stamp, 0u);
}


// ----------------------------------------------------------------------------
// 测试用例 5：异常处理 - 帧头魔数错误整帧丢弃
// 验证解析器能识别并丢弃 SoF 错误的帧
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, DropsInvalidSoF) {
    // 1. 构造一个合法帧
    auto frame = make_imu_frame(77u, 0.7f);
    const auto bytes = to_bytes(frame);
    
    // 2. 复制一份，篡改第一个字节（SoF）
    auto bad = bytes;
    bad[0] = 0x00;  // 正确应为 0x5A

    // 3. 解析坏帧：应被丢弃，imu_ 保持 0
    tg::Stm32Parser::parse(as_span(bad));
    EXPECT_EQ(imu_.time_stamp, 0u);

    // 4. 对照实验：解析合法帧，应成功
    tg::Stm32Parser::parse(as_span(bytes));
    EXPECT_EQ(imu_.time_stamp, 77u);
}


// ----------------------------------------------------------------------------
// 测试用例 6：异常处理 - 未知帧 ID 静默忽略
// 验证解析器支持前向兼容：下位机新增帧类型不会导致崩溃
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, IgnoresUnknownFrameId) {
    // 1. 构造一个帧，但把 ID 改为未知值 0x42
    auto frame = make_imu_frame(88u, 0.7f);
    frame.header.id = 0x42;  // 未定义的帧 ID

    // 2. 解析未知帧：应静默忽略
    tg::Stm32Parser::parse(as_span(to_bytes(frame)));
    
    // 3. 验证：两个输出槽都保持 0
    //    解析器找不到 ID=0x42 的分支，直接返回
    EXPECT_EQ(imu_.time_stamp, 0u);
    EXPECT_EQ(caps_.time_stamp, 0u);
}


// ----------------------------------------------------------------------------
// 测试用例 7：边界条件 - IMU 帧截断保护
// 验证长度不足 sizeof(ReceiveImuData) 时，保留旧值不被污染
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, KeepsOldValueOnTruncatedImuFrame) {
    // 1. 先解析一帧合法数据，让 imu_ 有旧值
    tg::Stm32Parser::parse(as_span(to_bytes(make_imu_frame(100u, 0.3f))));
    ASSERT_EQ(imu_.time_stamp, 100u);  // 确认旧值存在

    // 2. 构造一个新帧，但截断最后一个字节
    auto bytes = to_bytes(make_imu_frame(200u, 0.9f));
    bytes.resize(bytes.size() - 1);  // 删除 EoF，变成不完整帧

    // 3. 解析截断帧
    tg::Stm32Parser::parse(as_span(bytes));

    // 4. 验证：旧值保留，没有被截断数据污染
    //    解析器检测到 size < sizeof(ReceiveImuData)，直接返回
    EXPECT_EQ(imu_.time_stamp, 100u);
}


// ----------------------------------------------------------------------------
// 测试用例 8：边界条件 - 载荷中的魔数字节不受影响
// 验证解析器按 len 字段解析，不会把载荷中的 0x5A/0xA5 误判为帧头/帧尾
// ----------------------------------------------------------------------------
TEST_F(Stm32ParserTest, ToleratesMagicBytesInsidePayload) {
    // 1. 构造一个帧
    auto frame = make_imu_frame(555u, 0.25f);
    
    // 2. 在弹速字段中注入魔数 0x5A5A5A5A
    //    注意：这不是合法浮点数，但我们只测试字节模式
    uint32_t magic = 0x5A5A5A5Au;
    std::memcpy(&frame.data.bullet_speed, &magic, sizeof(magic));
    
    // 3. 计算期望的浮点值（同一字节模式解释为 float）
    float expected = 0.0f;
    std::memcpy(&expected, &magic, sizeof(expected));

    // 4. 解析帧
    tg::Stm32Parser::parse(as_span(to_bytes(frame)));

    // 5. 验证：时间戳正确
    EXPECT_EQ(imu_.time_stamp, 555u);
    
    // 6. 验证：弹速字段被解析为 float，值是同一字节模式解释的结果
    //    即使这个值看起来很奇怪，但解析器应该原样保留
    EXPECT_FLOAT_EQ(imu_.data.bullet_speed, expected);
}

// ============================================================================
// 第三层：SerialImpl 串口集成测试（PTY 模拟下位机）
// ============================================================================
class SerialImplTest : public ::testing::Test {
protected:
    void SetUp() override {
        //ASSERT_FALSE 是 Google Test 提供的断言宏，用于断言某个条件为假，如果条件为真则立即终止当前测试用例。
        ASSERT_FALSE(pty_.slave_path().empty()) << "PTY 创建失败";
        tg::Stm32Parser::latest_imu          = &imu_;
        tg::Stm32Parser::latest_capabilities = &caps_;
    }

    void TearDown() override {
        tg::Stm32Parser::latest_imu          = nullptr;
        tg::Stm32Parser::latest_capabilities = nullptr;
    }

    PtyPair pty_; // master=测试(下位机侧)，slave=被测串口
    tg::ReceiveImuData imu_{};
    tg::ReceiveCapabilitiesData caps_{};
};

// ============================================================================
// 测试套件：SerialImplTest
// 测试目标：SerialImpl 串口通信类的完整功能
// 测试策略：使用 PTY 伪终端对模拟真实串口设备
// ============================================================================

// ----------------------------------------------------------------------------
// 测试用例 1：连接不存在的设备 → 返回包含错误上下文的信息
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, ConnectFailureReturnsError) {
    // 1. 创建串口设备对象（未连接状态）
    tg::SerialDevice device;
    
    // 2. 尝试连接一个明确不存在的设备路径
    //    返回值类型：tl::expected<void, std::string>
    //    - 成功：std::nullopt（无值，表示成功）
    //    - 失败：包含错误信息字符串
    const auto result = device.connect("/dev/does-not-exist-talos-test", 115200);

    // 3. 断言：连接应该失败（result 没有值）
    //    ASSERT_FALSE：如果 result.has_value() 为 true，测试立即终止
    ASSERT_FALSE(result.has_value());
    
    // 4. 验证错误信息包含 "open serial device" 前缀
    //    这说明错误来自 open() 系统调用失败
    //    std::string::find() 返回位置，std::string::npos 表示未找到
    EXPECT_NE(result.error().find("open serial device"), std::string::npos);
    
    // 5. 验证错误信息包含了尝试连接的设备路径
    //    便于排障时确认是哪个设备出了问题
    EXPECT_NE(result.error().find("/dev/does-not-exist-talos-test"), std::string::npos);
    
    // 6. 验证设备状态标志为"未连接"
    //    is_connected() 检查内部文件描述符是否有效
    EXPECT_FALSE(device.is_connected());
}

// ----------------------------------------------------------------------------
// 测试用例 2：连接 PTY 成功 → 断开 → 断开后操作安全
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, ConnectAndDisconnect) {
    // 1. 创建串口设备对象
    tg::SerialDevice device;
    
    // 2. 连接 PTY slave 端（模拟真实串口设备）
    //    ASSERT_TRUE：连接失败则立即终止测试
    //    has_value() 检查 tl::expected 是否包含值
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());
    
    // 3. 验证：连接状态为 true
    EXPECT_TRUE(device.is_connected());

    // 4. 主动断开连接
    //    disconnect() 内部会：
    //    - 关闭文件描述符
    //    - 重置内部状态
    device.disconnect();
    
    // 5. 验证：断开后状态为 false
    EXPECT_FALSE(device.is_connected());

    // 6. 验证：断开后调用 handle_events() 不会崩溃
    //    内部应有 fd < 0 的守卫判断，直接返回
    device.handle_events();
    
    // 7. 验证：断开后调用 send_sync() 返回错误而非崩溃
    const uint8_t byte = 0x00;
    EXPECT_FALSE(device.send_sync(&byte, 1).has_value());
    // 预期错误信息："device not connected"
}

// ----------------------------------------------------------------------------
// 测试用例 3：未连接时发送数据 → 返回 "device not connected" 错误
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, SendSyncFailsWhenNotConnected) {
    // 1. 创建串口设备对象（默认未连接）
    tg::SerialDevice device;
    
    // 2. 准备要发送的数据（任意数据，这里准备了一个帧头）
    const uint8_t data[] = {0x5A, 0x01, 0x01};

    // 3. 尝试发送数据（此时设备未连接）
    const auto result = device.send_sync(data, sizeof(data));
    
    // 4. 断言：发送应该失败
    ASSERT_FALSE(result.has_value());
    
    // 5. 验证错误信息包含 "device not connected"
    //    确保错误信息明确指出了失败原因
    EXPECT_NE(result.error().find("device not connected"), std::string::npos);
}

// ----------------------------------------------------------------------------
// 测试用例 4：上位机 → 下位机 发送回环测试
// 验证 send_sync 正确地将数据写入串口
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, SendSyncRoundTrip) {
    // 1. 创建串口设备并连接到 PTY slave
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造一个视觉预测指令帧（上位机 → MCU）
    //    帧 ID 0x02：视觉预测帧，包含开火建议/目标角度/距离
    tg::SendVisionData vision{};
    vision.header.sof        = tg::HeaderFrame::SoF();  // 帧起始 0x5A
    vision.header.id         = 0x02;                    // 帧类型 ID
    vision.data.fire_advice  = true;                    // 建议开火
    vision.data.target_yaw   = 1.234f;                  // 目标偏航角
    vision.data.target_pitch = -0.567f;                 // 目标俯仰角
    vision.data.distance     = 6.28f;                   // 目标距离

    // 3. 序列化：结构体 → 字节流
    const auto sent = to_bytes(vision);
    
    // 4. 通过串口发送数据
    //    send_sync 内部调用 write() 写入 /dev/pts/N
    ASSERT_TRUE(device.send_sync(sent.data(), sent.size()).has_value());

    // 5. 从 PTY master 端读取数据（模拟下位机接收）
    //    read_master 内部用 poll + read 从 master fd 读取
    //    超时 500ms：如果收不到数据，返回已读部分
    const auto received = pty_.read_master(sent.size());
    
    // 6. 验证：收到数据的字节数等于发送的字节数
    ASSERT_EQ(received.size(), sent.size());
    
    // 7. 验证：收到数据的内容与发送的完全一致
    //    memcmp 返回 0 表示两段内存完全相同
    EXPECT_EQ(memcmp(received.data(), sent.data(), sent.size()), 0);

    // 8. 边界测试：零长度发送是合法空操作
    //    不应返回错误
    EXPECT_TRUE(device.send_sync(nullptr, 0).has_value());
}

// ----------------------------------------------------------------------------
// 测试用例 5：下位机 → 上位机 接收分发测试
// 验证 handle_events 正确地从串口读取并解析帧
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, ReceiveDispatchesImuFrame) {
    // 1. 创建串口设备并连接到 PTY slave
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造一个完整的 IMU 帧（时间戳 4242，偏航角 0.75 rad）
    const auto frame = make_imu_frame(4242u, 0.75f);
    
    // 3. 序列化为字节流
    const auto bytes = to_bytes(frame);
    
    // 4. 模拟下位机发送数据：向 PTY master 端写入
    //    数据通过内核 PTY 驱动出现在 slave 端
    ASSERT_TRUE(pty_.write_master(bytes.data(), bytes.size()));

    // 5. 调用事件处理函数
    //    handle_events 内部流程：
    //    a. 从串口 read() 读取数据
    //    b. try_parse_frame() 帧同步状态机
    //    c. Stm32Parser::parse() 按 ID 分发解析
    //    d. 结果存入 imu_（通过全局指针 latest_imu）
    device.handle_events();

    // 6. 验证：解析结果正确
    //    时间戳应为 4242
    EXPECT_EQ(imu_.time_stamp, 4242u);
    
    // 7. 验证：偏航角正确
    EXPECT_FLOAT_EQ(imu_.data.yaw, 0.75f);
    
    // 8. 验证：帧尾魔数正确
    EXPECT_EQ(imu_.eof, tg::HeaderFrame::EoF());
}

// ----------------------------------------------------------------------------
// 测试用例 6：帧前有乱码 → 乱码丢弃，后续帧正常解析
// 验证帧同步（resync）能力
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, ResyncAfterLeadingGarbage) {
    // 1. 连接串口
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造合法 IMU 帧
    const auto frame = make_imu_frame(111u, 0.1f);
    const auto bytes = to_bytes(frame);

    // 3. 准备乱码数据（不包含 SoF 0x5A）
    const uint8_t garbage[] = {0x11, 0x22, 0x33, 0x44};
    
    // 4. 先发送乱码，再发送合法帧
    //    模拟串口上先收到干扰数据，再收到有效数据
    ASSERT_TRUE(pty_.write_master(garbage, sizeof(garbage)));
    std::vector<uint8_t> stream(garbage, garbage + sizeof(garbage));
    stream.insert(stream.end(), bytes.begin(), bytes.end());
    ASSERT_TRUE(pty_.write_master(stream.data(), stream.size()));

    // 5. 处理事件
    //    try_parse_frame 会扫描数据，丢弃乱码，找到 SoF 后开始解析
    device.handle_events();

    // 6. 验证：合法帧被正确解析（乱码被丢弃）
    EXPECT_EQ(imu_.time_stamp, 111u);
}

// ----------------------------------------------------------------------------
// 测试用例 7：帧尾错误 → 坏帧丢弃，后续好帧恢复解析
// 验证 EoF 校验和恢复能力
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, RecoversAfterBadEof) {
    // 1. 连接串口
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造坏帧：EoF 被篡改为 0x00（正确应为 0xA5）
    auto bad = to_bytes(make_imu_frame(66u, 0.1f));
    bad.back() = 0x00;  // 篡改帧尾
    
    // 3. 构造好帧（EoF 正确）
    const auto good = to_bytes(make_imu_frame(77u, 0.2f));

    // 4. 先发坏帧，再发好帧
    std::vector<uint8_t> stream = bad;
    stream.insert(stream.end(), good.begin(), good.end());
    ASSERT_TRUE(pty_.write_master(stream.data(), stream.size()));

    // 5. 处理事件
    device.handle_events();

    // 6. 验证：坏帧被丢弃，好帧被正确解析
    EXPECT_EQ(imu_.time_stamp, 77u);
    EXPECT_EQ(imu_.eof, tg::HeaderFrame::EoF());
}

// ----------------------------------------------------------------------------
// 测试用例 8：帧分多次到达（半帧拼接）
// 验证串口层缓存和帧重组能力
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, HandlesSplitFrameArrival) {
    // 1. 连接串口
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造完整 IMU 帧
    const auto bytes = to_bytes(make_imu_frame(888u, 0.15f));

    // 3. 只发送前半帧（长度不足一帧）
    const size_t half = bytes.size() / 2;
    ASSERT_TRUE(pty_.write_master(bytes.data(), half));
    
    // 4. 处理事件：因为帧不完整，不应解析出数据
    device.handle_events();
    EXPECT_EQ(imu_.time_stamp, 0u);  // 保持旧值（初始为 0）

    // 5. 发送后半帧（补齐完整帧）
    ASSERT_TRUE(pty_.write_master(bytes.data() + half, bytes.size() - half));
    
    // 6. 再次处理事件：现在有完整帧，应解析成功
    device.handle_events();
    
    // 7. 验证：完整帧被正确解析
    EXPECT_EQ(imu_.time_stamp, 888u);
}

// ----------------------------------------------------------------------------
// 测试用例 9：一次读取包含多帧 → 循环解析全部消费
// 验证一个 read 缓冲区包含多帧时，循环解析能全部取出
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, ParsesMultipleFramesInOneRead) {
    // 1. 连接串口
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造两帧 IMU（时间戳 1 和 2）
    const auto f1 = to_bytes(make_imu_frame(1u, 0.05f));
    const auto f2 = to_bytes(make_imu_frame(2u, 0.10f));

    // 3. 构造一帧能力开关帧（时间戳 3）
    tg::ReceiveCapabilitiesData caps_frame{};
    caps_frame.header.sof     = tg::HeaderFrame::SoF();
    caps_frame.header.len     = payload_len<tg::ReceiveCapabilitiesData>();
    caps_frame.header.id      = 0x03;
    caps_frame.time_stamp     = 3u;
    caps_frame.data.following = 1;
    caps_frame.data.quanta    = 1;
    caps_frame.eof            = tg::HeaderFrame::EoF();
    const auto f3 = to_bytes(caps_frame);

    // 4. 拼接三帧，一次性全部写入 PTY master
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    stream.insert(stream.end(), f3.begin(), f3.end());
    ASSERT_TRUE(pty_.write_master(stream.data(), stream.size()));

    // 5. 一次性处理所有数据
    //    try_parse_frame 会在循环中反复解析，直到缓冲区耗尽
    device.handle_events();

    // 6. 验证：IMU 输出槽保留了最后一帧（时间戳 2）
    //    因为两帧 IMU 先后覆盖，最后保留的是第二帧
    EXPECT_EQ(imu_.time_stamp, 2u);
    EXPECT_FLOAT_EQ(imu_.data.yaw, 0.10f);
    
    // 7. 验证：能力帧也被正确解析
    EXPECT_EQ(caps_.time_stamp, 3u);
    EXPECT_EQ(caps_.data.following, 1);
    EXPECT_EQ(caps_.data.quanta, 1);
}

// ----------------------------------------------------------------------------
// 测试用例 10：len 字段声明超过实际数据 → 等待补齐，不越界
// 验证串口层能正确处理"声称长度 > 实际长度"的异常情况
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, WaitsForDeclaredFrameLength) {
    // 1. 连接串口
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 构造一个帧，但故意把 len 字段改大
    auto frame = make_imu_frame(999u, 0.2f);
    // 正常 len = 30，这里改为 40（多声明 10 字节）
    frame.header.len = payload_len<tg::ReceiveImuData>() + 10;
    const auto bytes = to_bytes(frame);

    // 3. 发送这个"声明过大"的帧（实际字节数不足）
    ASSERT_TRUE(pty_.write_master(bytes.data(), bytes.size()));
    
    // 4. 处理事件：帧不完整（还差 10 字节），不应解析
    device.handle_events();
    EXPECT_EQ(imu_.time_stamp, 0u);  // 未解析

    // 5. 补齐 10 字节额外数据
    //    注意：这些额外数据不是 EoF，所以帧尾错位，最终会因 EoF 校验失败丢弃
    std::vector<uint8_t> extra(10, 0x00);
    ASSERT_TRUE(pty_.write_master(extra.data(), extra.size()));
    
    // 6. 再次处理事件
    //    帧尾错位 → EoF 校验失败 → 整帧丢弃 → 无崩溃
    device.handle_events();
    
    // 7. SUCCEED() 显式标记测试通过
    //    只要没有崩溃/断言失败，这个测试就通过
    SUCCEED();
}

// ----------------------------------------------------------------------------
// 测试用例 11：长时间运行 → 缓存不无限增长
// 验证每帧消费后，接收缓冲区被正确清空
// ----------------------------------------------------------------------------
TEST_F(SerialImplTest, RxBufferDrainsAfterParse) {
    // 1. 连接串口
    tg::SerialDevice device;
    ASSERT_TRUE(device.connect(pty_.slave_path(), 115200).has_value());

    // 2. 连续发送 20 帧，逐帧验证
    for (uint32_t stamp = 0; stamp < 20; ++stamp) {
        // 发送一帧 IMU（时间戳 = stamp）
        const auto bytes = to_bytes(make_imu_frame(stamp, 0.01f));
        ASSERT_TRUE(pty_.write_master(bytes.data(), bytes.size()));
        
        // 处理事件
        device.handle_events();
        
        // 验证：解析结果的时间戳 = stamp
        // 如果缓存没清空，可能会有旧帧残留
        ASSERT_EQ(imu_.time_stamp, stamp);
    }
    
    // 3. 最终验证：最后一帧的时间戳是 19
    EXPECT_EQ(imu_.time_stamp, 19u);
}

// ============================================================================
// 第四层：McuDeviceHandle 统一设备句柄测试
// ============================================================================
/// 异常处理：create_serial 失败 → 错误信息含路径与波特率上下文
// ============================================================================
// 测试套件：McuDeviceHandleTest
// 测试目标：McuDeviceHandle 统一设备句柄的创建、通信、移动语义
// ============================================================================
// ----------------------------------------------------------------------------
// 测试用例 1：create_serial 失败时返回包含上下文信息的错误
// ----------------------------------------------------------------------------
TEST(McuDeviceHandleTest, CreateSerialFailureIncludesContext) {
    // 1. 调用 create_serial 尝试连接一个明确不存在的设备路径
    //    参数1：不存在的串口设备路径
    //    参数2：波特率 115200
    //    返回值类型：tl::expected<McuDeviceHandle, std::string>
    //    - 成功：包含 McuDeviceHandle 对象
    //    - 失败：包含错误信息字符串
    const auto result =
        tg::McuDeviceHandle::create_serial("/dev/does-not-exist-talos-test", 115200);

    // 2. 断言：result 应该没有值（即连接失败）
    //    ASSERT_FALSE：如果 result.has_value() 为 true，测试立即终止并报失败
    //    这里用 ASSERT 而非 EXPECT，因为后续代码依赖 result 是失败状态
    ASSERT_FALSE(result.has_value());

    // 3. 验证错误信息中是否包含 "connect serial mcu" 字符串
    //    result.error() 返回 std::string 类型的错误描述
    //    std::string::find() 返回查找位置，如果找不到返回 std::string::npos
    //    EXPECT_NE(... npos) 期望能找到该子串
    //    这确保错误信息说明了操作上下文（连接串口MCU）
    EXPECT_NE(result.error().find("connect serial mcu"), std::string::npos);

    // 4. 验证错误信息中是否包含 "baud_rate=115200" 字符串
    //    这确保错误信息包含了波特率参数，方便调试时确认配置
    EXPECT_NE(result.error().find("baud_rate=115200"), std::string::npos);
}

// ----------------------------------------------------------------------------
// 测试用例 2：create_serial 成功 → 验证完整收发回环
// ----------------------------------------------------------------------------
TEST(McuDeviceHandleTest, CreateSerialOverPty) {
    // 1. 创建 PTY 伪终端对
    //    - master 端：测试代码使用（模拟下位机 STM32）
    //    - slave 端：被测代码使用（模拟真实串口设备 /dev/pts/N）
    PtyPair pty;

    // 2. 断言：PTY 创建成功，slave 设备路径不为空
    //    如果路径为空，说明 posix_openpt/grantpt/unlockpt 失败
    //    后续测试无法进行，所以用 ASSERT_FALSE 立即终止
    ASSERT_FALSE(pty.slave_path().empty());

    // 3. 通过 McuDeviceHandle 的工厂方法创建串口设备句柄
    //    传入 PTY 的 slave 路径和波特率
    //    create_serial 内部会：
    //    - 打开 /dev/pts/N 设备文件
    //    - 配置 termios（波特率/数据位/停止位/校验位等）
    //    - 返回 McuDeviceHandle 对象（内部持有 SerialDevice）
    auto result = tg::McuDeviceHandle::create_serial(pty.slave_path(), 115200);

    // 4. 断言：创建成功
    //    ASSERT_TRUE：如果 result 没有值，测试立即终止
    //    << result.error()：如果失败，输出错误信息到测试日志
    ASSERT_TRUE(result.has_value()) << result.error();

    // 5. 获取句柄的引用（result 是 tl::expected，*result 返回 McuDeviceHandle&）
    auto& handle = *result;

    // 6. 验证：句柄识别为串口设备
    //    is_serial() 检查 variant 中存储的是 SerialDevice 类型
    //    如果是其他设备类型（如虚拟设备），返回 false
    EXPECT_TRUE(handle.is_serial());

    // 7. 验证：句柄已连接
    //    is_connected() 检查内部文件描述符是否有效
    EXPECT_TRUE(handle.is_connected());

    // ========== 测试方向1：上位机 → 下位机（发送指令） ==========

    // 8. 构造一个"极简视觉帧"（上位机 → MCU 的指令）
    //    帧 ID 0x04：极简视觉帧，只包含目标偏航角
    tg::SendSimpleVisionData simple{};
    simple.header.sof      = tg::HeaderFrame::SoF();  // 帧起始 0x5A
    simple.header.id       = 0x04;                    // 帧类型 ID
    simple.data.target_yaw = -0.5f;                   // 目标偏航角

    // 9. 序列化：结构体 → 字节流
    //    to_bytes() 用 memcpy 将结构体原始内存拷贝到 vector<uint8_t>
    const auto bytes = to_bytes(simple);

    // 10. 通过句柄发送数据（上位机 → 下位机）
    //     send_sync 内部调用 write() 写入串口设备
    //     ASSERT_TRUE：发送失败则终止测试
    ASSERT_TRUE(handle.send_sync(bytes.data(), bytes.size()).has_value());

    // 11. PTY master 端读取数据（模拟下位机接收）
    //     read_master 内部用 poll + read 从 master fd 读取
    //     参数 bytes.size()：期望读取的字节数
    //     超时 500ms：如果下位机没收到数据，返回已读部分
    const auto received = pty.read_master(bytes.size());

    // 12. 验证：读取到的数据大小等于发送的数据大小
    //     如果大小不匹配，说明数据在传输中丢失或 PTY 配置有问题
    ASSERT_EQ(received.size(), bytes.size());

    // 13. 验证：读取到的数据内容与发送的逐字节一致
    //     memcmp 返回 0 表示两段内存完全相同
    //     这验证了 send_sync 正确地将数据写入了串口
    EXPECT_EQ(memcmp(received.data(), bytes.data(), bytes.size()), 0);

    // ========== 测试方向2：下位机 → 上位机（接收数据） ==========

    // 14. 准备一个 IMU 数据帧的接收缓冲区
    //     这个结构体将被 Stm32Parser 填充
    tg::ReceiveImuData imu{};

    // 15. 设置全局解析器输出指针（指向 imu 缓冲区）
    //     Stm32Parser 是静态解析器，通过全局指针输出解析结果
    //     这是测试 Fixture 中 SetUp/TearDown 的手动版
    tg::Stm32Parser::latest_imu = &imu;

    // 16. 构造一个完整的 IMU 帧（时间戳 31337，偏航角 0.42 rad）
    //     make_imu_frame 填充所有字段：帧头/时间戳/颜色/姿态/角速度/帧尾
    const auto frame = to_bytes(make_imu_frame(31337u, 0.42f));

    // 17. 模拟下位机发送数据：通过 PTY master 端写入
    //     数据会通过内核 PTY 驱动出现在 slave 端（/dev/pts/N）
    //     被测的 SerialImpl 从 slave 端读取
    ASSERT_TRUE(pty.write_master(frame.data(), frame.size()));

    // 18. 调用句柄的事件处理函数
    //     handle_events 内部流程：
    //     a. 从串口设备 read() 读取所有可用数据
    //     b. 将数据喂给 try_parse_frame() 帧同步状态机
    //     c. 完整帧交给 Stm32Parser::parse() 解析
    //     d. 解析结果存入 latest_imu 指向的缓冲区
    handle.handle_events();

    // 19. 清理：解除全局指针绑定，防止后续测试误用
    tg::Stm32Parser::latest_imu = nullptr;

    // 20. 验证：解析结果正确
    //     imu.time_stamp 应该等于 31337
    EXPECT_EQ(imu.time_stamp, 31337u);

    // 21. 验证：偏航角正确（使用 EXPECT_FLOAT_EQ 处理浮点误差）
    EXPECT_FLOAT_EQ(imu.data.yaw, 0.42f);
    // 其他字段（颜色/弹速/俯仰/横滚/角速度）由于 make_imu_frame 的默认值
    // 也会正确解析，但此处只验证最关键的两个字段
}

// ----------------------------------------------------------------------------
// 测试用例 3：句柄支持移动语义（move constructor / move assignment）
// ----------------------------------------------------------------------------
TEST(McuDeviceHandleTest, HandleIsMovable) {
    // 1. 创建 PTY 伪终端对
    PtyPair pty;
    ASSERT_FALSE(pty.slave_path().empty());

    // 2. 创建串口设备句柄
    auto result = tg::McuDeviceHandle::create_serial(pty.slave_path(), 115200);
    ASSERT_TRUE(result.has_value());

    // 3. 移动构造：将 result 中的句柄移动到 moved
    //    std::move(*result) 将左值转为右值引用
    //    触发 McuDeviceHandle 的移动构造函数
    //    移动后：
    //    - moved 接管了内部的 unique_ptr 和文件描述符
    //    - result 中的原句柄变为"空"状态（类似 nullptr）
    tg::McuDeviceHandle moved = std::move(*result);

    // 4. 验证：移动后的新句柄仍然连接
    //     因为资源所有权已转移，moved 持有有效的文件描述符
    EXPECT_TRUE(moved.is_connected());

    // 5. 验证：移动后的新句柄仍然是串口类型
    //     variant 中存储的 active type 仍然是 SerialDevice
    EXPECT_TRUE(moved.is_serial());

    // 注意：这里没有验证原句柄 result 的状态，因为：
    // - 移动后的对象处于"有效但未指定"状态（valid but unspecified）
    // - 通常 is_connected() 返回 false，但不强制要求
    // - 实际项目中，移动后的对象不应再被使用
}
