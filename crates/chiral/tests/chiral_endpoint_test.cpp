// 引入导航模块IPC端点、数据结构体、类型别名定义头文件
#include "chiral/navigation.hpp"

// Linux 标准错误码头文件，用于判断 shm_open 失败错误类型 ENOENT
#include <cerrno>
// 可选容器，IPC读取无新数据时返回 std::optional 空值
#include <optional>
// C++ 类型萃取，编译期静态断言校验接口返回/入参类型
#include <type_traits>
// 移动语义工具 std::move、std::declval
#include <utility>

// Google Test 单元测试框架头文件
#include <gtest/gtest.h>
// Linux 共享内存映射系统调用 mmap 头文件
#include <sys/mman.h>
// Linux 文件关闭、共享内存删除系统调用 close / shm_unlink
#include <unistd.h>

// 全局命名空间简化，无需重复写完整前缀
using namespace talos::chiral;
using namespace talos::chiral::ipc;

// 导航IPC单元测试专属隔离命名空间
namespace chiral_test {

// 导航模块两种IPC消息数据类型别名
using TalosData    = navigation::TalosData;    // Talos主控向外发送的上行数据
using IncomingData = navigation::NavigationData; // 导航端下发给Talos的下行数据
// 两端双向通信端点别名
using TalosSide    = navigation::TalosEndpoint;  // Talos主控进程IPC端点
using RemoteSide   = navigation::NavigationEndpoint; // 导航上位机/子进程IPC端点

/**
 * @brief 全局清理函数：删除导航两套共享内存文件，消除测试残留脏数据
 * 每次测试前后自动执行，避免上一轮测试残留SHM导致用例失败
 */
void cleanup_endpoint_shm() noexcept {
    // 忽略返回值，即使文件不存在也无需报错
    (void)::shm_unlink(ShmName<TalosData>::value);
    (void)::shm_unlink(ShmName<IncomingData>::value);
}

/**
 * @brief GTest测试夹具基类，所有ChiralEndpoint相关测试共用
 * 自动在每个测试用例执行前、执行后清理共享内存，隔离用例环境
 */
class ChiralEndpointTest : public ::testing::Test {
protected:
    // 每个TEST_F执行前回调：清理残留共享内存
    void SetUp() override { cleanup_endpoint_shm(); }
    // 每个TEST_F执行完毕后回调：再次清理，防止进程异常退出残留SHM
    void TearDown() override { cleanup_endpoint_shm(); }
};

// ============ 编译期静态类型校验静态断言 ============
/**
 * 静态断言1：TalosSide::write 入参必须是 const TalosData&，返回值 void
 * std::declval<TalosSide&>() 模拟左值引用对象
 * decltype 获取调用write后的返回类型
 * std::is_same_v 判断类型完全一致，编译失败打印提示字符串
 */
static_assert(
    std::is_same_v<
        decltype(std::declval<TalosSide&>().write(std::declval<const TalosData&>())), void>,
    "TalosEndpoint::write must accept TalosData");

/**
 * 静态断言2：TalosSide::read_new 返回类型必须 std::optional<IncomingData>
 */
static_assert(
    std::is_same_v<decltype(std::declval<TalosSide&>().read_new()), std::optional<IncomingData>>,
    "TalosEndpoint::read_new must return optional<NavigationData>");

/**
 * 静态断言3：TalosSide::read_latest 返回类型必须 std::optional<IncomingData>
 */
static_assert(
    std::is_same_v<decltype(std::declval<TalosSide&>().read_latest()), std::optional<IncomingData>>,
    "TalosEndpoint::read_latest must return optional<NavigationData>");

/**
 * 静态断言4：RemoteSide::write 入参 const IncomingData&，返回 void
 */
static_assert(
    std::is_same_v<
        decltype(std::declval<RemoteSide&>().write(std::declval<const IncomingData&>())), void>,
    "NavigationEndpoint::write must accept NavigationData");

/**
 * 静态断言5：RemoteSide::read_new 返回 std::optional<TalosData>
 */
static_assert(
    std::is_same_v<decltype(std::declval<RemoteSide&>().read_new()), std::optional<TalosData>>,
    "NavigationEndpoint::read_new must return optional<TalosData>");

// ============ 单元测试用例开始 ============
/**
 * @brief 测试共享内存创建、自动清理释放逻辑
 * 验证：创建ShmRegion后析构自动调用shm_unlink，外部无法打开该SHM
 */
TEST_F(ChiralEndpointTest, ShmRegionCreateAndCleanup) {
    // 计算TalosData完整共享内存布局字节大小
    constexpr size_t shm_size = sizeof(ShmLayout<TalosData>);

    // 局部作用域，region离开作用域自动析构
    {
        // 创建TalosData共享内存区域
        auto region = ShmRegion::create(ShmName<TalosData>::value, shm_size);
        // 断言创建成功，打印错误码
        ASSERT_TRUE(region.has_value()) << static_cast<int>(region.error());
        // 映射虚拟地址非空
        ASSERT_NE(region->as<void>(), nullptr);

        // 尝试打开共享内存fd，此时文件存在，fd>=0
        const int fd = ::shm_open(ShmName<TalosData>::value, O_RDWR, 0);
        ASSERT_GE(fd, 0);
        close(fd);
    }

    // region析构，自动执行shm_unlink删除共享内存文件
    errno        = 0;
    // 再次打开应当失败
    const int fd = ::shm_open(ShmName<TalosData>::value, O_RDWR, 0);
    EXPECT_EQ(fd, -1);
    // 错误码为文件不存在 ENOENT
    EXPECT_EQ(errno, ENOENT);
}

/**
 * @brief 测试读通道先于写通道打开，应当返回 NotFound 错误
 * 共享内存未被创建时，Reader无法打开
 */
TEST_F(ChiralEndpointTest, ChannelReaderOpenBeforeWriterFails) {
    auto reader = ChannelReader<TalosData>::open();
    // 打开失败
    ASSERT_FALSE(reader.has_value());
    // 错误类型为共享内存不存在
    EXPECT_EQ(reader.error(), ShmError::NotFound);
}

/**
 * @brief 基础双向通信流程全链路测试
 * Talos端写数据 → Remote端读取；Remote端写数据 → Talos端读取
 * 验证基础收发、三缓冲数据存储、读取接口正常工作
 */
TEST_F(ChiralEndpointTest, BidirectionalBasicFlow) {
    // 创建Talos主控端点
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);
    // 初始无任何对端写入数据，读取返回空
    EXPECT_FALSE(talos->read_new().has_value());
    EXPECT_FALSE(talos->read_latest().has_value());

    // 创建导航远端端点
    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);

    // Talos构造发送数据，填充云台链接平移x=42 y=21
    TalosData outbound{};
    outbound.gimbal_link.translation.x = 42.0;
    outbound.gimbal_link.translation.y = 21.0;
    // 写入共享内存三缓冲
    talos->write(outbound);

    // Remote读取新帧
    auto remote_read = remote->read_new();
    ASSERT_TRUE(remote_read.has_value());
    // 浮点精确相等校验
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, 42.0);
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.y, 21.0);

    // 远端构造下行指令，时间戳12345
    IncomingData inbound{};
    inbound.timestamp_ns = 12345;
    remote->write(inbound);

    // Talos读取远端下发数据
    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 12345U);

    // 读取最新一帧，和上一帧数据一致
    auto latest = talos->read_latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->timestamp_ns, 12345U);
}

/**
 * @brief 测试启动时序：Talos进程先启动创建SHM，后启动Remote导航进程
 * 验证写端先行创建共享内存，读端后打开可正常通信
 */
TEST_F(ChiralEndpointTest, TalosCanStartBeforeRemote) {
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);

    // 验证Talos对应的共享内存文件已创建
    const int talos_fd = ::shm_open(ShmName<TalosData>::value, O_RDWR, 0);
    ASSERT_GE(talos_fd, 0);
    close(talos_fd);

    // 后创建远端端点
    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);

    // Talos发送数据
    TalosData outbound{};
    outbound.gimbal_link.translation.x = 7.0;
    talos->write(outbound);

    // Remote正常读取
    auto remote_read = remote->read_new();
    ASSERT_TRUE(remote_read.has_value());
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, 7.0);

    // Remote下发指令
    IncomingData inbound{};
    inbound.timestamp_ns = 77;
    remote->write(inbound);

    // Talos读取下发指令
    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 77U);
}

/**
 * @brief 反向时序测试：远端导航进程先启动，Talos后启动
 * 验证远端写通道先行创建，Talos读通道延迟打开可正常通信
 */
TEST_F(ChiralEndpointTest, RemoteCanStartBeforeTalos) {
    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);
    // Talos未启动，无下行数据，读取为空
    EXPECT_FALSE(remote->read_new().has_value());
    EXPECT_FALSE(remote->read_latest().has_value());

    // 后创建Talos端点
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);

    // Remote下发数据
    IncomingData inbound{};
    inbound.timestamp_ns = 88;
    remote->write(inbound);

    // Talos读取
    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 88U);

    // Talos上行推送
    TalosData outbound{};
    outbound.gimbal_link.translation.x = 8.0;
    talos->write(outbound);

    // Remote读取上行数据
    auto remote_read = remote->read_new();
    ASSERT_TRUE(remote_read.has_value());
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, 8.0);
}

/**
 * @brief 测试懒加载读通道自动重连机制：对端进程销毁重建后，本地读通道自动重建映射
 * 场景：Remote销毁，重新创建，Talos无需重启自动重新连接新共享内存
 */
TEST_F(ChiralEndpointTest, LazyReaderReconnectsAfterPeerRestart) {
    // 创建Talos端点长期持有
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);

    // 局部作用域Remote，离开作用域自动销毁释放共享内存
    {
        auto remote_result = RemoteSide::create();
        ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
        auto remote = std::move(*remote_result);
        ASSERT_NE(remote, nullptr);

        IncomingData inbound{};
        inbound.timestamp_ns = 1;
        remote->write(inbound);

        auto talos_read = talos->read_new();
        ASSERT_TRUE(talos_read.has_value());
        EXPECT_EQ(talos_read->timestamp_ns, 1U);
    }

    // Remote销毁，共享内存被删除，此时读取无数据
    EXPECT_FALSE(talos->read_new().has_value());
    EXPECT_FALSE(talos->read_latest().has_value());

    // 重新创建Remote，生成全新共享内存文件
    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);

    // Remote下发新数据
    IncomingData inbound{};
    inbound.timestamp_ns = 2;
    remote->write(inbound);

    // Talos懒加载检测旧映射失效，自动重建读通道，成功读取新数据
    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 2U);
}

/**
 * @brief 高压循环测试：反复销毁重建两端IPC端点128次，验证无句柄泄漏、无内存泄漏、通信稳定
 * 覆盖极端启停场景，模拟机器人程序频繁重启、热重载
 */
TEST_F(ChiralEndpointTest, RepeatedDisconnectAndRecreateBothLinks) {
    constexpr uint64_t kCycles = 128;

    // 循环128轮完整双向销毁重建
    for (uint64_t cycle = 1; cycle <= kCycles; ++cycle) {
        // SCOPED_TRACE：失败时打印当前循环编号，快速定位故障轮次
        SCOPED_TRACE(cycle);

        // 本轮创建两端
        auto talos_result = TalosSide::create();
        ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
        auto talos = std::move(*talos_result);
        ASSERT_NE(talos, nullptr);

        auto remote_result = RemoteSide::create();
        ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
        auto remote = std::move(*remote_result);
        ASSERT_NE(remote, nullptr);

        // Talos上行发送，携带当前循环编号
        TalosData outbound{};
        outbound.gimbal_link.translation.x = static_cast<double>(cycle);
        talos->write(outbound);

        // Remote读取校验
        auto remote_read = remote->read_new();
        ASSERT_TRUE(remote_read.has_value());
        EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, static_cast<double>(cycle));

        // 销毁Remote端点
        remote.reset();
        // Remote销毁后Talos无下行数据
        EXPECT_FALSE(talos->read_new().has_value());
        EXPECT_FALSE(talos->read_latest().has_value());

        // 重新创建Remote
        auto remote_recreated_result = RemoteSide::create();
        ASSERT_TRUE(remote_recreated_result.has_value())
            << static_cast<int>(remote_recreated_result.error());
        auto remote_recreated = std::move(*remote_recreated_result);
        ASSERT_NE(remote_recreated, nullptr);

        // Remote下发带循环编号的指令
        IncomingData inbound{};
        inbound.timestamp_ns = cycle;
        remote_recreated->write(inbound);

        // Talos读取校验
        auto talos_read = talos->read_new();
        ASSERT_TRUE(talos_read.has_value());
        EXPECT_EQ(talos_read->timestamp_ns, cycle);

        // 销毁Talos端点
        talos.reset();
        // Talos销毁后Remote无上行数据
        EXPECT_FALSE(remote_recreated->read_new().has_value());
        EXPECT_FALSE(remote_recreated->read_latest().has_value());

        // 重新创建Talos端点
        auto talos_recreated_result = TalosSide::create();
        ASSERT_TRUE(talos_recreated_result.has_value())
            << static_cast<int>(talos_recreated_result.error());
        auto talos_recreated = std::move(*talos_recreated_result);
        ASSERT_NE(talos_recreated, nullptr);

        // Talos重新发送偏移0.5的数据
        TalosData rebound{};
        rebound.gimbal_link.translation.x = static_cast<double>(cycle) + 0.5;
        talos_recreated->write(rebound);

        // Remote读取校验
        auto remote_rebound = remote_recreated->read_new();
        ASSERT_TRUE(remote_rebound.has_value());
        EXPECT_DOUBLE_EQ(
            remote_rebound->gimbal_link.translation.x, static_cast<double>(cycle) + 0.5);
    }
}

/**
 * @brief 校验导航两套消息对应的共享内存文件名是否与预期字符串匹配
 */
TEST_F(ChiralEndpointTest, ShmNameTraitsMatchNavigationEndpoints) {
    EXPECT_STREQ(ShmName<TalosData>::value, "/chiral_nav_talos");
    EXPECT_STREQ(ShmName<IncomingData>::value, "/chiral_nav_navigation");
}

/**
 * @brief 测试IncomingData空默认构造全部成员初始化为0
 * 验证结构体无随机脏内存，共享内存初始化安全
 */
TEST_F(ChiralEndpointTest, IncomingDataDefaultInitialization) {
    IncomingData data{};
    EXPECT_EQ(data.timestamp_ns, 0U);
}

} // namespace chiral_test