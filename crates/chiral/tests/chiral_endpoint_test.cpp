#include "chiral/navigation.hpp"

#include <cerrno>
#include <optional>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace talos::chiral;
using namespace talos::chiral::ipc;

namespace chiral_test {

using TalosData    = navigation::TalosData;
using IncomingData = navigation::NavigationData;
using TalosSide    = navigation::TalosEndpoint;
using RemoteSide   = navigation::NavigationEndpoint;

void cleanup_endpoint_shm() noexcept {
    (void)::shm_unlink(ShmName<TalosData>::value);
    (void)::shm_unlink(ShmName<IncomingData>::value);
}

class ChiralEndpointTest : public ::testing::Test {
protected:
    void SetUp() override { cleanup_endpoint_shm(); }
    void TearDown() override { cleanup_endpoint_shm(); }
};

static_assert(
    std::is_same_v<
        decltype(std::declval<TalosSide&>().write(std::declval<const TalosData&>())), void>,
    "TalosEndpoint::write must accept TalosData");

static_assert(
    std::is_same_v<decltype(std::declval<TalosSide&>().read_new()), std::optional<IncomingData>>,
    "TalosEndpoint::read_new must return optional<NavigationData>");

static_assert(
    std::is_same_v<decltype(std::declval<TalosSide&>().read_latest()), std::optional<IncomingData>>,
    "TalosEndpoint::read_latest must return optional<NavigationData>");

static_assert(
    std::is_same_v<
        decltype(std::declval<RemoteSide&>().write(std::declval<const IncomingData&>())), void>,
    "NavigationEndpoint::write must accept NavigationData");

static_assert(
    std::is_same_v<decltype(std::declval<RemoteSide&>().read_new()), std::optional<TalosData>>,
    "NavigationEndpoint::read_new must return optional<TalosData>");

TEST_F(ChiralEndpointTest, ShmRegionCreateAndCleanup) {
    constexpr size_t shm_size = sizeof(ShmLayout<TalosData>);

    {
        auto region = ShmRegion::create(ShmName<TalosData>::value, shm_size);
        ASSERT_TRUE(region.has_value()) << static_cast<int>(region.error());
        ASSERT_NE(region->as<void>(), nullptr);

        const int fd = ::shm_open(ShmName<TalosData>::value, O_RDWR, 0);
        ASSERT_GE(fd, 0);
        close(fd);
    }

    errno        = 0;
    const int fd = ::shm_open(ShmName<TalosData>::value, O_RDWR, 0);
    EXPECT_EQ(fd, -1);
    EXPECT_EQ(errno, ENOENT);
}

TEST_F(ChiralEndpointTest, ChannelReaderOpenBeforeWriterFails) {
    auto reader = ChannelReader<TalosData>::open();
    ASSERT_FALSE(reader.has_value());
    EXPECT_EQ(reader.error(), ShmError::NotFound);
}

TEST_F(ChiralEndpointTest, BidirectionalBasicFlow) {
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);
    EXPECT_FALSE(talos->read_new().has_value());
    EXPECT_FALSE(talos->read_latest().has_value());

    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);

    TalosData outbound{};
    outbound.gimbal_link.translation.x = 42.0;
    outbound.gimbal_link.translation.y = 21.0;
    talos->write(outbound);

    auto remote_read = remote->read_new();
    ASSERT_TRUE(remote_read.has_value());
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, 42.0);
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.y, 21.0);

    IncomingData inbound{};
    inbound.timestamp_ns = 12345;
    remote->write(inbound);

    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 12345U);

    auto latest = talos->read_latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->timestamp_ns, 12345U);
}

TEST_F(ChiralEndpointTest, TalosCanStartBeforeRemote) {
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);

    const int talos_fd = ::shm_open(ShmName<TalosData>::value, O_RDWR, 0);
    ASSERT_GE(talos_fd, 0);
    close(talos_fd);

    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);

    TalosData outbound{};
    outbound.gimbal_link.translation.x = 7.0;
    talos->write(outbound);

    auto remote_read = remote->read_new();
    ASSERT_TRUE(remote_read.has_value());
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, 7.0);

    IncomingData inbound{};
    inbound.timestamp_ns = 77;
    remote->write(inbound);

    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 77U);
}

TEST_F(ChiralEndpointTest, RemoteCanStartBeforeTalos) {
    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);
    EXPECT_FALSE(remote->read_new().has_value());
    EXPECT_FALSE(remote->read_latest().has_value());

    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);

    IncomingData inbound{};
    inbound.timestamp_ns = 88;
    remote->write(inbound);

    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 88U);

    TalosData outbound{};
    outbound.gimbal_link.translation.x = 8.0;
    talos->write(outbound);

    auto remote_read = remote->read_new();
    ASSERT_TRUE(remote_read.has_value());
    EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, 8.0);
}

TEST_F(ChiralEndpointTest, LazyReaderReconnectsAfterPeerRestart) {
    auto talos_result = TalosSide::create();
    ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
    auto talos = std::move(*talos_result);
    ASSERT_NE(talos, nullptr);

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

    EXPECT_FALSE(talos->read_new().has_value());
    EXPECT_FALSE(talos->read_latest().has_value());

    auto remote_result = RemoteSide::create();
    ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
    auto remote = std::move(*remote_result);
    ASSERT_NE(remote, nullptr);

    IncomingData inbound{};
    inbound.timestamp_ns = 2;
    remote->write(inbound);

    auto talos_read = talos->read_new();
    ASSERT_TRUE(talos_read.has_value());
    EXPECT_EQ(talos_read->timestamp_ns, 2U);
}

TEST_F(ChiralEndpointTest, RepeatedDisconnectAndRecreateBothLinks) {
    constexpr uint64_t kCycles = 128;

    for (uint64_t cycle = 1; cycle <= kCycles; ++cycle) {
        SCOPED_TRACE(cycle);

        auto talos_result = TalosSide::create();
        ASSERT_TRUE(talos_result.has_value()) << static_cast<int>(talos_result.error());
        auto talos = std::move(*talos_result);
        ASSERT_NE(talos, nullptr);

        auto remote_result = RemoteSide::create();
        ASSERT_TRUE(remote_result.has_value()) << static_cast<int>(remote_result.error());
        auto remote = std::move(*remote_result);
        ASSERT_NE(remote, nullptr);

        TalosData outbound{};
        outbound.gimbal_link.translation.x = static_cast<double>(cycle);
        talos->write(outbound);

        auto remote_read = remote->read_new();
        ASSERT_TRUE(remote_read.has_value());
        EXPECT_DOUBLE_EQ(remote_read->gimbal_link.translation.x, static_cast<double>(cycle));

        remote.reset();
        EXPECT_FALSE(talos->read_new().has_value());
        EXPECT_FALSE(talos->read_latest().has_value());

        auto remote_recreated_result = RemoteSide::create();
        ASSERT_TRUE(remote_recreated_result.has_value())
            << static_cast<int>(remote_recreated_result.error());
        auto remote_recreated = std::move(*remote_recreated_result);
        ASSERT_NE(remote_recreated, nullptr);

        IncomingData inbound{};
        inbound.timestamp_ns = cycle;
        remote_recreated->write(inbound);

        auto talos_read = talos->read_new();
        ASSERT_TRUE(talos_read.has_value());
        EXPECT_EQ(talos_read->timestamp_ns, cycle);

        talos.reset();
        EXPECT_FALSE(remote_recreated->read_new().has_value());
        EXPECT_FALSE(remote_recreated->read_latest().has_value());

        auto talos_recreated_result = TalosSide::create();
        ASSERT_TRUE(talos_recreated_result.has_value())
            << static_cast<int>(talos_recreated_result.error());
        auto talos_recreated = std::move(*talos_recreated_result);
        ASSERT_NE(talos_recreated, nullptr);

        TalosData rebound{};
        rebound.gimbal_link.translation.x = static_cast<double>(cycle) + 0.5;
        talos_recreated->write(rebound);

        auto remote_rebound = remote_recreated->read_new();
        ASSERT_TRUE(remote_rebound.has_value());
        EXPECT_DOUBLE_EQ(
            remote_rebound->gimbal_link.translation.x, static_cast<double>(cycle) + 0.5);
    }
}

TEST_F(ChiralEndpointTest, ShmNameTraitsMatchNavigationEndpoints) {
    EXPECT_STREQ(ShmName<TalosData>::value, "/chiral_nav_talos");
    EXPECT_STREQ(ShmName<IncomingData>::value, "/chiral_nav_navigation");
}

TEST_F(ChiralEndpointTest, IncomingDataDefaultInitialization) {
    IncomingData data{};
    EXPECT_EQ(data.timestamp_ns, 0U);
}

} // namespace chiral_test
