// ===========================================================================
// 阶段7：primitive 模块其余 6 个组件
//
// 文件对齐真实项目（crates/primitive/src/primitive/）：
//   lazy.hpp             延迟构造：参数包存 std::tuple，operator() 才构造
//   overloaded.hpp        C++17 pack expansion 继承 + using 引入 operator()
//   spin.hpp             SPIN_HINT：x86 _mm_pause / ARM yield / 其它空操作
//   performance_probe.*   LatencyProbe 单段计时；LatencyHistogram 无锁直方图
//                         （8192 滑动窗口 + 原子 min/max，mean/p50/p95/p99/stddev）
//   system_info.*         get_username / get_hostname（std::expected 返回）
//   channel.hpp          三缓冲之上的通道封装：make_spsc_channel / make_spmc_channel
//                         → split() 得 Writer/Reader，SPMC 可 clone_reader()
//
// 测试清单
// 测试1：lazy 两条路径（预绑定参数 / 运行时参数），构造次数恰为 1
// 测试2：overloaded + variant 三策略分发（对应 output_interface 的配置分发）
// 测试3：SPIN_HINT 定性对比：100 万次空转，加/不加 PAUSE 的耗时
// 测试4：LatencyProbe 分段计时 + LatencyHistogram 已知分布统计
//        （1000 个 100ns + 50 个 10000ns → p50=100, p95/p99=10000, count=1050）
// 测试5：SystemInfo 用户名/主机名（std::expected 语义）
// 测试6：channel：SPSC make→split→写读 + borrow_mut/publish；
//        SPMC clone_reader 两个消费者独立读取同一帧
// ===========================================================================

#include "primitive/channel.hpp"
#include "primitive/lazy.hpp"
#include "primitive/overloaded.hpp"
#include "primitive/performance_probe.hpp"
#include "primitive/spin.hpp"
#include "primitive/system_info.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

namespace tp = talos::primitive;

// ===========================================================================
// 轻量断言：失败打印位置并累计，main 末尾以非零退出码结束
// ===========================================================================
static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "  [CHECK 失败] " #cond "  (" << __FILE__ << ":" \
                      << __LINE__ << ")\n";                               \
            ++g_failures;                                                \
        }                                                                 \
    } while (0)

// ===========================================================================
// 测试1：lazy 延迟构造 —— 调用前绝不构造，调用时恰好构造 1 次
// ===========================================================================
struct Tracked {
    static inline int constructions = 0;

    Tracked(int x, std::string s)
        : a(x)
        , b(std::move(s)) {
        ++constructions;
    }

    int         a;
    std::string b;
};

void test_lazy() {
    std::cout << "=== 测试1：lazy 延迟构造 ===\n";

    // 路径A：预绑定参数 —— 直接构造 lazy<T, PArgs...>
    // ⚠ 读源码发现的真实 bug：工厂 make_lazy 的返回类型写的是 lazy<T>
    // （丢了 PArgs...），内部却构造 lazy<T, PArgs...> —— 传任何预绑定参数
    // 都无法编译通过，该工厂实际不可用，只能显式写模板参数直接构造。
    tp::lazy<Tracked, int, std::string> lz(41, std::string("hello"));
    CHECK(Tracked::constructions == 0); // ← 延迟的本质：构造参数已存，T 未构造

    auto p1 = lz(); // std::apply 解包 tuple → make_unique<Tracked>(41, "hello")
    CHECK(Tracked::constructions == 1);
    CHECK(p1 != nullptr);
    CHECK(p1->a == 41 && p1->b == "hello");

    // 路径B：无预绑定参数，运行时传参（requires(sizeof...(PArgs) == 0) 分支）
    tp::lazy<Tracked> lz_runtime;
    auto              p2 = lz_runtime(42, "world");
    CHECK(Tracked::constructions == 2); // 累计 2 次，各自恰好一次
    CHECK(p2->a == 42 && p2->b == "world");

    std::cout << "  预绑定路径: 调用前构造次数 = 0，调用后 = 1（延迟生效）\n";
    std::cout << "  运行时路径: 构造次数 +1，unique_ptr 托管\n";
    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：overloaded + variant 三策略分发
// 复刻 output_interface 的 OutputConfig 分发结构：三种输出后端
// （直连 / Daedalus 共享内存 / Foxglove 可视化）各走各的 lambda
// ===========================================================================
struct DirectConfig {
    int exposure_us;
};
struct DaedalusConfig {
    std::string shm_name;
};
struct FoxgloveConfig {
    int port;
};
using OutputConfig = std::variant<DirectConfig, DaedalusConfig, FoxgloveConfig>;

void test_overloaded() {
    std::cout << "=== 测试2：overloaded + variant 分发 ===\n";

    const std::vector<OutputConfig> configs = {
        DirectConfig{2000},
        DaedalusConfig{"/dev/shm/talos_frames"},
        FoxgloveConfig{8765},
    };

    // overloaded{...}：CTAD 推导出 overloaded<L1,L2,L3>，
    // using Ts::operator()... 把三个 lambda 的调用运算符合入一个 functor
    const auto describe = overloaded{
        [](const DirectConfig& c) {
            return "直连后端: 曝光 " + std::to_string(c.exposure_us) + "us";
        },
        [](const DaedalusConfig& c) {
            return "Daedalus 后端: 共享内存 " + c.shm_name;
        },
        [](const FoxgloveConfig& c) {
            return "Foxglove 后端: WebSocket 端口 " + std::to_string(c.port);
        },
    };

    std::string results[3];
    for (std::size_t i = 0; i < configs.size(); ++i) {
        results[i] = std::visit(describe, configs[i]); // 编译期穷尽，漏一个 lambda 直接报错
        std::cout << "  " << results[i] << "\n";
    }

    CHECK(results[0].find("2000") != std::string::npos);
    CHECK(results[1].find("/dev/shm/talos_frames") != std::string::npos);
    CHECK(results[2].find("8765") != std::string::npos);

    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：SPIN_HINT 定性对比
// x86 上 _mm_pause = PAUSE 指令：降低自旋功耗、缓解流水线冲刷。
// 定性演示（不 asserting 快慢，微架构行为受频率调节影响）
// ===========================================================================
void test_spin_hint() {
    std::cout << "=== 测试3：SPIN_HINT 自旋提示 ===\n";

    constexpr int    N    = 1'000'000;
    volatile int     sink = 0;
    tp::LatencyProbe probe;

    for (int i = 0; i < N; ++i) {
        sink = sink + i;
    }
    const auto t_plain_ns = probe.snapshot_and_reset(); // 分段计时

    for (int i = 0; i < N; ++i) {
        SPIN_HINT(); // x86: _mm_pause()；ARM: yield
        sink = sink + i;
    }
    const auto t_pause_ns = probe.snapshot_and_reset();

    CHECK(t_plain_ns > 0);
    CHECK(t_pause_ns > 0);

    std::cout << "  100 万次空转（含 volatile 加法）:\n";
    std::cout << "  无提示耗时: " << t_plain_ns / 1000 << " us\n";
    std::cout << "  加提示耗时: " << t_pause_ns / 1000 << " us\n";
    std::cout << "  （PAUSE 不保证更快，但显著降低自旋功耗与流水线冲刷）\n";
    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：LatencyProbe 分段计时 + LatencyHistogram 已知分布统计
// 灌入 1000 个 100ns + 50 个 10000ns：
//   count=1050, min=100, max=10000, p50=100, p95=p99=p999=10000, mean=571
// ===========================================================================
void test_performance_probe() {
    std::cout << "=== 测试4：LatencyProbe + LatencyHistogram ===\n";

    // 4a. LatencyProbe：elapsed_ns 与 snapshot_and_reset
    tp::LatencyProbe probe;
    volatile double  acc = 0.0;
    for (int i = 0; i < 100000; ++i) {
        acc = acc + 1.0;
    }
    const auto busy_ns = probe.snapshot_and_reset();
    const auto idle_ns = probe.snapshot_and_reset(); // 立即再拍，应远小于上一段
    CHECK(busy_ns > 0);
    CHECK(idle_ns <= busy_ns);

    // 4b. LatencyHistogram：已知分布
    tp::LatencyHistogram hist;
    for (int i = 0; i < 1000; ++i) {
        hist.record(100);
    }
    for (int i = 0; i < 50; ++i) {
        hist.record(10'000); // 尾部毛刺
    }

    const auto stats = hist.compute();
    CHECK(stats.count == 1050);          // 总采样数
    CHECK(stats.sample_count == 1050);   // 窗口未满，全部有效
    CHECK(stats.min_ns == 100);
    CHECK(stats.max_ns == 10'000);
    CHECK(stats.p50_ns == 100);          // 中位数落在主体
    // p95 下标 = 1050*95/100 = 997 < 1000 → 仍在主体内（5% 尾部不够命中毛刺）
    CHECK(stats.p95_ns == 100);
    CHECK(stats.p99_ns == 10'000);       // p99 下标 1039 ≥ 1000 → 命中尾部毛刺
    CHECK(stats.p999_ns == 10'000);
    CHECK(stats.mean_ns == 571);         // (1000*100 + 50*10000) / 1050
    CHECK(stats.stddev_ns > 0.0);

    std::cout << "  分布: 1000 个 100ns + 50 个 10000ns\n";
    std::cout << "  count=" << stats.count << "  min=" << stats.min_ns
              << "  p50=" << stats.p50_ns << "  p95=" << stats.p95_ns
              << "  p99=" << stats.p99_ns << "  max=" << stats.max_ns
              << "  mean=" << stats.mean_ns << "  stddev=" << stats.stddev_ns << "\n";
    std::cout << "  LatencyProbe 分段: 忙等段 " << busy_ns / 1000 << " us > 空闲段 "
              << idle_ns << " ns\n";
    std::cout << "测试4通过\n\n";
}

// ===========================================================================
// 测试5：SystemInfo —— std::expected 返回语义
// 读源码确认：真实 API 是用户名/主机名（get_username/get_hostname），
// 不是核心数；返回 expected<string_view, string>，失败携带错误文本
// ===========================================================================
void test_system_info() {
    std::cout << "=== 测试5：SystemInfo ===\n";

    // 两个分支都是合法结果，都要验证：
    // 有值 → 非空 string_view；无值 → error() 携带非空错误文本
    const auto username = tp::SystemInfo::get_username();
    if (username.has_value()) {
        CHECK(!username->empty());
        std::cout << "  username = " << *username << "\n";
        // 与环境变量交叉验证（仅当环境提供时）
        if (const char* env_user = std::getenv("USER")) {
            CHECK(*username == env_user);
        }
    } else {
        CHECK(!username.error().empty()); // 错误分支也要有可读信息
        std::cout << "  username 不可用（无登录会话）: " << username.error() << "\n";
    }

    const auto hostname = tp::SystemInfo::get_hostname();
    if (hostname.has_value()) {
        CHECK(!hostname->empty());
        std::cout << "  hostname = " << *hostname << "\n";
    } else {
        CHECK(!hostname.error().empty());
        std::cout << "  hostname 不可用: " << hostname.error() << "\n";
    }

    std::cout << "测试5通过\n\n";
}

// ===========================================================================
// 测试6：channel —— 把 stage4 的底层三缓冲接到调度器风格的上层用法
// SPSC：make → split → write/read + borrow_mut/publish 原地构造
// SPMC：clone_reader 出第二个消费者，两个读者独立消费同一帧
// ===========================================================================
void test_channel() {
    std::cout << "=== 测试6：channel 通道封装 ===\n";

    // 6a. SPSC 通道
    auto spsc = tp::make_spsc_channel<int>();
    static_assert(decltype(spsc)::is_spsc);
    auto [w, r] = spsc.split();
    CHECK(static_cast<bool>(w) && static_cast<bool>(r));

    w.write(42);
    auto v1 = r.read();
    CHECK(v1.has_value() && *v1 == 42);
    CHECK(!r.read().has_value()); // 已消费，无新帧

    // borrow_mut + publish：原地改写槽位后发布，省一次 move
    w.borrow_mut() = 99;
    w.publish();
    auto v2 = r.read();
    CHECK(v2.has_value() && *v2 == 99);

    // 6b. SPMC 通道：一个生产者，两个消费者
    auto spmc = tp::make_spmc_channel<std::string>();
    static_assert(decltype(spmc)::is_spmc);

    auto reader_b = spmc.clone_reader(); // 先克隆出第二个消费者
    auto [w2, r1] = spmc.split();        // 再拆出主读写对
    CHECK(static_cast<bool>(reader_b));

    w2.write(std::string("frame#1"));
    const auto gen = w2.generation();
    CHECK(gen == 1); // 首次写入，版本号 0 → 1

    auto a = r1.read();        // 消费者A
    auto b = reader_b.read();  // 消费者B：继承克隆点的版本标记，独立判断新帧
    CHECK(a.has_value() && *a == "frame#1");
    CHECK(b.has_value() && *b == "frame#1");
    CHECK(!r1.read().has_value());        // 各自消费完毕
    CHECK(!reader_b.read().has_value()); //

    std::cout << "  SPSC: write/read 42 -> borrow_mut/publish 99 均正确\n";
    std::cout << "  SPMC: generation=" << gen << "，两个消费者各自读到 frame#1\n";
    std::cout << "测试6通过\n\n";
}

// ===========================================================================
// 主函数：依次运行所有测试，任一断言失败返回非零
// ===========================================================================
int main() {
    test_lazy();            // 1. lazy 延迟构造
    test_overloaded();      // 2. overloaded + variant 分发
    test_spin_hint();       // 3. SPIN_HINT 定性对比
    test_performance_probe(); // 4. 时延探针 + 直方图
    test_system_info();     // 5. 系统信息
    test_channel();         // 6. 通道封装

    if (g_failures == 0) {
        std::cout << "=== stage7 primitive 模块全部测试通过 ===\n";
        return 0;
    }
    std::cerr << "=== stage7 失败断言数: " << g_failures << " ===\n";
    return 1;
}
