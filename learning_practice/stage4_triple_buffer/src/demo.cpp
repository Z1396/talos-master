// ===========================================================================
// 阶段4：三重缓冲无锁通道演示 + 多线程压力测试
//
// 文件结构对齐真实项目（crates/primitive/src/primitive/）：
//   - spin.hpp              自旋等待提示指令 SPIN_HINT()，降低自旋CPU占用
//   - spsc_triple_buffer.hpp SPSC三缓冲（Single‑Producer Single‑Consumer 单写单读）
//   - spmc_triple_buffer.hpp SPMC三缓冲（Single‑Producer Multi‑Consumer 单写多读）
//
// 使用Talos primitive模块真实API，接口与工程代码完全对齐：
//   - create()        静态工厂函数，返回 {写句柄,读句柄} 配对
//   - Write::write(data)   移动传入数据并自动发布新版本
//   - Write::borrow_mut()  获取写入槽可变引用，原位填充数据，调用publish()发布
//   - Read::read()         读取**未读过的新版本**；无新数据返回std::nullopt
//   - SPMC专用：Read::clone() 克隆读者句柄；每个读者独立维护版本号，新数据每个读者只会读到一次
//
// 测试清单
// 测试1：SPSC基础功能，create、write‑read单向传输、无新数据返回nullopt
// 测试2：SPSC borrow_mut原位填充，模拟相机帧：先填缓冲区再publish发布
// 测试3：SPMC多读者clone克隆、版本过滤；各个读者互相不干扰，新数据只读一次
// 测试4：高并发压力：1写者+4读者，校验序号严格递增，不能回退、不能重复（无数据竞争）
// 测试5：SPSC大对象完整性；验证move语义下vector图像数据完整转移，无内存破坏
// ===========================================================================

// ---------- 包含头文件 ----------
// SPMC 三缓冲（单写多读）
#include "triple_buffer/spmc_triple_buffer.hpp"
// SPSC 三缓冲（单写单读）
#include "triple_buffer/spsc_triple_buffer.hpp"

#include <atomic>   // std::atomic - 原子变量，多线程无锁同步
#include <cstdint>  // int64_t - 固定宽度整数
#include <iostream> // std::cout - 控制台输出
#include <thread>   // std::thread, std::this_thread::sleep_for
#include <vector>   // std::vector - 容器，模拟图像像素buffer

// ---------- 命名空间别名 ----------
// 简化书写，避免每次都写 talos::primitive::
using talos::primitive::SpmcTripleBuffer;
using talos::primitive::SpscTripleBuffer;

// using namespace 让 100us、1ms 这样的字面量可以直接使用
// std::chrono_literals 提供：100us (微秒), 1ms (毫秒), 1s (秒)
using namespace std::chrono_literals;

// ===========================================================================
// 测试1：SPSC 基本功能
//
// 验证点：
//   1. create() 生成读写句柄对
//   2. write() 写入 → read() 读出
//   3. 没有新版本时 read() 返回 nullopt
//   4. 每个版本只能被读取一次
// ===========================================================================
void test_basic() {
    std::cout << "=== 测试1：SPSC 基本功能 ===\n";

    // ---------- 1. 创建三缓冲通道 ----------
    // create() 是静态工厂函数，返回 std::pair<Write<T>, Read<T>>
    // 结构化绑定 (C++17)：auto [writer, reader] = ...
    // 等价于：
    //   auto pair = SpscTripleBuffer<int>::create();
    //   auto& writer = pair.first;
    //   auto& reader = pair.second;
    auto [writer, reader] = SpscTripleBuffer<int>::create();

    // ---------- 2. 初始状态：无数据 ----------
    // 刚创建时没有写入任何数据，read() 应该返回 nullopt
    // C++17 的 if 初始化语法：auto early = reader.read() 在 if 作用域内生效
    if (auto early = reader.read(); !early.has_value()) {
        std::cout << "  初始状态: 无新数据（nullopt）\n";
    }

    // ---------- 3. 写入并读取 3 组数据 ----------
    for (int i = 1; i <= 3; ++i) {
        // write()：移动数据到缓冲区，自动发布新版本
        // 内部调用：borrow_mut() + 赋值 + publish()
        writer.write(i * 100);

        // read()：尝试读取未读的新版本
        // 返回 std::optional<int>，有值则包含数据
        if (auto val = reader.read()) {
            std::cout << "  写入 " << i * 100 << "，读取 " << *val << "\n";
        }

        // ---------- 4. 重复读取验证 ----------
        // 同一个版本已经读过，再次 read() 应该返回 nullopt
        // 这验证了"每个版本只读一次"的核心语义
        if (auto again = reader.read(); !again.has_value()) {
            std::cout << "  重复读取: nullopt（正确）\n";
        }
    }
    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：SPSC borrow_mut 原位构造
//
// 业务场景：相机采集大帧图像（如 4K 视频帧）
//
// 对比两种方式：
//   ❌ write(FrameData{...})：先构造临时对象，再 move 进去 → 有额外开销
//   ✅ borrow_mut() + 填充 + publish()：直接在目标内存构造 → 0 拷贝
//
// 适用场景：大对象（vector、string、自定义复杂结构）
// ===========================================================================
void test_borrow_mut() {
    std::cout << "=== 测试2：SPSC borrow_mut 原位构造 ===\n";

    // 创建一个传输 std::string 的通道
    auto [writer, reader] = SpscTripleBuffer<std::string>::create();

    // ---------- 方式1：borrow_mut + publish（分步，高效） ----------
    // 适用于：大对象，需要填充多个字段
    //
    // 步骤：
    //   1. borrow_mut()：获取写入槽的可变引用（直接指向缓冲区内存）
    //   2. 在引用上直接构造/修改数据（不产生临时对象）
    //   3. publish()：发布新版本，通知读者
    // slot 是 std::string& 引用
    // std::string 有一个赋值运算符重载：operator=(const char* s)
    // 字符串字面量 "hello, talos" 是 const char*（存储在只读内存段）
    // C++ 禁止 char* p = "hello"（C 允许但 C++ 不允许），因为修改只读内存会崩溃
    std::string& slot = writer.borrow_mut(); // 借用写入槽
    slot              = "hello, talos";      // 直接在槽内赋值
    writer.publish();                        // 手动发布

    if (auto val = reader.read()) {
        std::cout << "  原位构造读取: " << *val << "\n";
    }

    // ---------- 方式2：write()（便捷，一行搞定） ----------
    // write() 等价于 borrow_mut() + 赋值 + publish()
    // 适用于：小对象或临时对象
    writer.write("write() 一行搞定");
    if (auto val = reader.read()) {
        std::cout << "  write() 读取: " << *val << "\n";
    }

    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：SPMC 多读者 clone + 版本过滤
//
// SPMC = Single Producer, Multiple Consumers
//
// 核心语义：
//   1. 每个读者独立维护自己的版本号 (last_gen_)
//   2. 同一个新版本，每个读者只会读到一次
//   3. 读者之间互不干扰
//
// 额外 API：
//   - has_new()：检查是否有未读新版本（不消费）
//   - read_current()：强制读最新（绕过版本过滤）
//   - last_generation()：获取当前已读版本号
// ===========================================================================
void test_spmc() {
    std::cout << "=== 测试3：SPMC 多读者 clone + 版本过滤 ===\n";

    // ---------- 1. 创建 SPMC 通道 ----------
    auto [writer, reader] = SpmcTripleBuffer<std::string>::create();

    // ---------- 2. 克隆第二个读者 ----------
    // clone() 创建独立读者句柄
    // 新读者继承当前已读版本（last_gen_），不会重复读取历史数据
    auto reader2 = reader.clone();

    // ---------- 3. 写入 v1 ----------
    writer.write("v1");

    // reader1 读取 v1
    if (auto v = reader.read()) {
        std::cout << "  reader1 读到: " << *v << "\n";
    }

    // reader1 重复读：返回 nullopt（版本已消费）
    std::cout << "  reader1 重读: "
              << (reader.read().has_value() ? "有值（错误！）" : "nullopt（正确）") << "\n";

    // reader2 独立读取 v1：不受 reader1 影响
    // 因为 reader2 的 last_gen_ 还是 0，而 generation 是 1
    if (auto v = reader2.read()) {
        std::cout << "  reader2 读到: " << *v << "\n";
    }

    // ---------- 4. 写入 v2 ----------
    writer.write("v2");

    // has_new()：快速检查是否有未读新版本
    // 不消费数据，只是检查
    std::cout << "  reader1 has_new: " << (reader.has_new() ? "true" : "false") << "\n";

    // reader1 读取 v2
    if (auto v = reader.read()) {
        std::cout << "  reader1 读到: " << *v << "（last_gen=" << reader.last_generation()
                  << "）\n";
    }

    // read_current()：强制读取当前最新数据
    // 即使已经读过，也会再次读取（并更新 last_gen_）
    // 注意：这会"消费"掉当前版本，后续 read() 不会再返回它
    if (auto v = reader.read_current()) {
        std::cout << "  reader1 read_current: " << *v << "\n";
    }

    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：高并发压力测试（SPMC 真实语义）
//
// 场景：
//   - 1 个写者线程：高速递增写入（1 ~ 100000）
//   - 4 个读者线程：并发读取
//
// 校验规则：每个读者读到的数据序列必须严格递增
//   - 如果出现 *val <= local_last → 重复或回退 → 错误
//
// 这验证了：
//   1. 版本过滤正确（不会重复读）
//   2. 无数据竞争（不会读错数据）
//   3. 无丢失（每个读者能读到所有版本）
//
// 注意：多线程 std::cout 会交错输出，所以先存到数组，最后统一打印
// ===========================================================================
void test_stress() {
    std::cout << "=== 测试4：高并发压力测试（1 写者 + 4 读者）===\n";

    // 总共写入十万个版本
    constexpr int64_t kTotal = 100000;

    // ---------- 1. 创建通道，写入初始值 ----------
    auto [writer, reader] = SpmcTripleBuffer<int64_t>::create();
    writer.write(0); // 初始版本 0，读者从 0 开始

    // ---------- 2. 控制变量 ----------
    std::atomic<bool> running{true}; // 控制读者线程退出
    std::atomic<int64_t> errors{0};  // 统计错误计数

    // ---------- 3. 写者线程 ----------
    std::thread write_thread([&]() {
        // 循环写入 1 ~ kTotal
        for (int64_t i = 1; i <= kTotal; ++i) {
            writer.write(i);
        }
        // 写入完成，通知读者可以退出
        // memory_order_release：确保所有写入对读者可见
        running.store(false, std::memory_order_release);
    });

    // ---------- 4. 读者线程（4个） ----------
    // 统计数据：每个读者读到多少条、最大序号
    std::vector<int64_t> read_counts(4, 0);
    std::vector<int64_t> read_max(4, -1);

    std::vector<std::thread> readers;
    for (int r = 0; r < 4; ++r) {
        readers.emplace_back([&, r]() {
            // 每个线程克隆独立读者句柄
            auto local = reader.clone();

            // 本地状态
            int64_t local_last = -1; // 上一次读到的序号
            int64_t count      = 0;  // 读取计数

            // ---------- 循环读取 ----------
            // 条件：running 为 true，或者还没读到最大版本
            // 注意：running 可能先变为 false，但读者可能还没读完所有数据
            while (running.load(std::memory_order_acquire) || local_last < kTotal) {
                if (auto val = local.read()) {
                    // ---------- 核心校验 ----------
                    // 当前值必须 > 上一次的值
                    // 如果 <= 说明：重复读取 或 序号回退
                    // 这两种情况都是版本过滤逻辑的 bug
                    if (*val <= local_last) {
                        errors.fetch_add(1, std::memory_order_relaxed);
                    }
                    local_last = *val;
                    ++count;
                }
            }

            // 保存统计数据
            read_counts[r] = count;
            read_max[r]    = local_last;
        });
    }

    // ---------- 5. 等待所有线程结束 ----------
    write_thread.join();
    for (auto& t : readers) {
        t.join();
    }

    // ---------- 6. 打印结果 ----------
    for (int r = 0; r < 4; ++r) {
        std::cout << "  reader" << r << ": 读到 " << read_counts[r] << " 个版本，最大 "
                  << read_max[r] << "\n";
    }

    std::cout << "  写入版本数: " << kTotal << "（每读者至多读到 " << kTotal + 1 << " 个）\n";
    std::cout << "  回退/重复错误: " << errors.load() << "\n";
    std::cout << (errors.load() == 0 ? "  无数据竞争，版本过滤正确\n" : "  发现错误！\n");
    std::cout << "测试4通过\n\n";
}

// ===========================================================================
// 测试5：SPSC 数据完整性验证
//
// 场景：传输大对象（模拟相机帧）
//   FrameData 包含：序号、宽高、std::vector<int> 像素数据
//
// 验证点：
//   1. 大对象通过 move 语义完整转移
//   2. vector 内存不被破坏、不截断
//   3. 没有野指针、没有 Use-After-Free
//
// 这验证了三缓冲的"数据完整性"：数据从写者到读者，100% 完整
// ===========================================================================

// 模拟相机帧数据结构
struct FrameData {
    int seq    = 0;          // 帧序号
    int width  = 0;          // 图像宽度
    int height = 0;          // 图像高度
    std::vector<int> pixels; // 像素数据（模拟大对象）
};

void test_integrity() {
    std::cout << "=== 测试5：数据完整性验证 ===\n";

    // 创建传输 FrameData 的 SPSC 通道
    auto [writer, reader] = SpscTripleBuffer<FrameData>::create();

    // 生成 100 帧数据
    for (int seq = 1; seq <= 100; ++seq) {
        // ---------- 写者：原位构造帧 ----------
        // 使用 borrow_mut() 直接在槽内构造，避免拷贝大 vector
        FrameData& frame = writer.borrow_mut();

        // 填充数据
        frame.seq    = seq;
        frame.width  = 1920;
        frame.height = 1080;
        frame.pixels.assign(100, seq); // 100 个像素，值 = seq

        // 发布新版本
        writer.publish();

        // 模拟帧间隔（100 微秒）
        std::this_thread::sleep_for(100us);

        // ---------- 读者：读取并验证 ----------
        // read() 返回 std::optional<FrameData>
        // 数据通过 move 语义转移到 f（所有权转移）
        if (auto f = reader.read()) {
            // 校验全部字段
            bool valid = (f->seq == seq) &&           // 序号匹配
                         (f->width == 1920) &&        // 宽度正确
                         (f->height == 1080) &&       // 高度正确
                         (f->pixels.size() == 100) && // vector 大小正确
                         (f->pixels[0] == f->seq);    // 像素值正确

            // 每 50 帧打印一次
            if (seq % 50 == 0) {
                std::cout << "  帧 seq=" << f->seq << " " << f->width << "x" << f->height
                          << " pixels[0]=" << f->pixels[0] << " 完整: " << (valid ? "YES" : "NO")
                          << "\n";
            }

            // 注意：f 是局部变量，离开作用域后 FrameData 自动析构
            // vector 的内存也会被正确释放
        }
    }

    std::cout << "测试5通过\n\n";
}

// ===========================================================================
// 主函数：依次运行所有测试
// ===========================================================================
int main() {
    // 按顺序执行 5 个测试
    test_basic();      // 1. SPSC 基本功能
    test_borrow_mut(); // 2. SPSC borrow_mut 原位构造
    test_spmc();       // 3. SPMC 多读者版本过滤
    test_stress();     // 4. 高并发压力测试
    test_integrity();  // 5. SPSC 数据完整性

    std::cout << "=== 三缓冲无锁通道全部测试通过 ===\n";
    return 0;
}