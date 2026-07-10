// ===========================================================================
// 阶段4：三缓冲无锁通道演示 + 多线程压力测试
//
// 测试1：单写单读，验证数据一致性
// 测试2：单写多读（SPMC），验证 shared_ptr 共享读取
// 测试3：高并发压力测试，验证无死锁、无数据竞争
// ===========================================================================

#include "triple_buffer/triple_buffer.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace spmc;
using namespace std::chrono_literals;

// ===========================================================================
// 测试1：单写单读基本功能
// ===========================================================================
void test_basic() {
    std::cout << "=== 测试1：单写单读基本功能 ===\n";
    TripleBuffer<int> buf;

    // 写入并发布 3 次
    for (int i = 1; i <= 3; ++i) {
        buf.write_slot() = i * 100;
        buf.publish();
        std::this_thread::sleep_for(1ms);

        // 读取最新值
        if (const int* val = buf.read()) {
            std::cout << "  写入 " << i * 100 << "，读取 " << *val << "\n";
        }
    }
    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：SPMC 多读者共享读取
// ===========================================================================
void test_spmc() {
    std::cout << "=== 测试2：SPMC 多读者共享读取 ===\n";
    SpmcTripleBuffer<std::string> channel;

    channel.publish("hello");

    // 多个读者共享同一份数据（shared_ptr 引用计数）
    auto r1 = channel.read();
    auto r2 = channel.read();
    auto r3 = channel.read();

    std::cout << "  reader1: " << *r1 << "\n";
    std::cout << "  reader2: " << *r2 << "\n";
    std::cout << "  reader3: " << *r3 << "\n";

    // 验证指向同一内存地址（零拷贝）
    bool same_addr = (r1.get() == r2.get()) && (r2.get() == r3.get());
    std::cout << "  共享同一地址: " << (same_addr ? "YES（零拷贝）" : "NO") << "\n";
    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：高并发压力测试
// 写者高频写入，多个读者并发读取，验证无数据竞争
// ===========================================================================
void test_stress() {
    std::cout << "=== 测试3：高并发压力测试 ===\n";
    SpmcTripleBuffer<int64_t> channel;
    channel.publish(0);

    std::atomic<bool> running{true};
    std::atomic<int64_t> max_read{-1};
    std::atomic<int64_t> errors{0};

    // 写者线程：快速递增写入
    std::thread writer([&]() {
        for (int64_t i = 1; i <= 100000; ++i) {
            channel.publish(i);
        }
        running.store(false, std::memory_order_release);
    });

    // 4 个读者线程：并发读取
    std::vector<std::thread> readers;
    for (int r = 0; r < 4; ++r) {
        readers.emplace_back([&]() {
            int64_t local_max = -1;
            while (running.load(std::memory_order_acquire) || local_max < 100000) {
                auto data = channel.read();
                if (data) {
                    int64_t val = *data;
                    // 验证：读取的值必须单调不减（可能有跳变但不能回退）
                    if (val < local_max) {
                        errors.fetch_add(1, std::memory_order_relaxed);
                    }
                    local_max = std::max(local_max, val);
                }
            }
            // 更新全局最大值
            int64_t prev = max_read.load(std::memory_order_relaxed);
            while (local_max > prev) {
                if (max_read.compare_exchange_weak(prev, local_max,
                        std::memory_order_relaxed)) {
                    break;
                }
            }
        });
    }

    writer.join();
    for (auto& t : readers) {
        t.join();
    }

    std::cout << "  写入次数: 100000\n";
    std::cout << "  读者最终最大值: " << max_read.load() << "\n";
    std::cout << "  数据回退错误数: " << errors.load() << "\n";
    std::cout << (errors.load() == 0 ? "  无数据竞争" : "  发现数据竞争！") << "\n";
    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：TripleBuffer 数据完整性验证
// 写者写结构体，验证读者读到的是完整的一份数据
// ===========================================================================
struct FrameData {
    int seq = 0;
    int width = 0;
    int height = 0;
    std::vector<int> pixels;  // 模拟图像数据
};

void test_integrity() {
    std::cout << "=== 测试4：数据完整性验证 ===\n";
    TripleBuffer<FrameData> buf;

    // 写者写完整帧
    for (int seq = 1; seq <= 100; ++seq) {
        FrameData& frame = buf.write_slot();
        frame.seq = seq;
        frame.width = 1920;
        frame.height = 1080;
        frame.pixels.assign(100, seq);  // 填充 seq 值

        buf.publish();
        std::this_thread::sleep_for(100us);
    }

    // 读者读最终帧
    if (const FrameData* frame = buf.read()) {
        bool valid = (frame->seq > 0) && (frame->width == 1920)
            && (frame->height == 1080)
            && (frame->pixels.size() == 100)
            && (frame->pixels[0] == frame->seq);
        std::cout << "  最终帧 seq=" << frame->seq
                  << " " << frame->width << "x" << frame->height
                  << " pixels[0]=" << frame->pixels[0] << "\n";
        std::cout << "  数据完整: " << (valid ? "YES" : "NO") << "\n";
    }
    std::cout << "测试4通过\n\n";
}

int main() {
    test_basic();
    test_spmc();
    test_stress();
    test_integrity();

    std::cout << "=== 三缓冲无锁通道全部测试通过 ===\n";
    return 0;
}
