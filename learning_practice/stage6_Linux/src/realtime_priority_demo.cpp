#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <sched.h>
#include <pthread.h>
#include <fmt/format.h>
#include <expected>
#include <cstring>

using AffinityResult = std::expected<void, std::string>;

// ---------- 工具函数 ----------

// 设置实时优先级（可以自定义调度策略）
AffinityResult set_realtime_priority(int priority, int policy = SCHED_FIFO) noexcept {
#ifdef __linux__
    sched_param param{};
    param.sched_priority = priority;
    int ret = pthread_setschedparam(pthread_self(), policy, &param);
    if (ret != 0) {
        return std::unexpected(
            fmt::format("设置实时优先级失败 (prio={}): {}", priority, std::strerror(ret)));
    }
    return {};
#else
    return std::unexpected("非 Linux 系统，不支持实时调度");
#endif
}

// 绑定线程到指定CPU核心
void pin_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

// 捣乱线程：占满一个CPU核心
void busy_worker(std::atomic<bool>& running) {
    while (running) {
        // 空转
    }
}

// 纯 CPU 密集计算任务（防优化）
void do_work() {
    auto start = std::chrono::high_resolution_clock::now();
    double x = 1.0;
    for (int i = 0; i < 1000000000; ++i) {
        x += i * 0.0000001;
    }
    if (x < 0) std::cout << "Debug" << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  处理耗时: " << duration << " ms" << std::endl;
}

// 带 sched_yield 的 CPU 密集任务（测试6用）
void do_work_with_yield() {
    auto start = std::chrono::high_resolution_clock::now();
    double x = 1.0;
    for (int i = 0; i < 1000000000; ++i) {
        x += i * 0.0000001;
        // 每隔 1000 万次主动让出一次CPU
        if (i % 10000000 == 0) {
            sched_yield();
        }
    }
    if (x < 0) std::cout << "Debug" << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  处理耗时: " << duration << " ms" << std::endl;
}

// ---------- 测试函数 ----------

// 测试3：优先级压制（99 vs 50）
void test_priority_pressure() {
    std::cout << "\n=== 测试3：优先级压制（99 vs 50）===\n";
    std::atomic<bool> running{true};

    std::thread busy([&]() {
        pin_to_core(0);
        busy_worker(running);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::thread high_prio([&]() {
        pin_to_core(0);
        set_realtime_priority(99);  // 最高优先级
        std::cout << "  [Prio 99] 线程开始跑:" << std::endl;
        do_work();
    });

    std::thread low_prio([&]() {
        pin_to_core(0);
        set_realtime_priority(50);  // 中等优先级
        std::cout << "  [Prio 50] 线程开始跑:" << std::endl;
        do_work();
    });

    high_prio.join();
    low_prio.join();

    running = false;
    busy.join();
}

// 测试4：FIFO vs RR
void test_fifo_vs_rr() {
    std::cout << "\n=== 测试4：SCHED_FIFO vs SCHED_RR（同优先级）===\n";
    std::atomic<bool> running{true};

    std::thread busy([&]() {
        pin_to_core(0);
        busy_worker(running);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 两个实时线程，优先级都是50
    std::thread t1([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_FIFO);
        std::cout << "  [FIFO] 线程1开始:" << std::endl;
        do_work();
    });

    std::thread t2([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_FIFO);
        std::cout << "  [FIFO] 线程2开始:" << std::endl;
        do_work();
    });

    t1.join();
    t2.join();

    running = false;
    busy.join();

    // 再测 RR
    std::cout << "  --- 切换为 SCHED_RR ---\n";
    running = true;
    std::thread busy2([&]() {
        pin_to_core(0);
        busy_worker(running);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::thread r1([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_RR);
        std::cout << "  [RR] 线程1开始:" << std::endl;
        do_work();
    });

    std::thread r2([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_RR);
        std::cout << "  [RR] 线程2开始:" << std::endl;
        do_work();
    });

    r1.join();
    r2.join();

    running = false;
    busy2.join();
}

// 测试5：绑核隔离
void test_bind_core_isolation() {
    std::cout << "\n=== 测试5：绑核隔离（独占核0 vs 自由飞）===\n";

    // 情况A：绑核
    {
        std::atomic<bool> running{true};
        std::thread busy([&]() {
            pin_to_core(0);
            busy_worker(running);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::thread worker([&]() {
            pin_to_core(0);
            set_realtime_priority(50);
            do_work();
        });
        worker.join();
        running = false;
        busy.join();
    }

    // 情况B：不绑核
    {
        std::atomic<bool> running{true};
        std::thread busy([&]() {
            pin_to_core(0);
            busy_worker(running);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::thread worker([&]() {
            // 不调用 pin_to_core，让操作系统随便分配
            set_realtime_priority(50);
            do_work();
        });
        worker.join();
        running = false;
        busy.join();
    }
}

// 测试6：主动让出对实时线程的破坏
void test_sched_yield() {
    std::cout << "\n=== 测试6：主动让出（sched_yield）对实时线程的破坏 ===\n";
    std::atomic<bool> running{true};

    std::thread busy([&]() {
        pin_to_core(0);
        busy_worker(running);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 不带 sched_yield
    std::thread no_yield([&]() {
        pin_to_core(0);
        set_realtime_priority(50);
        std::cout << "  [无让出] 实时线程:" << std::endl;
        do_work();
    });
    no_yield.join();

    // 带 sched_yield
    std::thread with_yield([&]() {
        pin_to_core(0);
        set_realtime_priority(50);
        std::cout << "  [有让出] 实时线程:" << std::endl;
        do_work_with_yield();
    });
    with_yield.join();

    running = false;
    busy.join();
}

int main() {
    std::cout << "当前系统核心数: " << std::thread::hardware_concurrency() << "\n" << std::endl;

    test_priority_pressure();    // 测试3
    test_fifo_vs_rr();           // 测试4
    test_bind_core_isolation();  // 测试5
    test_sched_yield();          // 测试6

    std::cout << "\n=== 全部测试完成 ===\n";
    return 0;
}