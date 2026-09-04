// ============================================================================
// 实时调度与CPU亲和性测试程序
// 
// 目的：演示和验证 Linux 实时调度策略对线程执行的影响
// 测试内容：
//   - 实时优先级调度（SCHED_FIFO/RR）
//   - CPU 核心绑定（CPU affinity）
//   - 优先级反转/压制
//   - sched_yield() 对实时线程的影响
// ============================================================================

#include <iostream>      // 标准输入输出
#include <thread>        // std::thread 多线程
#include <chrono>        // 时间相关（延时、计时）
#include <atomic>        // std::atomic 原子操作（线程安全标志）
#include <sched.h>       // sched 调度相关（sched_param, sched_yield）
#include <pthread.h>     // POSIX 线程（pthread_setschedparam, pthread_setaffinity_np）
#include <fmt/format.h>  // fmt 库：格式化字符串（类似 Python 的 f-string）
#include <expected>      // C++23 std::expected：带错误信息的返回值
#include <cstring>       // std::strerror：将 errno 转为字符串

// 定义别名：AffinityResult 表示"返回 void 或错误信息"
// std::expected<void, std::string> 类似于 Rust 的 Result<(), String>
// 成功时：返回 {}（空值），失败时：返回 std::unexpected("错误信息")
using AffinityResult = std::expected<void, std::string>;


// ============================================================================
// 工具函数
// ============================================================================

// ---------- 设置实时优先级 ----------
// 参数：
//   - priority: 实时优先级（1~99，数字越大优先级越高）
//   - policy: 调度策略，默认 SCHED_FIFO（先进先出）
// 返回：
//   - 成功：std::expected<void, std::string> 包含空值
//   - 失败：std::expected<void, std::string> 包含错误信息
AffinityResult set_realtime_priority(int priority, int policy = SCHED_FIFO) noexcept {
#ifdef __linux__
    // 1. 定义调度参数结构体
    sched_param param{};
    // 2. 设置实时优先级（1~99，SCHED_FIFO/SCHED_RR 的有效范围）
    param.sched_priority = priority;
    
    // 3. 调用 pthread_setschedparam 设置当前线程的调度策略和优先级
    //    pthread_self()：获取当前线程的 ID
    //    policy：SCHED_FIFO 或 SCHED_RR
    //    &param：调度参数（包含优先级）
    //    返回值：0 成功，非 0 失败（返回 errno）
    int ret = pthread_setschedparam(pthread_self(), policy, &param);
    
    // 4. 检查是否成功
    if (ret != 0) {
        // 失败：返回错误信息
        // fmt::format 类似 Python f"{priority}: {strerror(ret)}"
        // std::strerror(ret) 将 errno 转为可读字符串
        return std::unexpected(
            fmt::format("设置实时优先级失败 (prio={}): {}", priority, std::strerror(ret)));
    }
    
    // 5. 成功：返回空值（std::expected 的默认构造表示成功）
    return {};
#else
    // 非 Linux 系统不支持实时调度
    return std::unexpected("非 Linux 系统，不支持实时调度");
#endif
}


// ---------- 绑定线程到指定 CPU 核心 ----------
// 参数：core_id - 要绑定的 CPU 核心编号（0, 1, 2, ...）
// 作用：将当前线程固定运行在指定 CPU 核心上
void pin_to_core(int core_id) {
    // 1. 定义 CPU 集合变量（位图，每位代表一个 CPU 核心）
    cpu_set_t cpuset;
    
    // 2. 清空 CPU 集合（所有位设为 0）
    CPU_ZERO(&cpuset);
    
    // 3. 将指定的核心编号对应的位设为 1
    //    例如 core_id=0：绑定到 CPU 0
    CPU_SET(core_id, &cpuset);
    
    // 4. 调用 pthread_setaffinity_np 设置当前线程的 CPU 亲和性
    //    pthread_self()：当前线程 ID
    //    sizeof(cpu_set_t)：CPU 集合的大小
    //    &cpuset：CPU 集合（指定允许运行的核心）
    //    注：_np 表示 "non-portable"（非标准 POSIX 扩展，仅 Linux）
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}


// ---------- 捣乱线程 ----------
// 作用：占满一个 CPU 核心，制造高负载环境
// 参数：running - 原子布尔标志，控制线程是否继续运行
// 行为：空转循环，不断消耗 CPU 时间片
void busy_worker(std::atomic<bool>& running) {
    while (running) {
        // 空转（什么也不做，纯粹消耗 CPU）
        // 编译器不会优化掉这个循环，因为 running 是 atomic
    }
}


// ---------- CPU 密集计算任务（无主动让出） ----------
// 作用：执行大量浮点运算，模拟 CPU 密集型工作负载
// 特点：无 sched_yield，会一直占用 CPU 直到计算完成
// 防优化：通过累加 x 并最后判断（防止编译器优化掉整个循环）
void do_work() {
    // 1. 记录开始时间
    auto start = std::chrono::high_resolution_clock::now();
    
    // 2. 执行大量浮点运算（10 亿次迭代）
    double x = 1.0;
    for (int i = 0; i < 1000000000; ++i) {
        x += i * 0.0000001;  // 浮点累加
    }
    
    // 3. 防止编译器优化：如果 x 为负数才输出（永远为 false，但编译器不知道）
    //    这样编译器无法证明循环是无用的，所以不会优化掉
    if (x < 0) std::cout << "Debug" << std::endl;
    
    // 4. 记录结束时间并计算耗时
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  处理耗时: " << duration << " ms" << std::endl;
}


// ---------- CPU 密集计算任务（带主动让出） ----------
// 作用：同 do_work()，但每隔一定次数主动调用 sched_yield()
// 区别：sched_yield() 会主动放弃 CPU，让调度器重新选择下一个线程
// 用于测试：主动让出对实时线程执行时间的影响
void do_work_with_yield() {
    auto start = std::chrono::high_resolution_clock::now();
    double x = 1.0;
    for (int i = 0; i < 1000000000; ++i) {
        x += i * 0.0000001;
        
        // 每隔 1000 万次迭代主动让出一次 CPU
        // 注意：i % 10000000 == 0 的判断本身也有开销
        if (i % 10000000 == 0) {
            sched_yield();  // 主动让出 CPU，调度器可运行其他线程
        }
    }
    if (x < 0) std::cout << "Debug" << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  处理耗时: " << duration << " ms" << std::endl;
}


// ============================================================================
// 测试函数
// ============================================================================

// ---------- 测试3：优先级压制（99 vs 50） ----------
// 目的：验证高优先级线程能压制低优先级线程
// 场景：同一个 CPU 核心上，优先级 99 和 50 的两个实时线程竞争
// 预期：优先级 99 的线程先完成，然后优先级 50 的线程才开始
void test_priority_pressure() {
    std::cout << "\n=== 测试3：优先级压制（99 vs 50）===\n";
    std::atomic<bool> running{true};  // 控制捣乱线程

    // 1. 启动捣乱线程（占满 CPU 核心 0）
    //    这会制造高负载环境，让调度决策更明显
    std::thread busy([&]() {
        pin_to_core(0);          // 绑定到核心 0
        busy_worker(running);     // 空转消耗 CPU
    });

    // 2. 等待 50ms，让捣乱线程稳定运行
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 3. 启动高优先级线程（99）
    std::thread high_prio([&]() {
        pin_to_core(0);                    // 绑定到核心 0（与捣乱线程竞争）
        set_realtime_priority(99);         // 最高实时优先级（SCHED_FIFO 默认）
        std::cout << "  [Prio 99] 线程开始跑:" << std::endl;
        do_work();                         // 执行 CPU 密集计算
    });

    // 4. 启动低优先级线程（50）
    std::thread low_prio([&]() {
        pin_to_core(0);                    // 绑定到核心 0
        set_realtime_priority(50);         // 中等实时优先级
        std::cout << "  [Prio 50] 线程开始跑:" << std::endl;
        do_work();                         // 执行 CPU 密集计算
    });

    // 5. 等待两个线程完成
    high_prio.join();
    low_prio.join();

    // 6. 停止捣乱线程
    running = false;
    busy.join();
    
    // 预期结果：Prio 99 的耗时 < Prio 50 的耗时
    // 因为高优先级会抢占低优先级
}


// ---------- 测试4：SCHED_FIFO vs SCHED_RR ----------
// 目的：演示 FIFO 和 RR 在"同优先级"下的调度差异
// 场景：两个同优先级的实时线程同时运行
//   - FIFO：第一个线程运行完，第二个才开始（先进先出）
//   - RR：两个线程轮流执行（时间片轮转）
void test_fifo_vs_rr() {
    std::cout << "\n=== 测试4：SCHED_FIFO vs SCHED_RR（同优先级）===\n";
    std::atomic<bool> running{true};

    // 捣乱线程：占满 CPU 核心 0
    std::thread busy([&]() {
        pin_to_core(0);
        busy_worker(running);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ========== FIFO 测试 ==========
    // 两个线程都是 SCHED_FIFO，优先级都是 50
    // 预期：线程1 先执行完，然后线程2 开始执行
    std::thread t1([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_FIFO);  // 指定 FIFO 策略
        std::cout << "  [FIFO] 线程1开始:" << std::endl;
        do_work();
    });

    std::thread t2([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_FIFO);  // 同优先级，同策略
        std::cout << "  [FIFO] 线程2开始:" << std::endl;
        do_work();
    });

    t1.join();
    t2.join();

    running = false;
    busy.join();

    // ========== RR 测试 ==========
    std::cout << "  --- 切换为 SCHED_RR ---\n";
    running = true;
    std::thread busy2([&]() {
        pin_to_core(0);
        busy_worker(running);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 两个线程都是 SCHED_RR，优先级都是 50
    // 预期：两个线程交替执行（时间片轮转），总耗时相近
    std::thread r1([&]() {
        pin_to_core(0);
        set_realtime_priority(50, SCHED_RR);  // 指定 RR 策略
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
    
    // 预期结果：
    // - FIFO：线程1 先完成，然后线程2 完成（时间差大）
    // - RR：两个线程几乎同时完成（时间差小）
}


// ---------- 测试5：绑核隔离 ----------
// 目的：验证 CPU 绑定对线程执行的影响
// 场景：捣乱线程占满核心 0，工作线程在相同核心 vs 自由调度
// 预期：绑核到核心 0 的线程被捣乱线程干扰，耗时更长
//       不绑核的线程可被调度到其他核心，不受干扰
void test_bind_core_isolation() {
    std::cout << "\n=== 测试5：绑核隔离（独占核0 vs 自由飞）===\n";

    // ========== 情况A：绑核 ==========
    {
        std::atomic<bool> running{true};
        // 捣乱线程占满核心 0
        std::thread busy([&]() {
            pin_to_core(0);
            busy_worker(running);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 工作线程也绑定到核心 0（与捣乱线程竞争）
        // 预期：耗时长，因为核心 0 已被捣乱线程占满
        std::thread worker([&]() {
            pin_to_core(0);             // 绑定到核心 0
            set_realtime_priority(50);  // 实时优先级，可抢占捣乱线程
            do_work();
        });
        worker.join();
        running = false;
        busy.join();
    }

    // ========== 情况B：不绑核 ==========
    {
        std::atomic<bool> running{true};
        // 捣乱线程占满核心 0
        std::thread busy([&]() {
            pin_to_core(0);
            busy_worker(running);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 工作线程不绑定核心（操作系统自由调度）
        // 预期：耗时短，因为会被调度到其他空闲核心
        std::thread worker([&]() {
            // 注意：这里不调用 pin_to_core()
            // 让操作系统自行决定运行在哪个核心
            set_realtime_priority(50);  // 实时优先级
            do_work();
        });
        worker.join();
        running = false;
        busy.join();
    }
}


// ---------- 测试6：sched_yield() 对实时线程的影响 ----------
// 目的：演示实时线程中调用 sched_yield() 会导致性能下降
// 场景：同一优先级的实时线程，有无 sched_yield 的耗时对比
// 预期：有 sched_yield 的线程耗时更长（频繁让出 CPU）
void test_sched_yield() {
    std::cout << "\n=== 测试6：主动让出（sched_yield）对实时线程的破坏 ===\n";
    std::atomic<bool> running{true};

    std::thread busy([&]() {
        pin_to_core(0);
        busy_worker(running);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ========== 无 sched_yield ==========
    // 预期：执行速度快，因为持续占用 CPU
    std::thread no_yield([&]() {
        pin_to_core(0);
        set_realtime_priority(50);
        std::cout << "  [无让出] 实时线程:" << std::endl;
        do_work();  // 无 sched_yield
    });
    no_yield.join();

    // ========== 有 sched_yield ==========
    // 预期：执行速度慢，因为频繁让出 CPU
    std::thread with_yield([&]() {
        pin_to_core(0);
        set_realtime_priority(50);
        std::cout << "  [有让出] 实时线程:" << std::endl;
        do_work_with_yield();  // 有 sched_yield
    });
    with_yield.join();

    running = false;
    busy.join();
    
    // 结论：实时线程中应避免调用 sched_yield()
    // 因为它是"自愿"放弃 CPU，会破坏实时调度的确定性
}


// ============================================================================
// 主函数：依次执行所有测试
// ============================================================================
int main() {
    // 1. 输出系统 CPU 核心数
    //    std::thread::hardware_concurrency() 返回硬件支持的并发线程数
    std::cout << "当前系统核心数: " << std::thread::hardware_concurrency() << "\n" << std::endl;

    // 2. 依次执行测试
    test_priority_pressure();    // 测试3：优先级压制
    test_fifo_vs_rr();           // 测试4：FIFO vs RR
    test_bind_core_isolation();  // 测试5：绑核隔离
    test_sched_yield();          // 测试6：sched_yield 影响

    // 3. 测试完成
    std::cout << "\n=== 全部测试完成 ===\n";
    return 0;
}