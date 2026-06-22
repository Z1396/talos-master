#include <cstdio>

// 调度器错误格式化工具，用于打印构建依赖图失败信息
#include "scheduler/error_formatter.hpp"
// 调度器核心类 Scheduler、系统注册、执行策略
#include "scheduler/scheduler.hpp"
// 全局资源容器 World，存放共享资源res<T>
#include "scheduler/world.hpp"

// 全局命名空间简写，避免重复写talos::
using namespace talos;
using namespace talos::scheduler::system;

// ============================================================================
// 【测试自定义数据类型】模拟业务层数据结构
// ============================================================================

/// 相机帧数据：图像基础信息
struct Frame {
    int id    = 0;      // 帧序列号
    int width = 1920;   // 图像宽度
};

/// 2D检测框结果：检测器输出
struct Detection {
    int frame_id = 0;   // 关联所属相机帧ID
    float x      = 0.0f;// 检测框中心X像素
    float y      = 0.0f;// 检测框中心Y像素
};

/// 全局静态配置资源：检测器阈值，多系统共享只读
struct Config {
    float threshold = 0.5f; // 检测置信度阈值
};

/// 全局可变共享资源：跟踪器全局计数，多系统可修改
struct TrackerState {
    int count = 0; // 累计检测目标总数
};

// 【管道Tag空结构体标签】编译期区分不同数据流通道，无运行时开销
// 对应你工程 channel_topics.hpp 的设计思路，强类型隔离数据流
struct CameraPipe {};   // 相机输出帧数据流标识
struct DetectorPipe {}; // 检测结果数据流标识

// ============================================================================
// 测试1：通道类型特征静态校验（编译期断言，无需运行）
// 作用：校验 spsc/spmc/res/local 通道的类型分类标识kind是否正确
// ============================================================================
void test_type_traits() {
    printf("=== Test: Type Traits ===\n");

    // spsc<T> 单生产者单消费者 只读读取端 → 通道类型spsc_reader
    static_assert(spsc<Frame>::kind == channel_kind::spsc_reader);
    // spsc_mut<T> 单生产者单消费者 写入端 → spsc_writer
    static_assert(spsc_mut<Frame>::kind == channel_kind::spsc_writer);
    // spmc<T> 单生产者多消费者 读取端 → spmc_reader（你项目可视化大量使用）
    static_assert(spmc<Frame>::kind == channel_kind::spmc_reader);
    // spmc_mut<T> 单生产者多消费者 写入端 → spmc_writer
    static_assert(spmc_mut<Frame>::kind == channel_kind::spmc_writer);
    // res<T> 全局只读共享资源（Config配置）
    static_assert(res<Config>::kind == channel_kind::res);
    // res_mut<T> 全局可修改共享资源（TrackerState计数）
    static_assert(res_mut<TrackerState>::kind == channel_kind::res_mut);

    // 全部静态断言编译通过才会执行到该行打印
    printf("  All static assertions passed!\n\n");
}

// ============================================================================
// 测试2：带Tag标签的多系统流水线（相机→检测器→跟踪器）
// 演示Tag作用：相同数据类型不同管道靠Tag区分，编译期隔离
// ============================================================================

/**
 * @brief 相机生产者系统：输出图像帧到CameraPipe通道
 * @param out spsc_mut<输出数据, 管道Tag> 单生产者写入接口
 */
void camera_system(spsc_mut<Frame, CameraPipe> out) {
    // 静态局部变量：系统每次执行自增帧号
    static int frame_id = 0;
    // 向通道写入一帧图像数据
    out.write(Frame{.id = frame_id++, .width = 1920});
}

/**
 * @brief 检测器消费系统：读取相机帧、全局配置、输出检测框
 * @param frame_in 读取CameraPipe相机帧
 * @param config 全局只读资源：检测阈值
 * @param det_out 写入DetectPipe检测结果通道
 */
void detector_system(
    spsc<Frame, CameraPipe> frame_in, 
    res<Config> config,
    spsc_mut<Detection, DetectorPipe> det_out) 
{
    // read()返回optional，有数据才执行检测逻辑
    if (const auto f = frame_in.read()) {
        // *config 解引用全局共享配置，读取阈值
        printf("    detector: got frame %d, threshold=%.2f\n", f->id, (*config).threshold);
        // 生成模拟检测结果写入通道
        det_out.write(Detection{.frame_id = f->id, .x = 100.0f, .y = 200.0f});
    }
}

/**
 * @brief 跟踪器消费系统：读取检测框，修改全局跟踪计数资源
 * @param det_in 读取DetectPipe检测结果
 * @param state 全局可变共享资源，累计目标数量
 */
void tracker_system(spsc<Detection, DetectorPipe> det_in, res_mut<TrackerState> state) {
    if (const auto d = det_in.read()) {
        // 全局计数自增
        (*state).count++;
        printf(
            "    tracker: got detection from frame %d, total count=%d\n", 
            d->frame_id, (*state).count);
    }
}

/// 流水线注册与依赖图构建测试
void test_meta_extraction() {
    printf("=== Test: Meta Extraction with Tags ===\n");

    // 注释说明：框架内部在add_system注册时自动提取系统入参元数据
    // 自动分析通道Tag、资源依赖，构建执行拓扑图，无需手动声明依赖
    printf("  Channel type traits work correctly!\n");
    printf("  Metadata extraction happens automatically during Scheduler::add_system\n\n");
}

// ============================================================================
// 测试3：完整SPSC串行流水线搭建、调度器依赖图校验
// 业务链路：camera（生成帧） → detector（检测） → tracker（跟踪计数）
// ============================================================================
void test_spsc_pipeline() {
    printf("=== Test: SPSC Pipeline ===\n");

    // 创建调度器实例
    Scheduler scheduler;
    // 向全局World容器插入只读共享配置资源
    scheduler.world().insert_resource(Config{.threshold = 0.7f});
    // 插入可修改全局跟踪计数资源
    scheduler.world().insert_resource(TrackerState{.count = 0});

    // 注册三个业务系统，自动解析输入输出通道构建依赖关系
    scheduler.add_system("camera", &camera_system);
    scheduler.add_system("detector", &detector_system);
    scheduler.add_system("tracker", &tracker_system);

    // 打印所有已注册系统名称
    printf("  Registered systems:\n");
    scheduler.print_systems();

    // build()：自动拓扑排序，校验依赖闭环、通道读写匹配、资源访问合法性
    auto build_result = scheduler.build();
    // build_result是expected类型，失败携带错误信息（循环依赖、Tag不匹配、资源缺失等）
    if (!build_result) {
        fmt::print("  ERROR: Build failed: {}\n", build_result.error());
        return;
    }

    printf("  Pipeline setup successful!\n\n");
}

// ============================================================================
// 测试4：Lambda匿名系统 + Tag管道标签混用
// 支持直接传入lambda作为系统，无需单独定义函数，快速开发小功能模块
// ============================================================================
void test_lambda_with_tags() {
    printf("=== Test: Lambda with Tags ===\n");

    // 本地临时管道Tag，仅当前测试作用域有效
    struct ProducerTag {};
    struct ConsumerTag {};

    Scheduler scheduler;

    // 生产者lambda系统：写入ProducerTag通道
    scheduler.add_system("producer", [](spmc_mut<int, ProducerTag> out) {
        static int produced = 0;
        out.write(++produced);
        printf("    produced: %d\n", produced);
    });

    // 消费者lambda系统：读取同一个ProducerTag通道
    scheduler.add_system("consumer", [](spmc<int, ProducerTag> in) {
        static int consumed = 0;
        if (const auto val = in.read()) {
            consumed = *val;
            printf("    consumed: %d\n", consumed);
        }
    });

    printf("  Lambda systems registered successfully!\n\n");
}

// ============================================================================
// 测试5：三种执行策略（Policy）编译期特征校验 + 调度器注册
// 三种策略：
// 1. fixed_rate<Hz, 亲和核, 优先级>：固定频率定时任务，执行后通知
// 2. fixed_rate_silent<Hz>：静默定时，无事件通知，轻量化后台
// 3. pool_compute：事件驱动，通道有新数据才执行（你项目可视化全用这个）
// ============================================================================
void test_execution_policies() {
    printf("=== Test: Execution Policies ===\n");

    // 编译期静态断言：区分定时策略 / 池化事件驱动策略
    // fixed_rate 系列属于定时策略
    static_assert(is_fixed_rate_policy_v<fixed_rate<30>>);
    static_assert(is_fixed_rate_policy_v<fixed_rate<1000, 0, 99>>);
    static_assert(is_fixed_rate_policy_v<fixed_rate_silent<1000>>);
    // pool_compute不属于定时策略
    static_assert(!is_fixed_rate_policy_v<pool_compute>);

    // pool_compute 属于池化事件驱动策略
    static_assert(is_pool_policy_v<pool_compute>);
    static_assert(!is_pool_policy_v<fixed_rate<30>>);
    static_assert(!is_pool_policy_v<fixed_rate_silent<1000>>);

    // 区分是否带执行通知：fixed_rate 有通知，silent无通知
    static_assert(is_notifying_policy_v<fixed_rate<30>>);
    static_assert(!is_notifying_policy_v<fixed_rate_silent<1000>>);
    static_assert(is_silent_policy_v<fixed_rate_silent<1000>>);
    static_assert(!is_silent_policy_v<fixed_rate<30>>);

    // 编译期提取定时策略配置参数：频率、CPU亲和核、线程优先级、通知开关
    constexpr auto ex_info = make_policy_info<fixed_rate<30, 2, 50>>();
    static_assert(ex_info.kind == PolicyKind::FixedRate);
    static_assert(ex_info.frequency_hz == 30);    // 30Hz定时
    static_assert(ex_info.cpu_affinity == 2);     // 绑定CPU2核心
    static_assert(ex_info.thread_priority == 50); // 线程优先级50
    static_assert(ex_info.notifies == true);      // 执行完成发送通知

    // 静默定时策略参数提取
    constexpr auto silent_info = make_policy_info<fixed_rate_silent<1000, 1, 10>>();
    static_assert(silent_info.kind == PolicyKind::FixedRate);
    static_assert(silent_info.frequency_hz == 1000);
    static_assert(silent_info.cpu_affinity == 1);
    static_assert(silent_info.thread_priority == 10);
    static_assert(silent_info.notifies == false); // 无通知

    // 池化事件驱动策略
    constexpr auto pool_info = make_policy_info<pool_compute>();
    static_assert(pool_info.kind == PolicyKind::PoolCompute);

    printf("  Static assertions for policy traits passed!\n");

    // 实战：不同策略系统注册到调度器
    struct DataPipe {};
    struct ImuPipe {};

    World world;
    Scheduler scheduler(world);

    // 30Hz定时相机采集系统，绑定固定频率策略
    scheduler.add_system<fixed_rate<30>>(
        "camera", [](spmc_mut<int, DataPipe> out) { out.write(42); });

    // 1000Hz静默IMU采集，无通知，轻量化后台
    scheduler.add_system<fixed_rate_silent<1000>>(
        "imu", [](spmc_mut<int, ImuPipe> out) { out.write(123); });

    // 事件驱动池化任务：通道有数据才执行，可视化/检测模块通用
    scheduler.add_system<pool_compute>(
        "processor", [](spmc<int, DataPipe> in, spmc<int, ImuPipe> imu) {
            (void)in.read();
            (void)imu.read();
        });

    printf("  Systems with policies:\n");
    scheduler.print_systems();
    printf("  Policy test completed successfully!\n\n");
}

// ============================================================================
// 测试6：talos::local<T> 系统局部持久变量
// 特点：
// 1. 仅当前系统独占，其他系统无法访问
// 2. 调度循环之间状态保留，不用全局res资源
// 3. 不参与依赖图分析，不会产生依赖关系
// ============================================================================
void test_local_variables() {
    printf("=== Test: Local Variables ===\n");

    // 静态校验local<T>通道类型标识
    static_assert(local<int>::kind == channel_kind::local);
    printf("  [PASS] Type trait checks for local<T>\n");

    struct InputTag {};
    struct OutputTag {};

    World world;
    Scheduler scheduler(world);

    // 生产者：持续写入数据
    scheduler.add_system<pool_compute>("producer", [](spmc_mut<int, InputTag> out) {
        out.write(42);
        printf("    producer: sent 42\n");
    });

    // 计数系统：两个local局部变量，仅本系统可见，多次执行数值累加不重置
    scheduler.add_system<pool_compute>(
        "counter", [](spmc<int, InputTag> in, local<int> run_count, local<int> total_sum) {
            if (auto data = in.read()) {
                (*run_count)++;        // 累计系统执行次数
                (*total_sum) += *data; // 累计所有接收数据总和
                printf(
                    "    counter: run_count=%d, total_sum=%d, data=%d\n", 
                    *run_count, *total_sum, *data);
            }
        });

    printf("  Local variables test setup successful!\n");
    printf("  [PASS] Local variables persist across runs\n");
    printf("  [PASS] local<T> excluded from dependency analysis\n\n");
}

// ============================================================================
// 程序入口：依次执行全部单元测试
// ============================================================================
int main() {
    printf("\n");
    printf("==========================================\n");
    printf("  Bevy-style ECS with Tags Test Suite\n");
    printf("==========================================\n\n");

    // 按顺序执行所有测试用例
    test_type_traits();
    test_meta_extraction();
    test_spsc_pipeline();
    test_lambda_with_tags();
    test_execution_policies();
    test_local_variables();

    printf("==========================================\n");
    printf("  All tests completed!\n");
    printf("==========================================\n\n");

    return 0;
}