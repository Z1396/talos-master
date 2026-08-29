// ===========================================================================
// 阶段3：ECS 调度器演示（Talos 火控流水线简化版）
//
// 模拟 L1→L5 流水线：Sensor → Perception → Estimation → Planning → Weapon
// 每个系统通过 World 读写资源，调度器校验依赖、顺序执行
// ===========================================================================

#include "ecs/scheduler.hpp" // 引入调度器（包含 World 和系统管理）

#include <iostream>          // 标准输入输出
#include <stdexcept>         // std::runtime_error - 捕获异常
#include <string>            // 字符串支持
#include <typeindex>         // std::type_index - 依赖声明

// ===========================================================================
// 业务资源类型（对应 Talos core/types.hpp 中的领域类型）
//
// 这些结构体模拟了火控系统中的"数据包"，沿着流水线逐步被加工
// 每个资源只被一个系统写入，但可被多个系统读取（单向数据流）
// ===========================================================================

// L1 输出：摄像头捕获的原始图像（仅数据，无行为）
struct CameraImage {
    int width  = 1440; // 图像宽度（影响检测精度）
    int height = 1080; // 图像高度
};

// L2 输出：目标检测结果（由感知系统产生）
struct DetectionResult {
    int count        = 0;    // 检测到的目标数量
    float confidence = 0.0f; // 置信度 (0.0~1.0)
};

// L3 输出：跟踪状态（由估计系统产生）
struct TrackState {
    int track_id = 0;   // 目标唯一 ID（0 表示无目标）
    double x     = 0.0; // 目标 X 坐标（简化二维空间）
    double y     = 0.0; // 目标 Y 坐标
};

// L4 输出：开火指令（由规划系统产生）
struct FireCommand {
    bool should_fire = false; // 是否开火
    double pitch     = 0.0;   // 俯仰角（上下）
    double yaw       = 0.0;   // 偏航角（左右）
};

// ===========================================================================
// 系统实现（数据沿流水线单向流动）
//
// 每个系统都是一个函数：接收 World 引用，从其中读取/写入资源
// 系统之间通过 World 解耦，不直接调用彼此
// ===========================================================================

// ===========================================================================
// L1 Sensor（传感器系统）：读取初始状态，产出相机图像
//
// 职责：模拟硬件传感器，产生原始图像数据
// 写入：CameraImage（产出数据）
// 读取：无（流水线起点）
// ===========================================================================
void sensor_system(ecs::World& w) {
    // 获取 CameraImage 资源引用（此时应该已被 World 初始化）
    auto& img = w.get_resource<CameraImage>();
    std::cout << "    [L1] camera " << img.width << "x" << img.height << " captured\n";
}

// ===========================================================================
// L2 Perception（感知系统）：读取图像，产出检测结果
//
// 职责：对图像进行目标检测，识别潜在威胁
// 读取：CameraImage（依赖上游数据）
// 写入：DetectionResult（产出数据）
//
// 关键逻辑：数据驱动 - 图像分辨率影响检测效果
//   - 高分辨率 (>1500) → 检测失败（模拟算法缺陷或性能不足）
//   - 正常分辨率   → 检测到 3 个目标，置信度 0.95
// ===========================================================================
void perception_system(ecs::World& w) {
    const auto& img = w.get_resource<CameraImage>();     // 只读引用
    auto& det       = w.get_resource<DetectionResult>(); // 可写引用

    // 数据驱动分支：根据输入数据决定输出
    if (img.width > 1500) {
        // 高分辨率下检测失败（模拟真实场景：算法对高分辨率图像处理不佳）
        det.count      = 0;
        det.confidence = 0.0f;
    } else {
        // 正常检测结果
        det.count      = 3;
        det.confidence = 0.95f;
    }

    std::cout << "    [L2] detected " << det.count << " targets (conf=" << det.confidence
              << ") from " << img.width << "x" << img.height << "\n";
}

// ===========================================================================
// L3 Estimation（估计系统）：读取检测结果，产出跟踪状态
//
// 职责：对检测到的目标进行跟踪，预测其位置
// 读取：DetectionResult（依赖上游感知结果）
// 写入：TrackState（产出跟踪数据）
//
// 关键逻辑：
//   - 有目标 (count>0) → 分配 track_id=1，更新位置 (1.5, 2.0)
//   - 无目标 (count=0) → track_id=0，位置保持上一帧值（演示残留状态）
// ===========================================================================
void estimation_system(ecs::World& w) {
    const auto& det = w.get_resource<DetectionResult>(); // 只读
    auto& track     = w.get_resource<TrackState>();      // 可写

    // 根据检测结果决定跟踪状态
    track.track_id = det.count > 0 ? 1 : 0; // 有目标则分配 ID

    // 仅在有目标时更新位置（无目标时保留旧值，模拟目标丢失）
    if (det.count > 0) {
        track.x = 1.5;
        track.y = 2.0;
    }
    // 注意：当 det.count=0 时，track 的值保持不变（演示状态残留）

    std::cout << "    [L3] tracking id=" << track.track_id << " pos=(" << track.x << "," << track.y
              << ")\n";
}

// ===========================================================================
// L4 Planning（规划系统）：读取跟踪状态，产出开火指令
//
// 职责：根据目标位置决定是否开火及射击参数
// 读取：TrackState（依赖跟踪数据）
// 写入：FireCommand（产出开火指令）
//
// 关键逻辑：目标在 X>1.0 时开火（简化的射界判断）
// ===========================================================================
void planning_system(ecs::World& w) {
    const auto& track = w.get_resource<TrackState>();  // 只读
    auto& cmd         = w.get_resource<FireCommand>(); // 可写

    // 决策逻辑：目标在 X>1.0 范围内允许开火
    cmd.should_fire = (track.x > 1.0);
    cmd.pitch       = 0.5; // 固定俯仰角（简化）
    cmd.yaw         = 0.3; // 固定偏航角（简化）

    std::cout << "    [L4] fire=" << (cmd.should_fire ? "YES" : "NO") << " pitch=" << cmd.pitch
              << " yaw=" << cmd.yaw << "\n";
}

// ===========================================================================
// L5 Weapon（武器系统）：读取开火指令，执行开火
//
// 职责：实际执行开火动作
// 读取：FireCommand（依赖规划指令）
// 写入：无（流水线终点）
//
// 注意：这是流水线的终点，不产生新数据，只"消费"数据
// ===========================================================================
void weapon_system(ecs::World& w) {
    const auto& cmd = w.get_resource<FireCommand>(); // 只读

    if (cmd.should_fire) {
        std::cout << "    [L5] FIRE! pitch=" << cmd.pitch << " yaw=" << cmd.yaw << "\n";
    } else {
        std::cout << "    [L5] hold fire\n";
    }
}

// ===========================================================================
// 异常路径测试：调度器三条 throw 路径的防御性验证
//
//   1. World frozen 后 insert_resource → 抛异常（结构已锁定）
//   2. 两个系统写同一资源 → build() 抛异常（单写者约束）
//   3. 未 build 就 run() → 抛异常（生命周期校验）
// ===========================================================================
static int g_error_tests_failed = 0;

// 测试1：World frozen 后 insert_resource 必须抛异常
void test_frozen_insert() {
    std::cout << "    [异常1] frozen 后 insert_resource";
    ecs::World w;
    w.insert_resource(CameraImage{1440, 1080});
    w.freeze();                               // 模拟 build() 后的冻结状态

    bool threw = false;
    try {
        w.insert_resource(DetectionResult{}); // 冻结后插入 → 应抛
    } catch (const std::runtime_error& e) {
        threw = true;
        std::cout << " → 抛异常: " << e.what() << "\n";
    }
    if (!threw) {
        std::cerr << " → 未抛异常（失败！）\n";
        ++g_error_tests_failed;
    }
}

// 测试2：两个系统写同一资源 → build() 必须抛异常（单写者约束）
void test_duplicate_writer() {
    std::cout << "    [异常2] 多写者 build()";
    ecs::Scheduler scheduler;
    auto& world = scheduler.world();
    world.insert_resource(CameraImage{1440, 1080});
    world.insert_resource(DetectionResult{});

    // 两个系统都声明写入 CameraImage → 违反"单写者"约束
    auto& s1 = scheduler.add_system("S1_Writer", sensor_system);
    scheduler.writes(s1, typeid(CameraImage));
    auto& s2 = scheduler.add_system("S2_Writer", sensor_system);
    scheduler.writes(s2, typeid(CameraImage));

    bool threw = false;
    try {
        scheduler.build();
    } catch (const std::runtime_error& e) {
        threw = true;
        std::cout << " → 抛异常: " << e.what() << "\n";
    }
    if (!threw) {
        std::cerr << " → 未抛异常（失败！）\n";
        ++g_error_tests_failed;
    }
}

// 测试3：未 build 就 run() → 必须抛异常（生命周期校验）
void test_run_without_build() {
    std::cout << "    [异常3] 未 build 就 run()";
    ecs::Scheduler scheduler;
    (void)scheduler.add_system("S1", sensor_system); // 注册但从不 build

    bool threw = false;
    try {
        scheduler.run();
    } catch (const std::runtime_error& e) {
        threw = true;
        std::cout << " → 抛异常: " << e.what() << "\n";
    }
    if (!threw) {
        std::cerr << " → 未抛异常（失败！）\n";
        ++g_error_tests_failed;
    }
}

// ===========================================================================
// Demo 主程序
// ===========================================================================
int main() {
    // ---------- 0. 先验证异常路径（独立 World，不影响下方演示）----------
    std::cout << "=== 异常路径测试 ===\n";
    test_frozen_insert();
    test_duplicate_writer();
    test_run_without_build();
    if (g_error_tests_failed > 0) {
        std::cerr << "异常路径测试失败数: " << g_error_tests_failed << "\n";
        return 1;
    }
    std::cout << "三条 throw 路径全部按预期抛异常\n\n";

    // ---------- 1. 创建调度器并获取 World 引用 ----------
    ecs::Scheduler scheduler;        // 调度器拥有 World
    auto& world = scheduler.world(); // 获取 World 引用用于资源插入

    // ---------- 2. 初始化资源 ----------
    // 在注册系统前，先往 World 中放入初始资源
    // 这些资源将沿流水线被各系统逐步加工
    world.insert_resource(CameraImage{1440, 1080}); // 初始图像（1440x1080）
    world.insert_resource(DetectionResult{});       // 空检测结果
    world.insert_resource(TrackState{});            // 空跟踪状态
    world.insert_resource(FireCommand{});           // 空开火指令

    // ---------- 3. 注册系统并声明依赖（读写语义） ----------
    // 使用 Builder 模式链式声明，构建依赖图：
    //
    //   L1_Sensor    → writes CameraImage
    //   L2_Perception → reads CameraImage, writes DetectionResult
    //   L3_Estimation → reads DetectionResult, writes TrackState
    //   L4_Planning   → reads TrackState, writes FireCommand
    //   L5_Weapon     → reads FireCommand
    //
    // 这个依赖图本质上是一条链：L1 → L2 → L3 → L4 → L5
    // 调度器会按注册顺序执行，但依赖声明用于校验（确保写者唯一）

    // L1: 传感器（写入 CameraImage）
    auto& s1 = scheduler.add_system("L1_Sensor", sensor_system);
    scheduler.writes(s1, typeid(CameraImage)); // 声明写入 CameraImage

    // L2: 感知（读取 CameraImage，写入 DetectionResult）
    auto& s2 = scheduler.add_system("L2_Perception", perception_system);
    scheduler.reads(s2, typeid(CameraImage));      // 声明读取 CameraImage
    scheduler.writes(s2, typeid(DetectionResult)); // 声明写入 DetectionResult

    // L3: 估计（读取 DetectionResult，写入 TrackState）
    auto& s3 = scheduler.add_system("L3_Estimation", estimation_system);
    scheduler.reads(s3, typeid(DetectionResult)); // 声明读取 DetectionResult
    scheduler.writes(s3, typeid(TrackState));     // 声明写入 TrackState

    // L4: 规划（读取 TrackState，写入 FireCommand）
    auto& s4 = scheduler.add_system("L4_Planning", planning_system);
    scheduler.reads(s4, typeid(TrackState));   // 声明读取 TrackState
    scheduler.writes(s4, typeid(FireCommand)); // 声明写入 FireCommand

    // L5: 武器（读取 FireCommand，不写入任何资源）
    auto& s5 = scheduler.add_system("L5_Weapon", weapon_system);
    scheduler.reads(s5, typeid(FireCommand)); // 声明读取 FireCommand
    // 注意：L5 不写入任何资源，是流水线的终点

    // ---------- 4. 构建：校验依赖、冻结 World ----------
    // build() 会执行两项校验：
    //   1. 每个资源最多只有一个写者（防止冲突）
    //   2. 每个被读取的资源至少有一个写者（防止读空）
    // 校验通过后，World 被冻结（不能再插入新资源）
    scheduler.build();

    // ---------- 5. 第一帧：正常流水线 ----------
    std::cout << "\n=== 第一帧 ===\n";
    scheduler.run();
    // 预期输出：
    //   [L1] camera 1440x1080 captured
    //   [L2] detected 3 targets (conf=0.95) from 1440x1080
    //   [L3] tracking id=1 pos=(1.5,2.0)
    //   [L4] fire=YES pitch=0.5 yaw=0.3
    //   [L5] FIRE! pitch=0.5 yaw=0.3

    // ---------- 6. 第二帧：修改输入演示数据驱动 ----------
    std::cout << "\n=== 第二帧（高分辨率→检测失败→hold fire）===\n";

    // 关键演示：同一套系统代码，修改输入数据后，输出完全不同
    // 这就是"数据驱动"设计的核心价值：业务逻辑与数据解耦

    // 修改输入资源：提高分辨率，触发 L2 的"检测失败"分支
    world.get_resource<CameraImage>().width = 1920; // 1920 > 1500 → 检测失败

    // 重置跟踪状态：模拟"新一帧"开始（不重置会残留上一帧的位置）
    world.get_resource<TrackState>() = {}; // 重置为 0

    // 重新运行流水线（系统代码完全不变）
    scheduler.run();
    // 预期输出：
    //   [L1] camera 1920x1080 captured
    //   [L2] detected 0 targets (conf=0) from 1920x1080   ← 检测失败！
    //   [L3] tracking id=0 pos=(1.5,2.0)                  ← 注意位置残留！
    //   [L4] fire=NO pitch=0.5 yaw=0.3                    ← 不开火
    //   [L5] hold fire                                    ← 不开火

    std::cout << "\nECS 调度器演示完成\n";
}