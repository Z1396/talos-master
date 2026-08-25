// ===========================================================================
// 阶段3：ECS 调度器演示
//
// 模拟 Talos 的火控流水线：
//   Sensor → Perception → Estimation → Planning → Weapon
//
// 每个系统通过 World 读写资源，调度器校验依赖、顺序执行
// ===========================================================================

#include "ecs/scheduler.hpp"

#include <iostream>
#include <string>
#include <typeindex>

// ===========================================================================
// 业务资源类型定义
// ===========================================================================

// 原始图像（L1 Sensor 产出）
struct CameraImage {
    int width = 1440;
    int height = 1080;
    std::string data = "raw_pixels";
};

// 检测结果（L2 Perception 产出）
struct DetectionResult {
    int count = 0;
    float confidence = 0.0f;
};

// 跟踪状态（L3 Estimation 产出）
struct TrackState {
    int track_id = 0;
    double x = 0.0;
    double y = 0.0;
};

// 开火指令（L4 Planning 产出，L5 Weapon 消费）
struct FireCommand {
    bool should_fire = false;
    double pitch = 0.0;
    double yaw = 0.0;
};

// ===========================================================================
// 系统实现
// ===========================================================================

// L1 Sensor：读取相机，产出图像
void sensor_system(ecs::World& w) {
    auto& img = w.get_resource<CameraImage>();
    std::cout << "    [L1] camera " << img.width << "x" << img.height
              << " captured\n";
}

// L2 Perception：检测目标
void perception_system(ecs::World& w) {
    const auto& img = w.get_resource<CameraImage>();
    auto& det = w.get_resource<DetectionResult>();
    det.count = 3;
    det.confidence = 0.95f;
    std::cout << "    [L2] detected " << det.count << " targets (conf="
              << det.confidence << ") from " << img.width << "x"
              << img.height << "\n";
}

// L3 Estimation：跟踪目标
void estimation_system(ecs::World& w) {
    const auto& det = w.get_resource<DetectionResult>();
    auto& track = w.get_resource<TrackState>();
    track.track_id = det.count > 0 ? 1 : 0;  // 有目标则分配 track_id
    track.x = 1.5;
    track.y = 2.0;
    std::cout << "    [L3] tracking id=" << track.track_id
              << " pos=(" << track.x << "," << track.y << ")\n";
}

// L4 Planning：决策开火
void planning_system(ecs::World& w) {
    const auto& track = w.get_resource<TrackState>();
    auto& cmd = w.get_resource<FireCommand>();
    cmd.should_fire = (track.x > 1.0);
    cmd.pitch = 0.5;
    cmd.yaw = 0.3;
    std::cout << "    [L4] fire=" << (cmd.should_fire ? "YES" : "NO")
              << " pitch=" << cmd.pitch << " yaw=" << cmd.yaw << "\n";
}

// L5 Weapon：执行开火
void weapon_system(ecs::World& w) {
    const auto& cmd = w.get_resource<FireCommand>();
    if (cmd.should_fire) {
        std::cout << "    [L5] FIRE! pitch=" << cmd.pitch << " yaw=" << cmd.yaw << "\n";
    } else {
        std::cout << "    [L5] hold fire\n";
    }
}

// ===========================================================================
// 辅助：通过辅助函数声明系统依赖
// ===========================================================================
template <typename T>
std::type_index type_of() { return std::type_index(typeid(T)); }

int main() {
    ecs::Scheduler scheduler;

    // 1. 插入初始资源
    auto& world = scheduler.world();
    world.insert_resource(CameraImage{1440, 1080, "raw"});
    world.insert_resource(DetectionResult{});
    world.insert_resource(TrackState{});
    world.insert_resource(FireCommand{});

    // 2. 注册系统并声明依赖（读写语义）
    auto& s1 = scheduler.add_system("L1_Sensor", sensor_system);
    scheduler.writes(s1, type_of<CameraImage>());

    auto& s2 = scheduler.add_system("L2_Perception", perception_system);
    scheduler.reads(s2, type_of<CameraImage>());
    scheduler.writes(s2, type_of<DetectionResult>());

    auto& s3 = scheduler.add_system("L3_Estimation", estimation_system);
    scheduler.reads(s3, type_of<DetectionResult>());
    scheduler.writes(s3, type_of<TrackState>());

    auto& s4 = scheduler.add_system("L4_Planning", planning_system);
    scheduler.reads(s4, type_of<TrackState>());
    scheduler.writes(s4, type_of<FireCommand>());

    auto& s5 = scheduler.add_system("L5_Weapon", weapon_system);
    scheduler.reads(s5, type_of<FireCommand>());

    // 3. 构建：校验依赖、冻结 World
    scheduler.build();

    // 4. 运行：顺序执行流水线
    std::cout << "\n=== 第一帧 ===\n";
    scheduler.run();

    // 5. 修改输入再跑一帧（演示数据驱动）
    std::cout << "\n=== 第二帧（修改输入）===\n";
    world.get_resource<CameraImage>().width = 1920;
    world.get_resource<TrackState>().x = 0.5;  // 不满足开火条件
    scheduler.run();

    std::cout << "\nECS 调度器演示完成\n";
    return 0;
}
