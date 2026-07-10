// ===========================================================================
// scheduler.hpp - 调度器主实现（Talos Scheduler 简化版）
//
// 设计要点：
//   1. add_system：注册系统（lambda + 参数类型列表）
//   2. build：冻结 World，校验依赖（唯一写者、无孤立读者）
//   3. run：按注册顺序执行系统（单线程简化版）
//
// 生命周期：Configuring → build() → Built → run() → Running → stop()
// ===========================================================================
#pragma once

#include "ecs/channel.hpp"
#include "ecs/world.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_set>
#include <vector>

namespace ecs {

// ---------------------------------------------------------------------------
// System 包装：存储函数和依赖信息
// ---------------------------------------------------------------------------
struct SystemBase {
    std::string name;
    std::function<void(World&)> func;
    std::vector<std::type_index> reads;   // 读取的资源/通道类型
    std::vector<std::type_index> writes;  // 写入的资源/通道类型
};

// ---------------------------------------------------------------------------
// 调度器主类
// ---------------------------------------------------------------------------
class Scheduler {
public:
    Scheduler() = default;

    // 访问内部 World（用于插入资源）
    [[nodiscard]] World& world() { return world_; }

    // 注册系统：接收 lambda + 显式依赖类型列表
    // 简化设计：用户通过 builder API 声明读写类型
    template <typename Fn>
    SystemBase& add_system(std::string name, Fn fn) {
        systems_.push_back(SystemBase{
            .name = std::move(name),
            .func = [f = std::move(fn)](World& w) { f(w); },
            .reads = {},
            .writes = {},
        });
        return systems_.back();
    }

    // 为系统添加读取依赖
    SystemBase& reads(SystemBase& sys, std::type_index type) {
        sys.reads.push_back(type);
        return sys;
    }

    // 为系统添加写入依赖
    SystemBase& writes(SystemBase& sys, std::type_index type) {
        sys.writes.push_back(type);
        return sys;
    }

    // 构建：冻结 World，校验依赖
    void build() {
        if (built_) {
            throw std::runtime_error("Scheduler::build: already built");
        }

        // 校验1：同一资源/通道只能有一个写者
        std::unordered_set<std::type_index> writers;
        for (const auto& sys : systems_) {
            for (const auto& w : sys.writes) {
                if (writers.contains(w)) {
                    throw std::runtime_error(
                        "Scheduler::build: multiple writers for type");
                }
                writers.insert(w);
            }
        }

        // 校验2：无孤立读者（有读者但无写者，会一直读空）
        for (const auto& sys : systems_) {
            for (const auto& r : sys.reads) {
                if (!writers.contains(r)) {
                    std::cerr << "[warn] System '" << sys.name
                              << "' reads resource with no writer\n";
                }
            }
        }

        world_.freeze();
        built_ = true;
        std::cout << "[Scheduler] build complete: " << systems_.size()
                  << " systems registered\n";
    }

    // 运行：单线程顺序执行所有系统
    void run() {
        if (!built_) {
            throw std::runtime_error("Scheduler::run: not built yet");
        }
        running_ = true;
        std::cout << "[Scheduler] running...\n";
        for (const auto& sys : systems_) {
            std::cout << "  > " << sys.name << "\n";
            sys.func(world_);
        }
        running_ = false;
        std::cout << "[Scheduler] finished\n";
    }

    [[nodiscard]] bool is_running() const noexcept { return running_; }

private:
    World world_;
    std::vector<SystemBase> systems_;
    bool built_ = false;
    bool running_ = false;
};

}  // namespace ecs
