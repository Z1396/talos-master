// ===========================================================================
// scheduler.hpp - 调度器（Talos Scheduler 简化版）
//
// 核心设计：
//   1. add_system 注册系统，reads/writes 声明依赖（Builder 模式）
//   2. build() 校验：同一资源单写者、读者必须有写者
//   3. run() 顺序执行所有系统（单线程简化版）
//
// 生命周期：Configuring → build() → Built → run() → Running
// ===========================================================================
#pragma once             // 防止头文件被重复包含（等效于 include guards）

#include "ecs/world.hpp" // 引入 World（世界）类，它是所有组件和实体的容器

#include <functional>    // std::function - 可调用对象的包装器
#include <iostream>      // std::cout, std::cerr - 控制台输出
#include <stdexcept>     // std::runtime_error - 运行时异常
#include <string>        // std::string
#include <typeindex>     // std::type_index - 类型的唯一标识符（基于 typeid）
#include <unordered_set> // 哈希集合 - 用于快速查重
#include <vector>        // 动态数组 - 存储系统列表

namespace ecs {          // 所有代码放在 ecs 命名空间，避免命名冲突

// ===========================================================================
// SystemBase - 单个系统的元数据 + 执行函数
// ===========================================================================
struct SystemBase {
    std::string name;                    // 系统名称（用于日志输出）
    std::function<void(World&)> func;    // 实际执行的函数，接收 World 引用
    std::vector<std::type_index> reads;  // 声明此系统"读取"哪些组件类型
    std::vector<std::type_index> writes; // 声明此系统"写入"哪些组件类型
};

// ===========================================================================
// Scheduler - 调度器：管理系统注册、依赖校验、执行
// ===========================================================================
class Scheduler {
public:
    // 默认构造函数（无特殊初始化）
    Scheduler() = default;

    // 暴露 World 引用，允许用户在注册系统前添加实体/组件
    [[nodiscard]] World& world() { return world_; }
    // [[nodiscard]] 表示返回值不应被忽略，防止用户调用但不使用

    // =========================================================================
    // add_system - 注册一个系统
    // 模板参数 Fn：可调用对象（函数指针、lambda、函数对象等）
    // 返回 SystemBase& 引用，支持链式调用（Builder 模式）
    // =========================================================================
    template <typename Fn>
    SystemBase& add_system(std::string name, Fn fn) {
        // 构造 SystemBase 对象并添加到 systems_ 向量尾部
        systems_.push_back(
            SystemBase{
                .name   = std::move(name),                         // 转移字符串所有权，避免拷贝
                .func   = [f = std::move(fn)](World& w) { f(w); }, // 包装可调用对象
                .reads  = {},   // 初始为空，稍后通过 reads()/writes() 添加
                .writes = {},
            });
        return systems_.back(); // 返回刚添加的系统的引用
    }

    // =========================================================================
    // reads - 声明系统"读取"某类型
    // 参数：sys（要声明的系统引用），type（类型的 type_index）
    // 返回：sys 的引用，支持链式调用（如 sys.reads(...).writes(...)）
    // =========================================================================
    SystemBase& reads(SystemBase& sys, std::type_index type) {
        sys.reads.push_back(type);
        return sys;
    }

    // =========================================================================
    // writes - 声明系统"写入"某类型（写入=修改，可理解为"独占写"）
    // =========================================================================
    SystemBase& writes(SystemBase& sys, std::type_index type) {
        sys.writes.push_back(type);
        return sys;
    }

    // =========================================================================
    // build - 构建/校验阶段
    // 1. 检查是否重复 build
    // 2. 校验：同一资源只能有一个写者
    // 3. 校验：读者必须有写者（否则是空读，警告但不报错）
    // 4. 冻结 World（禁止再添加新组件类型）
    // =========================================================================
    void build() {
        // 防止重复构建
        if (built_) {
            throw std::runtime_error("already built");
        }

        // ---------- 规则1：同一资源只能有一个写者 ----------
        // 用 unordered_set 记录所有被写入过的类型
        std::unordered_set<std::type_index> writers;
        for (const auto& sys : systems_) {     // 遍历所有系统
            for (const auto& w : sys.writes) { // 遍历该系统的写入列表
                // 尝试插入，如果已存在则说明有多个写者
                /*### `set::insert` 返回值
                `insert(value)` 返回一个 **`std::pair<迭代器, bool>`***/
                if (!writers.insert(w).second) { // .second 表示是否插入成功
                    throw std::runtime_error("multiple writers for same type");
                }
            }
        }

        // ---------- 规则2：读者必须有写者 ----------
        // 注意：这里只是警告而不是报错，因为有些场景可能允许"只读空数据"
        for (const auto& sys : systems_) {
            for (const auto& r : sys.reads) {
                //`contains()` 是 **C++20** 新增，`std::set` / `std::unordered_set` / `std::map` / `std::unordered_map` 都有。
                /*bool contains(const T& key) const;
                - 返回 `true`：容器**已经存在这个 key**
                - 返回 `false`：容器**没有这个 key**
                > ⚠️ **contains 仅仅做查询，不会修改容器，不会插入元素！***/
                if (!writers.contains(r)) { // C++20 的 contains() 方法
                    std::cerr << "[warn] '" << sys.name << "' reads type with no writer\n";
                }
            }
        }

        // 冻结 World：禁止再注册新的组件类型
        // 因为依赖已经校验完毕，后续不能再改组件结构
        world_.freeze();
        built_ = true; // 标记为已构建

        std::cout << "[Scheduler] build complete: " << systems_.size() << " systems\n";
    }

    // =========================================================================
    // run - 执行所有已注册的系统
    // 按注册顺序依次调用每个系统的 func，传入 world_ 引用
    // =========================================================================
    void run() {
        // 必须先 build 才能 run
        if (!built_) {
            throw std::runtime_error("not built yet");
        }

        std::cout << "[Scheduler] running...\n";
        for (const auto& sys : systems_) {
            std::cout << "  > " << sys.name << "\n";
            sys.func(world_);         // 执行系统函数
        }
        std::cout << "[Scheduler] finished\n";
    }

private:
    World world_;                     // World 实例（存储所有实体和组件）
    std::vector<SystemBase> systems_; // 已注册的系统列表（按注册顺序存储）
    bool built_ = false;              // 是否已完成构建校验
};

} // namespace ecs