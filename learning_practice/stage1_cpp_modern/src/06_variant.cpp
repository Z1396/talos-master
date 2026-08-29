// ===========================================================================
// 练习6：std::variant + std::visit —— "能力进入类型系统"（项目核心范式）
//
// 对齐真实项目（src/fcs/L1_sensor/output_interface.cpp L278-280）：
//   OutputMode 是 std::variant<IpcOutput, McuOutput, ChiralOutput>
//   send 用一个 std::visit 完成三策略分发 —— 编译期展开成三个 send 调用，
//   不需要 if/else，不需要虚函数表，多态能力全部进入类型系统。
//
// 学习要点：
//   1. variant = 类型安全的"或"（C 风格 union 的升级版）：同一时刻只持有一种类型
//   2. std::visit = 编译期 switch：对 variant 的每种替代类型都实例化一份调用
//   3. overloaded = 多个 lambda 的合并（包展开继承 + using 引入 operator()）
//   4. 穷尽性保证：variant 新增类型后忘写对应 lambda → 编译报错（安全兜底，
//      比运行时 switch 的 default 分支可靠 —— 忘写分支编译器直接拦截）
//   5. 反面教材：holds_alternative + get_if 手写 if-else 分支（可读性差、易漏）
// ===========================================================================

#include <iostream>
#include <string>
#include <type_traits> // std::is_same_v, std::variant_alternative_t
#include <variant>

// ===========================================================================
// 1. 三种输出后端（模拟 output_interface.cpp 的三个策略类）
//    每个类独立持有自己的资源，各自实现 send()
// ===========================================================================
struct IpcOutput {
    std::string shm_name;

    // 真实项目：共享内存发布帧数据
    std::string send(const std::string& msg) const {
        return "[Ipc]  帧发布到共享内存 " + shm_name + ": " + msg;
    }
};

struct McuOutput {
    int baud;

    // 真实项目：串口下发云台指令
    std::string send(const std::string& msg) const {
        return "[Mcu]  串口下发 (baud=" + std::to_string(baud) + "): " + msg;
    }
};

struct ChiralOutput {
    int port;

    // 真实项目：Foxglove 可视化推送
    std::string send(const std::string& msg) const {
        return "[Chiral] WebSocket 推送 (port=" + std::to_string(port) + "): " + msg;
    }
};

// ===========================================================================
// 2. 模式类型：variant 三选一（对应 output_interface.hpp 的 OutputMode）
// ===========================================================================
using OutputMode = std::variant<IpcOutput, McuOutput, ChiralOutput>;

// ===========================================================================
// 3. overloaded：合并多个 lambda（C++17 惯用法，CTAD 推导指引）
//    包展开继承 Ts... + using Ts::operator()... 把各 lambda 的调用运算符合一
//    C++20 起编译器为聚合模板自动生成推导指引，这里的显式指引可省
// ===========================================================================
template <typename... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// ===========================================================================
// 4. 统一的 send 入口：一个 std::visit 完成全部策略分发
//    这正是 output_interface.cpp L278-280 的做法：
//    std::visit([&](const auto& out) { out.send(...); }, mode);
//    泛型 lambda 对三种类型各实例化一次 —— send 的存在性编译期检查
// ===========================================================================
std::string dispatch(const OutputMode& mode, const std::string& msg) {
    return std::visit([&](const auto& out) { return out.send(msg); }, mode);
}

// ===========================================================================
// 5. 反面教材：运行时 if-else 手写分支（与 std::visit 对比，不推荐）
//    缺点：
//      a) 每分支写两遍类型名（holds_alternative + get），冗长易错
//      b) 漏掉一个 else-if 编译照样通过 —— 运行时静默返回兜底值
//      c) variant 新增类型时没有编译器提醒
// ===========================================================================
std::string dispatch_runtime(const OutputMode& mode, const std::string& msg) {
    if (std::holds_alternative<IpcOutput>(mode)) {
        return std::get<IpcOutput>(mode).send(msg);
    }
    if (std::holds_alternative<McuOutput>(mode)) {
        return std::get<McuOutput>(mode).send(msg);
    }
    if (std::holds_alternative<ChiralOutput>(mode)) {
        return std::get<ChiralOutput>(mode).send(msg);
    }
    return "[?] 未知输出模式"; // ← 漏写分支时这里静默兜底，无编译期提醒
}

// ===========================================================================
// 轻量断言
// ===========================================================================
static int g_failures = 0;

#define CHECK(cond)                                                                             \
    do {                                                                                        \
        if (!(cond)) {                                                                          \
            std::cerr << "  [CHECK 失败] " #cond "  (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            ++g_failures;                                                                       \
        }                                                                                       \
    } while (0)

int main() {
    // ===== 测试1：三策略分发（泛型 lambda 版本，最贴近真实项目）=====
    std::cout << "=== 测试1：std::visit 三策略分发 ===\n";
    const OutputMode m1 = IpcOutput{"/dev/shm/talos_frames"};
    const OutputMode m2 = McuOutput{115200};
    const OutputMode m3 = ChiralOutput{8765};

    CHECK(dispatch(m1, "frame#1").find("/dev/shm/talos_frames") != std::string::npos);
    CHECK(dispatch(m2, "yaw 0.3").find("115200") != std::string::npos);
    CHECK(dispatch(m3, "armor@x1.2").find("8765") != std::string::npos);
    std::cout << "  " << dispatch(m1, "frame#1") << "\n";
    std::cout << "  " << dispatch(m2, "yaw 0.3") << "\n";
    std::cout << "  " << dispatch(m3, "armor@x1.2") << "\n";
    std::cout << "测试1通过\n\n";

    // ===== 测试2：overloaded 显式三 lambda 版本（对照 stage7）=====
    std::cout << "=== 测试2：overloaded 显式 lambda 分发 ===\n";
    const auto describe = overloaded{
        [](const IpcOutput& o) { return "Ipc@" + o.shm_name; },
        [](const McuOutput& o) { return "Mcu@" + std::to_string(o.baud); },
        [](const ChiralOutput& o) { return "Chiral@" + std::to_string(o.port); },
    };
    CHECK(std::visit(describe, m1) == "Ipc@/dev/shm/talos_frames");
    CHECK(std::visit(describe, m2) == "Mcu@115200");
    CHECK(std::visit(describe, m3) == "Chiral@8765");
    std::cout << "  " << std::visit(describe, m1) << "\n";
    std::cout << "  " << std::visit(describe, m2) << "\n";
    std::cout << "  " << std::visit(describe, m3) << "\n";
    std::cout << "测试2通过\n\n";

    // ===== 测试3：编译期穷尽性（本测试用 static_assert 变体）=====
    // std::variant 替代类型数 = 3，std::variant_size_v 编译期可查
    // 各位置的类型也可编译期抽取：variant_alternative_t<1, OutputMode> == McuOutput
    static_assert(std::variant_size_v<OutputMode> == 3);
    static_assert(std::is_same_v<std::variant_alternative_t<1, OutputMode>, McuOutput>);
    // 但"当前活跃的是哪种类型"是运行期概念 —— holds_alternative 不是 constexpr
    CHECK(std::holds_alternative<McuOutput>(m2));
    std::cout << "=== 测试3：编译期穷尽性 ===\n";
    std::cout << "  variant_size_v<OutputMode> = " << std::variant_size_v<OutputMode>
              << "（编译期常量）\n";
    std::cout << "  variant_alternative_t<1>  = McuOutput（编译期类型抽取）\n";
    std::cout << "  holds_alternative<McuOutput>(m2) = true（运行期活跃类型）\n";
    std::cout << "测试3通过\n\n";
    // ------------------------------------------------------------------
    // 【编译期错误演示】以下代码放开任何一行都直接编译报错——
    // 这就是"穷尽性"的威力：漏分支在编译期被拦截，而非运行时兜底。
    //
    // (a) overloaded 漏掉一个类型的 lambda → std::visit 无匹配重载
    //     error: no matching function for call to 'visit'
    //     const auto bad = overloaded{
    //         [](const IpcOutput& o) { return o.shm_name; },
    //         [](const McuOutput& o) { return std::to_string(o.baud); },
    //         // 忘记 ChiralOutput 分支
    //     };
    //     auto s = std::visit(bad, m3);
    //
    // (b) variant 新增第 4 种类型后，旧 visit 全部失效（漏类型即报错）
    //     注意：variant 不允许重复类型（同类型只能出现一次），
    //     所以新增必须定义新类型，如 struct DaedalusOutput {...};
    //     using OutputMode4 = std::variant<IpcOutput, McuOutput,
    //                                      ChiralOutput, DaedalusOutput>;
    //     auto s = std::visit(bad, OutputMode4{...});  // 同样 no match
    // ------------------------------------------------------------------

    // ===== 测试4：std::monostate —— variant 的"空状态"=====
    // 真实项目场景：输出模式未配置时持 monostate，避免定义"无效对象"
    std::cout << "=== 测试4：monostate 空状态 ===\n";
    using MaybeOutput = std::variant<std::monostate, IpcOutput, McuOutput, ChiralOutput>;
    const MaybeOutput none{}; // 默认构造 = 持有 monostate

    const auto visitor = overloaded{
        [](std::monostate) { return std::string("（未配置输出）"); },
        [](const IpcOutput& o) { return o.shm_name; },
        [](const McuOutput& o) { return std::to_string(o.baud); },
        [](const ChiralOutput& o) { return std::to_string(o.port); },
    };
    CHECK(std::visit(visitor, none) == "（未配置输出）");
    CHECK(std::holds_alternative<std::monostate>(none));
    std::cout << "  monostate 默认态描述: " << std::visit(visitor, none) << "\n";
    std::cout << "测试4通过\n\n";

    // ===== 测试5：反面教材 —— 运行时 if-else 与 visit 行为一致，但更脆弱 =====
    std::cout << "=== 测试5：反面教材对比 ===\n";
    CHECK(dispatch_runtime(m1, "frame#1") == dispatch(m1, "frame#1"));
    CHECK(dispatch_runtime(m2, "yaw 0.3") == dispatch(m2, "yaw 0.3"));
    CHECK(dispatch_runtime(m3, "armor@x1.2") == dispatch(m3, "armor@x1.2"));
    std::cout << "  dispatch_runtime 与 std::visit 输出一致（行为等价）\n";
    std::cout << "  但缺点：漏写分支编译不报错、每分支两遍类型名、新增类型无提醒\n";
    std::cout << "测试5通过\n\n";

    if (g_failures == 0) {
        std::cout << "=== variant 演示完成 ===\n";
        return 0;
    }
    std::cerr << "=== 失败断言数: " << g_failures << " ===\n";
    return 1;
}
