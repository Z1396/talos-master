// ===========================================================================
// 阶段10：math 模块 —— SO(2) 李群 + ROS2 RPY 欧拉角
//
// 文件对齐真实项目（crates/math/src/）：
//   - so2.hpp   SO(2) 平面旋转李群：减法 = 流形增量（自动归一化 [-π,π]）
//   - euler.hpp ROS2 Z-Y-X 欧拉角（rpy 工厂 + quat/matrix/so3 互转）
//
// 为什么不用裸 double 存角度（跟踪器 yaw 解算不跳变的基础）：
//   a - b 跨 ±π 边界时，裸减法得 2π-ε（如 350°-10°=340°），
//   SO2 流形减法得 ε（-20°），两者几何等价但数值上前者会让
//   EKF 的 yaw 增量瞬间出现 2π 的虚假跳变。
//
// ⚠ 读源码确认的真实 API 行为（与最初计划的差异）：
//   1. SO2 构造函数【不】归一化：SO2(350°) 内部就存 350°；
//   2. operator+ 【不】归一化：a+delta 原样相加（π-0.1 + 0.2 = π+0.1）；
//   3. 只有 operator- 用 std::remainder 归一化到 [-π,π] ——
//      所以“增量叠加后回绕”要靠再减一个 SO2(0) 实现（见测试2）。
//
// 测试清单
// 测试1：跨 ±π 边界减法：350° - 10° = -20°（而非 +340°），裸 double 反例对照
// 测试2：增量叠加跨界：π-0.1 + 0.2 = π+0.1（裸值），再减 0 归一化得 -π+0.1
// 测试3：跟踪器场景：179° → -179° 的真实增量是 +2°，裸减法得 -358°
// 测试4：euler 往返：rpy -> matrix -> rpy、rpy -> quat -> rpy 数值复原
// 测试5：旋转语义：yaw=90° 把机体 X 轴转到世界 Y 轴（Z-Y-X 内旋约定）
// ===========================================================================

// SO(2) 二维旋转李群（真实项目头文件）
#include "so2.hpp"
// ROS2 RPY 欧拉角（真实项目头文件，依赖 Eigen + 3dparty/lieplusplus）
#include "euler.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <numbers>
#include <string>

namespace mf = math_fuxk;

// ===========================================================================
// 轻量断言：失败打印位置并累计，main 末尾以非零退出码结束
// ===========================================================================
static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  [CHECK 失败] " #cond "  (" << __FILE__ << ":"      \
                      << __LINE__ << ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (0)

static double deg2rad(double deg) { return deg * std::numbers::pi / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / std::numbers::pi; }

// ===========================================================================
// 测试1：跨 ±π 边界的流形减法（SO2 存在的全部意义）
// 350° 与 10° 在圆上只差 20°，但裸减法得 340°（几何上绕远路）
// ===========================================================================
void test_so2_cross_boundary() {
    std::cout << "=== 测试1：SO2 跨 ±π 边界减法 ===\n";

    mf::SO2<double> a(deg2rad(350.0)); // 构造不归一化，内部存 350°
    mf::SO2<double> b(deg2rad(10.0));

    // operator- : std::remainder(340°, 360°) = -20°
    const mf::SO2<double> diff  = a - b;
    const double          so2_v = static_cast<double>(diff);

    // 裸 double 反例
    const double raw_v = deg2rad(350.0) - deg2rad(10.0); // = +340°

    CHECK(std::abs(so2_v - deg2rad(-20.0)) < 1e-12);
    CHECK(std::abs(raw_v - deg2rad(340.0)) < 1e-12);

    std::cout << "  SO2:  350° - 10° = " << rad2deg(so2_v) << "°  (流形增量，最短弧)\n";
    std::cout << "  raw:  350° - 10° = " << rad2deg(raw_v) << "°  (裸减法，绕远路)\n";
    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：增量叠加 + 回绕
// 注意：operator+ 不归一化（源码 so2.hpp:45 无 remainder），
// 叠加后再减 SO2(0) 才落回 [-π, π]
// ===========================================================================
void test_so2_increment_wrap() {
    std::cout << "=== 测试2：增量叠加与回绕 ===\n";

    mf::SO2<double> a(std::numbers::pi - 0.1);

    const auto sum = a + 0.2; // operator+：不归一化，π-0.1+0.2 = π+0.1
    const auto zero = mf::SO2<double>(0.0);
    const auto normalized = sum - zero; // 减法归一化：π+0.1 → -π+0.1

    const double raw_v     = static_cast<double>(sum);
    const double wrapped_v = static_cast<double>(normalized);

    CHECK(raw_v > std::numbers::pi); // 裸值已越界
    CHECK(std::abs(wrapped_v - (-(std::numbers::pi - 0.1))) < 1e-12);

    std::cout << "  (π-0.1) + 0.2 裸值    = " << raw_v << "  (越界，> π)\n";
    std::cout << "  再减 SO2(0) 归一化后   = " << wrapped_v << "  (= -π+0.1)\n";
    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：跟踪器 yaw 场景（armor tracker 解算不跳变的根因）
// 上一帧 yaw=179°，当前帧 yaw=-179°：云台只顺时针转了 2°，
// 裸减法却得 -358°，EKF 会把这一步当成一整圈的反向旋转
// ===========================================================================
void test_so2_tracker_scenario() {
    std::cout << "=== 测试3：跟踪器 yaw 跳变场景 ===\n";

    mf::SO2<double> prev_yaw(deg2rad(179.0));
    mf::SO2<double> curr_yaw(deg2rad(-179.0));

    const auto   delta_so2 = curr_yaw - prev_yaw;
    const double delta     = static_cast<double>(delta_so2);
    const double delta_raw = deg2rad(-179.0) - deg2rad(179.0);

    CHECK(std::abs(delta - deg2rad(2.0)) < 1e-12);   // SO2: +2°
    CHECK(std::abs(delta_raw - deg2rad(-358.0)) < 1e-12); // 裸: -358°

    std::cout << "  179° → -179°  SO2 增量 = " << rad2deg(delta) << "°  (真实转角)\n";
    std::cout << "               裸减法    = " << rad2deg(delta_raw) << "°  (虚假整圈跳变)\n";
    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：euler 往返一致性
// rpy(roll,pitch,yaw) → 旋转矩阵 → rpy(矩阵) 应复原三个角；
// 同理 rpy → 四元数 → rpy。覆盖 euler.hpp 的三条转换链
// ===========================================================================
void test_euler_roundtrip() {
    std::cout << "=== 测试4：RPY ↔ 矩阵/四元数往返 ===\n";

    const auto e = mf::rpy(0.1, -0.2, 0.3); // Ros2EulerRotd

    // 链路1：rpy -> matrix -> rpy
    const auto from_matrix = mf::rpy(e.matrix());
    CHECK(std::abs(from_matrix.roll - e.roll) < 1e-9);
    CHECK(std::abs(from_matrix.pitch - e.pitch) < 1e-9);
    CHECK(std::abs(from_matrix.yaw - e.yaw) < 1e-9);

    // 链路2：rpy -> quat -> rpy
    const auto from_quat = mf::rpy(e.quat());
    CHECK(std::abs(from_quat.roll - e.roll) < 1e-9);
    CHECK(std::abs(from_quat.pitch - e.pitch) < 1e-9);
    CHECK(std::abs(from_quat.yaw - e.yaw) < 1e-9);

    // 链路3：rpy().rpy() 元组解包
    const auto [r, p, y] = e.rpy();
    CHECK(r == e.roll && p == e.pitch && y == e.yaw);

    std::cout << "  原始 RPY   : roll=" << e.roll << ", pitch=" << e.pitch
              << ", yaw=" << e.yaw << "\n";
    std::cout << "  矩阵往返差 : < 1e-9\n";
    std::cout << "  四元数往返差: < 1e-9\n";
    std::cout << "测试4通过\n\n";
}

// ===========================================================================
// 测试5：旋转语义 —— Z-Y-X 内旋约定（Rz(yaw)·Ry(pitch)·Rx(roll)）
// yaw=90° 时，机体 X 轴（枪管指向）应转到世界 Y 轴
// ===========================================================================
void test_euler_rotation_semantics() {
    std::cout << "=== 测试5：Z-Y-X 旋转语义 ===\n";

    const auto e = mf::rpy(0.0, 0.0, std::numbers::pi / 2); // 仅 yaw = 90°

    const Eigen::Vector3d body_x  = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d world_v = e.matrix() * body_x;

    CHECK((world_v - Eigen::Vector3d::UnitY()).norm() < 1e-12);

    // so3()：转 lieplusplus 的 group::SO3 李群封装（euler.hpp:72）
    const auto g = e.so3();
    CHECK((g.R() * body_x - world_v).norm() < 1e-12);

    std::cout << "  yaw=90°: 机体X轴 (1,0,0) → 世界 (" << world_v.x() << ", " << world_v.y()
              << ", " << world_v.z() << ")，命中 +Y 轴\n";
    std::cout << "  so3().R() 与 matrix() 一致（误差 < 1e-12）\n";
    std::cout << "测试5通过\n\n";
}

// ===========================================================================
// 主函数：依次运行所有测试，任一断言失败返回非零
// ===========================================================================
int main() {
    test_so2_cross_boundary();   // 1. 跨 ±π 边界减法
    test_so2_increment_wrap();   // 2. 增量叠加与回绕
    test_so2_tracker_scenario(); // 3. 跟踪器 yaw 跳变场景
    test_euler_roundtrip();      // 4. RPY 往返一致性
    test_euler_rotation_semantics(); // 5. Z-Y-X 旋转语义

    if (g_failures == 0) {
        std::cout << "=== stage10 math 模块全部测试通过 ===\n";
        return 0;
    }
    std::cerr << "=== stage10 失败断言数: " << g_failures << " ===\n";
    return 1;
}
