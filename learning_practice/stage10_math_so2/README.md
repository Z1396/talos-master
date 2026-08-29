# stage10：math 模块 —— SO(2) 李群 + ROS2 RPY 欧拉角

对应真实代码：`crates/math/src/`（207 行，header-only）
- [so2.hpp](../../crates/math/src/so2.hpp)：SO(2) 平面旋转李群
- [euler.hpp](../../crates/math/src/euler.hpp)：ROS2 Z-Y-X 欧拉角（RPY）与四元数/旋转矩阵/SO3 互转

## 功能分析

### so2.hpp —— 为什么不用裸 double 存角度

跟踪器解算 yaw 时，`a - b` 跨 ±π 边界会出现数值跳变：

```
裸减法：350° - 10° = +340°   （几何上绕远路）
SO2   ：350° - 10° = -20°    （流形增量，最短弧）
```

EKF 拿到 +340° 的"增量"会把一步 20° 的旋转当成几乎一整圈，滤波器直接发散。
`SO2::operator-` 内部用 `std::remainder` 归一化到 [-π, π]，从根上消除跳变。

**读源码确认的 API 细节（与直觉不同）**：
1. 构造函数**不**归一化：`SO2(350°)` 内部就存 350°；
2. `operator+` **不**归一化：`a + delta` 原样相加；
3. 只有 `operator-` 归一化 —— 叠加后要落回主值区间就再减一个 `SO2(0)`。

### euler.hpp —— ROS2 TF2 约定

旋转顺序 `Rz(yaw) · Ry(pitch) · Rx(roll)`（先 roll 后 yaw 的内旋约定），
提供 `rpy(roll, pitch, yaw)` 工厂和从矩阵/四元数/SO3 反解的 `rpy()` 重载，
万向锁（pitch = ±90°）分支单独处理。

## 运行方法

```bash
cd learning_practice/stage10_math_so2
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/math_so2_demo     # 全部通过返回 0，任一断言失败返回 1
```

依赖：Eigen（系统安装或自动 FetchContent 3.4.0）、项目内 `3dparty/lieplusplus`。

## 预期输出

```
=== 测试1：SO2 跨 ±π 边界减法 ===
  SO2:  350° - 10° = -20°  (流形增量，最短弧)
  raw:  350° - 10° = 340°  (裸减法，绕远路)
测试1通过

=== 测试2：增量叠加与回绕 ===
  (π-0.1) + 0.2 裸值    = 3.24159  (越界，> π)
  再减 SO2(0) 归一化后   = -3.04159  (= -π+0.1)
测试2通过

=== 测试3：跟踪器 yaw 跳变场景 ===
  179° → -179°  SO2 增量 = 2°  (真实转角)
               裸减法    = -358°  (虚假整圈跳变)
测试3通过

=== 测试4：RPY ↔ 矩阵/四元数往返 ===
  原始 RPY   : roll=0.1, pitch=-0.2, yaw=0.3
  矩阵往返差 : < 1e-9
  四元数往返差: < 1e-9
测试4通过

=== 测试5：Z-Y-X 旋转语义 ===
  yaw=90°: 机体X轴 (1,0,0) → 世界 (0, 1, 0)，命中 +Y 轴
  so3().R() 与 matrix() 一致（误差 < 1e-12）
测试5通过

=== stage10 math 模块全部测试通过 ===
```

## 测试清单（src/demo.cpp）

| 测试 | 断言要点 |
|---|---|
| 1 跨边界减法 | `SO2(350°)-SO2(10°)` = -20°，裸减法 = +340° 反例对照 |
| 2 增量回绕 | `π-0.1 + 0.2` 裸值 > π；再减 `SO2(0)` 归一化 = -π+0.1 |
| 3 跟踪器场景 | 179°→-179°：SO2 增量 +2°，裸减法 -358° |
| 4 RPY 往返 | rpy→matrix→rpy、rpy→quat→rpy 误差 < 1e-9 |
| 5 旋转语义 | yaw=90° 把机体 X 轴转到世界 +Y；`so3().R()` 与 `matrix()` 一致 |
