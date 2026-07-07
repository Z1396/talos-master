#pragma once
// 头文件保护宏，防止重复包含导致重定义编译错误

// 数学库：std::remainder 角度取模、三角函数
#include <cmath>
// C++20 标准数学常量 π
#include <numbers>

namespace math_fuxk {

/**
 * @brief SO(2) 二维旋转李群封装
 * 用于平面单自由度旋转（偏航、航向角等二维旋转量）
 * 内部存储约束：角度自动归一化到 [-π, π] 弧度区间
 * 底层存储固定为 double，输入任意数值类型均转为双精度
 */
template <typename T>
struct SO2 {
public:
    /**
     * @brief 构造函数，接收任意数值类型角度，转为double存储
     * @param v 输入旋转角（右值引用，支持拷贝/临时变量）
     */
    explicit SO2(T&& v)
        : angle(v) {}

    /**
     * @brief 二元减法运算符：两个SO2旋转做流形减法（角度差，归一化到±π）
     * 几何含义：a 相对于 b 的旋转增量 Δθ = a - b
     * @param a 左操作数 SO2 旋转
     * @param b 右操作数 SO2 旋转
     * @return 差值归一化后的新 SO2 实例
     */
    friend SO2 operator-(const SO2& a, const SO2& b) {
        // std::remainder 将差值映射至 [-π, π] 周期区间
        return SO2(std::remainder(a.angle - b.angle, 2.0 * std::numbers::pi));
    }

    /**
     * @brief 旋转加法：SO2 旋转 + 标量角度增量
     * @param a 基础SO2旋转
     * @param delta 标量角度增量（任意数值类型T）
     * @return 叠加增量后的新SO2实例
     */
    friend SO2 operator+(const SO2& a, const T& delta) {
        return SO2(a.angle + static_cast<double>(delta));
    }

    /**
     * @brief 显式类型转换：把SO2对象转为double，取出内部角度值
     */
    explicit operator double() const { return angle; }

private:
    // 内部存储旋转角，固定双精度浮点，值域 [-π, π] rad
    double angle;
};

} // namespace math_fuxk