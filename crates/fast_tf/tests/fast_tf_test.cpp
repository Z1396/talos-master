// 引入Google测试框架头文件，提供TEST、EXPECT、ASSERT系列断言宏
#include <gtest/gtest.h>

// 自定义环形缓冲区、坐标帧定义、变换矩阵模板头文件
#include "buffer.hpp"
#include "frame.hpp"
#include "matrix.hpp"

// C++标准类型特性库，用于编译期静态断言检测模板返回类型
#include <type_traits>

// 简化命名空间，当前文件内直接使用fast_tf下所有类型
using namespace fast_tf;

// 匿名命名空间，仅当前编译单元可见，隔离测试用虚拟坐标系标签
namespace {

// 三个空结构体，仅作为模板标签区分不同坐标系（无任何成员，编译期标记）
struct test_a_frame {};
struct test_b_frame {};
struct test_c_frame {};
// 单位坐标系标签，用于无平移旋转的单位变换测试
struct test_identity_frame {};

// 类型别名：双精度变换矩阵，源帧/目标帧均为单位坐标系，通用测试变换类型
using TestTransform = TransformMatrixd<test_identity_frame, test_identity_frame>;

} // 匿名命名空间结束

// ===================== 环形缓冲区Buffer基础功能测试 =====================
/**
 * @brief 测试Buffer基础入队、获取最新帧逻辑
 * TEST(测试套件名, 测试用例名)：GTest标准测试宏
 */
TEST(BufferBasicsTest, BasicPushAndLatest) {
    // 实例化环形缓冲区：存储TestTransform，最大容量10，插值维度1
    Buffer<TestTransform, 10, 1> buffer;

    // 断言缓冲区初始为空
    EXPECT_TRUE(buffer.is_empty());
    // 断言初始元素数量为0
    EXPECT_EQ(buffer.size(), 0);

    // 构造平移(1,2,3)的单位变换矩阵
    const auto tf = TestTransform::from_translation(1.0, 2.0, 3.0);
    // 压入时间戳1000的变换数据
    buffer.push(1000, tf);

    // 缓冲区不再为空
    EXPECT_FALSE(buffer.is_empty());
    // 元素数量为1
    EXPECT_EQ(buffer.size(), 1);

    // 获取缓冲区最新一帧数据
    const auto result = buffer.latest();
    // 断言返回值包含有效数据（std::expected有值）
    ASSERT_TRUE(result.has_value());
    // 校验最新帧时间戳等于1000
    EXPECT_EQ(result->timestamp, 1000);
}

/**
 * @brief 测试Buffer获取存储数据的时间戳区间（最早、最晚时间）
 */
TEST(BufferBasicsTest, TimeRange) {
    Buffer<TestTransform, 10, 1> buffer;

    // 空缓冲区无时间区间，返回无值
    EXPECT_FALSE(buffer.time_range().has_value());

    // 依次压入3个不同时间戳变换
    buffer.push(1000, TestTransform{});
    buffer.push(2000, TestTransform{});
    buffer.push(3000, TestTransform{});

    // 获取时间范围
    const auto range = buffer.time_range();
    ASSERT_TRUE(range.has_value());
    // 最早时间戳1000
    EXPECT_EQ(range->first, 1000);
    // 最晚时间戳3000
    EXPECT_EQ(range->second, 3000);
}

/**
 * @brief 测试clamped查找模式：区间内线性插值，超出区间直接返回边界帧
 */
TEST(BufferBasicsTest, InterpolationClamped) {
    Buffer<TestTransform, 10, 1> buffer;

    // 压入3个时间戳变换，X轴平移分别0、1、2
    buffer.push(0, TestTransform::from_translation(0.0, 0.0, 0.0));
    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(2.0, 0.0, 0.0));

    // 精确匹配时间戳1000，直接返回对应变换
    auto result = buffer.lookup(1000, clamped);
    ASSERT_TRUE(result.has_value());
    // 允许1e-6浮点误差，X平移等于1
    EXPECT_NEAR(result->value.translation().x(), 1.0, 1e-6);

    // 时间500，介于0~1000之间，线性插值得到0.5
    result = buffer.lookup(500, clamped);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->value.translation().x(), 0.5, 1e-6);

    // 时间0，区间下界，直接返回首帧
    result = buffer.lookup(0, clamped);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->value.translation().x(), 0.0, 1e-6);

    // 时间3000，超出最大时间2000，钳位返回最后一帧X=2
    result = buffer.lookup(3000, clamped);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->value.translation().x(), 2.0, 1e-6);
}

/**
 * @brief 测试环形缓冲区覆盖逻辑：填满后新数据覆盖最旧数据
 */
TEST(BufferBasicsTest, CircularOverwrite) {
    // 缓冲区最大容量3
    Buffer<TestTransform, 3, 1> buffer;

    // 填满3帧
    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(2.0, 0.0, 0.0));
    buffer.push(3000, TestTransform::from_translation(3.0, 0.0, 0.0));

    // 缓冲区已满标记
    EXPECT_TRUE(buffer.is_full());
    // 当前存储数量3
    EXPECT_EQ(buffer.size(), 3);

    // 再压入一帧，覆盖最旧1000帧
    buffer.push(4000, TestTransform::from_translation(4.0, 0.0, 0.0));

    // 容量不变仍为3
    EXPECT_EQ(buffer.size(), 3);

    const auto range = buffer.time_range();
    ASSERT_TRUE(range.has_value());
    // 最早时间变为2000，最新4000
    EXPECT_EQ(range->first, 2000);
    EXPECT_EQ(range->second, 4000);
}

// ===================== TransformMatrix 变换矩阵插值测试 =====================
/**
 * @brief 测试SE3线性插值平移部分
 */
TEST(TransformMatrixTest, LerpPosition) {
    // 起始变换原点，终点平移(10,20,30)
    const auto a = TestTransform::from_translation(0.0, 0.0, 0.0);
    const auto b = TestTransform::from_translation(10.0, 20.0, 30.0);

    // t=0.5插值中间值
    const auto mid = TestTransform::lerp(a, b, 0.5);
    // 平移各分量插值一半
    EXPECT_NEAR(mid.translation().x(), 5.0, 1e-6);
    EXPECT_NEAR(mid.translation().y(), 10.0, 1e-6);
    EXPECT_NEAR(mid.translation().z(), 15.0, 1e-6);
}

/**
 * @brief 测试四元数球面插值，0~90度绕Z轴旋转中间值45度
 */
TEST(TransformMatrixTest, LerpRotation) {
    // 单位无旋转、绕Z轴90度旋转
    const auto a = TestTransform::from_rpy(0.0, 0.0, 0.0);
    const auto b = TestTransform::from_rpy(0.0, 0.0, M_PI / 2.0);

    // t=0.5插值
    const auto mid   = TestTransform::lerp(a, b, 0.5);
    // 提取欧拉角
    const auto euler = mid.euler_rot();

    // 滚转、俯仰0，偏航45度(π/4)
    EXPECT_NEAR(euler.roll, 0.0, 1e-6);
    EXPECT_NEAR(euler.pitch, 0.0, 1e-6);
    EXPECT_NEAR(euler.yaw, M_PI / 4.0, 1e-6);
}

/**
 * @brief 测试SE3左插值右不变一致性：lerp_se3 等价于 a ∘ exp( (a⁻b) * t )
 */
TEST(TransformMatrixTest, LerpRightInvariantConsistency) {
    // 两个随机位姿变换
    const auto a = TestTransform::from_rpy(0.1, -0.2, 0.3, 1.0, 2.0, 3.0);
    const auto b = TestTransform::from_rpy(-0.4, 0.5, -0.6, 4.0, 5.0, 6.0);

    constexpr double t = 0.3;
    // SE3插值函数
    const auto mid     = TestTransform::lerp_se3(a, b, t);
    // 理论等价计算式：a 右乘 对数差乘t
    const auto mid_expected = a.rplus(a.rminus(b) * t);

    // 转换4×4齐次矩阵
    const auto M  = mid.matrix();
    const auto Me = mid_expected.matrix();
    // 矩阵范数差值接近0，证明插值公式数学自洽
    EXPECT_NEAR((M - Me).norm(), 0.0, 1e-9);
}

/**
 * @brief from_quaternion_xyz 构造函数自动归一化非单位四元数
 */
TEST(TransformMatrixTest, FromQuaternionNormalizes) {
    // 模长2的非归一化四元数
    Eigen::Quaterniond q(2.0, 0.0, 0.0, 0.0);
    // 构造变换，平移(1,2,3)
    const auto tf = TestTransform::from_quaternion_xyz(q, 1.0, 2.0, 3.0);

    // 内部四元数自动归一化，模长≈1
    EXPECT_NEAR(tf.quaternion().norm(), 1.0, 1e-12);
    // 平移分量完整保留
    EXPECT_NEAR(tf.translation().x(), 1.0, 1e-12);
    EXPECT_NEAR(tf.translation().y(), 2.0, 1e-12);
    EXPECT_NEAR(tf.translation().z(), 3.0, 1e-12);
}

/**
 * @brief 强类型坐标系变换：矩阵乘法、逆变换编译期坐标系类型校验
 */
TEST(TypedTransformMatrixTest, CompositionAndInverseAreStronglyTyped) {
    // A→B 变换矩阵
    const auto ab = TransformMatrixd<test_a_frame, test_b_frame>::from_translation(1.0, 0.0, 0.0);
    // B→C 变换矩阵
    const auto bc = TransformMatrixd<test_b_frame, test_c_frame>::from_translation(0.0, 2.0, 0.0);

    // 矩阵相乘：A→B * B→C = A→C，编译期校验输出类型
    const auto ac = ab * bc;
    // 静态断言：乘积类型严格为 TransformMatrixd<test_a_frame, test_c_frame>
    static_assert(std::is_same_v<
                  std::remove_cvref_t<decltype(ac)>, TransformMatrixd<test_a_frame, test_c_frame>>);

    // 逆变换：A→C 逆矩阵为 C→A
    const auto ca = ac.inv();
    static_assert(std::is_same_v<
                  std::remove_cvref_t<decltype(ca)>, TransformMatrixd<test_c_frame, test_a_frame>>);

    // 合成平移 X=1,Y=2,Z=0
    EXPECT_NEAR(ac.translation().x(), 1.0, 1e-6);
    EXPECT_NEAR(ac.translation().y(), 2.0, 1e-6);
    EXPECT_NEAR(ac.translation().z(), 0.0, 1e-6);

    // 变换乘逆变换得到单位矩阵 A→A
    const auto aa = ac * ca;
    static_assert(std::is_same_v<
                  std::remove_cvref_t<decltype(aa)>, TransformMatrixd<test_a_frame, test_a_frame>>);
    // 平移向量模长接近0，单位变换无平移
    EXPECT_NEAR(aa.translation().norm(), 0.0, 1e-9);
}

// ===================== Buffer 专用接口 push_rotate_only 测试 =====================
/**
 * @brief push_rotate_only：仅更新旋转，复用缓冲区历史平移
 */
TEST(BufferTest, PushRotateOnlyKeepsLatestTranslation) {
    Buffer<TestTransform, 8, 1> buffer;

    // 先压入完整变换，平移(1,2,3)
    buffer.push(1000, TestTransform::from_translation(1.0, 2.0, 3.0));
    // 仅推送旋转，时间戳2000，平移复用历史值
    buffer.push_rotate_only(2000, math_fuxk::rpy(0.1, -0.2, 0.3));

    // 精确查找2000时间戳
    const auto result = buffer.lookup(2000, exact);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 2000);
    // 平移复用1000帧的1,2,3
    EXPECT_NEAR(result->value.translation().x(), 1.0, 1e-6);
    EXPECT_NEAR(result->value.translation().y(), 2.0, 1e-6);
    EXPECT_NEAR(result->value.translation().z(), 3.0, 1e-6);

    // 旋转使用新推送的rpy
    const auto euler = result->value.euler_rot();
    EXPECT_NEAR(euler.roll, 0.1, 1e-6);
    EXPECT_NEAR(euler.pitch, -0.2, 1e-6);
    EXPECT_NEAR(euler.yaw, 0.3, 1e-6);
}

// 注释：该测试依赖已移除的坐标系全局工厂函数，暂时注释
// TODO: Restore FrameLookupTest after implementing hero_new() factory or similar
// The test below depends on a coordinate system initialization function that no longer exists
// TEST(FrameLookupTest, TypedLookupReturnsFrameSpecificTransform) { ... }

/**
 * @brief exact精确查找模式：时间戳完全匹配才返回数据
 */
TEST(BufferTest, ExactLookupReturnsStoredValue) {
    Buffer<TestTransform, 8, 1> buffer;

    buffer.push(1000, TestTransform::from_translation(11.0, 0.0, 0.0));

    const auto result = buffer.lookup(1000, exact);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 1000);
    EXPECT_NEAR(result->value.translation().x(), 11.0, 1e-6);
}

/**
 * @brief exact模式时间戳不匹配返回空，错误信息包含exact关键字
 */
TEST(BufferTest, ExactRequiresMatchingTimestamp) {
    Buffer<TestTransform, 8, 1> buffer;

    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(2.0, 0.0, 0.0));

    // 查询1500无精确匹配
    const auto result = buffer.lookup(1500, exact);
    EXPECT_FALSE(result.has_value());
    // 错误字符串包含exact标识
    EXPECT_TRUE(result.error().find("exact") != std::string::npos)
        << "expected 'exact' in error, got: " << result.error();
}

/**
 * @brief nearest就近查找：取时间戳距离最近的样本
 */
TEST(BufferTest, NearestReturnsClosestInRangeSample) {
    Buffer<TestTransform, 8, 1> buffer;

    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(2.0, 0.0, 0.0));

    // 1400距离1000更近，返回1000帧
    auto result = buffer.lookup(1400, nearest);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 1000);
    EXPECT_NEAR(result->value.translation().x(), 1.0, 1e-6);

    // 1600距离2000更近，返回2000帧
    result = buffer.lookup(1600, nearest);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 2000);
    EXPECT_NEAR(result->value.translation().x(), 2.0, 1e-6);
}

/**
 * @brief 中点1500距离相等时，就近模式选择更早一帧（1000）
 */
TEST(BufferTest, NearestBreaksExactMidpointTowardOlderSample) {
    Buffer<TestTransform, 8, 1> buffer;

    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(2.0, 0.0, 0.0));

    const auto result = buffer.lookup(1500, nearest);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 1000);
    EXPECT_NEAR(result->value.translation().x(), 1.0, 1e-6);
}

/**
 * @brief interpolate插值模式：仅区间内线性插值，超出区间直接报错（无外推）
 */
TEST(BufferTest, InterpolateUsesAdjacentSamplesWithoutExtrapolation) {
    Buffer<TestTransform, 8, 1> buffer;

    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(3.0, 0.0, 0.0));

    // 1500区间内插值得到2.0
    auto result = buffer.lookup(1500, interpolate);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 1500);
    EXPECT_NEAR(result->value.translation().x(), 2.0, 1e-6);

    // 500早于最早帧，插值模式禁止外推，返回错误
    result = buffer.lookup(500, interpolate);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(result.error().find("past extrapolation") != std::string::npos)
        << "expected 'past extrapolation' in error, got: " << result.error();
}

/**
 * @brief clamped钳位模式：区间内插值，超出上下限直接返回边界帧
 */
TEST(BufferTest, ClampedReturnsBoundarySamplesOutsideRange) {
    Buffer<TestTransform, 8, 1> buffer;

    buffer.push(1000, TestTransform::from_translation(1.0, 0.0, 0.0));
    buffer.push(2000, TestTransform::from_translation(3.0, 0.0, 0.0));

    // 500低于区间下限，钳位返回1000帧
    auto result = buffer.lookup(500, clamped);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 1000);
    EXPECT_NEAR(result->value.translation().x(), 1.0, 1e-6);

    // 2500高于上限，钳位返回2000帧
    result = buffer.lookup(2500, clamped);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 2000);
    EXPECT_NEAR(result->value.translation().x(), 3.0, 1e-6);

    // 区间内正常插值
    result = buffer.lookup(1500, clamped);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 1500);
    EXPECT_NEAR(result->value.translation().x(), 2.0, 1e-6);
}

/**
 * @brief static静态模式缓冲区：固定不变变换，任意查询时间都返回该变换
 * Buffer模板第一个参数容量0代表静态固定变换
 */
TEST(BufferTest, StaticModeReturnsSameValueForAnyQueryTime) {
    // 容量0，静态缓冲区，初始固定平移5m
    Buffer<TestTransform, 0, 1> buffer(TestTransform::from_translation(5.0, 0.0, 0.0));

    // 任意时间戳4242查询，均返回初始变换
    const auto result = buffer.lookup(4242, exact);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->timestamp, 4242);
    EXPECT_NEAR(result->value.translation().x(), 5.0, 1e-6);
}

// ===================== Spherial 球面坐标插值测试（弹道球面坐标） =====================
/**
 * @brief 插值系数t=0，完全返回第一个球面坐标
 */
TEST(SpherialTest, LerpAtZeroReturnsFirst) {
    const Spherial<double> a{.yaw = 0.5, .pitch = 0.3, .distance = 4.0};
    const Spherial<double> b{.yaw = 1.0, .pitch = 0.6, .distance = 8.0};
    const auto result = Spherial<double>::lerp(a, b, 0.0);
    EXPECT_NEAR(result.yaw, a.yaw, 1e-6);
    EXPECT_NEAR(result.pitch, a.pitch, 1e-6);
    EXPECT_NEAR(result.distance, a.distance, 1e-6);
}

/**
 * @brief 插值系数t=1，完全返回第二个球面坐标
 */
TEST(SpherialTest, LerpAtOneReturnsSecond) {
    const Spherial<double> a{.yaw = 0.5, .pitch = 0.3, .distance = 4.0};
    const Spherial<double> b{.yaw = 1.0, .pitch = 0.6, .distance = 8.0};
    const auto result = Spherial<double>::lerp(a, b, 1.0);
    EXPECT_NEAR(result.yaw, b.yaw, 1e-6);
    EXPECT_NEAR(result.pitch, b.pitch, 1e-6);
    EXPECT_NEAR(result.distance, b.distance, 1e-6);
}

/**
 * @brief 插值系数t<0，自动钳位到t=0，返回A
 */
TEST(SpherialTest, LerpClampsBelowZero) {
    const Spherial<double> a{.yaw = 0.5, .pitch = 0.3, .distance = 4.0};
    const Spherial<double> b{.yaw = 1.0, .pitch = 0.6, .distance = 8.0};
    const auto result = Spherial<double>::lerp(a, b, -0.5);
    EXPECT_NEAR(result.yaw, a.yaw, 1e-6);
    EXPECT_NEAR(result.pitch, a.pitch, 1e-6);
    EXPECT_NEAR(result.distance, a.distance, 1e-6);
}

/**
 * @brief 插值系数t>1，自动钳位到t=1，返回B
 */
TEST(SpherialTest, LerpClampsAboveOne) {
    const Spherial<double> a{.yaw = 0.5, .pitch = 0.3, .distance = 4.0};
    const Spherial<double> b{.yaw = 1.0, .pitch = 0.6, .distance = 8.0};
    const auto result = Spherial<double>::lerp(a, b, 1.5);
    EXPECT_NEAR(result.yaw, b.yaw, 1e-6);
    EXPECT_NEAR(result.pitch, b.pitch, 1e-6);
    EXPECT_NEAR(result.distance, b.distance, 1e-6);
}

/**
 * @brief 同方向球面坐标，距离线性插值，角度不变
 */
TEST(SpherialTest, LerpLinearDistance) {
    // 偏航俯仰完全相同，仅距离不同
    const Spherial<double> a{.yaw = 0.0, .pitch = 0.0, .distance = 2.0};
    const Spherial<double> b{.yaw = 0.0, .pitch = 0.0, .distance = 6.0};
    const auto result = Spherial<double>::lerp(a, b, 0.5);
    EXPECT_NEAR(result.distance, 4.0, 1e-6);
    EXPECT_NEAR(result.yaw, 0.0, 1e-6);
    EXPECT_NEAR(result.pitch, 0.0, 1e-6);
}

/**
 * @brief A→B t=0.5 与 B→A t=0.5 插值距离相等，仅方向相反
 */
TEST(SpherialTest, LerpMidpointSymmetry) {
    const Spherial<double> a{.yaw = 0.3, .pitch = 0.1, .distance = 3.0};
    const Spherial<double> b{.yaw = 0.8, .pitch = -0.2, .distance = 7.0};
    const auto ab = Spherial<double>::lerp(a, b, 0.5);
    const auto ba = Spherial<double>::lerp(b, a, 0.5);
    EXPECT_NEAR(ab.distance, ba.distance, 1e-6);
}

/**
 * @brief 两个完全相同球面坐标插值，结果与原坐标一致
 */
TEST(SpherialTest, LerpIdentityAtSamePoint) {
    const Spherial<double> a{.yaw = 0.0, .pitch = 0.0, .distance = 5.0};
    const auto result = Spherial<double>::lerp(a, a, 0.5);
    EXPECT_NEAR(result.yaw, a.yaw, 1e-6);
    EXPECT_NEAR(result.pitch, a.pitch, 1e-6);
    EXPECT_NEAR(result.distance, a.distance, 1e-6);
}

/**
 * @brief 两个零距离球面坐标插值，距离始终为0
 */
TEST(SpherialTest, LerpZeroDistanceStaysZero) {
    const Spherial<double> a{.yaw = 0.0, .pitch = 0.0, .distance = 0.0};
    const Spherial<double> b{.yaw = 1.0, .pitch = 1.0, .distance = 0.0};
    const auto result = Spherial<double>::lerp(a, b, 0.5);
    EXPECT_NEAR(result.distance, 0.0, 1e-9);
}

/**
 * @brief 同方向远距离插值，浮点精度无丢失
 */
TEST(SpherialTest, LerpPreservesFloatPrecision) {
    const Spherial<double> a{.yaw = 0.0, .pitch = 0.0, .distance = 1.0};
    const Spherial<double> b{.yaw = 0.0, .pitch = 0.0, .distance = 2.0};
    const auto result = Spherial<double>::lerp(a, b, 0.5);
    EXPECT_NEAR(result.distance, 1.5, 1e-6);
    EXPECT_NEAR(result.yaw, 0.0, 1e-6);
}

/**
 * @brief 正负对称偏航球面插值，中点落在X轴正方向(yaw=0)
 */
TEST(SpherialTest, LerpNegativeYawProducesCorrectQuadrant) {
    // 左右对称偏航，距离均为1
    const Spherial<double> a{.yaw = -0.5, .pitch = 0.0, .distance = 1.0};
    const Spherial<double> b{.yaw = 0.5, .pitch = 0.0, .distance = 1.0};
    const auto result = Spherial<double>::lerp(a, b, 0.5);
    // 插值后偏航归零，距离为cos(0.5)
    EXPECT_NEAR(result.yaw, 0.0, 1e-6);
    EXPECT_NEAR(result.pitch, 0.0, 1e-6);
    EXPECT_NEAR(result.distance, std::cos(0.5), 1e-6);
}

// 注释：该测试依赖已移除的坐标系全局工厂函数，暂时注释禁用
// TODO: Restore FrameLookupTest after implementing hero_new() factory or similar
// The test below depends on a coordinate system initialization function that no longer exists
// TEST(FrameLookupTest, TypedLookupReturnsFrameSpecificTransform) { ... }