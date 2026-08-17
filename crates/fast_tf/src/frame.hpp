#pragma once
// 时序插值环形缓冲区定义
#include "buffer.hpp"
// RPY欧拉角数据结构
#include "euler.hpp"
// SE(3)强类型变换矩阵 TransformMatrix
#include "matrix.hpp"
// 全局基础类型别名、常量模板
#include "types.hpp"

#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

//================================================================================
// stdx 元编程辅助库：实现带cv限定符、左右值属性完美转发工具
// 作用：保存变量的const/volatile、左值/右值属性，实现C++23 deducing this 透明转发
//================================================================================
namespace stdx {

// ---------- 1. copy_cv：复制源类型 const/volatile 修饰符到目标类型 ----------
// 基础模板：默认不带任何cv修饰
template <class From, class To>
struct copy_cv {
    using type = To;
};
// 特化：源带const → 目标加上const
template <class From, class To>
struct copy_cv<const From, To> {
    using type = const To;
};
// 特化：源带volatile → 目标加上volatile
template <class From, class To>
struct copy_cv<volatile From, To> {
    using type = volatile To;
};
// 特化：源同时带const+volatile
template <class From, class To>
struct copy_cv<const volatile From, To> {
    using type = const volatile To;
};
// 别名简化使用
template <class From, class To>
using copy_cv_t = typename copy_cv<From, To>::type;

// ---------- 2. copy_ref：复制源类型左/右值引用属性 ----------
template <class From, class To>
struct copy_ref {
    using type = To;
};
// 左值引用特化
template <class From, class To>
struct copy_ref<From&, To> {
    using type = To&;
};
// 右值引用特化
template <class From, class To>
struct copy_ref<From&&, To> {
    using type = To&&;
};
template <class From, class To>
using copy_ref_t = typename copy_ref<From, To>::type;

// ---------- 3. forward_like_t：组合cv+引用复制，完整继承源类型属性 ----------
// 逻辑：剥离From自身引用，复制cv修饰符，再复制引用属性到To
template <class From, class To>
using forward_like_t =
    copy_ref_t<From, copy_cv_t<std::remove_reference_t<From>, std::remove_reference_t<To>>>;

// ---------- 4. forward_like：运行时转发函数，等价带属性继承的std::forward ----------
// Like：参照类型（继承它的const/左右值属性）；T：待转发变量
template <class Like, class T>
constexpr forward_like_t<Like, T&&> forward_like(T&& x) noexcept {
    return static_cast<forward_like_t<Like, T&&>>(x);
}

} // namespace stdx

//================================================================================
// fast_tf 顶层命名空间：树形强类型坐标变换系统
//================================================================================
namespace fast_tf {

// ====================== 帧树声明宏：编译期构建坐标系继承树 ======================
/**
 * @macro DECL_ROOT
 * 根帧宏：顶层坐标系world，无父节点(ancestor=void)
 * name_fuxk_frame：帧唯一类型名；frame_id：运行时打印用字符串名称
 */
#define DECL_ROOT(name)                                     \
    struct name##_fuxk_frame {                              \
        static constexpr std::string_view frame_id = #name; \
        using ancestor                             = void;  \
    };

/**
 * @macro DECL
 * 普通子帧宏：指定父帧parent
 * ancestor = parent##_fuxk_frame，建立编译期继承关系
 */
#define DECL(name, parent)                                                \
    struct name##_fuxk_frame {                                            \
        static constexpr std::string_view frame_id = #name;               \
        using ancestor                             = parent##_fuxk_frame; \
    };

/**
 * @macro DECL_WITH_ID
 * 自定义字符串ID子帧：帧类型名和打印字符串分离，适配ROS标准帧名 camera_optical_frame
 */
#define DECL_WITH_ID(name, parent, id)                                    \
    struct name##_fuxk_frame {                                            \
        static constexpr std::string_view frame_id = id;                  \
        using ancestor                             = parent##_fuxk_frame; \
    };

// ---------------------- 完整机器人坐标系树定义 ----------------------
// 根坐标系：世界全局坐标系
DECL_ROOT(world);
// odom里程计坐标系，父world
DECL(odom, world);
// 云台偏航yaw，父odom
DECL(gimbal_yaw, odom);
// 云台俯仰pitch，父gimbal_yaw
DECL(gimbal_pitch, gimbal_yaw);
// 相机机械安装基座，父俯仰云台
DECL(camera_link, gimbal_pitch);
// 相机光学帧（ROS标准名称camera_optical_frame）
DECL_WITH_ID(camera_optical, camera_link, "camera_optical_frame");
// 枪管发射点坐标系，父俯仰云台
DECL(muzzle_link, gimbal_pitch);

// 销毁宏，防止外部污染命名空间
#undef DECL_ROOT
#undef DECL
#undef DECL_WITH_ID

// 简短类型别名，业务代码不用写完整xx_fuxk_frame
using world          = world_fuxk_frame;
using odom           = odom_fuxk_frame;
using gimbal         = gimbal_yaw_fuxk_frame;
using gimbal_pitch   = gimbal_pitch_fuxk_frame;
using camera         = camera_link_fuxk_frame;
using camera_optical = camera_optical_fuxk_frame;
using muzzle         = muzzle_link_fuxk_frame;

// ====================== C++20 Concept：帧类型约束 ======================
/**
 * @concept frame
 * 合法坐标系标签必须满足两点：
 * 1. 存在静态字符串 frame_id（日志报错打印帧名称）
 * 2. 存在类型别名 ancestor（父帧类型，void代表根帧）
 */
template <typename T>
concept frame = requires {
    /*requires 表达式语法：`{表达式} -> 约束;`
    代表：**表达式`T::frame_id`求值得到的类型，必须可以隐式转换成`std::string_view`**。*/
    { T::frame_id } -> std::convertible_to<std::string_view>;
    // 要求2：T内部必须存在嵌套类型别名 ancestor
    typename T::ancestor;
};


/**
 * @concept root_frame
 * 根帧约束：合法frame，且父类型是void
 */
// template<typename T> 模板，接收一个类型T做编译期检查
// concept：C++20 概念，编译期类型约束，用来做静态断言、函数模板参数筛选，编译期报错提示友好
// 先确定T是否是合法frame，再判断ancestor是否是void
template <typename T>
concept root_frame = frame<T> && std::is_void_v<typename T::ancestor>;

/**
 * @concept non_root_frame
 * 非根子帧约束：合法frame，存在有效父帧
 */
template <typename T>
/*std::is_void<X>::value
编译期判断：类型 X 是不是`void`。
 X = void → `value = true`
 X = WorldFrame / int / double → `value = false`

完全发生在编译期，**零运行时开销**。*/
concept non_root_frame = frame<T> && !std::is_void_v<typename T::ancestor>;


// ====================== 编译期递归工具：判断子帧是否是目标帧后代 ======================
/**
 * @brief 编译期常量函数：判断 Child 坐标系是否在 Ancestor 的子树内
 * 递归向上遍历ancestor父链，编译期完成判断，无运行时开销
 * @return true Child是Ancestor后代，允许帧变换查询；false 两棵独立树，编译/运行报错
 */
template <frame Child, frame Ancestor>
constexpr bool is_descendant_of() noexcept {
    // 递归终止1：当前帧和目标帧相同，是后代
    if constexpr (std::is_same_v<Child, Ancestor>) {
        return true;
    }
    // 递归终止2：走到根帧仍不匹配，无继承关系
    else if constexpr (std::is_void_v<typename Child::ancestor>) {
        return false;
    }
    // 递归向上查询父帧
    else {
        return is_descendant_of<typename Child::ancestor, Ancestor>();
    }
}

//================================================================================
// 强类型绑定容器 Fuck<Coordinate, T>
// 核心设计：给任意数值/向量/矩阵绑定单一坐标系，运算符做帧校验
// 禁止不同坐标系向量直接加减（编译报错）；乘法缩放、矩阵变换允许跨帧
//================================================================================
template <frame Coordinate, typename T>
struct [[nodiscard]] Fuck {
    // 默认构造、移动构造
    explicit Fuck(T t)
        : val(std::move(t)) {}
    explicit Fuck() = default;

    // ========================================================================
    // C++23 Deducing this 透明解引用：完美转发const/左右值属性
    // *this 获取内部存储T，自动匹配外层const/左/右值
    // ========================================================================
    template <typename Self>
    [[nodiscard]] constexpr auto operator*(this Self&& self) noexcept -> decltype(auto) {
        // 继承Self的cv与引用属性转发内部成员val
        return stdx::forward_like<Self>(self.val);
    }

    // -> 指针解引用，返回带const属性的成员地址
    template <typename Self>
    [[nodiscard]] constexpr auto operator->(this Self&& self) noexcept -> decltype(auto) {
        return std::addressof(self.val);
    }

    // ========================================================================
    // 二元运算符：坐标系安全运算
    // 1. 标量乘法：同帧/跨帧均可缩放向量，结果保留当前坐标系
    // 2. 向量加减：强制要求两边坐标系完全一致，否则编译报错
    // ========================================================================
    // 向量 * 普通标量，结果仍属于Coordinate坐标系
    template <typename Self>
    [[nodiscard]] constexpr auto operator*(this Self&& self, const T& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * other)> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * other)>(
            stdx::forward_like<Self>(self.val) * other);
    }

    // 同类型包裹向量相乘（点积/矩阵乘），允许跨坐标系，结果坐标系为左侧
    template <typename Self, frame OtherCoord>
    [[nodiscard]] constexpr auto
        operator*(this Self&& self, const Fuck<OtherCoord, T>& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * (*other))> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) * (*other))>(
            stdx::forward_like<Self>(self.val) * (*other));
    }

    // 向量加法：requires 强制两个坐标系完全相同，跨帧直接编译失败
    template <typename Self, frame OtherCoord>
    requires std::is_same_v<Coordinate, OtherCoord> [[nodiscard]] constexpr auto
        operator+(this Self&& self, const Fuck<OtherCoord, T>& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) + (*other))> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) + (*other))>(
            stdx::forward_like<Self>(self.val) + (*other));
    }

    // 向量减法：同加法，仅同坐标系可运算
    template <typename Self, frame OtherCoord>
    requires std::is_same_v<Coordinate, OtherCoord> [[nodiscard]] constexpr auto
        operator-(this Self&& self, const Fuck<OtherCoord, T>& other) noexcept
        -> Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) - (*other))> {
        return Fuck<Coordinate, decltype(stdx::forward_like<Self>(self.val) - (*other))>(
            stdx::forward_like<Self>(self.val) - (*other));
    }

    // ========================================================================
    // 透明函数调用：如果T是可调用对象，完美转发全部参数并继承属性
    // ========================================================================
    template <typename Self, typename... Args>
    [[nodiscard]] constexpr auto operator()(this Self&& self, Args&&... args) noexcept(
        noexcept(stdx::forward_like<Self>(self.val)(std::forward<Args>(args)...)))
        -> decltype(stdx::forward_like<Self>(self.val)(std::forward<Args>(args)...)) {
        return stdx::forward_like<Self>(self.val)(std::forward<Args>(args)...);
    }

private:
    // 底层包裹原始数据：Eigen向量/矩阵/欧拉角/数值
    T val;
};

/**
 * @brief 辅助工厂函数，快速构造绑定坐标系的包裹对象
 * 示例：fucked<camera_optical>(vec) → Fuck<camera_optical, Eigen::Vector3d>
 */
template <typename Coordinate, typename T>
inline auto fucked(T&& t) {
    return Fuck<Coordinate, T>(std::forward<T>(t));
}

//================================================================================
// 变换矩阵类型别名封装，复用前文 TransformMatrix 强类型SE3
//================================================================================
// FrameTransform<A,B> = T_AB：B坐标系点转A坐标系
template <frame A, frame B>
using FrameTransform = TransformMatrixd<A, B>;
// 单位变换：同一坐标系T_CC
template <frame Coordinate>
using IdentityTransform = FrameTransform<Coordinate, Coordinate>;

/**
 * @brief EdgeTransform<Layer> 单条树边变换：父帧 → 当前帧
 * EdgeTransform<camera_link> = FrameTransform<camera_link::ancestor, camera_link>
 * 含义：把camera_link下的点转换到gimbal_pitch（父帧），对应时序缓冲区存储的每条边变换
 */
template <non_root_frame Layer>
using EdgeTransform = FrameTransform<typename Layer::ancestor, Layer>;

// 常用浮点包裹别名
template <frame Coordinate>
using FuckDouble = Fuck<Coordinate, double>;
template <frame Coordinate>
using FuckFloat = Fuck<Coordinate, float>;
template <frame Coordinate>
using FuckIntrinsicEulerRotd = Fuck<Coordinate, math_fuxk::Ros2EulerRotd>;
template <frame Coordinate>
using FuckIntrinsicEulerRotf = Fuck<Coordinate, math_fuxk::Ros2EulerRotf>;

//================================================================================
// 时序缓冲区、全局坐标系存储容器
//================================================================================
/**
 * @brief EphemeralBuffer：单条树边时序环形缓存
 * EdgeTransform<T>：存储父→子帧变换；5=最小缓存容量；500=最大采样点
 */
template <typename T>
using EphemeralBuffer = Buffer<EdgeTransform<T>, 5, 500>;

/**
 * @brief CoordinateSystem 全局坐标系根容器
 * tuple依次存储所有非根帧的时序变换缓冲区：
 * odom、gimbal_yaw、gimbal_pitch、camera_link、camera_optical、muzzle_link
 * 完整保存整棵坐标系树所有边的时序变换历史，支持插值查询任意时刻T_AB
 */
using CoordinateSystem = std::tuple<
    EphemeralBuffer<odom_fuxk_frame>, EphemeralBuffer<gimbal_yaw_fuxk_frame>,
    EphemeralBuffer<gimbal_pitch_fuxk_frame>, EphemeralBuffer<camera_link_fuxk_frame>,
    EphemeralBuffer<camera_optical_fuxk_frame>, EphemeralBuffer<muzzle_link_fuxk_frame>>;

// ====================== 缓冲区元编程匹配工具 ======================
// buffer_coord_of：提取Buffer存储变换的子帧B（EdgeTransform<Layer>的Layer）
template <typename Layer>
struct buffer_coord_of;

template <typename Layer>
using buffer_coord_t = buffer_coord_of<Layer>::type;

// 偏特化：从Buffer<Transform<Scalar,A,B>>提取目标帧B
template <typename Scalar, typename A, typename B, sec Range, sample_per_sec Density>
struct buffer_coord_of<Buffer<TransformMatrix<Scalar, A, B>, Range, Density>> {
    using type = B;
};

/**
 * @brief buffer_of 编译期递归遍历tuple，找到对应Layer的缓冲区并返回引用
 * @param system CoordinateSystem全局tuple容器
 * @return EphemeralBuffer<Layer>& 该帧的时序变换缓存
 * static_assert 找不到帧直接编译报错
 */
template <non_root_frame Layer, typename System, size_t I = 0>
[[nodiscard]] constexpr decltype(auto) buffer_of(System&& system) noexcept {
    using tuple_t = std::remove_cvref_t<System>;
    // 递归遍历超出tuple长度，编译报错
    /*static_assert(条件, "报错提示字符串");
    - 如果**条件为 true**：啥事没有，编译器直接跳过。
    - 如果**条件为 false**：编译直接失败，打印你写的提示文字，程序连二进制都生成不出来。
    > 和普通运行时 `assert()` 的核心区别：
    - `assert(cond)`：运行时才检查，程序跑起来才会崩；release 版本可以直接被关掉。
    - `static_assert`：**编译阶段就拦截错误，还没生成 exe 就报错。*/
    static_assert(I < std::tuple_size_v<tuple_t>, "Layer not found in CoordinateSystem");

    // 获取第I个缓冲区对应的坐标系标签
    //拿`std::tuple_element_t<I>`取出第 I 个缓冲区的类型，通过`buffer_coord_of`元 trait 提取这个缓冲区绑定的坐标系标签。
    using buffer_coord =
        typename buffer_coord_of<std::remove_cvref_t<std::tuple_element_t<I, tuple_t>>>::type;

    // 匹配到目标帧，返回当前tuple元素，匹配`Layer == buffer_coord`，就执行`return std::get<I>(system)`返回缓冲区引用。
    if constexpr (std::is_same_v<Layer, buffer_coord>) {
        return std::get<I>(std::forward<System>(system));
    }
    // 未匹配，递归查找下一个tuple下标
    else {
        return buffer_of<Layer, System, I + 1>(std::forward<System>(system));
    }
}

// 提取整个CoordinateSystem所有存储的坐标系标签元组
template <typename System>
struct coordinate_of;
template <typename... Buffers>
struct coordinate_of<std::tuple<Buffers...>> {
    using type = std::tuple<typename buffer_coord_of<std::remove_cvref_t<Buffers>>::type...>;
};
template <typename System>
using coordinate_of_t = typename coordinate_of<std::remove_cvref_t<System>>::type;

/**
 * @brief for_each_coordinate_buffer 编译期展开tuple，遍历全部缓冲区执行回调fn
 * std::apply + 参数包折叠，遍历所有帧时序缓存
 整体：std::apply + 折叠表达式( ... ) + std::type_identity + 类型萃取buffer_coord_of。
 */
template <typename System, typename Fn>
/*System&& system：万能引用。system实际是一个std::tuple<BufA,BufB,BufC,...>，tuple 里面存放全部坐标变换缓冲区实例。
Fn&& fn：回调函数，就是你上一段代码里面那个 lambda。
constexpr：编译期可执行；当然也完全可以运行时调用。*/
constexpr void for_each_coordinate_buffer(System&& system, Fn&& fn) {
    //std::apply(tuple)的功能：把 tuple 的所有元素拆成一包参数包传给 lambda。
    /*假设：
    using CoordSystem = std::tuple<BufferIMU, BufferGimbal, BufferChassis>;
    CoordSystem system;
    调用std::apply(lambda, system)之后，lambda 里面：
    xs... 就展开成：BufferIMU& xs0, BufferGimbal& xs1, BufferChassis& xs2。*/
    std::apply(
        //auto&&... xs 是参数包，把 tuple 每一个元素全部接进来。
        [&](auto&&... xs) {
            // 回调参数：帧类型标识 + 缓冲区引用
            /*(expr , ...) 是 C++17 逗号折叠表达式。
            对参数包里每一个xs，依次执行：fn(arg1, arg2)。
            等价循环逻辑（伪代码）：
            for(auto& xs : tuple里面每一个buffer)
            {
                fn(类型标签, xs);
            }
            ⚠️ 这不是运行时循环，编译期展开，tuple 有多少个 buffer，编译就生成多少份 fn 调用代码。*/
            (
            /*std::type_identity<T>{} 构造一个空的、零大小的编译期标签对象。
            它唯一作用：把类型 T 当作一个函数参数传给回调 fn。
            运行时这个对象大小为 0，不会产生任何内存开销。
            回到你上一段代码回调内部：
            [&](auto frame_tag, const auto& buffer) {
                using Descendant = typename decltype(frame_tag)::type;
                // Descendant就拿到 GimbalFrame / ChassisFrame
            }
            decltype(frame_tag) = std::type_identity<GimbalFrame>；
            decltype(frame_tag)::type取出里面的内部别名GimbalFrame。
            关键点：类型只能编译期传递，不能直接当作运行时参数；于是用type_identity包一层，把 “类型” 包装成一个可以传进函数的空对象。*/
            fn(std::type_identity<
                //decltype(xs)会带上引用、const；remove_cvref_t剥掉const & &&，拿到 buffer 的原始类型，例如BufferGimbal。
                /*buffer_coord_of是项目自定义的 type‑trait（元函数）。
                输入：buffer 的类型（例如BufferGimbal）
                输出：这个 buffer 对应的子坐标系标签类型（就是前面代码的Descendant）。
                示例：
                // buffer_coord_of<BufferGimbal>::type → GimbalFrame;
                // buffer_coord_of<BufferChassis>::type → ChassisFrame;
                这个GimbalFrame、ChassisFrame是空的标签结构体，编译期元信息，没有运行时成员，里面存静态常量frame_id、using ancestor = xxx。*/
                typename buffer_coord_of<std::remove_cvref_t<decltype(xs)>>::type>{},
                //完美转发，保持原来的左值 / 右值、const 属性，把 buffer 引用传给回调。
                std::forward<decltype(xs)>(xs)
            ),
             ...
            );
        },
        std::forward<System>(system));
}

//================================================================================
// 对外业务API：更新时序缓冲区、递归查询变换矩阵
//================================================================================
/**
 * @brief update：向指定帧的时序缓冲区推入新的完整SE3变换
 * @param system 全局坐标系容器
 * @param transform EdgeTransform<Layer> 父→子帧SE3变换
 * @param ns 变换对应时间戳（纳秒）
 */
template <non_root_frame Layer>
constexpr void update(
    CoordinateSystem& system, const EdgeTransform<Layer>& transform, timestamp_ns_t ns) noexcept {
    buffer_of<Layer>(system).push(ns, transform);
}

/**
 * @brief update_rotate_only：仅推入欧拉角旋转（平移为0），轻量化更新
 * 适用于云台纯旋转无位移场景，节省计算
 */
template <non_root_frame Layer>
constexpr void update_rotate_only(
    CoordinateSystem& system, math_fuxk::Ros2EulerRotd rot, timestamp_ns_t ns) noexcept {
    buffer_of<Layer>(system).push_rotate_only(ns, rot);
}

// ====================== 递归查询核心 lookup ======================
/**
 * @brief lookup 内部递归实现：向上拼接变换链，得到 T<Target, Source>
 * 数学逻辑：
 * Source 向上遍历父链，每一层边变换 T<Ancestor, Current>
 * 复合：T_Ancestor_Current * T_Current_Source = T_Ancestor_Source
 * 递归直到Current == Target，返回复合变换
 * @param acc 累积变换：T<CurrentFrame, Source>
 * @param ns 查询时刻时间戳，缓冲区内部插值
 * @return expected<变换矩阵, 错误字符串>，无抛异常
 */
template <frame Target, frame Source, frame CurrentFrame>
constexpr auto lookup(
    const CoordinateSystem& system, TransformMatrixd<CurrentFrame, Source> acc,
    timestamp_ns_t ns) noexcept -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    // 编译期强校验：Source必须是Target后代，否则直接编译失败，杜绝非法跨树查询
    static_assert(is_descendant_of<Source, Target>(), "Source must be a descendant of Target");
    // 递归终止：当前帧等于目标帧，返回累积复合变换
    if constexpr (std::is_same_v<CurrentFrame, Target>) {
        return acc;
    }
    // 遍历到根帧仍未命中，返回错误信息
    else if constexpr (root_frame<CurrentFrame>) {
        return std::unexpected(
            fmt::format(
                "lookup({} -> {}, {}ns): traversed past root without finding target, "
                "stopped at root frame '{}'",
                Source::frame_id, Target::frame_id, ns, CurrentFrame::frame_id));
    }
    // 正常向上一层父帧递归
    else {
        // 获取当前帧时序缓冲区，线性插值对应时刻变换
        const auto& buffer = buffer_of<CurrentFrame>(system);
        auto result        = buffer.lookup(ns, interpolate);
        // 缓冲区插值失败（无足够样本），返回错误
        if (!result) {
            return std::unexpected(
                fmt::format(
                    "lookup({} -> {}, {}ns): failed at edge {} -> {}: {}", Source::frame_id,
                    Target::frame_id, ns, CurrentFrame::ancestor::frame_id, CurrentFrame::frame_id,
                    result.error()));
        }
        // 复合变换：父→当前 * 当前→源 = 父→源
        return lookup<Target, Source>(system, result->value * acc, ns);
    }
}

/**
 * @brief lookup 对外顶层API：查询 T<Target, Source>
 * @param system 全局坐标系存储
 * @param ns 查询时间戳
 * @return std::expected<TransformMatrixd<Target, Source>, std::string>
 * 示例：lookup<odom, camera_optical>(sys, t) 获取相机光学帧转odom的变换矩阵
 */
template <frame Target, frame Source>
constexpr auto lookup(const CoordinateSystem& system, timestamp_ns_t ns) noexcept
    -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    // 初始累积变换为Source自身单位矩阵 I<Source>
    return lookup<Target, Source>(system, I<Source>(), ns);
}

// ====================== lookup_clamped 夹紧插值查询 ======================
/**
 * @brief lookup_clamped 内部递归：夹紧插值（不做外推）
 * 若查询时间超出缓冲区最早/最晚样本，直接使用边界样本，不预测未来/过去
 * 对标ROS2 TF2 TimePointZero逻辑，工业机器人常用，避免外推漂移
 */
template <frame Target, frame Source, frame CurrentFrame>
constexpr auto lookup_clamped(
    const CoordinateSystem& system, TransformMatrixd<CurrentFrame, Source> acc,
    timestamp_ns_t ns) noexcept -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    static_assert(is_descendant_of<Source, Target>(), "Source must be a descendant of Target");
    if constexpr (std::is_same_v<CurrentFrame, Target>) {
        return acc;
    } else if constexpr (root_frame<CurrentFrame>) {
        return std::unexpected(
            fmt::format(
                "lookup_clamped({} -> {}, {}ns): traversed past root without finding target, "
                "stopped at root frame '{}'",
                Source::frame_id, Target::frame_id, ns, CurrentFrame::frame_id));
    } else {
        const auto& buffer = buffer_of<CurrentFrame>(system);
        // 区别：使用clamped夹紧插值策略
        auto result        = buffer.lookup(ns, clamped);
        if (!result) {
            return std::unexpected(
                fmt::format(
                    "lookup_clamped({} -> {}, {}ns): failed at edge {} -> {}: {}", Source::frame_id,
                    Target::frame_id, ns, CurrentFrame::ancestor::frame_id, CurrentFrame::frame_id,
                    result.error()));
        }
        return lookup_clamped<Target, Source>(system, result->value * acc, ns);
    }
}

/**
 * @brief lookup_clamped 顶层对外API，夹紧插值查询变换矩阵
 */
template <frame Target, frame Source>
constexpr auto lookup_clamped(const CoordinateSystem& system, timestamp_ns_t ns) noexcept
    -> std::expected<TransformMatrixd<Target, Source>, std::string> {
    return lookup_clamped<Target, Source, Source>(system, I<Source>(), ns);
}

} // namespace fast_tf