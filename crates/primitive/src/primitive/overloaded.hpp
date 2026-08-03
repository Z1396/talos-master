#pragma once
namespace {

/// std::visit helper for variant exhaustiveness.
template <typename... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
/*2. 逐段拆解 overloaded(Ts...) -> overloaded<Ts...>
左边：overloaded(Ts...)
匹配构造函数的入参形式：
当你调用 overloaded{ a, b, c }，参数是一堆 Ts... 类型。
箭头 ->
固定语法分隔符，左边是构造实参，右边是要实例化的模板类型。
右边：overloaded<Ts...>
告诉编译器：
用刚才从构造参数推出来的类型包 Ts...，去实例化 overloaded 模板。
整体逻辑：
如果构造 overloaded 时传入一堆 Ts 类型参数，那这个对象的类型就是 overloaded<Ts...>
3. 完整匹配流程（对应你的代码）
overloaded{
    [](DirectConfig){},
    [](DaedalusConfig){}
}
编译器看到构造传入 2 个 lambda，推导出 Ts... = Lambda1, Lambda2
套用指引 -> overloaded<Ts...>
确定类型 = overloaded<Lambda1, Lambda2>
自动实例化，不用你手动写 <...>
4. 为什么必须单独写这一行，构造函数不行？
普通构造函数无法反向推导类模板参数。
模板类的构造函数只能推导函数模板参数，不能推导类本身的模板参数，所以 C++17 专门加了 CTAD 语法来补足这个能力。*/
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

} // namespace
