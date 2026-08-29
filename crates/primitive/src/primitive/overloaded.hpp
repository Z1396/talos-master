#pragma once
namespace {

/*- 继承所有传入的 lambda：`Ts...` 就是一堆 lambda 类型，多重继承。
- `using Ts::operator()...;`：把每一个 lambda 的 `operator()` 全部引入到子类，形成**重载集合**。
示例：两个 lambda
```
auto l1 = [](int){};
auto l2 = [](std::string){};
// overloaded<decltype(l1), decltype(l2)> 多重继承 l1,l2
// using 把两个 operator() 都拿进来，于是对象同时可以调用 int版本、string版本。
```*/
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
/*`overloaded` 是类模板 `template<typename...Ts> struct overloaded`。
编译器需要知道 `Ts...` 是什么。
**没有指引的情况下：构造函数的参数，不会用来推导类模板的 Ts...**。
编译器不知道 `overloaded<???>`，编译失败。
> 
> 普通函数模板可以推导参数；**类模板本身不会自动从构造实参推导模板参数**，这是 C++ 语法规则。CTAD 指引就是专门补这个缺口。
### 指引语法格式
```
模板声明
类名(参数类型...) -> 类名<模板参数...>;
```
- `overloaded(Ts...)`：**构造函数的形参模式**：传入若干个 Ts 类型的对象。
- `->` 固定符号，CTAD 语法。
- `overloaded<Ts...>`：告诉编译器：当你看到 `overloaded{arg1,arg2,arg3}` 这种构造形式，**推导出来的类模板实例化类型就应该是 `overloaded<Ts...>`**，Ts 就是各个入参的类型。*/
} // namespace
