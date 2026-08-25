// ===========================================================================
// 练习4：CRTP（奇异递归模板模式）
// 目标：实现 Animal<Derived> 基类，零虚函数开销静态多态
// 学习要点：CRTP 模式、静态多态、与虚函数对比
// ===========================================================================

#include <iostream>    // std::cout, std::endl
#include <string>      // std::string
#include <string_view> // std::string_view（轻量级字符串视图，避免拷贝）

// ---------------------------------------------------------------------------
// 1. CRTP 基类：把派生类作为模板参数传给自己
// 优势：编译期确定调用，无虚表开销，且能复用通用接口
// ---------------------------------------------------------------------------

/**
 * CRTP 基类模板：Animal<Derived>
 * 
 * 关键语法：template <typename Derived> class Animal
 * 派生类继承时：class Dog : public Animal<Dog>
 *               ↑ 把自己作为模板参数传给基类
 * 
 * 为什么叫"奇异递归"？
 * 1. 递归：派生类把自己作为模板参数传给基类
 * 2. 奇异：这种自我引用看起来有点奇怪
 * 
 * 核心优势：
 * - 零虚函数开销（无 vtable，无运行时类型识别）
 * - 编译期多态（所有调用在编译期确定）
 * - 代码复用（基类提供通用接口）
 * - 类型安全（派生类类型在编译期已知）
 * 
 * 典型应用场景：
 * - 模板方法模式（Template Method Pattern）
 * - 静态接口（Static Interface）
 * - 对象池（Object Pool）
 * - 性能敏感场景（游戏、嵌入式）
 */
template <typename Derived>
class Animal {
public:
    // -----------------------------------------------------------------------
    // 对外统一接口：转发到派生类的 _impl 方法
    // -----------------------------------------------------------------------
    
    /**
     * make_sound() - 让动物发出声音
     * 
     * 实现原理：
     * 1. static_cast 将 this 转换为 Derived* 类型
     * 2. 调用派生类的 make_sound_impl() 方法
     * 3. 这一切都在编译期确定，无运行时开销
     * 
     * 为什么用 static_cast 而不是 dynamic_cast？
     * - static_cast：编译期转换，无运行时开销 ✅
     * - dynamic_cast：运行时检查，有开销 ❌
     * 
     * 安全性保证：
     * - 只有正确继承 Animal<Derived> 的类才能实例化
     * - 编译器会检查类型兼容性
     */
    void make_sound() const {
        // static_cast 把 this 转为派生类指针，编译期绑定
        // const auto& derived = static_cast<const Derived&>(*this);
        // 这行代码在编译期展开，生成的汇编直接调用 Derived::make_sound_impl()
        const auto& derived = static_cast<const Derived&>(*this);
        derived.make_sound_impl();
    }

    // -----------------------------------------------------------------------
    // 模板方法模式：基类提供算法骨架，派生类提供具体步骤
    // -----------------------------------------------------------------------
    
    /**
     * describe() - 返回动物的描述信息
     * 
     * 这是"模板方法模式"（Template Method Pattern）的典型应用：
     * 1. 基类提供算法骨架（describe 方法本身）
     * 2. 派生类提供具体步骤（species_impl 和 sound_text_impl）
     * 3. 基类控制整体流程（组合字符串）
     * 
     * [[nodiscard]]：C++17 属性，警告调用者不要忽略返回值
     */
    [[nodiscard]] std::string describe() const {
        const auto& derived = static_cast<const Derived&>(*this);
        // 调用派生类的 species_impl() 和 sound_text_impl()
        // std::string 构造：拼接字符串
        return "I am a " + std::string(derived.species_impl())
             + " and I say: " + std::string(derived.sound_text_impl());
    }

protected:
    // -----------------------------------------------------------------------
    // 默认实现（protected：派生类可访问，外部不可访问）
    // 派生类可以选择覆盖这些方法，也可以使用默认实现
    // -----------------------------------------------------------------------
    
    /**
     * 这些是"钩子方法"（Hook Methods）
     * 派生类可以覆盖它们来提供具体行为
     * 
     * 使用 std::string_view 的好处：
     * - 轻量级字符串视图（不拷贝字符串）
     * - 只保存指针和长度
     * - 零拷贝，高性能
     */
    std::string_view species_impl() const { return "unknown animal"; }
    std::string_view sound_text_impl() const { return "..."; }
};

// ---------------------------------------------------------------------------
// 2. 派生类：Dog，CRTP 传入自身类型
// ---------------------------------------------------------------------------

/**
 * Dog 类：继承自 Animal<Dog>
 * 
 * 关键点：class Dog : public Animal<Dog>
 *          ↑ 把自己作为模板参数传给基类
 * 
 * 这种写法让基类知道派生类的具体类型
 * 从而可以在编译期调用派生类的方法
 */
class Dog : public Animal<Dog> {
public:
    // -----------------------------------------------------------------------
    // 实现细节：CRTP 基类会转发到这里
    // -----------------------------------------------------------------------
    
    /**
     * make_sound_impl() - 狗叫的具体实现
     * 
     * 这个方法会被基类的 make_sound() 调用
     * 命名约定：_impl 后缀表示"实现细节"
     * 
     * 可见性：public
     * - 因为基类需要调用它（通过 static_cast）
     * - 虽然基类是友元，但 public 更直观
     */
    void make_sound_impl() const {
        std::cout << "Woof! Woof!\n";
    }

    /**
     * friend class Animal<Dog>;
     * 
     * 为什么要友元？
     * 因为基类需要访问派生类的 private 成员（species_impl 等）
     * 通过友元声明，基类可以访问派生类的私有方法
     * 
     * 注意：基类模板参数是 Dog，所以必须是 Animal<Dog>
     */
    friend class Animal<Dog>;

private:
    /**
     * private 方法：只有基类（友元）能访问
     * 外部代码不能直接调用，必须通过基类接口
     * 
     * 这样设计的优点：
     * 1. 强制使用基类统一接口
     * 2. 派生类实现细节被隐藏
     * 3. 基类可以在调用前后添加额外逻辑（如日志、校验）
     */
    std::string_view species_impl() const { return "dog"; }
    std::string_view sound_text_impl() const { return "Woof Woof"; }
};

// ---------------------------------------------------------------------------
// 3. 派生类：Cat
// ---------------------------------------------------------------------------

/**
 * Cat 类：继承自 Animal<Cat>
 * 
 * 和 Dog 结构相同，但实现不同
 * 展示了 CRTP 如何实现"静态多态"
 */
class Cat : public Animal<Cat> {
public:
    void make_sound_impl() const {
        std::cout << "Meow~\n";
    }

    friend class Animal<Cat>;

private:
    std::string_view species_impl() const { return "cat"; }
    std::string_view sound_text_impl() const { return "Meow"; }
};

// ---------------------------------------------------------------------------
// 4. 通用函数：用模板约束接受任意 Animal<Derived>
// 静态多态：无需 virtual，编译期即确定调用哪个 make_sound_impl
// ---------------------------------------------------------------------------

/**
 * interact() - 与任何动物交互
 * 
 * 这是"静态多态"的体现：
 * 1. 函数是模板，接受任何 Animal<Derived> 类型
 * 2. 编译器为 Dog 和 Cat 分别生成不同版本的函数
 * 3. 每个版本直接调用对应的派生类方法（无虚表开销）
 * 
 * 对比虚函数方式：
 * - 虚函数：void interact(const VirtualAnimal& animal) -> 运行时查找 vtable
 * - CRTP：template <typename Derived> void interact(const Animal<Derived>& animal) -> 编译期确定
 * 
 * 性能差异：
 * - 虚函数：每次调用需要解引用 vptr，查找 vtable，间接跳转
 * - CRTP：直接函数调用（可能内联），零开销
 * 
 * 代价：
 * - CRTP 是模板，会生成多份代码（代码膨胀）
 * - 虚函数只需要一份代码（运行时多态）
 */
template <typename Derived>
void interact(const Animal<Derived>& animal) {
    // 调用基类接口，实际上会转发到派生类实现
    animal.make_sound();
    std::cout << animal.describe() << "\n";
}

// ---------------------------------------------------------------------------
// 5. 对比：传统虚函数方式（运行期多态，有虚表开销）
// ---------------------------------------------------------------------------

/**
 * VirtualAnimal - 传统虚函数基类
 * 
 * 特点：
 * 1. 使用 virtual 关键字声明虚函数
 * 2. 编译器生成 vtable（虚函数表）
 * 3. 每个对象有 vptr（虚函数指针）
 * 4. 运行时通过 vptr 查找 vtable 调用函数
 * 
 * 开销：
 * - 内存：每个对象多一个 vptr（8 字节）
 * - 时间：每次调用需要间接寻址（比直接调用慢）
 * - 无法内联（因为调用在运行时确定）
 * 
 * 优点：
 * - 可以存储不同类型的对象（多态容器）
 * - 可以在运行时动态切换行为
 */
class VirtualAnimal {
public:
    // 虚析构函数：确保派生类正确析构
    virtual ~VirtualAnimal() = default;
    
    // 纯虚函数：派生类必须实现
    virtual void make_sound() const = 0;
    virtual std::string describe() const = 0;
};

// ---------------------------------------------------------------------------
// 6. main 函数：测试所有功能
// ---------------------------------------------------------------------------

int main() {
    // ===== 创建派生类对象 =====
    Dog dog;
    Cat cat;

    // ===== 测试1：CRTP 静态多态调用 =====
    std::cout << "=== Dog ===\n";
    // interact<Dog>(dog); 编译器自动推导模板参数
    // 编译器实例化：void interact(const Animal<Dog>& animal)
    // 内部调用：Animal<Dog>::make_sound() -> Dog::make_sound_impl()
    interact(dog);
    // 输出：
    // Woof! Woof!
    // I am a dog and I say: Woof Woof

    std::cout << "\n=== Cat ===\n";
    interact(cat);
    // 输出：
    // Meow~
    // I am a cat and I say: Meow

    // ===== 测试2：直接调用 =====
    std::cout << "\n=== Direct ===\n";
    // 直接调用派生类方法（但通常通过基类接口）
    dog.make_sound();  // Woof! Woof!
    cat.make_sound();  // Meow~
    // 注意：这里调用的其实是基类的 make_sound()，但转发到了派生类
    
    std::cout << "\nCRTP 演示完成\n";
    return 0;
}