// ===========================================================================
// 练习4：CRTP（奇异递归模板模式）
// 目标：实现 Animal<Derived> 基类，零虚函数开销静态多态
// 学习要点：CRTP 模式、静态多态、与虚函数对比
// ===========================================================================

#include <iostream>
#include <string>
#include <string_view>

// ---------------------------------------------------------------------------
// 1. CRTP 基类：把派生类作为模板参数传给自己
// 优势：编译期确定调用，无虚表开销，且能复用通用接口
// ---------------------------------------------------------------------------
template <typename Derived>
class Animal {
public:
    // 对外统一接口：转发到派生类的 _impl 方法
    void make_sound() const {
        // static_cast 把 this 转为派生类指针，编译期绑定
        const auto& derived = static_cast<const Derived&>(*this);
        derived.make_sound_impl();
    }

    // 模板方法模式：基类提供算法骨架，派生类提供具体步骤
    [[nodiscard]] std::string describe() const {
        const auto& derived = static_cast<const Derived&>(*this);
        return "I am a " + std::string(derived.species_impl())
             + " and I say: " + std::string(derived.sound_text_impl());
    }

protected:
    // 默认实现，派生类可覆盖
    std::string_view species_impl() const { return "unknown animal"; }
    std::string_view sound_text_impl() const { return "..."; }
};

// ---------------------------------------------------------------------------
// 2. 派生类：Dog，CRTP 传入自身类型
// ---------------------------------------------------------------------------
class Dog : public Animal<Dog> {
public:
    // 实现细节：CRTP 基类会转发到这里
    void make_sound_impl() const {
        std::cout << "Woof! Woof!\n";
    }

    // friend 让基类能访问 private 成员
    friend class Animal<Dog>;

private:
    std::string_view species_impl() const { return "dog"; }
    std::string_view sound_text_impl() const { return "Woof Woof"; }
};

// ---------------------------------------------------------------------------
// 3. 派生类：Cat
// ---------------------------------------------------------------------------
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
template <typename Derived>
void interact(const Animal<Derived>& animal) {
    animal.make_sound();
    std::cout << animal.describe() << "\n";
}

// ---------------------------------------------------------------------------
// 5. 对比：传统虚函数方式（运行期多态，有虚表开销）
// ---------------------------------------------------------------------------
class VirtualAnimal {
public:
    virtual ~VirtualAnimal() = default;
    virtual void make_sound() const = 0;
    virtual std::string describe() const = 0;
};

int main() {
    Dog dog;
    Cat cat;

    // CRTP 静态多态调用
    std::cout << "=== Dog ===\n";
    interact(dog);

    std::cout << "\n=== Cat ===\n";
    interact(cat);

    // 直接调用
    std::cout << "\n=== Direct ===\n";
    dog.make_sound();
    cat.make_sound();

    std::cout << "\nCRTP 演示完成\n";
    return 0;
}
