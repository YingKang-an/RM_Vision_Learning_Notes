// ============================================================
// 本节要学什么？
// 1. 委托构造函数（Delegating Constructor）：C++11 新特性
// 2. 委托构造的语法、作用、使用场景
// 3. 委托构造的调用顺序、注意事项（避免循环委托）
// 4. 继承构造函数（Inheriting Constructor）：C++11 新特性
// 5. 继承构造的语法、作用、使用场景
// 6. 构造函数复用的三种方式：普通复用 vs 委托构造 vs 继承构造
//
// 为什么要学？
// - 构造函数是对象初始化的核心，代码复用能减少冗余、降低出错率
// - 委托构造让一个构造调用另一个构造，避免重复初始化代码
// - 继承构造让子类直接复用父类的所有构造函数，大幅简化代码
// - 这两个特性是现代 C++ 构造函数设计的必备技能，工程中大量使用
//
// 核心重点（必须记住）：
// 1. 委托构造：构造函数 -> 初始化列表 -> 调用另一个构造函数
// 2. 委托构造不能同时出现普通初始化列表和委托构造，只能二选一
// 3. 禁止循环委托（A 委托 B，B 委托 A），会导致编译错误或运行时崩溃
// 4. 继承构造：using Base::Base; 让子类直接继承父类所有构造函数
// 5. 继承构造不会继承默认、拷贝、移动构造函数，需手动处理
// ============================================================

#include <iostream>
#include <string>

// --- 第一部分：委托构造函数（C++11）---
// --- 演示类：使用委托构造实现构造函数复用 ---
class MyClass {
private:
  int m_a;
  double m_b;
  std::string m_c;

  // --- 核心构造函数：负责真正的初始化 ---
  // 作用：被其他构造函数委托，集中处理所有成员初始化
  MyClass(int a, double b, const std::string& c) 
    : m_a(a), m_b(b), m_c(c) {
    std::cout << "核心构造函数: " << m_a << ", " << m_b << ", " << m_c << std::endl;
  }
  // - 解释：
  // - 1. 这是一个私有构造函数，专门用于被委托，不对外暴露。
  // - 2. 集中处理所有成员的初始化，避免代码冗余。

public:
  // --- 1. 委托构造：无参构造委托核心构造 ---
  // 语法：构造函数() : 委托构造函数(参数) {}
  MyClass() : MyClass(0, 0.0, "default") {
    std::cout << "无参构造（委托核心构造）" << std::endl;
  }
  // - 解释：
  // - 1. 无参构造通过初始化列表，委托给核心构造函数。
  // - 2. 执行顺序：先执行核心构造，再执行当前构造的函数体。
  // - 3. 好处：无需重复写 m_a(0), m_b(0.0), m_c("default")。

  // --- 2. 委托构造：单参构造委托核心构造 ---
  MyClass(int a) : MyClass(a, 0.0, "default") {
    std::cout << "单参构造（int a）（委托核心构造）" << std::endl;
  }

  // --- 3. 委托构造：双参构造委托核心构造 ---
  MyClass(int a, double b) : MyClass(a, b, "default") {
    std::cout << "双参构造（int a, double b）（委托核心构造）" << std::endl;
  }

  // --- 4. 普通构造：三参构造（对外暴露的接口）---
  MyClass(int a, double b, const char* c) : MyClass(a, b, std::string(c)) {
    std::cout << "三参构造（const char* c）（委托核心构造）" << std::endl;
  }

  // --- 错误演示：循环委托（绝对禁止！）---
  // MyClass(char c) : MyClass(int(c)) {}
  // MyClass(int a) : MyClass(char(a)) {}
  // - 解释：
  // - 1. 循环委托会导致无限递归调用，编译可能不报错，但运行时会栈溢出崩溃。
  // - 2. 编译器无法检测所有循环委托，必须自己避免。

  // --- 错误演示：委托构造 + 普通初始化列表（禁止！）---
  // MyClass(int a, int b) : MyClass(a), m_b(b) {}
  // - 解释：
  // - 1. 委托构造的类，初始化列表只能有一个委托构造，不能同时初始化其他成员。
  // - 2. 所有成员初始化必须在被委托的构造函数中完成。
};

// --- 测试委托构造 ---
void testDelegatingConstructor() {
  std::cout << "\n--- 测试委托构造函数 ---" << std::endl;

  MyClass obj1;                  /* < 无参构造 → 委托核心构造 */
  MyClass obj2(10);              /* < 单参构造 → 委托核心构造 */
  MyClass obj3(20, 3.14);        /* < 双参构造 → 委托核心构造 */
  MyClass obj4(30, 6.28, "test");/* < 三参构造 → 委托核心构造 */
}

// --- 第二部分：继承构造函数（C++11）---
// --- 父类：定义多个构造函数 ---
class Base {
public:
  int m_x;

  // --- 父类构造函数 ---
  Base() : m_x(0) {
    std::cout << "Base 无参构造" << std::endl;
  }

  Base(int x) : m_x(x) {
    std::cout << "Base 单参构造: " << m_x << std::endl;
  }

  Base(int x, double y) : m_x(x) {
    std::cout << "Base 双参构造: " << m_x << ", " << y << std::endl;
  }

  Base(const std::string& s) : m_x(s.size()) {
    std::cout << "Base 字符串构造: " << m_x << std::endl;
  }
};

// --- 子类1：不使用继承构造，手动实现所有构造（冗余代码）---
class DerivedManual : public Base {
public:
  // --- 手动实现所有父类构造函数，代码冗余 ---
  DerivedManual() : Base() {}
  DerivedManual(int x) : Base(x) {}
  DerivedManual(int x, double y) : Base(x, y) {}
  DerivedManual(const std::string& s) : Base(s) {}
  // - 解释：
  // - 1. 子类没有新增成员，却要手动写一遍所有父类构造，代码冗余。
  // - 2. 父类新增构造，子类必须同步修改，维护成本高。
};

// --- 子类2：使用继承构造，一行代码复用父类所有构造（C++11）---
// 语法：using Base::Base;
class DerivedInherit : public Base {
public:
  // --- 继承构造函数：直接复用父类所有构造函数 ---
  using Base::Base;
  // - 解释：
  // - 1. 这一行代码，让 DerivedInherit 自动拥有 Base 类的所有构造函数。
  // - 2. 编译器会自动生成对应的子类构造函数，委托给父类构造。
  // - 3. 父类新增构造，子类自动拥有，无需修改代码。
  // - 4. 注意：不会继承默认、拷贝、移动构造函数，这些需要手动实现。

  // --- 子类新增构造函数（与继承构造不冲突）---
  DerivedInherit(double d) : Base(int(d)) {
    std::cout << "DerivedInherit 新增构造: " << d << std::endl;
  }
};

// --- 测试继承构造 ---
void testInheritingConstructor() {
  std::cout << "\n--- 测试继承构造函数 ---" << std::endl;

  // --- 使用继承构造，直接调用父类的所有构造 ---
  DerivedInherit obj1;                  /* < 继承 Base 无参构造 */
  DerivedInherit obj2(100);             /* < 继承 Base 单参构造 */
  DerivedInherit obj3(200, 3.1415);     /* < 继承 Base 双参构造 */
  DerivedInherit obj4("hello");         /* < 继承 Base 字符串构造 */

  // --- 调用子类新增构造 ---
  DerivedInherit obj5(6.28);
}

// --- 第三部分：构造函数复用方式对比 ---
// 1. 普通复用：提取公共函数，在每个构造中调用（C++98 传统方式）
// 2. 委托构造：构造函数之间互相调用（C++11，推荐，集中初始化）
// 3. 继承构造：子类复用父类构造（C++11，推荐，减少冗余）

int main() {
  testDelegatingConstructor();
  testInheritingConstructor();
  return 0;
}

