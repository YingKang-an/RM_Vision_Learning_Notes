// ============================================================
// 本节要学什么？
// 1. 继承的基本语法与三种继承方式：public / protected / private
// 2. 访问控制权限：public / protected / private 在继承中的变化
// 3. 子类访问父类成员的规则（超级详细）
// 4. 菱形继承（钻石继承）的问题：数据冗余、二义性
// 5. 虚继承（virtual inheritance）：解决菱形继承问题
// 6. 继承中的构造函数调用顺序、析构函数调用顺序
//
// 为什么要学？
// - 继承是面向对象三大特性之一，实现代码复用与扩展
// - 访问控制是封装的核心，决定了成员的可见性与安全性
// - 菱形继承是多继承中的经典问题，虚继承是标准解决方案
// - 工程中大量用于框架设计、模块扩展、接口实现
//
// 核心重点（必须记住）：
// 1. 继承方式决定了父类成员在子类中的“可见性降级”
// 2. 子类永远无法访问父类的 private 成员
// 3. 菱形继承导致数据冗余与二义性，必须用虚继承解决
// 4. 虚继承让公共基类在子类中只存在一份实例
// 5. 构造顺序：先父类，再子类；析构顺序：先子类，再父类
// ============================================================

#include <iostream>

// --- 第一部分：访问控制与三种继承方式 ---
// --- 基类：定义三种访问权限 ---
class Base {
public:
  int m_public;    /* < 公有成员：类内、类外、子类都可访问 */
protected:
  int m_protected; /* < 保护成员：类内、子类可访问，类外不可访问 */
private:
  int m_private;   /* < 私有成员：只有类内自己可访问，子类、类外都不可访问 */

public:
  Base() : m_public(1), m_protected(2), m_private(3) {}

  void showBase() {
    // 类内：三种权限都可访问
    std::cout << "Base 类内访问: " 
              << m_public << ", " 
              << m_protected << ", " 
              << m_private << std::endl;
  }
};

// --- 子类1：公有继承 (public inheritance) ---
// 语法：class DerivedPublic : public Base
class DerivedPublic : public Base {
public:
  void access() {
    m_public = 10;    /* < 父类 public → 子类 public，可访问 */
    m_protected = 20; /* < 父类 protected → 子类 protected，可访问 */
    // m_private = 30; /* < 错误！父类 private，子类永远不可访问 */
    // - 解释：
    // - 1. 公有继承是最常用、最符合“is-a”关系的继承方式。
    // - 2. 父类 public → 子类 public。
    // - 3. 父类 protected → 子类 protected。
    // - 4. 父类 private → 子类不可见。
  }
};

// --- 子类2：保护继承 (protected inheritance) ---
// 语法：class DerivedProtected : protected Base
class DerivedProtected : protected Base {
public:
  void access() {
    m_public = 10;    /* < 父类 public → 子类 protected，可访问 */
    m_protected = 20; /* < 父类 protected → 子类 protected，可访问 */
    // m_private = 30; /* < 错误！父类 private，子类不可访问 */
    // - 解释：
    // - 1. 保护继承将父类的公有成员降级为保护成员。
    // - 2. 父类 public → 子类 protected。
    // - 3. 父类 protected → 子类 protected。
    // - 4. 父类 private → 子类不可见。
    // - 5. 外部代码无法通过子类对象访问父类的任何成员。
  }
};

// --- 子类3：私有继承 (private inheritance) ---
// 语法：class DerivedPrivate : private Base
class DerivedPrivate : private Base {
public:
  void access() {
    m_public = 10;    /* < 父类 public → 子类 private，可访问 */
    m_protected = 20; /* < 父类 protected → 子类 private，可访问 */
    // m_private = 30; /* < 错误！父类 private，子类不可访问 */
    // - 解释：
    // - 1. 私有继承将父类的所有可见成员都降级为私有成员。
    // - 2. 父类 public → 子类 private。
    // - 3. 父类 protected → 子类 private。
    // - 4. 父类 private → 子类不可见。
    // - 5. 子类的子类将无法访问任何父类成员（实现了“实现继承”而非“接口继承”）。
  }
};

// --- 测试访问控制 ---
void testAccessControl() {
  std::cout << "\n--- 测试访问控制 ---" << std::endl;

  Base b;
  b.m_public = 100; /* < 类外：只能访问 public 成员 */
  // b.m_protected = 200; /* < 错误！类外不可访问 protected */
  // b.m_private = 300;   /* < 错误！类外不可访问 private */
  b.showBase();

  DerivedPublic dp;
  dp.m_public = 100; /* < 类外：公有继承，public 成员仍可访问 */
  // dp.m_protected = 200; /* < 错误！类外不可访问 protected */

  DerivedProtected dprot;
  // dprot.m_public = 100; /* < 错误！保护继承，public 已降级为 protected */

  DerivedPrivate dpriv;
  // dpriv.m_public = 100; /* < 错误！私有继承，public 已降级为 private */
}

// --- 第二部分：菱形继承（钻石继承）问题 ---
// --- 顶层基类 ---
class TopBase {
public:
  int m_data;
  TopBase() : m_data(100) {
    std::cout << "TopBase 构造" << std::endl;
  }
};

// --- 中间层子类A：普通继承 ---
class DerivedA : public TopBase {
public:
  DerivedA() {
    std::cout << "DerivedA 构造" << std::endl;
  }
};

// --- 中间层子类B：普通继承 ---
class DerivedB : public TopBase {
public:
  DerivedB() {
    std::cout << "DerivedB 构造" << std::endl;
  }
};

// --- 最终子类：同时继承 A 和 B ---
// 问题：DerivedA 和 DerivedB 都继承了 TopBase，导致 FinalDerived 中有两份 TopBase 数据
class FinalDerived : public DerivedA, public DerivedB {
public:
  FinalDerived() {
    std::cout << "FinalDerived 构造" << std::endl;
  }

  void testAmbiguity() {
    // m_data = 200; /* < 错误！二义性，不知道是 DerivedA::m_data 还是 DerivedB::m_data */
    // - 解释：
    // - 1. 这就是“菱形继承”问题。
    // - 2. FinalDerived 对象内部包含了两个 TopBase 子对象。
    // - 3. 直接访问 m_data 会产生歧义，编译报错。
    // - 4. 解决方案：使用 虚继承 (virtual inheritance)。

    // 必须指定作用域才能访问
    DerivedA::m_data = 200;
    DerivedB::m_data = 300;
    std::cout << "DerivedA::m_data = " << DerivedA::m_data << std::endl;
    std::cout << "DerivedB::m_data = " << DerivedB::m_data << std::endl;
    // - 解释：
    // - 1. 此时 DerivedA::m_data 和 DerivedB::m_data 是两个独立的变量。
    // - 2. 这就造成了“数据冗余”，浪费内存，且不符合逻辑。
  }
};

// --- 第三部分：虚继承（解决菱形继承问题）---
// --- 顶层基类 ---
class VirtualTopBase {
public:
  int m_data;
  VirtualTopBase() : m_data(100) {
    std::cout << "VirtualTopBase 构造" << std::endl;
  }
};

// --- 中间层子类A：虚继承 ---
// 语法：class DerivedVA : virtual public VirtualTopBase
class DerivedVA : virtual public VirtualTopBase {
public:
  DerivedVA() {
    std::cout << "DerivedVA 构造" << std::endl;
  }
};

// --- 中间层子类B：虚继承 ---
class DerivedVB : virtual public VirtualTopBase {
public:
  DerivedVB() {
    std::cout << "DerivedVB 构造" << std::endl;
  }
};

// --- 最终子类：同时继承 VA 和 VB ---
// 解决：因为 A 和 B 都是虚继承，所以 FinalVD 中只有一份 VirtualTopBase 实例
class FinalVD : public DerivedVA, public DerivedVB {
public:
  FinalVD() {
    std::cout << "FinalVD 构造" << std::endl;
  }

  void testNoAmbiguity() {
    m_data = 200; /* < 正确！无歧义，只有一份 m_data */
    // - 解释：
    // - 1. 虚继承的核心作用：让公共基类在最终子类中只存在一份实例。
    // - 2. 编译器会通过“虚基类表”和“虚基类指针”来实现共享。
    // - 3. 构造顺序：最先构造虚基类，然后是普通父类，最后是子类。
    std::cout << "m_data = " << m_data << std::endl;
    std::cout << "DerivedVA::m_data = " << DerivedVA::m_data << std::endl;
    std::cout << "DerivedVB::m_data = " << DerivedVB::m_data << std::endl;
    // - 解释：
    // - 1. 此时 DerivedVA::m_data、DerivedVB::m_data 和 m_data 指向同一块内存。
    // - 2. 彻底解决了数据冗余和二义性问题。
  }
};

// --- 测试菱形继承与虚继承 ---
void testDiamondInheritance() {
  std::cout << "\n--- 测试普通菱形继承（有问题）---" << std::endl;
  FinalDerived fd;
  fd.testAmbiguity();

  std::cout << "\n--- 测试虚继承（解决问题）---" << std::endl;
  FinalVD fvd;
  fvd.testNoAmbiguity();
}

// --- 第四部分：构造与析构顺序 ---
class Base1 {
public:
  Base1() { std::cout << "Base1 构造" << std::endl; }
  ~Base1() { std::cout << "Base1 析构" << std::endl; }
};

class Base2 {
public:
  Base2() { std::cout << "Base2 构造" << std::endl; }
  ~Base2() { std::cout << "Base2 析构" << std::endl; }
};

class DerivedOrder : public Base1, public Base2 {
public:
  DerivedOrder() { std::cout << "DerivedOrder 构造" << std::endl; }
  ~DerivedOrder() { std::cout << "DerivedOrder 析构" << std::endl; }
  // - 解释：
  // - 1. 构造顺序：严格按照继承声明的顺序（Base1 → Base2 → DerivedOrder）。
  // - 2. 析构顺序：与构造顺序完全相反（DerivedOrder → Base2 → Base1）。
  // - 3. 虚继承时，虚基类会最先构造。
};

void testOrder() {
  std::cout << "\n--- 测试构造与析构顺序 ---" << std::endl;
  DerivedOrder d;
}

int main() {
  testAccessControl();
  testDiamondInheritance();
  testOrder();
  return 0;
}

