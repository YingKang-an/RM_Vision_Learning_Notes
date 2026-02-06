// ============================================================
// 本节要学什么？
// 1. const 正确性（const correctness）：C++ 最重要的设计思想之一
// 2. const 成员函数：承诺不修改对象状态，提高代码安全性与可读性
// 3. const 成员函数内部的 this 指针：是指向 const 对象的指针（const T* const this）
// 4. const 对象：只能调用 const 成员函数，不能调用非 const 成员函数
// 5. 非 const 对象：可以调用 const 和非 const 成员函数
// 6. mutable 关键字：突破 const 限制，允许在 const 成员函数中修改特定成员
// 7. 重载：const 成员函数与非 const 成员函数可以同时存在，构成重载
// 8. 返回值 const：防止返回值被修改，提高接口安全性
// 9. 引用与 const：const 引用可以绑定到临时对象，延长生命周期
// 10. 各种组合场景：const 对象 / 非 const 对象 + const 成员 / 非 const 成员 / mutable 成员
//
// 为什么要学？
// - const 正确性是写出安全、健壮、可维护、高效代码的基础
// - 编译器利用 const 信息做优化，const 代码比非 const 代码更快
// - 标准库大量使用 const，不懂 const 无法看懂和使用标准库
// - 面试高频考点：const 成员函数、this 指针、mutable、const 正确性
//
// 核心重点（必须记住）：
// 1. const 成员函数：不能修改成员变量（除 mutable 外），不能调用非 const 成员函数
// 2. const 对象：只能调用 const 成员函数，不能调用非 const 成员函数
// 3. this 指针在 const 成员函数中是：const T* const this（指向 const 对象的常量指针）
// 4. mutable 成员：可以在 const 成员函数中修改，用于缓存、计数、互斥锁等内部状态
// 5. const 与非 const 成员函数可以重载，根据对象常量性选择调用
// 6. const 正确性是“设计思想”，不是“语法糖”，贯穿整个 C++ 设计
// ============================================================

#include <iostream>
#include <string>
#include <mutex>
#include <cmath>  // 修复：sqrt 需要这个头文件
// - 解释：
// - 1. <cmath> 提供 sqrt、sin、cos、pow 等数学函数。
// - 2. 之前报错就是因为没包含它，编译器找不到 sqrt。

// --- 第一部分：const 成员函数与 this 指针 ---
// --- 演示类：包含 const 与非 const 成员函数 ---
class MyClass {
private:
  int m_value;
  std::string m_name;

  // --- mutable 成员：突破 const 限制 ---
  mutable int m_cache;
  mutable std::mutex m_mutex;
  // - 解释 mutable：
  // - 1. mutable 是“可变的”，可以修饰非静态、非引用、非 const 的成员变量。
  // - 2. 被 mutable 修饰的成员，可以在 const 成员函数中被修改。
  // - 3. 用途：缓存计算结果、统计调用次数、多线程互斥锁、内部状态更新等。
  // - 4. 注意：不要滥用 mutable，只用于真正需要“逻辑上不改变对象状态”的内部变量。

public:
  // --- 构造函数 ---
  MyClass(int val, const std::string& name)
    : m_value(val), m_name(name), m_cache(0) {}

  // --- 1. 非 const 成员函数 ---
  void setValue(int val) {
    m_value = val;
    std::cout << "非 const 成员函数 setValue: 修改 m_value = " << m_value << std::endl;
  }

  void setName(const std::string& name) {
    m_name = name;
    std::cout << "非 const 成员函数 setName: 修改 m_name = " << m_name << std::endl;
  }

  // --- 2. const 成员函数 ---
  int getValue() const {
    // m_value = 100; // 错误！const 成员函数不能修改普通成员
    // setValue(100); // 错误！const 成员函数不能调用非 const 成员函数

    // --- this 指针在 const 成员函数中的类型 ---
    // this 的类型是：const MyClass* const this
    // - 解释：
    // - 1. 第一个 const：指向的对象是 const，不能通过 this 修改对象状态。
    // - 2. 第二个 const：this 指针本身是常量，不能修改 this 的指向。
    // - 3. 这就是 const 成员函数不能修改成员的根本原因：this 指向 const 对象。

    std::cout << "const 成员函数 getValue: 读取 m_value = " << m_value << std::endl;
    return m_value;
  }

  const std::string& getName() const {
    std::cout << "const 成员函数 getName: 读取 m_name = " << m_name << std::endl;
    return m_name;
  }

  // --- 3. const 成员函数中修改 mutable 成员 ---
  int getValueWithCache() const {
    if (m_cache == 0) {
      m_cache = m_value * 2;
      std::cout << "const 成员函数 getValueWithCache: 计算并缓存 m_cache = " << m_cache << std::endl;
    } else {
      std::cout << "const 成员函数 getValueWithCache: 使用缓存 m_cache = " << m_cache << std::endl;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache;
  }

  // --- 4. 重载：const 与非 const 成员函数同时存在 ---
  std::string& getName() {
    std::cout << "非 const 成员函数 getName: 返回可修改引用" << std::endl;
    return m_name;
  }
  // - 解释：
  // - 1. 这两个 getName 构成重载：一个 const，一个非 const。
  // - 2. 非 const 对象调用：调用非 const 版本，返回可修改引用。
  // - 3. const 对象调用：调用 const 版本，返回 const 引用。
  // - 4. 这是标准库容器（如 vector::operator[]）的通用做法。

  // --- 5. 返回值 const：防止返回值被修改 ---
  const int getValueConstReturn() const {
    std::cout << "const 成员函数 getValueConstReturn: 返回 const int" << std::endl;
    return m_value;
  }
  // - 解释：
  // - 1. 返回值是 const，防止外部代码修改返回值。
  // - 2. 例如：obj.getValueConstReturn() = 100; 会编译报错。
  // - 3. 提高接口安全性，明确表示返回值是只读的。
};

// --- 测试：const 对象 vs 非 const 对象 ---
void testConstObject() {
  std::cout << "\n--- 测试：非 const 对象 ---" << std::endl;
  MyClass obj(10, "test");

  obj.setValue(20);
  obj.setName("new test");
  obj.getValue();
  obj.getName();
  obj.getValueWithCache();
  obj.getValueWithCache();

  std::cout << "\n--- 测试：const 对象 ---" << std::endl;
  const MyClass constObj(30, "const test");

  // constObj.setValue(40); // 错误
  // constObj.setName("new const test"); // 错误

  constObj.getValue();
  constObj.getName();
  constObj.getValueWithCache();
  constObj.getValueWithCache();

  // constObj.getValueConstReturn() = 50; // 错误
}

// --- 第二部分：const 引用与临时对象 ---
void printString(const std::string& str) {
  std::cout << "const 引用打印: " << str << std::endl;
}
// - 解释 const 引用：
// - 1. const 引用可以绑定到临时对象（如 printString("hello")）。
// - 2. 非 const 引用不能绑定到临时对象。
// - 3. const 引用会延长临时对象的生命周期。

void testConstReference() {
  std::cout << "\n--- 测试：const 引用 ---" << std::endl;

  std::string s = "normal string";
  printString(s);
  printString("temporary string");
}

// --- 第三部分：const 指针与 const 引用（补充）---
void testConstPointer() {
  std::cout << "\n--- 测试：const 指针 ---" << std::endl;

  int x = 10;
  int y = 20;

  // --- 1. 指向 const 的指针 ---
  const int* p1 = &x;
  // *p1 = 100; // 错误
  p1 = &y;
  std::cout << "pointer to const: *p1 = " << *p1 << std::endl;

  // --- 2. const 指针 ---
  int* const p2 = &x;
  *p2 = 100;
  // p2 = &y; // 错误
  std::cout << "const pointer: *p2 = " << *p2 << std::endl;

  // --- 3. 指向 const 的 const 指针 ---
  const int* const p3 = &x;
  std::cout << "const pointer to const: *p3 = " << *p3 << std::endl;
}

// --- 第四部分：const 正确性的完整场景演示 ---
class Complex {
private:
  double m_real;
  double m_imag;
  mutable double m_absCache;

public:
  Complex(double r = 0, double i = 0)
    : m_real(r), m_imag(i), m_absCache(-1) {}

  // --- 非 const 成员函数 ---
  void setReal(double r) { m_real = r; m_absCache = -1; }
  void setImag(double i) { m_imag = i; m_absCache = -1; }

  // --- const 成员函数 ---
  double getReal() const { return m_real; }
  double getImag() const { return m_imag; }

  // --- const 成员函数，使用 mutable 缓存 ---
  double abs() const {
    if (m_absCache < 0) {
      m_absCache = std::sqrt(m_real * m_real + m_imag * m_imag);
      std::cout << "const 成员函数 abs: 计算模长 = " << m_absCache << std::endl;
    } else {
      std::cout << "const 成员函数 abs: 使用缓存 = " << m_absCache << std::endl;
    }
    return m_absCache;
  }
  // - 解释：
  // - 1. 这里用 std::sqrt 更规范，避免全局命名空间冲突。
  // - 2. m_absCache 是 mutable，所以在 const 函数里可以修改。

  // --- 重载：const 与非 const ---
  Complex& operator+=(const Complex& other) {
    m_real += other.m_real;
    m_imag += other.m_imag;
    m_absCache = -1;
    return *this;
  }

  Complex operator+(const Complex& other) const {
    Complex res = *this;
    res += other;
    return res;
  }
  // - 解释：
  // - 1. operator+ 是 const 成员函数，因为它不修改当前对象。
  // - 2. operator+= 是非 const 成员函数，因为它修改当前对象。
  // - 3. 这是运算符重载的标准 const 正确性实践。
};

void testComplex() {
  std::cout << "\n--- 测试：Complex 类 const 正确性 ---" << std::endl;

  Complex c1(3, 4);
  const Complex c2(1, 1);

  c1.getReal();
  c1.abs();
  c1.abs();
  c1.setReal(5);
  c1.abs();

  c2.getImag();
  c2.abs();
  c2.abs();

  Complex c3 = c1 + c2;
  c3 += c1;
}

int main() {
  testConstObject();
  testConstReference();
  testConstPointer();
  testComplex();
  return 0;
}

