// ============================================================
// 本节要学什么？
// 1. 单参数构造函数的隐式类型转换（坑点）
// 2. explicit 关键字：禁止构造函数的隐式转换，强制显式构造
// 3. 类型转换运算符（operator Type()）：自定义对象到其他类型的转换
// 4. explicit 类型转换运算符（C++11）：禁止隐式转换，只允许显式转换
// 5. explicit operator bool()：安全的“判空”/“有效性”检查（避免隐式转 int）
// 6. 各种转换场景的对比：隐式 vs 显式，安全 vs 危险
//
// 为什么要学？
// - 隐式类型转换是 C++ 中最隐蔽、最容易出 bug 的地方之一
// - explicit 是写出安全、健壮、可读性高代码的必备关键字
// - explicit operator bool 是智能指针、迭代器、容器等标准库的通用做法
// - 面试高频考点：explicit 作用、类型转换运算符、安全 bool 惯用法
//
// 核心重点（必须记住）：
// 1. 单参数构造函数（或除第一个外其余有默认值的构造）会被编译器用于隐式转换
// 2. explicit 构造：只能显式调用，禁止隐式转换，防止意外构造
// 3. 类型转换运算符：定义对象 → 其他类型的转换规则
// 4. explicit 转换运算符：只能显式转换（如 static_cast），禁止隐式转换
// 5. explicit operator bool()：安全的布尔测试，避免对象隐式转为 0/1 参与算术运算
// ============================================================

#include <iostream>
#include <string>

// --- 第一部分：单参数构造函数与隐式转换（坑点演示）---
// --- 危险类：无 explicit，允许隐式转换 ---
class DangerousInt {
private:
  int m_val;

public:
  // --- 单参数构造函数 ---
  DangerousInt(int val) : m_val(val) {
    std::cout << "DangerousInt(int) 构造: " << m_val << std::endl;
  }
  // - 解释：
  // - 1. 这是一个单参数构造函数，编译器会把它当作“int → DangerousInt”的隐式转换规则。
  // - 2. 这意味着：任何需要 DangerousInt 的地方，都可以直接传 int，编译器自动构造临时对象。
  // - 3. 这种行为非常隐蔽，容易导致意料之外的对象构造，引发 bug。

  int getVal() const { return m_val; }
};

// 接受 DangerousInt 类型的函数
void printDangerous(const DangerousInt& di) {
  std::cout << "DangerousInt 值: " << di.getVal() << std::endl;
}

void testImplicitCtor() {
  std::cout << "\n--- 测试：无 explicit，隐式转换构造 ---" << std::endl;

  DangerousInt di1(10);      // 显式构造（正常）
  DangerousInt di2 = 20;     // 隐式转换构造（危险！等价于 DangerousInt temp(20); di2 = temp;）
  // - 解释：
  // - 这里没有写构造函数，但是编译器自动用 20 构造了一个临时对象。

  printDangerous(30);        // 更危险！直接传 int 给需要 DangerousInt 的函数
  // - 解释：
  // - 编译器偷偷构造了一个临时 DangerousInt(30) 传给函数，你完全没意识到。
}

// --- 第二部分：explicit 构造函数（禁止隐式转换）---
// --- 安全类：有 explicit，禁止隐式转换 ---
class SafeInt {
private:
  int m_val;

public:
  // --- explicit 单参数构造函数 ---
  explicit SafeInt(int val) : m_val(val) {
    std::cout << "SafeInt(int) 显式构造: " << m_val << std::endl;
  }
  // - 解释：
  // - 1. explicit 关键字：禁止该构造函数参与隐式类型转换。
  // - 2. 只能通过显式调用（SafeInt(...)）来创建对象。
  // - 3. 这是现代 C++ 的最佳实践：所有单参数构造函数都应该加 explicit，除非你故意要隐式转换。

  int getVal() const { return m_val; }
};

// 接受 SafeInt 类型的函数
void printSafe(const SafeInt& si) {
  std::cout << "SafeInt 值: " << si.getVal() << std::endl;
}

void testExplicitCtor() {
  std::cout << "\n--- 测试：有 explicit，禁止隐式转换 ---" << std::endl;

  SafeInt si1(10);          // 显式构造（正常）
  // SafeInt si2 = 20;      // 错误！explicit 禁止隐式转换构造
  // printSafe(30);         // 错误！不能直接传 int，编译器无法隐式构造

  printSafe(SafeInt(30));   // 正确！必须显式构造
  printSafe(static_cast<SafeInt>(40)); // 正确！显式转换
}

// --- 第三部分：类型转换运算符（自定义对象 → 其他类型）---
// --- 危险类：无 explicit，允许隐式转换到其他类型 ---
class DangerousConverter {
private:
  int m_val;

public:
  DangerousConverter(int val) : m_val(val) {}

  // --- 类型转换运算符：对象 → int ---
  operator int() const {
    std::cout << "隐式转换: DangerousConverter → int" << std::endl;
    return m_val;
  }
  // - 解释：
  // - 1. 语法：operator 目标类型() const { ... }
  // - 2. 作用：定义了对象如何自动转换为 int 类型。
  // - 3. 危险：会在任何需要 int 的地方隐式触发，导致意外行为。

  // --- 类型转换运算符：对象 → std::string ---
  operator std::string() const {
    std::cout << "隐式转换: DangerousConverter → string" << std::endl;
    return std::to_string(m_val);
  }
};

void testImplicitConverter() {
  std::cout << "\n--- 测试：无 explicit，隐式类型转换运算符 ---" << std::endl;

  DangerousConverter dc(100);

  int x = dc;               // 隐式转换：dc → int
  std::cout << "x = " << x << std::endl;

  std::string s = dc;       // 隐式转换：dc → string
  std::cout << "s = " << s << std::endl;

  if (dc > 50) {            // 隐式转换：dc → int，然后比较
    std::cout << "dc > 50" << std::endl;
  }
  // - 解释：
  // - 上面这行代码看起来是在比较对象，实际上是先转成 int 再比较，非常隐蔽。
}

// --- 第四部分：explicit 类型转换运算符（C++11，安全）---
// --- 安全类：有 explicit，禁止隐式转换到其他类型 ---
class SafeConverter {
private:
  int m_val;

public:
  SafeConverter(int val) : m_val(val) {}

  // --- explicit 类型转换运算符：对象 → int ---
  explicit operator int() const {
    std::cout << "显式转换: SafeConverter → int" << std::endl;
    return m_val;
  }
  // - 解释：
  // - 1. explicit 禁止隐式转换，只能通过显式转换（static_cast）调用。
  // - 2. 防止意外转换，让代码意图更清晰。

  explicit operator std::string() const {
    std::cout << "显式转换: SafeConverter → string" << std::endl;
    return std::to_string(m_val);
  }
};

void testExplicitConverter() {
  std::cout << "\n--- 测试：有 explicit，显式类型转换运算符 ---" << std::endl;

  SafeConverter sc(200);

  // int x = sc;            // 错误！explicit 禁止隐式转换
  // std::string s = sc;    // 错误！

  int x = static_cast<int>(sc);    // 正确！显式转换
  std::cout << "x = " << x << std::endl;

  std::string s = static_cast<std::string>(sc); // 正确！显式转换
  std::cout << "s = " << s << std::endl;

  // if (sc > 50) {}        // 错误！不能隐式转 int 比较
  if (static_cast<int>(sc) > 50) { // 正确！显式转换后比较
    std::cout << "sc > 50" << std::endl;
  }
}

// --- 第五部分：explicit operator bool()（最常用、最重要！）---
// --- 演示类：模拟智能指针/文件句柄，实现安全的有效性检查 ---
class SmartPtr {
private:
  int* m_ptr;

public:
  SmartPtr(int* ptr = nullptr) : m_ptr(ptr) {}

  ~SmartPtr() {
    delete m_ptr;
  }

  // --- 危险版本：operator bool() 无 explicit ---
  // operator bool() const {
  //   return m_ptr != nullptr;
  // }
  // - 解释：
  // - 1. 可以在 if 中使用：if (sp) { ... }
  // - 2. 但是！也会隐式转换为 int：int x = sp; if (sp + 5) { ... } 这完全是无意义且危险的。

  // --- 安全版本：explicit operator bool() (C++11) ---
  explicit operator bool() const {
    std::cout << "显式 bool 转换: 检查指针是否为空" << std::endl;
    return m_ptr != nullptr;
  }
  // - 解释：
  // - 1. 这是标准库（如 unique_ptr, string, fstream）的通用做法。
  // - 2. 允许在 if、while、for 的条件中使用（上下文转换，是个例外，允许隐式）。
  // - 3. 禁止在其他地方隐式转换为 bool/int，防止滥用。
};

void testExplicitBool() {
  std::cout << "\n--- 测试：explicit operator bool() ---" << std::endl;

  SmartPtr sp1(new int(10));
  SmartPtr sp2;

  // --- 正确用法：条件判断（上下文转换，允许）---
  if (sp1) {
    std::cout << "sp1 持有有效指针" << std::endl;
  }

  if (!sp2) {
    std::cout << "sp2 是空指针" << std::endl;
  }

  // --- 错误用法：隐式转换为 int ---
  // int x = sp1;          // 错误！explicit 禁止
  // if (sp1 + 5) {}       // 错误！

  // --- 正确显式用法 ---
  bool b1 = static_cast<bool>(sp1);
  std::cout << "sp1 有效: " << std::boolalpha << b1 << std::endl;
}

int main() {
  testImplicitCtor();
  testExplicitCtor();
  testImplicitConverter();
  testExplicitConverter();
  testExplicitBool();
  return 0;
}

