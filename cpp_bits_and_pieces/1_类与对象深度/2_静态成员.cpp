// ============================================================
// 本节要学什么？
// 1. 静态成员变量（static member variable）：类级共享，不属于任何对象
// 2. 静态成员函数（static member function）：无 this 指针，只能访问静态成员
// 3. 静态成员的底层逻辑：存储在全局/静态存储区，不属于对象内存布局
// 4. 静态成员变量的初始化：类内声明、类外初始化（重点）
// 5. const 静态成员变量：整型可以类内初始化，其他类型仍需类外
// 6. 静态成员的访问控制：public/protected/private 依然有效
// 7. 静态成员与普通成员的区别：this 指针、访问方式、生命周期、存储位置
// 8. 静态成员的优缺点：共享数据、全局状态、多线程风险、单例模式基础
// 9. 静态与多线程：静态变量是共享资源，需要同步
// 10. 静态与单例模式：最经典的单例实现方式（Meyers' Singleton）
//
// 为什么要学？
// - 静态成员是实现“类级共享”和“全局状态”的核心机制
// - 静态成员是单例模式、工厂模式、工具类的基础
// - 理解 static 底层逻辑是掌握 C++ 内存模型的关键
// - 面试高频考点：静态成员初始化、this 指针、静态与普通成员区别、单例
//
// 核心重点（必须记住）
// 1. 静态成员属于类，不属于任何对象，所有对象共享同一份数据
// 2. 静态成员变量必须在类内声明、类外初始化（const 整型除外）
// 3. 静态成员函数没有 this 指针，只能访问静态成员，不能访问普通成员
// 4. 静态成员存储在全局/静态存储区，生命周期从程序开始到结束
// 5. 静态成员的访问方式：类名::成员 或 对象.成员（但本质与对象无关）
// 6. 静态成员是全局状态的封装，多线程访问需要加锁
// ============================================================

#include <iostream>
#include <string>
#include <mutex>
#include <thread>

// --- 第一部分：静态成员变量与静态成员函数 ---
// --- 演示类：包含静态变量、静态函数、普通变量、普通函数 ---
class MyClass {
private:
  // --- 1. 普通成员变量（非静态）---
  int m_normalVar; // 属于每个对象，存储在对象内存中

  // --- 2. 静态成员变量（类内声明）---
  static int s_staticVar; // 仅声明，不分配内存
  // - 解释：
  // - 1. static 关键字表示“静态”，属于类，不属于任何对象。
  // - 2. 类内声明只是告诉编译器“有这个成员”，**不分配内存**。
  // - 3. 必须在类外进行定义（初始化），才能分配内存。
  // - 4. 所有 MyClass 对象共享同一份 s_staticVar。

  // --- 3. const 静态成员变量（特殊情况）---
  static const int s_constStaticInt = 100; // 整型 const 静态可以类内初始化
  // - 解释：
  // - 1. C++ 允许整型（int, char, bool 等）的 const 静态成员在类内初始化。
  // - 2. 非整型（如 double, string）的 const 静态成员仍需类外初始化。

  static const double s_constStaticDouble; // 非整型，必须类外初始化

public:
  // --- 构造函数 ---
  MyClass(int normalVal) : m_normalVar(normalVal) {
    // 构造函数中可以访问静态变量
    s_staticVar++; // 每创建一个对象，静态变量加 1
    std::cout << "MyClass 构造: 普通变量 = " << m_normalVar 
              << ", 静态变量 = " << s_staticVar << std::endl;
  }

  // --- 析构函数 ---
  ~MyClass() {
    s_staticVar--; // 每销毁一个对象，静态变量减 1
    std::cout << "MyClass 析构: 普通变量 = " << m_normalVar 
              << ", 静态变量 = " << s_staticVar << std::endl;
  }

  // --- 4. 普通成员函数 ---
  void normalFunc() {
    // 普通函数可以访问：普通成员 + 静态成员
    std::cout << "普通函数: 普通变量 = " << m_normalVar 
              << ", 静态变量 = " << s_staticVar << std::endl;
    // 普通函数有 this 指针
    std::cout << "普通函数 this 指针 = " << this << std::endl;
  }

  // --- 5. 静态成员函数 ---
  static void staticFunc() {
    // 静态函数**只能访问静态成员**，不能访问普通成员
    std::cout << "静态函数: 静态变量 = " << s_staticVar << std::endl;
    // 错误！静态函数没有 this 指针，不能访问普通成员
    // std::cout << "普通变量 = " << m_normalVar << std::endl;
    // 错误！静态函数没有 this 指针
    // std::cout << "this 指针 = " << this << std::endl;
    // - 解释：
    // - 1. 静态成员函数属于类，不属于任何对象，因此**没有 this 指针**。
    // - 2. 这是静态函数与普通函数最根本的区别。
    // - 3. 静态函数只能访问静态成员变量和其他静态成员函数。
  }

  // --- 6. 静态函数访问私有静态变量 ---
  static int getStaticVar() {
    return s_staticVar;
  }

  static void setStaticVar(int val) {
    s_staticVar = val;
  }

  // --- 7. 获取 const 静态变量 ---
  static int getConstStaticInt() {
    return s_constStaticInt;
  }

  static double getConstStaticDouble() {
    return s_constStaticDouble;
  }
};

// --- 静态成员变量：类外定义（初始化）---
int MyClass::s_staticVar = 0; // 分配内存，初始化为 0
// - 解释：
// - 1. 必须在类外、全局作用域中定义。
// - 2. 不需要再写 static 关键字（类内已经声明过）。
// - 3. 这行代码才真正为 s_staticVar 分配内存。
// - 4. 初始化值可以是任意表达式，甚至调用函数。

// --- const 静态成员变量：类外定义（非整型）---
const double MyClass::s_constStaticDouble = 3.1415926;
// - 解释：
// - 1. 非整型的 const 静态成员必须在类外初始化。
// - 2. 必须加 const 关键字，与类内声明一致。

// --- 测试：静态成员基本用法 ---
void testStaticBasic() {
  std::cout << "\n--- 测试：静态成员基本用法 ---" << std::endl;

  // --- 静态成员的访问方式 1：通过类名访问（推荐，体现类级属性）---
  MyClass::staticFunc();
  std::cout << "类名访问静态变量: " << MyClass::getStaticVar() << std::endl;
  std::cout << "类名访问 const 静态 int: " << MyClass::getConstStaticInt() << std::endl;
  std::cout << "类名访问 const 静态 double: " << MyClass::getConstStaticDouble() << std::endl;

  // --- 创建对象 ---
  MyClass obj1(10);
  MyClass obj2(20);

  // --- 静态成员的访问方式 2：通过对象访问（不推荐，容易误解）---
  obj1.staticFunc();
  obj2.staticFunc();
  // - 解释：
  // - 1. 虽然通过对象访问，但本质还是访问类的静态成员，与对象无关。
  // - 2. 编译器会自动转换为 MyClass::staticFunc()。
  // - 3. 即使对象是 nullptr，也能调用静态函数（危险，不推荐）。

  // --- 普通函数访问 ---
  obj1.normalFunc();
  obj2.normalFunc();

  // --- 修改静态变量（所有对象共享）---
  MyClass::setStaticVar(999);
  std::cout << "修改后，obj1 看到的静态变量: " << obj1.getStaticVar() << std::endl;
  std::cout << "修改后，obj2 看到的静态变量: " << obj2.getStaticVar() << std::endl;
  // - 解释：
  // - 1. 静态变量是共享的，修改后所有对象都能看到变化。
  // - 2. 这就是“类级共享”的含义。
}

// --- 第二部分：静态成员的底层逻辑与内存布局 ---
// --- 演示类：查看对象大小与内存布局 ---
class MemoryTest {
public:
  int m_normal;       // 普通变量，占 4 字节
  static int s_static; // 静态变量，不占对象内存
};

int MemoryTest::s_static = 0;

void testStaticMemory() {
  std::cout << "\n--- 测试：静态成员底层逻辑与内存布局 ---" << std::endl;

  MemoryTest obj;
  std::cout << "MemoryTest 对象大小 = " << sizeof(obj) << " 字节" << std::endl;
  // - 解释：
  // - 1. 输出是 4 字节，仅包含普通变量 m_normal。
  // - 2. 静态变量 s_static 不占对象内存，存储在全局/静态存储区。
  // - 3. 静态成员与对象内存布局完全分离。

  std::cout << "对象地址 = " << &obj << std::endl;
  std::cout << "普通变量地址 = " << &obj.m_normal << std::endl;
  std::cout << "静态变量地址 = " << &MemoryTest::s_static << std::endl;
  // - 解释：
  // - 1. 普通变量地址在对象地址范围内。
  // - 2. 静态变量地址与对象地址无关，通常在全局数据区。
}

// --- 第三部分：静态成员的访问控制 ---
// --- 演示类：私有静态成员 ---
class PrivateStatic {
private:
  static int s_privateStatic; // 私有静态变量

public:
  static void setPrivateStatic(int val) {
    s_privateStatic = val;
  }

  static int getPrivateStatic() {
    return s_privateStatic;
  }
};

int PrivateStatic::s_privateStatic = 0;

void testStaticAccessControl() {
  std::cout << "\n--- 测试：静态成员访问控制 ---" << std::endl;

  // 错误！私有静态成员不能直接访问
  // std::cout << PrivateStatic::s_privateStatic << std::endl;

  // 正确！通过公有静态函数访问
  PrivateStatic::setPrivateStatic(123);
  std::cout << "私有静态变量值 = " << PrivateStatic::getPrivateStatic() << std::endl;
  // - 解释：
  // - 1. 静态成员同样受访问控制（public/protected/private）约束。
  // - 2. 私有静态成员只能通过类的公有静态函数访问，实现封装。
}

// --- 第四部分：静态成员与多线程（风险与安全）---
class ThreadSafeStatic {
private:
  static int s_sharedData;
  static std::mutex s_mutex; // 静态互斥锁，保护静态数据

public:
  static void increment() {
    std::lock_guard<std::mutex> lock(s_mutex); // 加锁
    s_sharedData++;
    std::cout << "线程 " << std::this_thread::get_id() << ": 共享数据 = " << s_sharedData << std::endl;
  }

  static int getSharedData() {
    std::lock_guard<std::mutex> lock(s_mutex);
    return s_sharedData;
  }
};

int ThreadSafeStatic::s_sharedData = 0;
std::mutex ThreadSafeStatic::s_mutex;

void testStaticMultiThread() {
  std::cout << "\n--- 测试：静态成员与多线程 ---" << std::endl;

  // 创建 10 个线程，同时修改静态变量
  std::thread t1(ThreadSafeStatic::increment);
  std::thread t2(ThreadSafeStatic::increment);
  std::thread t3(ThreadSafeStatic::increment);
  std::thread t4(ThreadSafeStatic::increment);
  std::thread t5(ThreadSafeStatic::increment);

  t1.join();
  t2.join();
  t3.join();
  t4.join();
  t5.join();

  std::cout << "最终共享数据 = " << ThreadSafeStatic::getSharedData() << std::endl;
  // - 解释：
  // - 1. 静态成员是全局共享资源，多线程同时访问会导致竞态条件。
  // - 2. 必须使用互斥锁（mutex）等同步机制保护。
  // - 3. 这是静态成员最大的缺点之一：引入全局状态，多线程不安全。
}

// --- 第五部分：静态成员与单例模式（最经典应用）---
// --- Meyers' Singleton：C++ 最推荐的单例实现，利用静态局部变量 ---
class Singleton {
private:
  // --- 构造函数私有：禁止外部创建对象 ---
  Singleton() {
    std::cout << "Singleton 构造" << std::endl;
  }

  // --- 禁止拷贝和赋值 ---
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

public:
  // --- 静态函数：获取单例实例 ---
  static Singleton& getInstance() {
    // --- 静态局部变量：C++11 保证线程安全的初始化 ---
    static Singleton instance;
    // - 解释：
    // - 1. 静态局部变量在第一次调用 getInstance() 时初始化。
    // - 2. C++11 标准保证静态局部变量的初始化是线程安全的，无需手动加锁。
    // - 3. 这是目前 C++ 中最简单、最安全的单例实现方式。
    return instance;
  }

  void doSomething() {
    std::cout << "Singleton 做某事" << std::endl;
  }
};

void testSingleton() {
  std::cout << "\n--- 测试：静态成员与单例模式 ---" << std::endl;

  // 获取单例实例
  Singleton& s1 = Singleton::getInstance();
  Singleton& s2 = Singleton::getInstance();

  // 验证是同一个实例
  std::cout << "s1 地址 = " << &s1 << std::endl;
  std::cout << "s2 地址 = " << &s2 << std::endl;
  std::cout << "s1 和 s2 是同一个实例: " << std::boolalpha << (&s1 == &s2) << std::endl;

  s1.doSomething();
  s2.doSomething();
}

// --- 第六部分：静态成员的优缺点总结 ---
// 优点：
// 1. 实现类级共享数据，无需全局变量，封装性好
// 2. 静态函数可以作为工具函数，无需创建对象即可调用
// 3. 是单例模式、工厂模式等设计模式的基础
// 4. 静态局部变量实现线程安全的延迟初始化（C++11）
//
// 缺点：
// 1. 引入全局状态，降低代码的可测试性和可维护性
// 2. 多线程访问需要同步，增加复杂度
// 3. 静态成员生命周期长，占用内存直到程序结束
// 4. 过度使用静态会导致代码耦合度高，难以扩展

int main() {
  testStaticBasic();
  testStaticMemory();
  testStaticAccessControl();
  testStaticMultiThread();
  testSingleton();
  return 0;
}

