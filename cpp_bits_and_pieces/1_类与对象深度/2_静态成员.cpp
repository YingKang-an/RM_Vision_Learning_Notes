// ============================================================
// 本节要学什么？
// 1. 类的静态成员：静态成员变量、静态成员函数
// 2. 静态成员的本质：属于类，不属于任何对象，全局共享
// 3. 静态成员的定义、初始化、访问方式
// 4. 静态成员与普通成员的区别、底层存储原理
// 5. 静态成员的使用场景、注意事项、常见坑点
//
// 为什么要学？
// - 静态成员是“类级共享”的核心，用于统计对象个数、全局状态、工具函数
// - 面试高频考点，工程中大量使用（单例模式、计数器、配置管理）
// - 理解静态成员的底层逻辑，才能写出安全、高效、可维护的代码
//
// 核心重点（必须记住）：
// 1. 静态成员属于类，所有对象共享同一份内存，不占对象大小
// 2. 静态成员变量必须在类外初始化（const static 整型除外）
// 3. 静态成员函数没有 this 指针，只能访问静态成员，不能访问普通成员
// 4. 访问方式：类名::成员 或 对象.成员（效果一样）
// 5. 静态成员的生命周期：程序开始到结束，和全局变量一样
//
// 扩展对比：
// - 普通成员：属于对象，每个对象有独立副本，占对象内存
// - 静态成员：属于类，所有对象共享，不占对象内存
// - 全局变量：无封装，暴露全局，易冲突
// - 静态成员：有类封装，作用域受限，更安全
// ============================================================

#include <iostream>

// --- 演示类：包含静态成员变量和静态成员函数 ---
class MyClass {
private:
  // --- 1. 静态成员变量（声明） ---
  // 作用：类级共享，所有对象共用一份
  // 注意：类内只是声明，必须在类外定义+初始化
  static int s_count;  /* < 静态计数器，统计对象个数 */

  // 普通成员变量（对比用）
  int m_value;

public:
  // --- 构造函数 ---
  // 每次创建对象，静态计数器+1
  MyClass(int val = 0) : m_value(val) {
    s_count++;  /* < 访问静态成员变量 */
    std::cout << "构造函数: s_count = " << s_count << std::endl;
  }

  // --- 析构函数 ---
  // 每次销毁对象，静态计数器-1
  ~MyClass() {
    s_count--;
    std::cout << "析构函数: s_count = " << s_count << std::endl;
  }

  // --- 2. 静态成员函数 ---
  // 作用：类级函数，无需创建对象即可调用
  // 特点：没有 this 指针，只能访问静态成员，不能访问普通成员
  static int getCount() {  /* < 静态成员函数 */
    // 错误：静态函数不能访问普通成员
    // std::cout << m_value << std::endl;

    return s_count;  /* < 只能访问静态成员 */
  }

  // --- 普通成员函数 ---
  // 可以访问静态成员和普通成员
  void print() const {
    std::cout << "m_value = " << m_value 
              << ", s_count = " << s_count << std::endl;
  }

  // --- 静态常量成员（特殊情况） ---
  // const static 整型可以在类内直接初始化
  static const int s_max = 100;
};

// --- 静态成员变量的定义与初始化（必须在类外） ---
// 为什么必须在类外？
// 1. 类内只是声明，不分配内存
// 2. 静态成员属于类，不属于对象，需要在全局作用域分配内存
// 3. 避免头文件重复包含导致重复定义
int MyClass::s_count = 0;  /* < 类外初始化，初始值为0 */

// --- 测试静态成员 ---
void testStaticMember() {
  std::cout << "\n--- 初始状态：s_count = " << MyClass::getCount() << " ---" << std::endl;

  // --- 1. 创建对象，静态计数器变化 ---
  MyClass obj1(10);
  MyClass obj2(20);
  MyClass obj3(30);

  std::cout << "\n--- 访问静态成员：类名::函数 ---" << std::endl;
  std::cout << "MyClass::getCount() = " << MyClass::getCount() << std::endl;

  std::cout << "\n--- 访问静态成员：对象.函数 ---" << std::endl;
  std::cout << "obj1.getCount() = " << obj1.getCount() << std::endl;
  std::cout << "obj2.getCount() = " << obj2.getCount() << std::endl;

  std::cout << "\n--- 访问静态常量成员 ---" << std::endl;
  std::cout << "MyClass::s_max = " << MyClass::s_max << std::endl;

  std::cout << "\n--- 普通成员函数访问静态成员 ---" << std::endl;
  obj1.print();
  obj2.print();

  std::cout << "\n--- 销毁对象，静态计数器变化 ---" << std::endl;
}

// --- 静态成员的底层原理 ---
void testStaticMemory() {
  std::cout << "\n--- 静态成员不占对象内存 ---" << std::endl;
  MyClass obj;
  // MyClass 的大小 = 普通成员 m_value 的大小（int）
  // 静态成员 s_count 存在全局数据区，不属于对象
  std::cout << "sizeof(MyClass) = " << sizeof(MyClass) << std::endl;
  std::cout << "sizeof(obj) = " << sizeof(obj) << std::endl;
}

int main() {
  testStaticMember();
  testStaticMemory();
  return 0;
}
