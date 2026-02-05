/**
 * 静态成员函数 / 静态成员变量 完整示例
 * 包含：
 * 1. 静态成员函数定义与调用
 * 2. 静态成员变量定义与初始化
 * 3. 静态函数访问限制（只能访问静态成员）
 * 4. 对象计数（经典应用）
 * 5. 类级工具函数（不需要对象）
 * 6. 单例模式思想（静态函数获取实例）
 */

#include <iostream>
#include <string>

// ==============================
// 示例 1：基础静态成员 + 对象计数
// ==============================
class MyClass {
private:
  // 普通成员变量（属于每个对象）
  int m_id;

  // 静态成员变量（属于类，所有对象共享）
  static int s_object_count;

public:
  // 构造函数：每创建一个对象，计数+1
  MyClass(int id) : m_id(id) {
    s_object_count++;
  }

  // 析构函数：每销毁一个对象，计数-1
  ~MyClass() {
    s_object_count--;
  }

  // 普通成员函数：可以访问 静态 + 非静态
  void showInfo() const {
    std::cout << "对象ID: " << m_id << " | 当前总对象数: " << s_object_count << std::endl;
  }

  // ==============================
  // 静态成员函数：属于类，无 this 指针
  // ==============================
  static int getObjectCount() {
    // 静态函数里不能访问非静态成员（如 m_id）
    // 因为不知道是哪个对象的 m_id
    // m_id = 100; // 错误！

    // 只能访问静态成员
    return s_object_count;
  }

  // 静态工具函数：类级别的功能，不需要对象
  static void showClassInfo(const std::string& msg) {
    std::cout << "[MyClass 静态工具] " << msg << std::endl;
  }
};

// 静态成员变量必须在类外初始化（非常重要！）
int MyClass::s_object_count = 0;

// ==============================
// 示例 2：静态工具类（类似 Math 类）
// ==============================
class MathUtil {
public:
  // 纯静态工具类，不需要创建对象
  static double pi() {
    return 3.1415926535;
  }

  static int abs(int x) {
    return x < 0 ? -x : x;
  }

  static int max(int a, int b) {
    return a > b ? a : b;
  }
};

// ==============================
// 示例 3：单例模式（静态函数获取唯一实例）
// ==============================
class Singleton {
private:
  // 构造函数私有化：禁止外部创建对象
  Singleton() {
    std::cout << "Singleton 构造（只执行一次）" << std::endl;
  }

  // 禁用拷贝和赋值
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

  // 静态实例：整个程序只有一份
  static Singleton* s_instance;

public:
  // 静态成员函数：获取唯一实例
  static Singleton* getInstance() {
    if (s_instance == nullptr) {
      s_instance = new Singleton();
    }
    return s_instance;
  }

  void doSomething() {
    std::cout << "单例对象在工作" << std::endl;
  }
};

// 静态成员初始化
Singleton* Singleton::s_instance = nullptr;

// ==============================
// 主函数：演示所有用法
// ==============================
int main() {
  std::cout << "===== 1. 静态成员函数与对象计数 =====" << std::endl;

  // 直接调用静态成员函数（不需要对象）
  std::cout << "初始对象数: " << MyClass::getObjectCount() << std::endl;

  // 创建对象
  MyClass obj1(101);
  MyClass obj2(102);
  MyClass obj3(103);

  obj1.showInfo();
  obj2.showInfo();
  obj3.showInfo();

  // 仍然可以通过类名调用静态函数
  std::cout << "通过类名获取总数: " << MyClass::getObjectCount() << std::endl;

  // 通过对象调用静态函数（也可以，但不推荐，容易误解）
  std::cout << "通过对象获取总数: " << obj1.getObjectCount() << std::endl;

  // 调用静态工具函数
  MyClass::showClassInfo("这是一个类级别的工具调用");

  // -------------------------------------------------------------------------

  std::cout << "\n===== 2. 静态工具类 MathUtil =====" << std::endl;
  std::cout << "π = " << MathUtil::pi() << std::endl;
  std::cout << "abs(-123) = " << MathUtil::abs(-123) << std::endl;
  std::cout << "max(10, 20) = " << MathUtil::max(10, 20) << std::endl;

  // -------------------------------------------------------------------------

  std::cout << "\n===== 3. 单例模式 =====" << std::endl;
  Singleton* s1 = Singleton::getInstance();
  Singleton* s2 = Singleton::getInstance();

  // s1 和 s2 是同一个指针
  if (s1 == s2) {
    std::cout << "s1 和 s2 指向同一个对象" << std::endl;
  }

  s1->doSomething();

  return 0;
}
