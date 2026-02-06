// ============================================================
// 本节要学什么？
// 1. C++ 类的 6 大特殊成员函数：构造、析构、拷贝构造、拷贝赋值、移动构造、移动赋值
// 2. 每个函数的作用、写法、调用时机、注意事项
// 3. 新手必懂的灵魂问题：
//    - 拷贝构造为什么必须是 const &？
//    - 移动构造/赋值为什么要加 noexcept？
//    - 自赋值检查为什么重要？
// 4. 深拷贝 vs 浅拷贝、拷贝 vs 移动的本质区别
//
// 为什么要学？
// - 特殊成员函数是类的“生命周期骨架”，不懂就会写出内存泄漏、重复释放、崩溃的代码
// - 移动语义（C++11）是现代 C++ 性能优化的核心，容器、算法、异步都依赖它
// - 面试必考、工程必用，是写出安全、高效、可维护代码的基础
//
// 核心重点（必须记住）：
// 1. 六大函数：构造 → 析构 → 拷贝构造 → 拷贝赋值 → 移动构造 → 移动赋值
// 2. 拷贝构造必须是 const &：避免无限递归、兼容 const/临时对象、保证源对象不被修改
// 3. 移动操作必须加 noexcept：容器扩容才敢用移动，否则降级为拷贝，性能暴跌
// 4. 拷贝赋值必须做自赋值检查：防止释放自己的内存后再拷贝，导致崩溃
// 5. 移动后源对象必须置空：避免析构时重复释放资源
//
// 扩展对比：
// - C++98：只有 4 个特殊成员函数（无移动）
// - C++11：新增移动构造/移动赋值，形成“六大特殊成员函数”
// - 拷贝：复制资源（安全但慢，适合小对象/不可移动对象）
// - 移动：转移资源（快但源对象失效，适合大对象/临时对象）
// ============================================================

#include <iostream>
#include <cstring> // strlen, strcpy

// --- 演示类：带动态内存的字符串类 ---
class MyString {
private:
  char* m_data;  /* < 动态分配的字符串 */
  size_t m_len;  /* < 字符串长度 */

public:
  // --- 1. 默认构造函数 ---
  // 作用：创建空对象，初始化成员
  // 调用时机：MyString s;
  MyString() : m_data(nullptr), m_len(0) {
    std::cout << "【默认构造】空字符串" << std::endl;
  }

  // --- 带参构造（非特殊，但常用） ---
  MyString(const char* str) {
    if (str) {
      m_len = strlen(str);
      m_data = new char[m_len + 1];
      strcpy(m_data, str);
    } else {
      m_data = nullptr;
      m_len = 0;
    }
    std::cout << "【带参构造】: " << m_data << std::endl;
  }

  // --- 2. 析构函数 ---
  // 作用：释放动态内存，避免泄漏
  // 调用时机：对象生命周期结束时
  ~MyString() {
    std::cout << "【析构】: " << (m_data ? m_data : "空") << std::endl;
    delete[] m_data; // 必须释放，否则内存泄漏
  }

  // --- 3. 拷贝构造函数 ---
  // 作用：用已有对象创建新对象（深拷贝）
  // 调用时机：MyString s2(s1); 或 MyString s2 = s1;
  // ------------------------------
  // 为什么必须是 const &？（灵魂注释）
  // 1. 必须加 &：传值会触发拷贝构造，导致无限递归；引用是别名，不创建新对象
  // 2. 必须加 const：
  //    - 保证不修改源对象，符合“拷贝”语义
  //    - 兼容 const 对象（const MyString s1; s2 = s1;）
  //    - 兼容临时对象（右值），如 MyString s2 = MyString("hello");
  // ------------------------------
  MyString(const MyString& other) {
    m_len = other.m_len;
    if (other.m_data) {
      m_data = new char[m_len + 1];
      strcpy(m_data, other.m_data);
    } else {
      m_data = nullptr;
    }
    std::cout << "【拷贝构造】: " << m_data << std::endl;
  }

  // --- 4. 拷贝赋值运算符 ---
  // 作用：把一个对象赋值给另一个已存在的对象（深拷贝）
  // 调用时机：s2 = s1;
  // 注意：必须处理自赋值，否则释放自己的内存后再拷贝，导致崩溃
  MyString& operator=(const MyString& other) {
    if (this == &other) { // 自赋值检查
      return *this;
    }

    // 先释放旧资源
    delete[] m_data;

    // 再深拷贝新资源
    m_len = other.m_len;
    if (other.m_data) {
      m_data = new char[m_len + 1];
      strcpy(m_data, other.m_data);
    } else {
      m_data = nullptr;
    }

    std::cout << "【拷贝赋值】: " << m_data << std::endl;
    return *this;
  }

  // --- 5. 移动构造函数 (C++11) ---
  // 作用：从临时对象/右值偷资源，避免深拷贝，提升性能
  // 调用时机：MyString s2(std::move(s1)); 或返回临时对象
  // ------------------------------
  // 为什么必须加 noexcept？（灵魂注释）
  // 1. 承诺不抛异常，让编译器生成更高效的代码
  // 2. 容器（如 vector）扩容时，只有看到 noexcept 才敢用移动，否则降级为拷贝
  // 3. 移动操作只做指针转移，本身不会失败，加 noexcept 符合语义
  // 4. 标准库算法/容器会根据 noexcept 选择最优实现
  // ------------------------------
  MyString(MyString&& other) noexcept {
    // 直接偷资源
    m_data = other.m_data;
    m_len = other.m_len;

    // 源对象置空，防止析构时重复释放
    other.m_data = nullptr;
    other.m_len = 0;

    std::cout << "【移动构造】资源转移" << std::endl;
  }

  // --- 6. 移动赋值运算符 (C++11) ---
  // 作用：从右值赋值，转移资源，避免深拷贝
  // 调用时机：s2 = std::move(s1);
  // ------------------------------
  // 为什么必须加 noexcept？（同上，灵魂注释）
  // 1. 承诺不抛异常，编译器优化
  // 2. 容器扩容才敢用移动，否则降级为拷贝
  // 3. 移动操作无失败可能，符合语义
  // ------------------------------
  MyString& operator=(MyString&& other) noexcept {
    if (this == &other) {
      return *this;
    }

    // 先释放旧资源
    delete[] m_data;

    // 再偷新资源
    m_data = other.m_data;
    m_len = other.m_len;

    // 源对象置空
    other.m_data = nullptr;
    other.m_len = 0;

    std::cout << "【移动赋值】资源转移" << std::endl;
    return *this;
  }

  // --- 辅助函数：打印内容 ---
  void print() const {
    if (m_data) {
      std::cout << "内容: " << m_data << std::endl;
    } else {
      std::cout << "内容: 空" << std::endl;
    }
  }
};

// --- 测试函数：触发所有特殊成员函数 ---
MyString createTemp() {
  return MyString("临时对象");
}

void test() {
  std::cout << "\n--- 1. 默认构造 ---" << std::endl;
  MyString s1;

  std::cout << "\n--- 2. 带参构造 ---" << std::endl;
  MyString s2("hello");

  std::cout << "\n--- 3. 拷贝构造 ---" << std::endl;
  MyString s3 = s2;

  std::cout << "\n--- 4. 拷贝赋值 ---" << std::endl;
  MyString s4("world");
  s4 = s2;

  std::cout << "\n--- 5. 移动构造（临时对象） ---" << std::endl;
  MyString s5 = createTemp();

  std::cout << "\n--- 6. 移动赋值（临时对象） ---" << std::endl;
  MyString s6("abc");
  s6 = createTemp();

  std::cout << "\n--- 7. 显式移动 ---" << std::endl;
  MyString s7 = std::move(s2);

  std::cout << "\n--- 测试结束，局部对象析构 ---" << std::endl;
}

int main() {
  test();
  return 0;
}

