/**
 * C++ 类的 6 大特殊成员函数 完整讲解
 * 包含：
 * 1. 默认构造函数
 * 2. 析构函数
 * 3. 拷贝构造函数
 * 4. 拷贝赋值运算符
 * 5. 移动构造函数
 * 6. 移动赋值运算符
 *
 * 每个函数都有：作用 + 写法 + 调用时机 + 注意事项
 */

#include <iostream>
#include <cstring> // strlen, strcpy

// ==============================
// 演示类：带动态内存的字符串类
// ==============================
class MyString {
private:
  char* m_data;   // 动态分配的字符串
  size_t m_len;   // 字符串长度

public:
  // ==============================
  // 1. 默认构造函数
  // 作用：创建对象时初始化
  // 调用时机：MyString s;
  // ==============================
  MyString() : m_data(nullptr), m_len(0) {
    std::cout << "【默认构造】空字符串" << std::endl;
  }

  // 带参构造（非特殊，但常用）
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

  // ==============================
  // 2. 析构函数
  // 作用：释放资源（如 new 的内存）
  // 调用时机：对象生命周期结束时
  // ==============================
  ~MyString() {
    std::cout << "【析构】: " << (m_data ? m_data : "空") << std::endl;
    delete[] m_data; // 必须释放动态内存
  }

  // ==============================
  // 3. 拷贝构造函数
  // 作用：用一个已有对象创建新对象
  // 调用时机：MyString s2(s1); 或 MyString s2 = s1;
  // 深拷贝：拷贝内容，不是只拷贝指针
  //-----------------------------
  // 拷贝构造函数为什么必须是 const &
  // const & 可以接收左值和右值，(但绑定后都被当作“左值”使用)
  // - 必须加 &（引用）：传值会触发拷贝构造，导致无限递归；引用是别名，不创建新对象，避免递归且高效。
  // - 必须加 const：保证不修改源对象，兼容 const 对象和临时对象（右值），符合拷贝“只读复制”的语义。
  // ==============================
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

  // ==============================
  // 4. 拷贝赋值运算符
  // 作用：把一个对象赋值给另一个已存在的对象
  // 调用时机：s2 = s1;
  // 注意：要处理自赋值、释放旧资源
  // ==============================
  MyString& operator= (const MyString& other) {
    if (this == &other) { // 自赋值检查,防止拷贝已释放的内存
      return *this;
    }

    // 释放当前对象的旧资源
    delete[] m_data;

    // 深拷贝
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

  // ==============================
  // 5. 移动构造函数
  // 作用：从“临时对象/右值”偷资源，避免拷贝
  // 调用时机：MyString s2(std::move(s1)); 或返回临时对象
  // 特点：高效，不拷贝数据，只转移指针
  //------------------------------
  // 移动构造/移动赋值加 noexcept 的全部原因
  // - 承诺不抛异常，让编译器优化代码，生成更高效的指令
  // - 容器（如 vector）扩容时，只有看到 noexcept 才敢用移动，否则降级为拷贝，性能差很多
  // - 移动操作只做指针转移，本身不会失败，加 noexcept 符合语义，也更安全
  // - 标准库很多组件（如算法、容器）会根据 noexcept 选择更优实现
  // ==============================
  MyString(MyString&& other) noexcept {
    // 直接“偷” other 的资源
    m_data = other.m_data;
    m_len = other.m_len;

    // 把 other 置空，防止它析构时释放资源
    other.m_data = nullptr;
    other.m_len = 0;

    std::cout << "【移动构造】资源转移" << std::endl;
  }

  // ==============================
  // 6. 移动赋值运算符
  // 作用：从右值赋值，转移资源
  // 调用时机：s2 = std::move(s1);
  //------------------------------
  // 移动构造/移动赋值加 noexcept 的全部原因
  // - 承诺不抛异常，让编译器优化代码，生成更高效的指令
  // - 容器（如 vector）扩容时，只有看到 noexcept 才敢用移动，否则降级为拷贝，性能差很多
  // - 移动操作只做指针转移，本身不会失败，加 noexcept 符合语义，也更安全
  // - 标准库很多组件（如算法、容器）会根据 noexcept 选择更优实现
  // ==============================
  MyString& operator= (MyString&& other) noexcept {
    if (this == &other) {
      return *this;
    }

    // 释放当前对象的旧资源
    delete[] m_data;

    // 偷资源
    m_data = other.m_data;
    m_len = other.m_len;

    // 置空源对象
    other.m_data = nullptr;
    other.m_len = 0;

    std::cout << "【移动赋值】资源转移" << std::endl;
    return *this;
  }

  // 辅助函数：打印内容
  void print() const {
    if (m_data) {
      std::cout << "内容: " << m_data << std::endl;
    } else {
      std::cout << "内容: 空" << std::endl;
    }
  }
};

// ==============================
// 主函数：演示 6 大函数的调用
// ==============================
int main() {
  std::cout << "\n===== 1. 默认构造 =====" << std::endl;
  MyString s1;

  std::cout << "\n===== 2. 带参构造 =====" << std::endl;
  MyString s2("hello");

  std::cout << "\n===== 3. 拷贝构造 =====" << std::endl;
  MyString s3 = s2;

  std::cout << "\n===== 4. 拷贝赋值 =====" << std::endl;
  MyString s4;
  s4 = s2;

  std::cout << "\n===== 5. 移动构造 =====" << std::endl;
  MyString s5 = std::move(s2);

  std::cout << "\n===== 6. 移动赋值 =====" << std::endl;
  MyString s6;
  s6 = std::move(s3);

  std::cout << "\n===== 最终状态 =====" << std::endl;
  s1.print();
  s2.print(); // 被移动后为空
  s3.print(); // 被移动后为空
  s4.print();
  s5.print();
  s6.print();

  return 0;
}
