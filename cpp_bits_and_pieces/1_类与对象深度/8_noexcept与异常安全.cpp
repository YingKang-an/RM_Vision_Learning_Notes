// ============================================================
// 本节要学什么？
// 1. noexcept 修饰符：声明函数“保证不抛异常”（C++11 核心特性）
// 2. noexcept 运算符：编译期判断一个表达式是否保证不抛异常
// 3. 异常规格说明的演变：throw() -> noexcept（C++11 弃用旧语法）
// 4. 构造函数的异常安全：构造抛异常会导致对象未完全构造，资源泄漏
// 5. 析构函数的异常安全：析构抛异常是未定义行为，必须保证不抛
// 6. 异常安全的三个等级：基本保证、强保证（事务）、不抛保证
// 7. RAII（资源获取即初始化）：C++ 异常安全的基石
// 8. 标准库中的 noexcept：容器、智能指针、std::move 等的 noexcept 保证
// 9. noexcept 对性能的影响：允许编译器优化（如不生成栈展开代码）
//
// 为什么要学？
// - 异常安全是写出健壮、可靠、无泄漏代码的基础
// - noexcept 是现代 C++ 接口设计的必备，影响容器、移动语义、标准库行为
// - 构造/析构异常是最隐蔽、最危险的 bug 来源之一
// - RAII 是 C++ 区别于其他语言的核心资源管理思想
// - 面试高频考点：noexcept、异常安全等级、RAII、构造析构异常
//
// 核心重点（必须记住）：
// 1. noexcept(expression)：表达式为 true 时，函数保证不抛异常
// 2. 析构函数默认是 noexcept(true)，绝对不要在析构中抛异常
// 3. 构造函数抛异常：已构造的子对象会被析构，未释放的资源会泄漏
// 4. 异常安全三等级：基本（无泄漏）、强（回滚）、不抛（noexcept）
// 5. RAII：资源生命周期绑定到对象生命周期，析构自动释放，异常安全
// 6. 标准库容器仅在元素的移动构造/移动赋值是 noexcept 时才使用移动
// 7. noexcept 是接口契约，违反会调用 std::terminate() 终止程序
// ============================================================

#include <iostream>
#include <vector>
#include <memory>
#include <stdexcept>
#include <string>
#include <algorithm>

// --- 第一部分：noexcept 修饰符与运算符 ---
// --- 1. 基础 noexcept 函数 ---
void mayThrow() {
  throw std::runtime_error("mayThrow 抛异常");
}

void noThrow() noexcept {
  std::cout << "noThrow() 执行，保证不抛异常" << std::endl;
}
// - 解释 noexcept 修饰符：
// - 1. 语法：返回类型 函数名(参数) noexcept(可选条件) { ... }
// - 2. noexcept 等价于 noexcept(true)，明确承诺不抛异常。
// - 3. 如果违反承诺（函数内真的抛了），程序会直接调用 std::terminate() 终止。
// - 4. 这是一个“契约”，告诉编译器和调用者：放心调用，不会有异常。

// --- 2. 条件 noexcept ---
template <typename T>
void wrapper(const T& obj) noexcept(noexcept(obj.func())) {
  obj.func();
}
// - 解释：
// - 1. 内层 noexcept(obj.func()) 是“noexcept 运算符”，编译期判断 obj.func() 是否不抛。
// - 2. 外层 noexcept(...) 是“noexcept 修饰符”，根据运算符结果决定是否不抛。
// - 3. 这是模板中实现完美转发 noexcept 的标准写法。

class TestClass {
public:
  // 修复：func 加上 const，才能被 const TestClass& 调用
  void func() const noexcept { 
    std::cout << "TestClass::func() 不抛" << std::endl; 
  }
  void badFunc() const { throw std::exception(); }
};

// --- 3. noexcept 运算符演示 ---
void testNoexceptOperator() {
  std::cout << "\n--- 测试 noexcept 运算符 ---" << std::endl;

  std::cout << "noexcept(mayThrow()) = " << std::boolalpha << noexcept(mayThrow()) << std::endl;
  std::cout << "noexcept(noThrow()) = " << noexcept(noThrow()) << std::endl;

  TestClass t;
  std::cout << "noexcept(t.func()) = " << noexcept(t.func()) << std::endl;
  std::cout << "noexcept(t.badFunc()) = " << noexcept(t.badFunc()) << std::endl;

  std::cout << "\n--- 测试条件 noexcept 模板 ---" << std::endl;
  std::cout << "noexcept(wrapper(t)) = " << noexcept(wrapper(t)) << std::endl;
}

// --- 第二部分：构造函数的异常安全 ---
class DangerousResource {
private:
  int* m_ptr;
  std::string m_name;

public:
  DangerousResource(const std::string& name, bool shouldThrow)
    : m_ptr(new int(10)), m_name(name) {
    std::cout << "DangerousResource 构造: " << m_name << std::endl;

    if (shouldThrow) {
      delete m_ptr;
      m_ptr = nullptr;
      throw std::runtime_error("DangerousResource 构造失败，抛异常");
    }
  }

  ~DangerousResource() {
    std::cout << "DangerousResource 析构: " << m_name << std::endl;
    delete m_ptr;
  }
};

class SafeResource {
private:
  std::unique_ptr<int> m_ptr;
  std::string m_name;

public:
  SafeResource(const std::string& name, bool shouldThrow)
    : m_ptr(std::make_unique<int>(10)), m_name(name) {
    std::cout << "SafeResource 构造: " << m_name << std::endl;

    if (shouldThrow) {
      throw std::runtime_error("SafeResource 构造失败，抛异常");
    }
  }

  ~SafeResource() {
    std::cout << "SafeResource 析构: " << m_name << std::endl;
  }
};

void testConstructorException() {
  std::cout << "\n--- 测试：构造函数抛异常 ---" << std::endl;

  try {
    DangerousResource dr("危险对象", true);
  } catch (const std::exception& e) {
    std::cout << "捕获异常: " << e.what() << std::endl;
  }

  try {
    SafeResource sr("安全对象", true);
  } catch (const std::exception& e) {
    std::cout << "捕获异常: " << e.what() << std::endl;
  }
}

// --- 第三部分：析构函数的异常安全 ---
class EvilDestructor {
public:
  ~EvilDestructor() {
    // 绝对禁止在析构中抛异常
  }
};

class GoodDestructor {
public:
  ~GoodDestructor() noexcept {
    std::cout << "GoodDestructor 析构，保证不抛异常" << std::endl;
  }
};

void testDestructorNoexcept() {
  std::cout << "\n--- 测试：析构函数 noexcept ---" << std::endl;

  GoodDestructor gd;
  std::cout << "noexcept(gd.~GoodDestructor()) = " << noexcept(gd.~GoodDestructor()) << std::endl;
}

// --- 第四部分：异常安全的三个等级 ---
template <typename T>
class Stack {
private:
  T* m_data;
  size_t m_size;
  size_t m_capacity;

  void resize() {
    T* newData = new T[m_capacity * 2];
    std::copy(m_data, m_data + m_size, newData);
    delete[] m_data;
    m_data = newData;
    m_capacity *= 2;
  }

public:
  Stack() : m_data(nullptr), m_size(0), m_capacity(0) {}

  ~Stack() { delete[] m_data; }

  void push(const T& val) {
    if (m_size == m_capacity) {
      resize();
    }
    m_data[m_size++] = val;
  }

  T pop() {
    if (m_size == 0) {
      throw std::underflow_error("栈空");
    }
    T val = m_data[--m_size];
    return val;
  }

  void clear() noexcept {
    m_size = 0;
  }
};

void testExceptionSafetyLevels() {
  std::cout << "\n--- 测试：异常安全三个等级 ---" << std::endl;

  Stack<int> s;
  s.push(1);
  s.push(2);
  s.push(3);

  s.clear();
  try {
    s.pop();
  } catch (const std::exception& e) {
    std::cout << "pop 异常: " << e.what() << std::endl;
  }
}

// --- 第五部分：RAII 与异常安全基石 ---
class FileHandler {
private:
  FILE* m_file;

public:
  FileHandler(const char* filename, const char* mode)
    : m_file(fopen(filename, mode)) {
    if (!m_file) {
      throw std::runtime_error("文件打开失败");
    }
    std::cout << "文件打开成功" << std::endl;
  }

  ~FileHandler() noexcept {
    if (m_file) {
      fclose(m_file);
      std::cout << "文件关闭成功" << std::endl;
    }
  }

  FileHandler(const FileHandler&) = delete;
  FileHandler& operator=(const FileHandler&) = delete;

  FileHandler(FileHandler&& other) noexcept
    : m_file(other.m_file) {
    other.m_file = nullptr;
  }

  FileHandler& operator=(FileHandler&& other) noexcept {
    if (this != &other) {
      if (m_file) fclose(m_file);
      m_file = other.m_file;
      other.m_file = nullptr;
    }
    return *this;
  }

  void write(const char* str) {
    if (fputs(str, m_file) == EOF) {
      throw std::runtime_error("文件写入失败");
    }
  }
};

void testRAII() {
  std::cout << "\n--- 测试：RAII 异常安全 ---" << std::endl;

  try {
    FileHandler fh("test.txt", "w");
    fh.write("Hello, RAII!");
  } catch (const std::exception& e) {
    std::cout << "捕获异常: " << e.what() << std::endl;
  }
}

// --- 第六部分：noexcept 与标准库、移动语义 ---
class NoexceptMoveClass {
public:
  NoexceptMoveClass() {}
  NoexceptMoveClass(NoexceptMoveClass&&) noexcept {
    std::cout << "NoexceptMoveClass 移动构造（noexcept）" << std::endl;
  }
};

class ThrowMoveClass {
public:
  ThrowMoveClass() {}
  ThrowMoveClass(ThrowMoveClass&&) {
    std::cout << "ThrowMoveClass 移动构造（可能抛异常）" << std::endl;
  }
};

void testNoexceptAndVector() {
  std::cout << "\n--- 测试：noexcept 与 vector 扩容 ---" << std::endl;

  std::vector<NoexceptMoveClass> v1;
  v1.emplace_back();
  v1.emplace_back();

  std::vector<ThrowMoveClass> v2;
  v2.emplace_back();
  v2.emplace_back();
}

int main() {
  testNoexceptOperator();
  testConstructorException();
  testDestructorNoexcept();
  testExceptionSafetyLevels();
  testRAII();
  testNoexceptAndVector();
  return 0;
}

