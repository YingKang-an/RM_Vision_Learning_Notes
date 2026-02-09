// ============================================================
// 本节要学什么？
// 1. 什么是自定义删除器：替换 shared_ptr/unique_ptr 默认的 delete/delete[]
// 2. 为什么需要自定义删除器：管理非堆内存、C 资源、文件、socket、特殊内存
// 3. unique_ptr 删除器：类型参与模板参数，两种写法（函数指针、仿函数、lambda）
// 4. shared_ptr 删除器：类型不影响智能指针本身，删除器存在控制块，类型擦除
// 5. 函数指针删除器、仿函数删除器、lambda 删除器的写法与区别
// 6. 管理 C 语言资源：fopen/fclose、malloc/free、socket/close
// 7. 数组删除器、自定义对齐内存、池化内存释放
// 8. 删除器的生命周期、拷贝、移动行为（尤其 unique_ptr）
// 9. 删除器为空状态、空指针释放安全处理
// 10. 工程场景：日志、监控、资源统计、释放前回调
// 11. unique_ptr 与 shared_ptr 删除器核心差异（模板参数 vs 类型擦除）
// 12. 常见错误：删除器类型不匹配、栈对象、重复释放、空删除器调用
//
// 为什么要学？
// - 智能指针不只管理 new/delete，几乎所有资源都能用 RAII 封装
// - 工程中大量 C 库、系统调用、自定义内存池必须靠自定义删除器
// - 不懂删除器等于只会用智能指针皮毛，无法写出真正安全的 RAII 代码
// - 面试高频：删除器原理、unique/shared 删除器区别、lambda 删除器
// - 是实现安全句柄管理、自动关闭文件/套接字/锁的核心技术
//
// 核心重点（必须记住）
// 1. unique_ptr 删除器是**类型一部分**，必须在模板参数指定
// 2. shared_ptr 删除器**不影响类型**，存储在控制块，类型擦除实现
// 3. lambda 是最常用、最简洁的删除器，支持捕获与就地逻辑
// 4. 管理 C 资源必须匹配：fopen→fclose、malloc→free、socket→close
// 5. 删除器必须保证：空指针安全、重复释放安全、不抛异常
// 6. 数组用 default_delete<T[]>，自定义数组也要匹配 delete[]
// 7. 不要用自定义删除器管理栈对象，会导致未定义行为
// 8. shared_ptr 可随时更换删除器，unique_ptr 删除器随指针移动/交换
// ============================================================

#include <iostream>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// 演示类
class Resource {
private:
  int m_id;
  const char* m_name;
public:
  Resource(int id, const char* name) : m_id(id), m_name(name) {
    std::cout << "[Resource] 构造: " << m_id << " " << m_name << "\n";
  }
  ~Resource() {
    std::cout << "[Resource] 析构: " << m_id << " " << m_name << "\n";
  }
  void show() const {
    std::cout << "[show] " << m_id << " " << m_name << "\n";
  }
};

//------------------- 1. 默认删除器（default_delete） -------------------
// unique_ptr/shared_ptr 默认使用 std::default_delete，调用 delete/delete[]
void demo_default_deleter() {
  std::cout << "\n===== 1. 默认删除器 =====\n";
  std::unique_ptr<Resource> u1 = std::make_unique<Resource>(1, "default_delete");
  std::shared_ptr<Resource> s1 = std::make_shared<Resource>(2, "default_delete");
}

//------------------- 2. 函数指针删除器 -------------------
// 传统 C 风格函数，适合简单、无状态释放逻辑
void raw_deleter(Resource* p) {
  if (p) {
    std::cout << "[函数删除器] 释放: " << p << "\n";
    delete p;
  }
}

void demo_function_deleter() {
  std::cout << "\n===== 2. 函数指针删除器 =====\n";

  // unique_ptr 必须指定删除器类型（函数指针）
  using UniqueDel = void(*)(Resource*);
  std::unique_ptr<Resource, UniqueDel> u2(new Resource(3, "func deleter"), raw_deleter);

  // shared_ptr 自动推导，无需指定类型
  std::shared_ptr<Resource> s2(new Resource(4, "func deleter"), raw_deleter);
}

//------------------- 3. 仿函数删除器（函数对象） -------------------
// 可携带状态、可拷贝、无额外开销，适合复杂删除逻辑
struct FunctorDeleter {
  const char* m_tag;
  FunctorDeleter(const char* tag = "functor") : m_tag(tag) {}

  void operator()(Resource* p) const {
    if (p) {
      std::cout << "[仿函数删除器] " << m_tag << " 释放: " << p << "\n";
      delete p;
    }
  }
};

void demo_functor_deleter() {
  std::cout << "\n===== 3. 仿函数删除器 =====\n";

  std::unique_ptr<Resource, FunctorDeleter> u3(
    new Resource(5, "functor deleter"), FunctorDeleter("custom_tag"));

  std::shared_ptr<Resource> s3(
    new Resource(6, "functor deleter"), FunctorDeleter());
}

//------------------- 4. Lambda 删除器（最常用、最灵活） -------------------
// 就地定义、可捕获、简洁、类型安全，现代 C++ 首选
void demo_lambda_deleter() {
  std::cout << "\n===== 4. Lambda 删除器 =====\n";

  auto lambda_del = [](Resource* p) {
    if (p) {
      std::cout << "[Lambda 删除器] 释放: " << p << "\n";
      delete p;
    }
  };

  // unique_ptr 需要 decltype 推导 Lambda 类型
  std::unique_ptr<Resource, decltype(lambda_del)> u4(
    new Resource(7, "lambda deleter"), lambda_del);

  std::shared_ptr<Resource> s4(
    new Resource(8, "lambda deleter"), lambda_del);
}

//------------------- 5. 管理 C 语言资源：文件（FILE*） -------------------
// fopen → fclose，典型系统资源 RAII
void demo_c_file() {
  std::cout << "\n===== 5. C 文件资源管理 =====\n";

  auto file_del = [](FILE* f) {
    if (f) {
      std::cout << "[文件删除器] fclose\n";
      fclose(f);
    }
  };

  std::shared_ptr<FILE> file(fopen("test.txt", "w"), file_del);
  if (file) {
    std::cout << "文件已打开，自动关闭\n";
  }
}

//------------------- 6. 管理 malloc / free 内存 -------------------
void demo_c_malloc() {
  std::cout << "\n===== 6. malloc/free 管理 =====\n";

  auto malloc_del = [](void* p) {
    if (p) {
      std::cout << "[malloc 删除器] free\n";
      free(p);
    }
  };

  std::shared_ptr<void> mem(malloc(1024), malloc_del);
}

//------------------- 7. 数组自定义删除器 -------------------
// 必须匹配 delete[]，unique_ptr<T[]> 默认调用 delete[]
void demo_array_deleter() {
  std::cout << "\n===== 7. 数组删除器 =====\n";

  auto arr_del = [](Resource* p) {
    std::cout << "[数组删除器] delete[]\n";
    delete[] p;
  };

  std::unique_ptr<Resource[], decltype(arr_del)> u_arr(
    new Resource[3]{Resource(10,"arr"),Resource(11,"arr"),Resource(12,"arr")}, arr_del);
}

//------------------- 8. unique_ptr 删除器 vs shared_ptr 删除器 -------------------
// 核心差异：类型是否包含删除器 + 存储位置
void demo_deleter_difference() {
  std::cout << "\n===== 8. 两种智能指针删除器差异 =====\n";
  std::cout << "- unique_ptr: 删除器是类型一部分，存储在指针内，无额外开销\n";
  std::cout << "- shared_ptr: 删除器存在控制块，类型擦除，同类型可装不同删除器\n";

  auto delA = [](int* p) { delete p; };
  auto delB = [](int* p) { delete p; };

  std::shared_ptr<int> sA(new int, delA);
  std::shared_ptr<int> sB(new int, delB);
  sA = sB; // 合法：类型相同

  // std::unique_ptr<int, decltype(delA)> uA(new int, delA);
  // std::unique_ptr<int, decltype(delB)> uB(new int, delB);
  // uA = uB; // 错误：类型不同
}

//------------------- 9. 释放前附加逻辑：日志、统计、检查 -------------------
void demo_extra_logic() {
  std::cout << "\n===== 9. 删除器附加业务逻辑 =====\n";

  auto log_del = [](Resource* p) {
    std::cout << "[日志删除器] 即将释放资源\n";
    delete p;
    std::cout << "[日志删除器] 资源已释放\n";
  };

  std::shared_ptr<Resource> s_log(new Resource(99, "with log"), log_del);
}

//------------------- 10. 常见错误与禁止行为 -------------------
void demo_mistakes() {
  std::cout << "\n===== 10. 常见错误 =====\n";
  std::cout << "- 不要用删除器释放栈对象 → 未定义行为\n";
  std::cout << "- 不要让删除器抛出异常 → 析构中抛异常会终止程序\n";
  std::cout << "- unique_ptr 必须匹配删除器类型，否则编译失败\n";
  std::cout << "- 避免空删除器调用，必须判空\n";
}

//================================================================
int main() {
  std::cout << "========== 自定义删除器 完整学习 ==========\n";

  demo_default_deleter();
  demo_function_deleter();
  demo_functor_deleter();
  demo_lambda_deleter();
  demo_c_file();
  demo_c_malloc();
  demo_array_deleter();
  demo_deleter_difference();
  demo_extra_logic();
  demo_mistakes();

  std::cout << "\n========== 演示结束 ==========\n";
  return 0;
}

