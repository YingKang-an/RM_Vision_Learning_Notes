/**
 * C++ std::bind 使用介绍
 * 来源：【C++ std::bind使用介绍-哔哩哔哩】https://b23.tv/sJ6Mg5e
 * 功能：将可调用对象与参数绑定，生成新的可调用对象
 * 核心作用：绑定参数、适配函数签名、绑定成员函数
 */

#include <iostream>
#include <functional> // std::bind, std::placeholders

// ------------------------------
// 1. 普通函数绑定
// ------------------------------
void print(int a, int b) {
  std::cout << "a = " << a << ", b = " << b << std::endl;
}

void test_normal_func() {
  std::cout << "\n--- 1. 普通函数绑定 ---" << std::endl;

  // 绑定：a=10，b 用占位符 _1
  auto f1 = std::bind(print, 10, std::placeholders::_1);
  f1(20); // 等价 print(10,20)

  // 绑定：a 用 _1，b 用 _2
  auto f2 = std::bind(print, std::placeholders::_1, std::placeholders::_2);
  f2(1, 2); // print(1,2)

  // 绑定：a 用 _2，b 用 _1（调换参数顺序）
  auto f3 = std::bind(print, std::placeholders::_2, std::placeholders::_1);
  f3(1, 2); // print(2,1)
}

// ------------------------------
// 2. 函数对象绑定
// ------------------------------
struct Add {
  void operator()(int a, int b) {
    std::cout << "a + b = " << a + b << std::endl;
  }
};

void test_functor() {
  std::cout << "\n--- 2. 函数对象绑定 ---" << std::endl;

  Add add;
  auto f = std::bind(add, 10, 20);
  f(); // 调用 add(10,20)
}

// ------------------------------
// 3. 绑定类成员函数
// ------------------------------
class Test {
public:
  void show(int x) {
    std::cout << "Test::show x = " << x << std::endl;
  }

  static void static_show(int x) {
    std::cout << "Test::static_show x = " << x << std::endl;
  }
};

void test_member_func() {
  std::cout << "\n--- 3. 绑定类成员函数 ---" << std::endl;

  Test t;

  // 绑定普通成员函数：必须传对象地址/引用
  auto f1 = std::bind(&Test::show, &t, std::placeholders::_1);
  f1(100);

  // 绑定静态成员函数：不需要对象
  auto f2 = std::bind(&Test::static_show, std::placeholders::_1);
  f2(200);
}

// ------------------------------
// 4. 绑定到 std::function
// ------------------------------
void test_bind_to_function() {
  std::cout << "\n--- 4. 绑定到 std::function ---" << std::endl;

  // function 需要无参函数
  std::function<void()> func;

  // 用 bind 绑定参数，适配成无参
  func = std::bind(print, 10, 20);
  func();
}

// ------------------------------
// 5. 绑定引用参数
// ------------------------------
void increment(int& a) {
  a++;
}

void test_ref() {
  std::cout << "\n--- 5. 绑定引用参数 ---" << std::endl;

  int x = 10;

  // 错误：bind 默认拷贝参数，不会传引用
  // auto f = std::bind(increment, x);

  // 正确：用 std::ref 传引用
  auto f = std::bind(increment, std::ref(x));
  f();
  std::cout << "x = " << x << std::endl; // 11
}

// ------------------------------
// 主函数：运行所有示例
// ------------------------------
int main() {
  test_normal_func();
  test_functor();
  test_member_func();
  test_bind_to_function();
  test_ref();

  return 0;
}
