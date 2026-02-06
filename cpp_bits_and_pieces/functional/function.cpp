/**
 * C++ std::function 使用说明
 * 来源：【C++ function使用说明-哔哩哔哩】https://b23.tv/I0qCdYA
 * 功能：通用可调用对象包装器，可包装函数、lambda、函数对象、成员函数等
 * 核心作用：统一可调用对象类型、存储回调、延迟执行、适配函数签名
 */

#include <iostream>
#include <functional> // std::function

// ------------------------------
// 1. 包装普通函数
// ------------------------------
int add(int a, int b) {
  return a + b;
}

void test_normal_func() {
  std::cout << "\n--- 1. 包装普通函数 ---" << std::endl;

  // 定义 function：返回 int，接收两个 int
  std::function<int(int, int)> func;

  // 赋值普通函数
  func = add;
  std::cout << "add(10, 20) = " << func(10, 20) << std::endl;
}

// ------------------------------
// 2. 包装 lambda 表达式
// ------------------------------
void test_lambda() {
  std::cout << "\n--- 2. 包装 lambda 表达式 ---" << std::endl;

  std::function<int(int, int)> func;

  // 赋值 lambda
  func = [](int a, int b) { return a * b; };
  std::cout << "lambda(10, 20) = " << func(10, 20) << std::endl;
}

// ------------------------------
// 3. 包装函数对象（仿函数）
// ------------------------------
struct Multiply {
  int operator()(int a, int b) {
    return a * b;
  }
};

void test_functor() {
  std::cout << "\n--- 3. 包装函数对象 ---" << std::endl;

  std::function<int(int, int)> func;

  // 赋值函数对象
  func = Multiply();
  std::cout << "Multiply()(10, 20) = " << func(10, 20) << std::endl;
}

// ------------------------------
// 4. 包装类成员函数
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
  std::cout << "\n--- 4. 包装类成员函数 ---" << std::endl;

  Test t;

  // 包装普通成员函数：需要 bind 绑定对象
  // 成员函数真实签名是 void(Test*, int)，比 std::function<void(int)> 多一个 this 参数
   // 所以不能直接赋值，必须用 bind 做两件事：
   // 1. 固定 this：把 &t 绑定到第一个参数位（this 指针）
   // 2. 预留用户参数：用 std::placeholders::_1 占住第二个参数位，表示“调用时再传 int a”
   // bind 后，函数签名变成 void(int)，与 std::function<void(int)> 匹配
  std::function<void(int)> func1 =
    std::bind(&Test::show, &t, std::placeholders::_1);
  func1(100);

  // 包装静态成员函数：直接赋值
  std::function<void(int)> func2 = Test::static_show;
  func2(200);
}

// ------------------------------
// 5. 作为函数参数（回调）
// ------------------------------
void run_task(std::function<void()> task) {
  std::cout << "\n--- 5. 作为函数参数（回调） ---" << std::endl;
  task();
}

void test_callback() {
  run_task([]() {
    std::cout << "我是 lambda 回调任务" << std::endl;
  });
}

// ------------------------------
// 6. 空 function 判断
// ------------------------------
void test_empty() {
  std::cout << "\n--- 6. 空 function 判断 ---" << std::endl;

  std::function<void()> func;

  if (!func) {
    std::cout << "function 为空，不能调用" << std::endl;
  } else {
    func();
  }
}

// ------------------------------
// 7. 存储到容器（vector）
// ------------------------------
void test_container() {
  std::cout << "\n--- 7. 存储到容器 ---" << std::endl;

  std::vector<std::function<void()>> tasks;

  tasks.emplace_back([]() { std::cout << "任务 1" << std::endl; });
  tasks.emplace_back([]() { std::cout << "任务 2" << std::endl; });
  tasks.emplace_back([]() { std::cout << "任务 3" << std::endl; });

  for (auto& task : tasks) {
    task();
  }
}

// ------------------------------
// 主函数：运行所有示例
// ------------------------------
int main() {
  test_normal_func();
  test_lambda();
  test_functor();
  test_member_func();
  test_callback();
  test_empty();
  test_container();

  return 0;
}
