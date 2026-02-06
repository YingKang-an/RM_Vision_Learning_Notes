// ==================== 必须记住：unique_ptr 核心 ====================
// 1. 独占所有权：同一时间只有一个指针管理对象
// 2. 不能拷贝，只能移动（std::move）
// 3. 最轻量、最快，无引用计数开销
// 4. C++ 中默认首选的智能指针
// 5. 创建方式：std::make_unique<T>(...)（C++14 推荐）
// ==================================================================

#include <iostream>
#include <memory>
#include <string>

class Person {
public:
  Person(std::string name, int age) : name(name), age(age) {
    std::cout << "构造: " << name << "\n";
  }
  ~Person() {
    std::cout << "析构: " << name << "\n";
  }
  void show() const {
    std::cout << "姓名: " << name << ", 年龄: " << age << "\n";
  }
private:
  std::string name;
  int age;
};

int main() {
  // 1. 推荐创建方式
  std::unique_ptr<Person> u1 = std::make_unique<Person>("小明", 18);
  u1->show();

  // 2. 不能拷贝（编译报错）
  // std::unique_ptr<Person> u2 = u1;

  // 3. 可以移动（转移所有权）
  std::unique_ptr<Person> u2 = std::move(u1);
  if (!u1) {
    std::cout << "u1 已空\n";
  }
  u2->show();

  // 4. 主动释放
  u2.reset();
  std::cout << "u2 已释放\n";

  return 0;
}
