// ==================== 必须记住：make_unique 核心 ====================
// 1. 是创建 std::unique_ptr 的“工厂函数”，C++14 引入
// 2. 安全、简洁，避免直接写 new
// 3. 语法：std::make_unique<T>(构造参数...)
// 4. 是创建 unique_ptr 的首选方式
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
  // C++14 推荐创建方式
  std::unique_ptr<Person> u = std::make_unique<Person>("小花", 19);
  u->show();

  return 0;
}
