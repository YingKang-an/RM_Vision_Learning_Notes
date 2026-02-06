// ==================== 必须记住：make_shared 核心 ====================
// 1. 是创建 std::shared_ptr 的“工厂函数”，不是类型
// 2. 一次内存分配（对象 + 引用计数），比 new 高效
// 3. 异常安全，避免内存泄漏
// 4. 语法：std::make_shared<T>(构造参数...)
// 5. 是创建 shared_ptr 的首选方式，永远优先用它
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
  // 最推荐：make_shared 创建 shared_ptr
  std::shared_ptr<Person> p = std::make_shared<Person>("张三", 22);
  p->show();
  std::cout << "计数: " << p.use_count() << "\n"; // 1

  std::shared_ptr<Person> p2 = p;
  std::cout << "计数: " << p.use_count() << "\n"; // 2

  return 0;
}
