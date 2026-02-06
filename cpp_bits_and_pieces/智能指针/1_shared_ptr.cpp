// ==================== 必须记住：shared_ptr 核心 ====================
// 1. 共享所有权：多个指针可以管理同一个对象
// 2. 用引用计数控制生命周期，计数为 0 才析构
// 3. 推荐创建方式：std::make_shared<T>(...)（安全、高效）
// 4. 存在循环引用问题，需配合 weak_ptr 解决
// 5. 比 unique_ptr 重 [因为要维护引用计数，多一个指针，有原子操作开销]
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
  std::shared_ptr<Person> s1 = std::make_shared<Person>("小红", 20);
  s1->show();
  std::cout << "计数: " << s1.use_count() << "\n"; // 1

  // 2. 拷贝共享（计数+1）
  std::shared_ptr<Person> s2 = s1;
  std::cout << "计数: " << s1.use_count() << "\n"; // 2

  // 3. 主动释放（计数-1）
  s1.reset();
  std::cout << "计数: " << s2.use_count() << "\n"; // 1

  return 0;
}
