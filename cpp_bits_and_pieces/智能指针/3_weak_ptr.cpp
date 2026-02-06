// ==================== 必须记住：weak_ptr 核心 ====================
// 1. 不拥有对象，不影响引用计数
// 2. 专门解决 shared_ptr 循环引用导致的内存泄漏
// 3. 必须从 shared_ptr 构造
// 4. 使用前必须 lock() 转为 shared_ptr，否则可能访问已释放对象
// 5. 常用场景：观察者模式、缓存、弱引用关系
// ==================================================================

#include <iostream>
#include <memory>
#include <string>

class Person {
public:
  Person(std::string name) : name(name) {
    std::cout << "构造: " << name << "\n";
  }
  ~Person() {
    std::cout << "析构: " << name << "\n";
  }
  void show() const {
    std::cout << "我是: " << name << "\n";
  }
private:
  std::string name;
};

int main() {
  std::shared_ptr<Person> s = std::make_shared<Person>("小刚");
  std::cout << "计数: " << s.use_count() << "\n"; // 1

  // 1. 从 shared_ptr 构造 weak_ptr
  std::weak_ptr<Person> w = s;
  std::cout << "weak_ptr 不影响计数: " << s.use_count() << "\n"; // 1

  // 2. 使用前必须 lock()
  if (auto temp = w.lock()) {
    temp->show();
    std::cout << "lock 后计数: " << temp.use_count() << "\n"; // 2
  }

  // 3. 释放 shared_ptr
  s.reset();
  std::cout << "shared_ptr 已释放\n";

  // 4. weak_ptr 已过期
  if (w.expired()) {
    std::cout << "weak_ptr 已过期\n";
  }

  return 0;
}
