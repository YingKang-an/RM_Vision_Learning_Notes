// ============================================================
// 本节要学什么？
// 1. 什么是 enable_shared_from_this：允许类内部安全获取 this 的 shared_ptr
// 2. 为什么必须用它：直接用 this 构造 shared_ptr 会产生多个控制块 → 重复释放崩溃
// 3. 底层原理：对象内置弱指针，预先绑定到控制块，避免重复创建引用计数
// 4. 正确用法：public 继承、调用 shared_from_this() 获取 shared_ptr<自身>
// 5. 错误用法：栈对象、未被 shared_ptr 管理时调用 shared_from_this() → 未定义行为
// 6. 构造函数中不能调用 shared_from_this()：此时控制块还未建立
// 7. 适用场景：回调注册、异步任务、事件分发、需要传递自身智能指针的场景
// 8. 与裸指针 this 的区别：安全、生命周期可控、不破坏引用计数
// 9. 多线程下的安全性：lock() 保证原子获取有效指针
// 10. 常见崩溃场景与规避方法
// 11. 继承体系下的 enable_shared_from_this 注意事项
//
// 为什么要学？
// - 只要在类内部需要传递自身的 shared_ptr，就必须用它，否则必崩
// - 手写回调、异步任务、观察者、组件系统几乎都离不开
// - 直接用 this 构造 shared_ptr 是 C++ 智能指针最经典、最隐蔽的崩溃点
// - 面试必考：原理、崩溃原因、正确使用时机、构造函数限制
// - 是写出稳定、安全、可维护的共享对象代码的必备技术
//
// 核心重点（必须记住）
// 1. 类必须 public 继承 std::enable_shared_from_this<自身类名>
// 2. 只能在对象**已经被 shared_ptr 托管**后调用 shared_from_this()
// 3. 禁止在构造函数中调用：控制块未绑定，直接崩溃
// 4. 禁止栈对象使用：没有控制块，调用即未定义行为
// 5. shared_from_this() 返回的是与外部共享同一控制块的安全强引用
// 6. 底层是内置 weak_ptr，不额外增加强引用，安全不泄漏
// 7. 多线程获取自身指针必须用 shared_from_this()，绝对不能用 this 直接构造
// ============================================================

#include <iostream>
#include <memory>

//------------------- 正确示例：public 继承 enable_shared_from_this -------------------
class MyClass : public std::enable_shared_from_this<MyClass> {
private:
  int m_id;

public:
  MyClass(int id) : m_id(id) {
    std::cout << "[构造] MyClass " << m_id << "\n";
    // 绝对不能在这里调用 shared_from_this()
  }

  ~MyClass() {
    std::cout << "[析构] MyClass " << m_id << "\n";
  }

  void show() const {
    std::cout << "[show] MyClass " << m_id << "\n";
  }

  // 安全获取 this 的 shared_ptr
  std::shared_ptr<MyClass> get_ptr() {
    return shared_from_this();
  }
};

//------------------- 错误示例：直接用 this 构造 shared_ptr（崩溃根源） -------------------
void demo_bad_this() {
  std::cout << "\n===== 【错误】this 直接构造 shared_ptr（会崩溃）=====\n";

  MyClass* raw = new MyClass(999);

  // 两个 shared_ptr 用同一个 this，但**控制块独立** → 引用计数各自为1
  std::shared_ptr<MyClass> s1(raw);
  // std::shared_ptr<MyClass> s2(raw); // 解开注释：离开作用域重复释放 → 崩溃

  std::cout << "s1 use_count: " << s1.use_count() << "\n";
  // std::cout << "s2 use_count: " << s2.use_count() << "\n";
}

//------------------- 正确用法演示 -------------------
void demo_correct() {
  std::cout << "\n===== 【正确】enable_shared_from_this =====\n";

  // 必须用 shared_ptr 托管堆对象
  std::shared_ptr<MyClass> s = std::make_shared<MyClass>(100);
  s->show();

  // 安全获取自身指针，共享同一控制块
  std::shared_ptr<MyClass> s2 = s->get_ptr();

  std::cout << "s  use_count: " << s.use_count() << "\n";
  std::cout << "s2 use_count: " << s2.use_count() << "\n";

  s2->show();
}

//------------------- 错误：栈对象调用 shared_from_this()（未定义行为） -------------------
void demo_stack_obj_bad() {
  std::cout << "\n===== 【错误】栈对象使用（禁止）=====\n";

  MyClass stack_obj(200);
  // auto s = stack_obj.get_ptr(); // 崩溃！栈对象无控制块
}

//------------------- 错误：裸指针直接调用（未被 shared_ptr 管理） -------------------
void demo_unmanaged_bad() {
  std::cout << "\n===== 【错误】未托管对象调用（禁止）=====\n";

  MyClass* raw = new MyClass(300);
  // auto s = raw->get_ptr(); // 崩溃！没有 shared_ptr 控制块
  std::shared_ptr<MyClass> safe(raw); // 托管后才能调用
}

//------------------- 典型工程场景：回调/异步任务传递自身 -------------------
// 模拟异步任务需要持有自身智能指针
struct Task : public std::enable_shared_from_this<Task> {
  int m_task_id;

  Task(int id) : m_task_id(id) {}

  void start_async() {
    // 安全传递自身给异步逻辑
    std::shared_ptr<Task> self = shared_from_this();

    std::cout << "异步任务启动，持有自身指针: use_count = " << self.use_count() << "\n";

    // 模拟异步：self 延长生命周期，保证任务结束前对象不被销毁
  }
};

void demo_async_use_case() {
  std::cout << "\n===== 工程场景：异步任务安全持有自身 =====\n";

  auto t = std::make_shared<Task>(1001);
  t->start_async();
  std::cout << "task use_count: " << t.use_count() << "\n";
}

//------------------- 继承体系注意事项 -------------------
// 基类继承，子类自动生效
class Base : public std::enable_shared_from_this<Base> {
public:
  virtual ~Base() = default;
  void base_func() {}
};

class Derived : public Base {
public:
  void derived_func() {}
};

void demo_inheritance() {
  std::cout << "\n===== 继承体系下的使用 =====\n";

  std::shared_ptr<Derived> d = std::make_shared<Derived>();
  std::shared_ptr<Base> b = d->shared_from_this(); // 基类也安全

  std::cout << "Base use_count: " << b.use_count() << "\n";
}

//------------------- 常见崩溃总结 -------------------
void demo_common_crashes() {
  std::cout << "\n===== 所有崩溃场景总结 =====\n";
  std::cout << "1. 栈对象调用 shared_from_this()\n";
  std::cout << "2. 裸指针未被 shared_ptr 托管就调用\n";
  std::cout << "3. 构造函数中调用 shared_from_this()\n";
  std::cout << "4. 直接用 this 构造多个 shared_ptr（多控制块）\n";
  std::cout << "5. 多次重复获取但不理解引用计数变化\n";
}

//================================================================
int main() {
  std::cout << "========== enable_shared_from_this 完整学习 ==========\n";

  // demo_bad_this();        // 错误（注释掉避免崩溃）
  demo_correct();
  // demo_stack_obj_bad();   // 错误
  // demo_unmanaged_bad();   // 错误
  demo_async_use_case();
  demo_inheritance();
  demo_common_crashes();

  std::cout << "\n========== 演示结束 ==========\n";
  return 0;
}

