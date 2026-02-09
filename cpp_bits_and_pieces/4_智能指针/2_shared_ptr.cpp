// ============================================================
// 本节要学什么？
// 1. shared_ptr 是什么：共享所有权智能指针，多个指针管理同一个对象
// 2. 核心原理：引用计数（reference count）+ 控制块（control block）
// 3. 基础用法：创建、赋值、拷贝、移动、自动释放
// 4. make_shared：推荐创建方式，内存合并、效率更高、异常安全
// 5. 引用计数变化：拷贝+1、赋值/析构-1、减到0自动释放对象
// 6. 控制块结构：引用计数、弱计数、删除器、分配器
// 7. 自定义删除器：支持函数、lambda、仿函数、C 风格资源
// 8. 数组支持：shared_ptr<T[]>（C++17）、注意与 unique_ptr 区别
// 9. enable_shared_from_this：获取自身安全的 shared_ptr
// 10. 循环引用：经典陷阱、内存泄漏、weak_ptr 解决思路
// 11. 线程安全：引用计数原子操作，对象本身非线程安全
// 12. 常见错误：裸指针重复构造、栈对象、空指针、循环引用
// 13. 与 unique_ptr 对比：适用场景、性能、开销、语义
// 14. 底层原理：控制块分配、make_shared 内存合并、弱引用机制
//
// 为什么要学？
// - shared_ptr 是 C++ 最常用的共享资源管理工具
// - 解决多个所有者共同管理同一块内存的生命周期问题
// - 面试必考：引用计数、控制块、循环引用、线程安全、make_shared
// - 工程中大量用于缓存、共享资源、多态对象、异步任务、观察者模式
// - 不懂原理会写出泄漏、崩溃、循环引用、性能极差的代码
//
// 核心重点（必须记住）
// 1. shared_ptr = 对象 + 控制块（引用计数+弱计数+删除器）
// 2. 拷贝 → 引用计数+1；析构/重置 → 引用计数-1
// 3. 引用计数减到 0 时，自动释放对象（不一定释放控制块）
// 4. make_shared 合并对象与控制块内存，更高效、更安全
// 5. 不要用同一个裸指针构造多个 shared_ptr，会导致重复释放
// 6. 循环引用 = 引用计数永远不为 0 → 内存泄漏
// 7. 引用计数是原子操作（线程安全），但对象访问不是线程安全
// 8. shared_ptr 有开销：间接访问、控制块、原子操作、不可移动优化
// 9. 优先用 unique_ptr，需要共享时才用 shared_ptr
// 10. enable_shared_from_this 用于类内部获取 this 的安全共享指针
// ============================================================

#include <iostream>
#include <memory>
#include <cstdio>
#include <cstring>

// 演示类：观察构造、析构、生命周期
class SharedDemo {
private:
  int m_id;
  const char* m_info;

public:
  SharedDemo(int id, const char* info) : m_id(id), m_info(info) {
    std::cout << "[构造] SharedDemo " << m_id << " : " << m_info << "\n";
  }

  ~SharedDemo() {
    std::cout << "[析构] SharedDemo " << m_id << " : " << m_info << "\n";
  }

  void show() const {
    std::cout << "[show] SharedDemo " << m_id << " : " << m_info << "\n";
  }

  int id() const { return m_id; }
};

//------------------- 基础用法：创建、拷贝、赋值、释放 -------------------
// 核心：拷贝/赋值会增加引用计数，所有人都释放才真正删除对象
void demo_basic() {
  std::cout << "\n===== 1. 基础用法 =====\n";

  // 裸指针写法（不推荐）
  std::shared_ptr<SharedDemo> s1(new SharedDemo(1, "new 构造"));
  std::cout << "s1 use_count: " << s1.use_count() << "\n";

  // 拷贝：计数+1
  std::shared_ptr<SharedDemo> s2 = s1;
  std::cout << "s2 拷贝后 use_count: " << s1.use_count() << "\n";

  // 赋值：计数+1
  std::shared_ptr<SharedDemo> s3;
  s3 = s1;
  std::cout << "s3 赋值后 use_count: " << s1.use_count() << "\n";

  // s1/s2/s3 都管理同一个对象
  s1->show();
  s2->show();
  s3->show();

  // 局部重置：计数-1
  s1.reset();
  std::cout << "s1 reset 后 use_count: " << s2.use_count() << "\n";

  // 最后一个离开作用域时，计数变为0，释放对象
}

//------------------- make_shared（最推荐、最高效） -------------------
// 合并对象内存 + 控制块为一块内存，减少分配、更高效、异常安全
void demo_make_shared() {
  std::cout << "\n===== 2. make_shared =====\n";

  auto s4 = std::make_shared<SharedDemo>(2, "make_shared");
  std::cout << "use_count: " << s4.use_count() << "\n";
  s4->show();
}

//------------------- 引用计数详细变化 -------------------
// 清晰展示每一步引用计数如何增减
void demo_ref_count() {
  std::cout << "\n===== 3. 引用计数变化 =====\n";

  auto s5 = std::make_shared<SharedDemo>(3, "ref count demo");
  std::cout << "创建后: " << s5.use_count() << "\n";

  auto s6 = s5;
  std::cout << "s6 拷贝: " << s5.use_count() << "\n";

  auto s7 = s5;
  std::cout << "s7 拷贝: " << s5.use_count() << "\n";

  s6.reset();
  std::cout << "s6 reset: " << s5.use_count() << "\n";

  s7.reset();
  std::cout << "s7 reset: " << s5.use_count() << "\n";

  s5.reset();
  std::cout << "s5 reset: 对象已释放\n";
}

//------------------- 自定义删除器 -------------------
// 支持文件、C 内存、自定义释放逻辑，控制块存储删除器
void demo_custom_deleter() {
  std::cout << "\n===== 4. 自定义删除器 =====\n";

  // 文件指针：自动 fclose
  auto file_deleter = [](FILE* f) {
    if (f) {
      std::cout << "自定义删除：关闭文件\n";
      fclose(f);
    }
  };

  std::shared_ptr<FILE> fp(fopen("test.txt", "w"), file_deleter);
  if (fp) {
    std::cout << "文件已打开，离开作用域自动关闭\n";
  }

  // 对象自定义删除器
  auto obj_deleter = [](SharedDemo* p) {
    std::cout << "自定义删除对象: id=" << p->id() << "\n";
    delete p;
  };

  std::shared_ptr<SharedDemo> s8(new SharedDemo(4, "custom deleter"), obj_deleter);
}

//------------------- enable_shared_from_this -------------------
// 类内部安全获取 this 的 shared_ptr，避免裸指针构造重复控制块
class SharedObj : public std::enable_shared_from_this<SharedObj> {
public:
  SharedObj() { std::cout << "SharedObj 构造\n"; }
  ~SharedObj() { std::cout << "SharedObj 析构\n"; }

  std::shared_ptr<SharedObj> get_this() {
    return shared_from_this(); // 安全获取 this 的 shared_ptr
  }
};

void demo_shared_from_this() {
  std::cout << "\n===== 5. enable_shared_from_this =====\n";

  auto obj = std::make_shared<SharedObj>();
  auto another = obj->get_this();

  std::cout << "use_count: " << obj.use_count() << "\n";
}

//------------------- 循环引用（经典陷阱 & 内存泄漏） -------------------
// 互相持有对方 shared_ptr → 引用计数永远 ≥1 → 不释放
struct Node;

struct Node {
  std::shared_ptr<Node> m_next;
  ~Node() { std::cout << "Node 析构\n"; }
};

void demo_cycle_ref() {
  std::cout << "\n===== 6. 循环引用（泄漏示例）=====\n";

  auto a = std::make_shared<Node>();
  auto b = std::make_shared<Node>();

  a->m_next = b;
  b->m_next = a;

  std::cout << "a use_count: " << a.use_count() << "\n";
  std::cout << "b use_count: " << b.use_count() << "\n";

  // 离开作用域：a、b 都不析构 → 内存泄漏
}

//------------------- 线程安全说明 -------------------
// 引用计数是原子操作（线程安全）
// 对象本身的读写不是线程安全，必须加锁
void demo_thread_safety() {
  std::cout << "\n===== 7. 线程安全 =====\n";
  std::cout << "- 引用计数增减是原子操作，多线程拷贝/赋值安全\n";
  std::cout << "- 对象成员读写非线程安全，必须加锁\n";
}

//------------------- shared_ptr 与 unique_ptr 对比 -------------------
// 语义、性能、开销、适用场景
void demo_compare() {
  std::cout << "\n===== 8. shared_ptr vs unique_ptr =====\n";
  std::cout << "- unique_ptr：独占、零开销、可移动、不可拷贝、默认首选\n";
  std::cout << "- shared_ptr：共享、控制块开销、可拷贝、引用计数、需要共享时用\n";
}

//------------------- 常见错误（禁止行为） -------------------
// 1. 裸指针构造多个 shared_ptr → 多个控制块 → 重复释放崩溃
// 2. 栈对象地址构造 shared_ptr → 栈内存非法释放
// 3. 循环引用 → 泄漏
// 4. 混用裸指针与智能指针
void demo_mistakes() {
  std::cout << "\n===== 9. 常见错误 =====\n";

  SharedDemo* raw = new SharedDemo(99, "错误演示");

  std::shared_ptr<SharedDemo> bad1(raw);
  // std::shared_ptr<SharedDemo> bad2(raw); // 错误：重复控制块，崩溃

  // 栈对象（错误）
  SharedDemo stack_obj(100, "stack");
  // std::shared_ptr<SharedDemo> bad_stack(&stack_obj); // 崩溃
}

//================================================================
int main() {
  std::cout << "========== shared_ptr 完整学习开始 ==========\n";

  demo_basic();
  demo_make_shared();
  demo_ref_count();
  demo_custom_deleter();
  demo_shared_from_this();
  demo_cycle_ref();
  demo_thread_safety();
  demo_compare();
  demo_mistakes();

  std::cout << "\n========== 全部演示结束 ==========\n";
  return 0;
}
