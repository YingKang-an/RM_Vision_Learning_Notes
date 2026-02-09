// ============================================================
// 本节要学什么？
// 1. 什么是 shared_ptr 循环引用：两个或多个对象互相强引用对方
// 2. 循环引用为什么会造成内存泄漏：引用计数永远无法减到 0
// 3. 典型场景：链表节点、树节点、双向引用、观察者模式
// 4. 泄漏现象：对象永远不析构，资源永久占用
// 5. 根本原因：强引用形成环，生命周期互相依赖、无法退出
// 6. 标准解决方案：把至少一方的 shared_ptr 改为 weak_ptr
// 7. 改造前后对比、计数变化、析构行为验证
// 8. 工程常见场景：双向链表、父子对象、事件回调、缓存系统
// 9. 多线程下的循环引用与失效安全
// 10. 如何快速识别、排查、避免循环引用
//
// 为什么要学？
// - 只要用 shared_ptr，几乎一定会遇到循环引用，是最隐蔽的泄漏源
// - 这种bug不崩溃、不报错，长期运行才暴露出内存暴涨
// - 面试必考：循环引用成因、危害、weak_ptr 解决原理
// - 写链表、树、图、双向依赖、观察者模式必须掌握
// - 不懂循环引用 = 写不出安全的现代C++资源管理代码
//
// 核心重点（必须记住）
// 1. 循环引用 = 强引用形成闭环 → use_count 永远 ≥1 → 不析构 → 泄漏
// 2. 解决方法：打破环，将**至少一方**改为 weak_ptr（弱引用，不增加强计数）
// 3. weak_ptr 不拥有对象、不影响生命周期，只安全观察
// 4. 改造后强计数可正常减到0，对象正常析构
// 5. 访问弱引用必须用 lock()，判断是否有效再使用
// 6. 优先设计单向依赖，减少双向强引用，从源头避免循环
// ============================================================

#include <iostream>
#include <memory>

// 演示用基础节点
struct Node {
  int m_id;
  std::shared_ptr<Node> m_next;

  Node(int id) : m_id(id) {
    std::cout << "[构造] Node " << m_id << "\n";
  }

  ~Node() {
    std::cout << "[析构] Node " << m_id << " （若没出现就是泄漏）\n";
  }
};

//------------------- 错误版本：循环引用 + 内存泄漏 -------------------
// 两个节点互相强引用 → 计数永远 >=1 → 永不析构
void demo_cycle_bad() {
  std::cout << "\n===== 【错误】循环引用（泄漏）=====\n";

  auto a = std::make_shared<Node>(10);
  auto b = std::make_shared<Node>(20);

  // 互相强引用，形成环
  a->m_next = b;
  b->m_next = a;

  std::cout << "a use_count: " << a.use_count() << "\n";
  std::cout << "b use_count: " << b.use_count() << "\n";

  // 离开作用域：a、b 局部指针销毁，但对象强引用仍为1，不释放
}

//------------------- 修复版本：一方改用 weak_ptr 打破循环 -------------------
struct NodeFixed {
  int m_id;
  std::shared_ptr<NodeFixed> m_next;
  std::weak_ptr<NodeFixed> m_prev; // 弱引用，不增加强计数

  NodeFixed(int id) : m_id(id) {
    std::cout << "[构造] NodeFixed " << m_id << "\n";
  }

  ~NodeFixed() {
    std::cout << "[析构] NodeFixed " << m_id << " （正常释放）\n";
  }
};

void demo_cycle_fixed() {
  std::cout << "\n===== 【修复】weak_ptr 打破循环 =====" << std::endl;

  auto a = std::make_shared<NodeFixed>(1);
  auto b = std::make_shared<NodeFixed>(2);

  // 单向强引用，反向用弱引用
  a->m_next = b;
  b->m_prev = a; // 弱引用，不强占生命周期

  std::cout << "a use_count: " << a.use_count() << "\n";
  std::cout << "b use_count: " << b.use_count() << "\n";

  // 离开作用域：强计数正常减到0，全部析构
}

//------------------- 更复杂：双向链表典型循环引用 -------------------
struct ListNode {
  int m_val;
  std::shared_ptr<ListNode> m_next;
  std::shared_ptr<ListNode> m_prev; // 错误：双向强引用 → 循环

  ListNode(int val) : m_val(val) {
    std::cout << "ListNode " << m_val << " 构造\n";
  }
  ~ListNode() {
    std::cout << "ListNode " << m_val << " 析构\n";
  }
};

void demo_list_bad() {
  std::cout << "\n===== 双向链表强引用（泄漏）=====\n";

  auto n1 = std::make_shared<ListNode>(100);
  auto n2 = std::make_shared<ListNode>(200);

  n1->m_next = n2;
  n2->m_prev = n1; // 循环形成
}

//------------------- 链表修复：prev 改为 weak_ptr -------------------
struct ListNodeFixed {
  int m_val;
  std::shared_ptr<ListNodeFixed> m_next;
  std::weak_ptr<ListNodeFixed> m_prev; // 修复点

  ListNodeFixed(int val) : m_val(val) {
    std::cout << "ListNodeFixed " << m_val << " 构造\n";
  }
  ~ListNodeFixed() {
    std::cout << "ListNodeFixed " << m_val << " 析构\n";
  }
};

void demo_list_fixed() {
  std::cout << "\n===== 链表修复：弱引用前驱 =====" << std::endl;

  auto n1 = std::make_shared<ListNodeFixed>(101);
  auto n2 = std::make_shared<ListNodeFixed>(201);

  n1->m_next = n2;
  n2->m_prev = n1;
}

//------------------- 工程最佳实践：如何避免循环引用 -------------------
void best_practices() {
  std::cout << "\n===== 避免循环引用：工程原则 =====\n";
  std::cout << "1. 优先设计单向依赖，少用双向强引用\n";
  std::cout << "2. 父→子用 shared_ptr，子→父用 weak_ptr\n";
  std::cout << "3. 观察者模式：观察者持被观察者用 weak_ptr\n";
  std::cout << "4. 缓存系统：缓存存 weak_ptr，外部用 shared_ptr\n";
  std::cout << "5. 访问弱引用必须 lock()，判断有效再使用\n";
  std::cout << "6. 定期检查 use_count()，定位异常滞留对象\n";
}

//------------------- 安全访问 weak_ptr 范式（必须这么写） -------------------
void access_weak(std::weak_ptr<NodeFixed> wp) {
  if (auto sp = wp.lock(); sp) {
    std::cout << "安全访问 NodeFixed: " << sp->m_id << "\n";
  } else {
    std::cout << "对象已失效，无法访问\n";
  }
}

//================================================================
int main() {
  std::cout << "========== 循环引用问题完整演示 ==========\n";

  demo_cycle_bad();     // 泄漏版（无析构）
  demo_cycle_fixed();   // 修复版（正常析构）
  demo_list_bad();      // 链表泄漏
  demo_list_fixed();    // 链表修复
  best_practices();     // 工程规范

  std::cout << "\n========== 演示结束 ==========\n";
  return 0;
}

