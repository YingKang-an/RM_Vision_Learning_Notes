// ============================================================
// 本节要学什么？
// 1. weak_ptr 是什么：弱引用智能指针，不参与引用计数
// 2. 核心作用：解决 shared_ptr 循环引用导致的内存泄漏
// 3. 与 shared_ptr 的关系：共享控制块，只观察、不拥有
// 4. 控制块中的弱计数（weak count）作用与生命周期
// 5. lock()：安全获取有效的 shared_ptr（为空则返回空）
// 6. expired()：快速判断被观察对象是否已销毁
// 7. use_count()：查看当前引用计数，用于调试
// 8. 构造方式：从 shared_ptr / 其他 weak_ptr 构造
// 9. 典型场景：缓存、观察者模式、解决循环引用、非拥有式引用
// 10. 底层原理：不增加强引用、不延长对象生命周期、控制块最后释放
// 11. 常见错误：直接访问对象、忘记 lock、多线程竞争
// 12. weak_ptr 与 unique_ptr 不兼容，只能配合 shared_ptr
//
// 为什么要学？
// - 只要用 shared_ptr，几乎必定遇到循环引用，必须靠 weak_ptr 解决
// - 不理解 weak_ptr 会写出长期隐藏的内存泄漏，极难排查
// - 缓存、观察者、回调、异步任务、图/树结构都大量依赖弱引用
// - 面试高频：循环引用、weak_ptr 作用、lock/expired、线程安全
// - 是现代 C++ 资源管理、生命周期安全的必备工具
//
// 核心重点（必须记住）
// 1. weak_ptr 不拥有对象，不增加强引用计数，不影响生命周期
// 2. 只能观察、不能直接访问对象，必须通过 lock() 转为 shared_ptr
// 3. expired() = true 表示对象已销毁，lock() 会返回空 shared_ptr
// 4. 循环引用：把至少一方的 shared_ptr 改为 weak_ptr 即可解决泄漏
// 5. 控制块在**强计数=0 时释放对象，弱计数=0 时才释放控制块**
// 6. lock() 是原子操作，多线程下安全获取有效指针
// 7. weak_ptr 没有 operator* / operator->，不能直接使用
// 8. 适用场景：非拥有引用、缓存、观察者、回指、打破循环
// ============================================================

#include <iostream>
#include <memory>

// 演示用类，观察构造/析构
class WeakDemo {
private:
  int m_id;
  const char* m_info;

public:
  WeakDemo(int id, const char* info) : m_id(id), m_info(info) {
    std::cout << "[构造] WeakDemo " << m_id << " : " << m_info << "\n";
  }

  ~WeakDemo() {
    std::cout << "[析构] WeakDemo " << m_id << " : " << m_info << "\n";
  }

  void show() const {
    std::cout << "[show] WeakDemo " << m_id << " : " << m_info << "\n";
  }
};

//------------------- weak_ptr 基础用法：构造、expired、lock -------------------
// 最核心：弱引用只观察，不拥有，不增加强引用计数
void demo_basic() {
  std::cout << "\n===== 1. 基础用法 =====\n";

  // 先有 shared_ptr
  std::shared_ptr<WeakDemo> s = std::make_shared<WeakDemo>(1, "shared 源");
  std::cout << "强引用计数: " << s.use_count() << "\n";

  // 从 shared_ptr 构造 weak_ptr：不增加强引用
  std::weak_ptr<WeakDemo> w = s;
  std::cout << "weak_ptr 构造后，强引用: " << s.use_count() << "\n";

  // 判断对象是否存活
  if (!w.expired()) {
    std::cout << "对象未销毁\n";
  }

  // 必须 lock() 转为 shared_ptr 才能安全访问
  std::shared_ptr<WeakDemo> s2 = w.lock();
  if (s2) {
    s2->show();
  }

  // 销毁源 shared_ptr，对象立即释放
  s.reset();
  std::cout << "s reset 后，expired: " << std::boolalpha << w.expired() << "\n";

  // 此时 lock() 返回空
  std::shared_ptr<WeakDemo> s3 = w.lock();
  if (!s3) {
    std::cout << "lock() 返回空，对象已销毁\n";
  }
}

//------------------- 经典问题：shared_ptr 循环引用（内存泄漏） -------------------
// 互相持有强引用 → 计数永远 ≥1 → 永不析构 → 泄漏
struct Node {
  std::shared_ptr<Node> m_next;
  ~Node() { std::cout << "Node 析构\n"; }
};

void demo_cycle_shared() {
  std::cout << "\n===== 2. 循环引用（泄漏版）=====\n";

  auto a = std::make_shared<Node>();
  auto b = std::make_shared<Node>();

  a->m_next = b;
  b->m_next = a;

  std::cout << "a use_count: " << a.use_count() << "\n";
  std::cout << "b use_count: " << b.use_count() << "\n";
  // 离开作用域：a、b 都不会析构 → 内存泄漏
}

//------------------- weak_ptr 解决循环引用（正确版） -------------------
// 把其中一方改为 weak_ptr，打破强引用循环
struct NodeWeak {
  std::shared_ptr<NodeWeak> m_next;
  std::weak_ptr<NodeWeak> m_prev; // 弱引用，不增加强计数

  ~NodeWeak() { std::cout << "NodeWeak 析构\n"; }
};

void demo_cycle_fixed() {
  std::cout << "\n===== 3. weak_ptr 解决循环引用 =====\n";

  auto a = std::make_shared<NodeWeak>();
  auto b = std::make_shared<NodeWeak>();

  a->m_next = b;
  b->m_prev = a; // 弱引用，不增加强计数

  std::cout << "a use_count: " << a.use_count() << "\n";
  std::cout << "b use_count: " << b.use_count() << "\n";

  // 离开作用域：正常析构，无泄漏
}

//------------------- lock() 安全使用范式（工程标准写法） -------------------
// 任何使用 weak_ptr 的地方，必须：lock → 判断非空 → 使用
void use_weak(std::weak_ptr<WeakDemo> w) {
  if (auto s = w.lock(); s) { // C++17 初始化+判断
    s->show();
  } else {
    std::cout << "对象已失效，无法访问\n";
  }
}

void demo_lock_pattern() {
  std::cout << "\n===== 4. lock() 安全范式 =====\n";

  auto s = std::make_shared<WeakDemo>(2, "lock test");
  use_weak(s);

  s.reset();
  use_weak(s);
}

//------------------- expired() 与 use_count() 注意事项 -------------------
// expired() 是原子的，但 use_count() 仅用于调试，多线程不可靠
void demo_expired_use_count() {
  std::cout << "\n===== 5. expired / use_count =====\n";

  auto s = std::make_shared<WeakDemo>(3, "expired test");
  std::weak_ptr<WeakDemo> w = s;

  std::cout << "expired: " << std::boolalpha << w.expired() << "\n";
  std::cout << "use_count: " << w.use_count() << "\n";

  s.reset();
  std::cout << "expired: " << w.expired() << "\n";
}

//------------------- 多线程安全说明 -------------------
// lock() 是原子安全的，是多线程访问对象的唯一安全方式
void demo_thread_safe() {
  std::cout << "\n===== 6. 多线程安全 =====\n";
  std::cout << "- weak_ptr::lock() 是原子操作，多线程安全\n";
  std::cout << "- expired() 也是原子的\n";
  std::cout << "- 但对象本身的读写仍需加锁\n";
  std::cout << "- 绝对不要先 expired() 再 lock()，存在竞争\n";
}

//------------------- 典型工程场景：缓存系统 -------------------
// 缓存持有 weak_ptr，外部使用 shared_ptr，无引用时自动失效
void demo_cache_use_case() {
  std::cout << "\n===== 7. 典型场景：缓存（弱引用）=====\n";

  std::weak_ptr<WeakDemo> cache;

  // 加载数据
  {
    auto data = std::make_shared<WeakDemo>(100, "cached data");
    cache = data;
    std::cout << "缓存已存入\n";
  } // data 离开作用域，对象销毁

  // 尝试从缓存获取
  if (auto data = cache.lock(); data) {
    std::cout << "命中缓存\n";
  } else {
    std::cout << "缓存失效，需要重新加载\n";
  }
}

//------------------- 常见错误（禁止行为） -------------------
// 1. 直接访问对象（weak_ptr 没有 -> / *）
// 2. 先 expired() 再 lock()，多线程下竞争失效
// 3. 用 weak_ptr 管理 unique_ptr（不兼容）
// 4. 依赖 use_count() 做业务逻辑（仅调试用）
void demo_mistakes() {
  std::cout << "\n===== 8. 常见错误 =====\n";

  std::weak_ptr<WeakDemo> w;
  // w->show(); // 错误：编译失败
  // *w = ...;  // 错误

  // 错误写法（多线程不安全）
  if (!w.expired()) {
    // 这里对象可能已被销毁
    auto s = w.lock(); // 仍可能为空
  }

  // 正确写法：只使用 lock()
}

//================================================================
int main() {
  std::cout << "========== weak_ptr 完整学习开始 ==========\n";

  demo_basic();
  demo_cycle_shared();
  demo_cycle_fixed();
  demo_lock_pattern();
  demo_expired_use_count();
  demo_thread_safe();
  demo_cache_use_case();
  demo_mistakes();

  std::cout << "\n========== 全部演示结束 ==========\n";
  return 0;
}

