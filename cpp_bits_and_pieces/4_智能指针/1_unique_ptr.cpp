// ============================================================
// 本节要学什么？
// 1. unique_ptr 是什么：独占式智能指针，同一时间只有一个拥有者
// 2. 核心作用：自动释放堆内存，杜绝泄漏、重复删除、野指针
// 3. 基础用法：创建、访问、作用域自动释放、make_unique
// 4. 移动语义：不可拷贝、只能转移所有权、std::move 的意义
// 5. reset()：手动释放或替换管理对象
// 6. release()：放弃所有权，返回裸指针但不删除
// 7. get()：安全获取内部裸指针，不转移所有权
// 8. 数组特化版本 unique_ptr<T[]>，自动匹配 delete[]
// 9. 自定义删除器：支持文件、C 内存、自定义释放逻辑
// 10. 函数传参、返回值、工厂模式的正确写法
// 11. 常见错误与工程陷阱：重复删除、栈指针、拷贝、野指针
// 12. 底层理解：独占所有权、零成本抽象、无引用计数
//
// 为什么要学？
// - 裸指针极易出现忘记 delete、重复 delete、异常泄漏、野指针
// - unique_ptr 是现代 C++ 最基础、最安全、性能最好的智能指针
// - 竞赛、机器人、工程代码中默认优先使用 unique_ptr
// - 理解它能彻底打通所有权、生命周期、移动语义、资源管理
// - 面试必考：独占语义、移动、删除器、数组、工厂、内存模型
//
// 核心重点（必须记住）
// 1. 独占所有权：同一时刻只能有一个 unique_ptr 管理对象
// 2. 不可拷贝，只能移动：std::move 转移所有权
// 3. 离开作用域自动释放，绝对不会内存泄漏
// 4. make_unique 是最推荐、最安全的创建方式（C++14）
// 5. get() 只获取指针，不转移所有权，绝对不要 delete
// 6. release() 释放所有权但不释放内存，需手动接管
// 7. 数组必须用 unique_ptr<T[]>，自动调用 delete[]
// 8. 自定义删除器会改变 unique_ptr 的类型
// 9. 传参优先使用值传递（移动）或 const&（只读）
// 10. 移动后源对象变为 nullptr，不可再访问
// ============================================================

#include <iostream>
#include <memory>
#include <cstdio>
#include <cstring>

// 演示生命周期：构造、析构、打印
class Demo {
private:
  int m_id;
  const char* m_info;

public:
  Demo(int id, const char* info) : m_id(id), m_info(info) {
    std::cout << "[构造] Demo " << m_id << " : " << m_info << "\n";
  }

  ~Demo() {
    std::cout << "[析构] Demo " << m_id << " : " << m_info << "\n";
  }

  void show() const {
    std::cout << "[show] Demo " << m_id << " : " << m_info << "\n";
  }
};

//----------------------  基础创建与自动释放  -----------------------
// 最核心：unique_ptr 托管堆对象，离开作用域自动 delete
// 不用手动释放，不会泄漏，这是它最核心的价值
void demo_basic() {
  std::cout << "\n===== 基础用法 =====\n";

  // 裸指针（容易忘 delete）
  Demo* raw = new Demo(1, "裸指针，需手动 delete");
  raw->show();

  // unique_ptr：离开作用域自动释放
  std::unique_ptr<Demo> u1(new Demo(2, "unique_ptr 托管"));
  u1->show();
}

//----------------------  make_unique（C++14 推荐）  -----------------------
// 优点：异常安全、代码简洁、不暴露裸指针、不会写错 new/delete
// 工程里 99% 都应该用 make_unique，而不是手动 new
void demo_make_unique() {
  std::cout << "\n===== make_unique =====\n";

  auto u2 = std::make_unique<Demo>(3, "make_unique 创建");
  u2->show();
}

//----------------------  移动语义（不可拷贝）  -----------------------
// 独占指针不能拷贝，否则会有两个指针管同一个对象
// 只能转移所有权：std::move，转移后原指针变为 nullptr
void demo_move() {
  std::cout << "\n===== 移动语义 =====\n";

  auto u3 = std::make_unique<Demo>(4, "移动源对象");

  // 错误！unique_ptr 没有拷贝构造
  // auto u4 = u3;

  // 正确：移动所有权
  auto u4 = std::move(u3);

  if (!u3) {
    std::cout << "u3 已空，所有权移交给 u4\n";
  }
  u4->show();
}

//----------------------  reset：手动释放/替换对象  -----------------------
// reset() 会释放旧对象，并可托管新对象
// 相当于：先 delete，再指向新内容
void demo_reset() {
  std::cout << "\n===== reset =====\n";

  auto u5 = std::make_unique<Demo>(5, "待 reset 对象");
  u5.reset(); // 立即释放

  if (!u5) std::cout << "u5 已被 reset 置空\n";

  // 重新托管新对象
  u5.reset(new Demo(6, "reset 后的新对象"));
  u5->show();
}

//----------------------  release：释放所有权不删除  -----------------------
// release() 只交出所有权，不 delete 对象
// 返回裸指针，需要手动接管，否则内存泄漏
void demo_release() {
  std::cout << "\n===== release =====\n";

  auto u6 = std::make_unique<Demo>(7, "待 release 对象");
  Demo* ptr = u6.release();

  if (!u6) std::cout << "u6 已空，ptr 手动接管\n";

  // 必须手动 delete
  delete ptr;
}

//----------------------  get：获取裸指针（不转移）  -----------------------
// get() 仅返回内部裸指针，不改变所有权
// 绝对不能 delete get() 出来的指针，会导致重复释放
void demo_get() {
  std::cout << "\n===== get =====\n";

  auto u7 = std::make_unique<Demo>(8, "get 获取裸指针");
  Demo* ptr = u7.get();

  if (ptr) ptr->show();
}

//----------------------  数组 unique_ptr<T[]>  -----------------------
// 管理数组必须用 T[] 版本，自动调用 delete[]
// 普通 unique_ptr 管理数组是未定义行为
void demo_array() {
  std::cout << "\n===== 数组管理 =====\n";

  std::unique_ptr<Demo[]> u_arr(new Demo[2]{
    Demo(9, "数组元素1"),
    Demo(10, "数组元素2")
  });

  u_arr[0].show();
  u_arr[1].show();
}

//----------------------  自定义删除器  -----------------------
// 用于文件、C 语言资源、特殊内存、自定义释放逻辑
// 如 fopen 需要 fclose，malloc 需要 free
void demo_custom_deleter() {
  std::cout << "\n===== 自定义删除器 =====\n";

  // 文件指针：自动 fclose
  using FilePtr = std::unique_ptr<FILE, int(*)(FILE*)>;
  FilePtr f(std::fopen("test.txt", "w"), std::fclose);

  if (f) std::cout << "文件已打开，离开作用域自动关闭\n";

  // Lambda 删除器
  auto del = [](Demo* p) {
    std::cout << "自定义释放：";
    delete p;
  };

  std::unique_ptr<Demo, decltype(del)> u_del(new Demo(11, "Lambda 删除器"), del);
}

//----------------------  函数传参与返回值  -----------------------
// 工厂函数返回 unique_ptr，函数传参用移动语义
// 外部传参必须 std::move
std::unique_ptr<Demo> create_demo(int id, const char* info) {
  return std::make_unique<Demo>(id, info);
}

void use_demo(std::unique_ptr<Demo> p) {
  if (p) p->show();
}

void demo_func() {
  std::cout << "\n===== 函数传参与返回 =====\n";

  auto u10 = create_demo(12, "工厂创建");
  use_demo(std::move(u10));

  if (!u10) std::cout << "u10 已移动，为空\n";
}

//----------------------  常见错误（禁止行为）  -----------------------
// 1. 同一裸指针构造多个 unique_ptr → 重复释放崩溃
// 2. 栈对象地址交给 unique_ptr → 崩溃
// 3. delete get() → 重复释放
// 4. 拷贝 unique_ptr → 编译错误
void demo_mistakes() {
  std::cout << "\n===== 常见错误 =====\n";

  Demo* raw = new Demo(13, "错误示例");
  std::unique_ptr<Demo> bad1(raw);

  // 错误示例（解开注释会崩溃）
  // std::unique_ptr<Demo> bad2(raw);
  // delete bad1.get();
  // Demo stack_obj(14, "栈对象");
  // std::unique_ptr<Demo> bad_stack(&stack_obj);
}

//================================================================
int main() {
  std::cout << "========== unique_ptr 完整学习 ==========\n";

  demo_basic();
  demo_make_unique();
  demo_move();
  demo_reset();
  demo_release();
  demo_get();
  demo_array();
  demo_custom_deleter();
  demo_func();
  demo_mistakes();

  std::cout << "\n========== 全部演示完成 ==========\n";
  return 0;
}

