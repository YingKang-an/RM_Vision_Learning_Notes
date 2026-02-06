// ============================================================
// 本节要学什么？
// 1. 多态的概念：静态多态 vs 动态多态
// 2. 虚函数（virtual）的两种形式：普通虚函数 vs 纯虚函数
// 3. 普通虚函数（不带 =0）：可实例化，子类可选重写
// 4. 纯虚函数（带 =0）：不可实例化，子类必须重写
// 5. 虚析构函数：防止内存泄漏的关键
// 6. C++11 新关键字：override / final
// 7. 底层原理：虚表（vtable）与虚指针（vptr）
//
// 为什么要学？
// - 多态是面向对象编程的核心，实现“一个接口，多种实现”
// - 工程中用于解耦、插件化开发、框架设计（如机器人控制模块）
// - 不懂虚函数，就无法理解 C++ 面向对象的精髓
//
// 核心重点（必须记住）：
// 1. 动态多态 = 虚函数 + 继承 + 基类指针/引用指向子类对象
// 2. 纯虚函数所在的类是抽象类，不能实例化
// 3. 基类指针删除子类对象时，基类析构函数必须是 virtual
// 4. override 用于检查重写正确性，final 用于禁止重写/继承
// ============================================================

#include <iostream>

// --- 基类：包含【普通虚函数】和【纯虚函数】---
class Shape {
public:
  // ==========================================================
  // --- 1. 普通虚函数（不带 =0）---
  // 作用：基类提供默认实现，子类可以重写，也可以不重写
  // 特点：包含它的类【可以实例化】
  // ==========================================================
  virtual void commonFunc() const {
    std::cout << "Shape::commonFunc() 基类默认实现" << std::endl;
  }
  // - 解释：
  // - 1. 这是普通虚函数，基类有实现。
  // - 2. 子类可以重写（override）它，也可以直接用基类的版本。
  // - 3. 因为有普通虚函数，所以这个类【不是抽象类】，可以创建对象。

  // ==========================================================
  // --- 2. 纯虚函数（带 =0）---
  // 作用：基类只声明接口，不提供实现，强制子类重写
  // 特点：包含它的类【是抽象类，不能实例化】
  // ==========================================================
  virtual double area() const = 0;
  // - 解释：
  // - 1. 这是纯虚函数，只有声明，没有实现（=0）。
  // - 2. 子类【必须】重写实现这个函数，否则子类也变成抽象类。
  // - 3. 因为有纯虚函数，所以这个类【是抽象类】，不能直接 new Shape()。

  virtual void draw() const = 0;

  // --- 3. 虚析构函数 ---
  virtual ~Shape() {
    std::cout << "Shape 析构" << std::endl;
  }
  // - 解释：
  // - 1. 当基类指针指向子类对象（如 Shape* s = new Circle），delete s 时。
  // - 2. 如果析构函数不是 virtual，只会调用基类析构，子类析构不执行，导致内存泄漏。
  // - 3. 加上 virtual 后，会先调用子类析构，再调用基类析构。
  // - 4. 规则：只要类里有虚函数，析构函数就应该是 virtual。
};

// --- 子类：圆形 ---
class Circle : public Shape {
private:
  double m_radius;

public:
  Circle(double r) : m_radius(r) {}

  // --- 重写纯虚函数（必须重写！）---
  double area() const override {
    return 3.14159 * m_radius * m_radius;
  }
  // - 解释：
  // - 1. override 是 C++11 关键字，显式声明“我要重写父类的虚函数”。
  // - 2. 作用：编译器会检查父类是否真的有这个同名同参的虚函数。
  // - 3. 好处：防止手误写错函数名（如写成 areo），编译器提前报错，避免运行时错误。

  void draw() const override {
    std::cout << "绘制圆形，半径: " << m_radius << std::endl;
  }

  // --- 重写普通虚函数（可选，这里我们重写了）---
  void commonFunc() const override {
    std::cout << "Circle::commonFunc() 子类重写实现" << std::endl;
  }

  ~Circle() override {
    std::cout << "Circle 析构" << std::endl;
  }
};

// --- 子类：矩形 ---
class Rectangle : public Shape {
private:
  double m_w, m_h;

public:
  Rectangle(double w, double h) : m_w(w), m_h(h) {}

  double area() const override {
    return m_w * m_h;
  }

  void draw() const override {
    std::cout << "绘制矩形，宽: " << m_w << " 高: " << m_h << std::endl;
  }

  // --- 不重写普通虚函数（直接继承基类版本）---
  // 这里没有写 commonFunc()，所以会调用 Shape 里的默认实现

  ~Rectangle() override {
    std::cout << "Rectangle 析构" << std::endl;
  }
};

// --- 测试：普通虚函数 vs 纯虚函数 ---
void testVirtualTypes() {
  std::cout << "\n--- 测试：普通虚函数 vs 纯虚函数 ---" << std::endl;

  // 错误演示：Shape 是抽象类（有纯虚函数），不能实例化
  // Shape s; // 编译报错！

  Shape* s1 = new Circle(5);
  Shape* s2 = new Rectangle(4, 6);

  std::cout << "\n--- 调用普通虚函数 ---" << std::endl;
  s1->commonFunc(); // 调用子类重写的版本
  s2->commonFunc(); // 调用基类默认版本（因为 Rectangle 没重写）

  std::cout << "\n--- 调用纯虚函数 ---" << std::endl;
  s1->draw();
  s2->draw();

  // 清理
  delete s1;
  delete s2;
}

// --- 底层原理讲解 ---
void testVTable() {
  std::cout << "\n--- 虚表与虚指针 ---" << std::endl;
  Circle c(1);
  // 64位系统下，大小为 16字节
  // - m_radius (double 8字节) + vptr (虚指针 8字节)
  std::cout << "sizeof(Circle) = " << sizeof(Circle) << std::endl;
  // - 解释：
  // - 1. 只要类有虚函数（普通或纯虚），编译器就会为这个类生成一张“虚表”(vtable)。
  // - 2. 每个对象内部会多一个隐藏的“虚指针”(vptr)，指向类的虚表。
  // - 3. 调用虚函数时，通过 vptr 找到虚表，再查表找到函数地址调用。
  // - 4. 这就是“动态绑定”/“运行时多态”的底层实现。
}

int main() {
  testVirtualTypes();
  testVTable();
  return 0;
}
