#define HIDE_CODE 0
#include <iostream>
/** 分类         名称          写法             参数
 * 构造函数   默认构造函数   A()
 *           拷贝构造函数   A(const A& other)              [常量引用]
 *                        A(A& other)                    [类的引用]
 *           移动构造函数   A(A&& other)                   [右值引用]
 * 赋值运算符  拷贝赋值运算符 A& operator=(const A& other)   [常量引用]
 *                        A& operator=(A& other)         [类的引用]
 *           移动赋值运算符 A& operator=(A&& other)        [右值引用]
 * 析构函数                ~A()
 */



//默认构造函数
class A {
public:
  A();     /**< 默认构造函数,没有参数的构造函数 */
};

//如果类中没有定义任何构造函数,则编译器会自动生成一个默认构造函数
class A {
public:
  int m_a = 0;
};

class A {
public:
  A(){ }; /**< 自动生成默认构造函数 */
public:
  int m_a = 0;
};

#if HIDE_CODE
A a;
A* pObj = new A();   /**< 动态创造一个A类对象,返回指针给pObj */
#endif

//如果不想让外部调用默认构造函数:
class A {
private:   
  A();      /**< 设置默认构造函数为私有成员 */
public:
  int m_a = 0;
};

class A {
public:
  A() = delete; /**< 删除默认构造函数.C++11后 */
public:
  int m_a = 0;
};

//---------------------------------------------------------------------------
// delete 
bool isEven(int num) {
  return (num& 1)^1;
}

bool r = isEven(2.31);
std::cout << r << std::endl;
// 输出: true [2.31会隐式转换为int]

//可使用delete删除不必要的传参类型
bool isEven(int num) {
  return (num& 1)^1;
}
bool isEven(double num) = delete;

//---------------------------------------------------------------------------
// default
class A {
public:
  A(int a) : m_a(a) {}
public:
  int m_a = 0;
};
// 在一个类中,如果定义了普通构造函数,则会自动删除默认构造函数
// 这时,A a 会报错
class A {
public:
  A(int a) : m_a(a) {}
  A = default; /**< 手动添加默认构造函数.C++11后 */
public:
  int m_a = 0;
};































































