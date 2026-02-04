// 在C++中,存在大量拷贝行为
// 1.左值 持久对象拷贝
// 2.右值 临时对象拷贝
// 3.将亡对象 拷贝

// 其中,2,3是可以优化的. 左值可能之后还会用到,移动后会出问题(eg:重复调用析构函数)
// 右值 和 将亡对象[要std::move(obj)转化为右值] 都可以用移动构造(浅拷贝)优化

// 所以 左值 深拷贝, 右值 浅拷贝 的前提:要准确判断是左值还是右值!

// C++11 前  
// Demo& 是左值引用
// const Demo& 是万能引用
// 没办法区分 左值 右值

// C++11 后
// 增加了右值引用 Demo&&

// 下面是相关代码
#include <iostream>

class Demo {
public:
  Demo(int a) {
    std::cout << "构造函数" << std::endl;
  }
  // 既能匹配组织对象又能匹配右值对象,即无法区分左值右值
  // 也就无法实现分别处理到底是浅拷贝还是深拷贝
  Demo(const Demo& other) {
    std::cout << "拷贝构造" << std::endl;
  }
  Demo& operator= (const Demo& other) {
    std::cout << "拷贝赋值" << std::endl;
    return *this;
  }
  ~Demo() {
    std::cout << "析构函数" << std::endl;
  }
public:
  int* p_arr;   /**< 指向动态分配数组的指针 */
  int m_size;   /**< 指向动态分配数组的大小 */
};

void fun(Demo&  l_val) { /* 处理左值*/ }
void fun(Demo&& r_val) { /* 处理右值*/ }

void test() {
  // 左值引用
  Demo demo(10);
  Demo& r = demo;
  //常量引用(万能引用)  当函数进行重载时,不可区分左值右值
  const Demo& r2 = demo;
  const Demo& r3 = Demo(10);
  // 右值引用
  Demo&& r4 = Demo(10);
}

int main() {
  test();
}

// yinkangan@Linux:~/Desktop/Vision/cpp_bits_and_pieces/移动语义$ ./_2右值引用 
// 构造函数
// 构造函数
// 构造函数
// 析构函数
// 析构函数
// 析构函数


































































































































































































































































































































































































































































































































