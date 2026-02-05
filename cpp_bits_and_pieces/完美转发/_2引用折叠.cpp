#include <iostream>
#include <type_traits>
using namespace std;


struct Person
{
  Person(string&)
  {
    cout << "左值" << endl;
  }

  Person(string&&)
  {
    cout << "右值" << endl;
  }
};

// 思考过程：
// 为了避免拷贝，需要使用引用参数
// 如果写左值引用，则无法匹配右值对象
// 如果写右值引用，则无法匹配左值对象
// 如果写常量引用，则无法匹配 Person 构造函数
// 所以，通过模板推导类型
// 写 T&，就无法匹配右值
// 写 const T& 会给传递进来的参数增加 const 性，也不合理
// 只能写 T&&
// 写 T&& 可以匹配右值，为什么可以匹配左值？
// 这就需要了解 C++11 中的引用折叠

// 函数要能够实现无拷贝，就需要实现对左值和右值的引用
template<class T>
Person make_person(T&& param)
{
  cout << "是否左值:" << is_lvalue_reference<T&&>::value << endl;
  cout << "是否右值:" << is_rvalue_reference<T&&>::value << endl;
  // 如果传递的参数 param 是左值
  return Person(param);
}


void test()
{
  // 1. 传递左值
  string name = "Obama";
  make_person(name);

  cout << "---------" << endl;

  // 2. 传递右值
  make_person(move(name));
}


int main()
{
  test();
  return 0;
}