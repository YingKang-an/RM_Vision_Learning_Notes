#include <iostream>
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

// 问题：
// 1. param 参数会导致对象发生拷贝、移动操作
// 2. param 传递到构造函数，无法区分值的类别(左值、右值)
Person make_person(string param)
{
  // 如果传递的参数 param 是左值
  return Person(param);
}

void test()
{
  // 1. 传递左值
  string name = "Obama";
  make_person(name);

  // 2. 传递右值
  make_person(move(name));
}

int main()
{
  test();
  return 0;
}