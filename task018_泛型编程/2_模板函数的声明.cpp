/**
 * --------------------------------------------------
 * @b       函数模板的声明
 * --------------------------------------------------
 * 模板函数放在同一文件中,隐藏在HIDE_CODE中,是为了讲解多文件时的模板函数声明.
 */

//------------------------------------------------------------------------------------
// 模板函数声明(多文件中)
#include "2_模板函数的声明.hpp"
#include <iostream>
using namespace std;

#define HIDE_CODE 0 
#if HIDE_CODE
//------------------------------------------------------------------------------------
// 模板函数声明(单文件中)

//template只作用于仅挨着的函数,不能跨函数使用
template<typename T> //不能少,两个函数的 T 互不影响
T add(T a, T c);     //模板函数"声明"

//每个模板通用类型名互不影响，即使同名，相当于函数局部变量
template<typename T> //不能少，两个函数的 T 互不影响
void print(T a);
#endif

int main(void) {
  std::cout << add(2, 3) << std::endl;       //T 是 int 类型
  std::cout << add(2.4, 3.2) << std::endl;   //T 是 double 类型
  std::cout << add(2.5f, 3.3f) << std::endl; //T 是 float 类型
  print(add(2, 3));
  print(add(2.4, 3.2));
  print(add(2.5f, 3.3f));
  return 0;
}

#if HIDE_CODE
template<typename T> //不能少，两个函数的 T 互不影响
T add(T a, T c) {
  return a + c;
}

template<typename T>
void print(T a) {
  std::cout << a << std::endl;
}
#endif