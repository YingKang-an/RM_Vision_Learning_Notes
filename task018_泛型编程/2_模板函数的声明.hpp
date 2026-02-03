#pragma once
#include <iostream>
/*
 * C++中模板函数的声明和定义必须放在一起(通常都放在头文件中).
 * 无法像普通函数那样声明放头文件、定义放源文件(.cpp).
 * 核心原因:模板是编译期实例化的泛型机制,编译器在编译调用模板的代码时,
 * 必须能看到模板的完整定义(函数体),才能根据调用的具体类型(如int/double)生成对应的具体函数代码;
 * 若定义放在源文件,编译器编译时无法找到模板体,会报未定义的引用错误.
 */

template<typename T> //不能少，两个函数的 T 互不影响
T add(T a, T c) {
  return a + c;
}

template<typename T>
void print(T a) {
  std::cout << a << std::endl;
}