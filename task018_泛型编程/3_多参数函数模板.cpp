/**
 * --------------------------------------------------
 * @b       多参数函数模板
 * --------------------------------------------------
 * 多个模板参数,逗号隔开
 */

//------------------------------------------------------------------------------------
// 多个模板参数

#include <iostream>

template<typename T, typename Y>
void PrintType(T a, Y c) {        
  std::cout << typeid(a).name() << std::endl;
  std::cout << typeid(c).name() << std::endl;    /**< 输出类型名 */ 
}

int main() {
  double d = -2.0;
  PrintType(1, 2.0);
  PrintType("hello", 'a');
  PrintType(d, 2.9f);
}

//------------------------------------------------------------------------------------
// 模板函数默认参数

template<typename T = int, typename Y = double>
void PrintTypeDefault(T a = 0, Y c = 0.0) {
  std::cout << typeid(a).name() << std::endl;
  std::cout << typeid(c).name() << std::endl;    /**< 输出类型名 */ 
}

//------------------------------------------------------------------------------------
