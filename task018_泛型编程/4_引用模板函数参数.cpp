#define HIDE_CODE 0
// 引用不是类型,它是功能(起别名,传入时是被起别名的类型)
#if HIDE_CODE
#include <iostream>

template<typename T>
void PrintType(T a) {        
  std::cout << typeid(a).name() << std::endl;    /**< 输出类型名 */ 
}

int main() {
  int a = 1;
  int& b = a;
  int* c = &a;
  PrintType(a);
  PrintType(b);
  PrintType(c);
}

// i
// i
// Pi
// [1] + Done                       "/usr/bin/gdb" --interpreter=mi --tty=${DbgTerm} 0<"/tmp/Microsoft-MIEngine-In-dkyxz2y3.sck" 1>"/tmp/Microsoft-MIEngine-Out-aiwgw5bh.za5"
// yinkangan@Linux:~/Desktop/Vision$ 
//-----------------------------------------------------
#endif

#include <iostream>
#include <typeinfo>

template<typename T>
void PrintType(T& a) {
  std::cout << "参数类型: " << typeid(a).name() << std::endl;
  std::cout << "模板参数T: " << typeid(T).name() << std::endl;
  std::cout << "---" << std::endl;
}

int main() {
  int a = 1;
  int& b = a;
  int* c = &a;
  PrintType(a);
  PrintType(b);
  PrintType(c);
}
