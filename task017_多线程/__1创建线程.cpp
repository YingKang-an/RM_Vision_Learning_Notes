#include <thread>
#include <iostream>

//这是C++11及以后版本推荐的方式.可以直接创建一个std::thread对象来启动一个新的线程,并传递一个可调用对象(如函数、函数对象、Lambda表达式等)作为线程的执行体.

void func(int a) {
  while(true) {
    std::cout << "Hello World" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));   /**< 线程休眠1秒 */
    std::this_thread::
  }
}

int main() {  /**< main所在的线程是主线程 */
  int a = 0;
  std::thread thread_1(func, 1);
  std::cout << std::endl<<thread_1.hardware_concurrency() << std::endl;    /**< 输出当前系统的线程数 */
  thread_1.join();    /**< 阻塞主线程,当thread_1执行完毕后,主线程才会继续执行 */
  thread_1.detach();    /**< 分离线程,当线程执行完毕后,线程会自动销毁 */
  while(true) {
  std::cout << "thread_1 id: " << thread_1.get_id() << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  while(true);
}


#ifdef NO_H
#define NO_H
/**
 * @file       创建线程.cpp
 * 
 * @brief      创建线程
 *             该文件演示了如何创建一个线程
 * 
 * @author @b  YinKang'an
 * @date   @b  2026-01-26
 */


#include <thread>                             /** 包含线程库 */
#include <iostream>

void SayHello() {
  std::cout << "hello,thread1" << std::endl;
}

int main() {
  
  std::cout << "this is main" << std::endl;    /**< main所在的线程是主线程 */

  // 传入的参数是"可调对象",可以是函数指针,lambda表达式等
  std::thread t1(SayHello);                    /**< 创建线程t1，执行SayHello函数 */
  t1.join();                                   /**< 阻塞主线程 */

  return 0;
}


#include <thread>                             /** 包含线程库 */
#include <iostream>

class MyClass {
public:
  void MemberFunction() {
    std::cout << "hello,memberfunction" << std::endl;
  }
};

int main() {
  
  std::cout << "this is main" << std::endl;    /**< main所在的线程是主线程 */
 
  MyClass Obj;
  // 传入的参数是"可调对象",可以是函数指针,lambda表达式等
  std::thread t1(&MyClass::MemberFunction, &Obj);                    /**< 创建线程t1，执行MemberFunction函数 */
  t1.join();                                   /**< 阻塞主线程 */

  return 0;
}


#include <thread>                             /** 包含线程库 */
#include <iostream>

int main() {

  std::thread t1([](){std::cout << "hello,lambda" << std::endl;});    /**< 创建线程t1，执行lambda表达式 */
  t1.join();                                   /**< 阻塞主线程 */

  return 0;
}

#include <thread>                             /** 包含线程库 */
#include <iostream>

struct Functor {
  void operator()() {
    std::cout << "hello,functor" << std::endl;
  }
};

int main() {

  Functor f;
  std::thread t1(f);                           /**< 创建线程t1，执行Functor函数对象 */
  t1.join();                                   /**< 阻塞主线程 */

  return 0;
}
#endif // NO_H