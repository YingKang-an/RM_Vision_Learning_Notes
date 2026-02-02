#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

// lock_guard 是一个模板类,位于 <mutex>头文件中. 
// 它符合 RAII风格,它主要用于管理 mutex 的生命周期,确保 mutex 在锁定的作用域内被正确地上锁和解锁.
// 它主要解决了手动管理 mutex 锁定和解锁时可能出现的问题,如：忘记解锁、异常情况下未解锁等问题.
// 你可以简单把这个东西,看成是对mutex的一种管理封装,就是说用原生的mutex,有些时候不顺手,有瑕疵.
// 使用lock_guard 比较省心,它来更好地管理你的mutex.

size_t count = 0;
std::mutex mtx;

void sum() {
  std::lock_guard<std::mutex> lock(mtx);            /**< 我保护的是mutex类型的锁,它叫mtx */
  for (size_t i = 0; i < 10000000; i++) {
    // std::lock_guard<std::mutex> lock(mtx);       /**< 效率低,不建议在循环中使用 */
    count++;
  }
  // 出了作用域,lock_guard 会自动解锁,无需手动调用 unlock().[析构函数会自动调用].
}

int main() {
  std::vector<std::thread> Threads;
  for (size_t i = 0; i < 10; i++) {
    Threads.emplace_back(sum);
  }
  for(std::thread& t : Threads) {                    /**< 线程不能复制,必须引用!!! */
    t.join();
  }
  std::cout << "count = " << count << std::endl;
}

// std::lock_guard 核心特性:不可复制,不可移动(禁用拷贝/移动构造+赋值)
// 设计原因:lock_guard 基于RAII机制,构造时自动加锁,析构时自动解锁,需与互斥锁严格一对一绑定.
// 若允许复制/移动,会导致多个lock_guard对象绑定同一个互斥锁,引发重复解锁（未定义行为,程序崩溃/死锁）.
// 从语法层面杜绝锁管理混乱,保证加锁/解锁的严格匹配,彻底避免手动锁的漏解锁,重复解锁问题.
// std::lock_guard<std::mutex> lg(mtx);
