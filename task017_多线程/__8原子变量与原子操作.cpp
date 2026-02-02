#include <iostream>
#include <thread>
#include <atomic>

// 原子变量是指使用std::atomic模板类定义的变量.这些变量提供了对其所表示的值进行原子操作的能力.
// 原子变量确保在多线程环境中,对变量的读写操作是线程安全的,即操作不会被其他线程中断或干扰.
// std::atomic<int> atomicInc(0); /**< 原子变量atomicInc,初始值为0 */

// int num = 0;
std::atomic<int> num(0);

void add() {
  for (size_t i = 0; i < 1000000; i++) {
    // C++11的 std::atomic 对 ++ 运算符做了原子重载,此时的 num++ 并非原生的自增语法
    num++;    /**< 在cpu上进行原子操作 */

    // num = num + 1; /**< !!!!这不是原子操作 */
    // 拆分为3步
    // 1. num.load():原子读取num的值到寄存器(底层lock mov)
    // 2. 寄存器中执行+1(普通操作)
    // 3. num.store(结果):原子写入(底层lock mov)
  }
}

int main() {
  std::thread t1(add);
  std::thread t2(add);
  t1.join();
  t2.join();
  std::cout << "num: " << num << std::endl;

  return 0;
}

#define HIDE_CODE 0 

// 原子操作的内存序问题
//-------------------------------------------------------------------------------------------
#if HIDE_CODE
// 在多线程编程中,内存顺序问题是由编译器和处理器的优化行为引起的.
// 为了提高性能,译器和处理器可能会对内存操作(读取和写入)行重排.
// 重排在单线程程序中通常不会引起问题,在多线程程序中,排可能会导致线程间的数据竞争和不可预测的行为.

// - memory_order_relaxed: 没有同步或顺序约束，仅保证原子性
// - memory_order_acquire: 确保此操作之前的所有读操作不会被重排到此操作之后
// - memory_order_release: 确保此操作之后的所有写操作不会被重排到此操作之前
// - memory_order_acq_rel: 同时具备 acquire 和 release 的特性
// - memory_order_seq_cst: 顺序一致性，保证所有线程的操作按顺序发生

num.fetch_add(100, std::memory_order_relaxed);    /**< 原子操作,将num增加100,内存序为std::memory_order_relaxed */

// 内存序不会影响原子操作的原子性.即使在最弱的 std::memory_order_relaxed 内存序下,
// 操作仍然是原子的,不可中断.然而,内存序会影响操作的可见性和顺序.

#endif

// 高级操作
//-------------------------------------------------------------------------------------------
#if HIDE_CODE
// 1.std::atomic 类 进行重载的运算符: ++, --, +=, -= 等,确保在多线程环境中对变量的操作是原子的.
  std::atomic<int> atomicNum(0); /**< 原子变量atomicNum,初始值为0 */
  atomicNum += 100; /**< 原子操作,将atomicNum增加100 */

// 2.compare_exchange_strong() 和 compare_exchange_weak(): 用于原子比较并交换操作,
  int expected = 100; /**< 期望的值为100 */
  // strong无伪失败,确定性强.weak允许伪失败,性能更优且必须配循环使用
  bool success = atomicNum.compare_exchange_strong(expected, 200); /**< 若atomicNum当前值为100,则将其交换为200,返回true;否则返回false */
  if (success) {
    std::cout << "compare_exchange_strong: 交换成功, atomicNum: " << atomicNum << std::endl;
  } else {
    std::cout << "compare_exchange_strong: 交换失败, atomicNum: " << atomicNum << std::endl;
  }


#endif