// 以下是一个不安全的线程操作示例,两个线程同时对GolbalVal进行++和--操作, 结果是不确定的
#define HIDE_CODE 0
#if HIDE_CODE
#include <thread>
#include <iostream>

int GolbalVal = 0;

void task_1() {
  for(uint64_t i = 0; i < 100000000; i++) {    /**<不安全的线程操作, 两个线程同时对GolbalVal进行++和--操作, 结果是不确定的 */
    GolbalVal++;
    GolbalVal--;
  }
}

int main() {
  std::thread thread_1(task_1);
  std::thread thread_2(task_1);
  thread_1.join();
  thread_2.join();
  std::cout << "GolbalVal: " << GolbalVal << std::endl;
}
#endif

// GolbalVal: -1188
// [1] + Done                       "/usr/bin/gdb" --interpreter=mi --tty=${DbgTerm} 0<"/tmp/Microsoft-MIEngine-In-wlsssilp.m2x" 1>"/tmp/Microsoft-MIEngine-Out-nvcvblg4.rw0"*/
//---------------------------------------------------------------------------------------------
// 线程同步(Thread Synchronization)是多线程编程中的一个重要概念,它指的是通过一定的机制来控制多个线程之间的执行顺序,以确保它们能够正确地访问和修改共享资源,从而避免数据竞争和不一致性的问题.
// 在多线程环境中,多个线程可能同时访问和修改共享资源(如变量、数据结构或文件等).如果没有适当的同步机制,这些线程可能会以不可预测的顺序执行,导致数据竞争,脏读、脏写或其他不可预期的行为.
// 线程同步的目标就是确保线程之间的有序执行,以维护数据的一致性和完整性.
//---------------------------------------------------------------------------------------------
#include <thread>
#include <iostream>
#include <mutex>

std::mutex mtx;

int GolbalVal = 0;

void task_1() {
  for(uint64_t i = 0; i < 100000000; i++) {
    mtx.lock();  /**< lock():尝试获取锁.如果锁是空闲的,立即获取并返回.如果锁已经被其他线程占用,当前线程会进入阻塞状态(不占用CPU),直到锁被释放 */
    GolbalVal++;
    GolbalVal--;
    mtx.unlock(); /**< unlock():释放锁.将锁的状态设置为空闲,"唤醒"所有正在等待这把锁的线程,让它们竞争获取锁 */
  }
}

int main() {
  std::thread thread_1(task_1);
  std::thread thread_2(task_1);
  thread_1.join();
  thread_2.join();
  std::cout << "GolbalVal: " << GolbalVal << std::endl;
}
