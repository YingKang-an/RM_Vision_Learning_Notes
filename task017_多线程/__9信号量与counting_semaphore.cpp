#include <iostream>
#include <thread>
#include <semaphore>
#include <vector>
#define HIDE_CODE 0

// 信号量(Semaphore)是一种用于管理和协调多线程或多进程访问共享资源的同步机制.
// 它通过计数器来控制对资源的访问数量,确保多个线程或进程能够安全地使用共享资源而不会发生数据竞争或死锁.
// 传统的锁(如互斥锁)可以用来保护共享资源.但对于某些场景(如资源的计数管理),信号量提供了更灵活和高效的解决方案.

// P操作(Proberen,尝试):
// 1. 尝试将信号量的计数值减1
// 2. 如果当前计数值 > 0,减1成功,线程继续执行(获取到资源的访问权)
// 3. 如果当前计数值 = 0,减1失败,线程阻塞等待,直到其他线程释放信号量(计数值>0)
// 作用：保证同一时间只有一个线程能进入后续的临界区(共享资源访问代码)

// V操作(Verhogen,释放):
// 1. 将信号量的计数值加1
// 2. 如果有线程因acquire()阻塞等待,会唤醒其中一个等待的线程
// 作用:释放对共享资源的占用,让其他等待的线程有机会获取资源

//-------------------------------------------------------------------------------------------
// 信号量实现互斥

std::counting_semaphore<1> sem(1); /**< 第一个1代表最大信号量数量[互斥访问],第二个1表示初始值为1 */

void worker(int id) {
  sem.acquire();             /**< P操作,获取信号量,如果信号量为0,则阻塞等待 */
  // std::cout << "max = " << sem.max() << std::endl;  /**< 最大信号量数量 */
  std::cout << "Worker " << id << " is working" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1)); // 模拟工作
  std::cout << "Worker " << id << " is done" << std::endl;
  sem.release();             /**< V操作,释放信号量,将信号量值加1 */
}

int main() {
  std::vector<std::thread> Threads;
  for (size_t i = 0; i < 5; ++i) {
    Threads.emplace_back(worker, i);
  }
  for (auto& Thread : Threads) {
    Thread.join();
  }
  return 0;
}

#if HIDE_CODE
//-------------------------------------------------------------------------------------------
// 信号量实现线程同步

std::counting_semaphore<1> ready(0); /**< 初始值为0,表示线程未准备好,用于work线程的等待 */
std::counting_semaphore<1> done(1);  /**< 初始值为1,表示线程可执行,用于prepare线程的执行 */

void prepare() {
  done.acquire(); /**< 减少信号量done的计数值,保证prepare线程在work线程之前执行 */
  std::cout << "preparing..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1)); // 模拟准备工作
  std::cout << "preparation done!" << std::endl;
  ready.release(); /**< 增加信号量ready的计数值,表示线程准备好,唤醒等待的work线程 */
}

void work() {
  ready.acquire(); /**< 减少信号量ready的计数值,如果为0,则阻塞等待,直到prepare线程释放信号量 */
  std::cout << "working..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1)); // 模拟工作
  std::cout << "work done!" << std::endl;
  done.release(); /**< 增加信号量done的计数值,表示线程执行完成,唤醒等待的prepare线程 */
}

int main() {
  std::vector<std::thread> Threads;
  for (size_t i = 0; i < 5; ++i) {    // 5个prepare线程
    Threads.emplace_back(prepare);
  }
  for (size_t i = 0; i < 5; ++i) {    // 5个work线程
    Threads.emplace_back(work);
  }
  for (auto& Thread : Threads) {
    Thread.join();
  }
  return 0;
}
#endif
