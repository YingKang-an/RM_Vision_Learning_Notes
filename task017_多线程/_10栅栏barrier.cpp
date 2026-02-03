#include <iostream>
#include <thread>
#include <barrier>
#include <vector>
#include <chrono>

// C++20 引入了 std::barrier,它提供了栅栏的功能.
// 栅栏(barrier)对象是一种同步原语,用于协调多个线程的执行,使它们能够在某个特定的点(即栅栏)等待,
// 直到所有线程都到达这一点,然后它们才能继续执行.栅栏可以确保并发任务在某些关键时刻同步,
// 比如等待所有线程完成某个阶段的工作,然后再进入下一阶段.
//--------------------------------------------------------------------------------------
// 特点
// 1. 同步点: 栅栏用于创建一个同步点,确保多个线程在同一时刻同步.
// 2. 计数器: 栅栏内部维护一个计数器,记录到达栅栏的线程数量.当计数器达到预设值时,所有等待的线程被同时唤醒.
// 3. 可重用性: C++20中的std::barrier是可重用的,线程可以反复使用同一个栅栏对象进行同步.
//--------------------------------------------------------------------------------------
// 作用
// 1. 阶段同步: 确保多个线程在某一阶段完成后再进入下一阶段.例如,在并行计算中,各个线程可能需要同步它们的计算结果,然后再继续下一步计算.
// 2. 批处理: 多个线程可能需要在某些关键时刻汇聚数据或状态,然后再继续各自的任务.
//--------------------------------------------------------------------------------------

void worker(int id, std::barrier< >& sync_point) {
  std::cout << "worker " << id << " is doing phase 1 work" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(3*id)); /**< 模拟不同的工作时间 */
  std::cout << "worker " << id << " is done with phase 1 work" << std::endl;
  std::cout << "worker " << id << " is waiting for all other workers to complete phase 1" << std::endl;
  sync_point.arrive_and_wait(); /**< 等待所有线程到达同步点 */
  std::cout << "worker " << id << " is doing phase 2 work" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(3*id)); /**< 模拟不同的工作时间 */
  std::cout << "worker " << id << " is done with phase 2 work" << std::endl;
}

int main() {
  const int num_workers = 5;
  std::barrier< > sync_point(num_workers); /**< 创建一个栅栏对象,用于同步 num_workers 个线程 */
  std::vector<std::thread> workers;
  for (int id = 0; id < num_workers; ++id) {
    workers.emplace_back(worker, id, std::ref(sync_point)); /**< 创建并启动线程,将栅栏对象作为引用传递 */
  }
  for (auto& worker : workers) {
    worker.join(); /**< 等待所有线程完成 */
  }
  std::cout << "All workers have completed work!!!!!!!!!!!!!!!!!!!!" << std::endl;
  return 0;
}


