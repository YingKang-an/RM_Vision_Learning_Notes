#include <iostream>         // 控制台输出
#include <chrono>           // 时间相关操作（睡眠、计时）
#include <thread>           // 线程睡眠函数
#include "thread_pool.hpp"  // 包含线程池头文件

/**
 * 测试任务1：无返回值的任务（模拟普通业务逻辑）
 * @param name 任务名称（用于区分不同任务）
 * @param sleep_ms 任务执行耗时（毫秒，模拟真实任务的处理时间）
 */
void test_task(const std::string& name, int sleep_ms) {
  std::cout << "Task [" << name << "] started (expected time: " << sleep_ms << "ms)" << std::endl;
  // 模拟任务执行耗时：线程睡眠指定时间
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  std::cout << "Task [" << name << "] completed" << std::endl;
}

/**
 * 测试任务2：带返回值的任务（模拟需要获取结果的业务逻辑）
 * @param a 第一个参数
 * @param b 第二个参数
 * @return a + b 的结果（模拟任务计算结果）
 */
int add_task(int a, int b) {
  std::cout << "Add task started: " << a << " + " << b << std::endl;
  // 模拟任务执行耗时（500毫秒）
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  int result = a + b;
  std::cout << "Add task completed: " << a << " + " << b << " = " << result << std::endl;
  return result;
}

int main() {
  // 1. 创建线程池：4个工作线程（可根据CPU核心数调整，比如8核心设为8）
  ThreadPool pool(4);
  // 存储带返回值任务的future对象，后续用于获取结果
  std::vector<std::future<int>> futures;

  std::cout << "=== Step 1: Test task priority (verify priority queue) ===" << std::endl;
  // 提交不同优先级的任务，验证优先级调度：高优先级任务先执行
  pool.enqueue(TaskPriority::LOW, test_task, "LOW-1", 1000);          // 优先级1（最低），耗时1000ms
  pool.enqueue(TaskPriority::HIGH, test_task, "HIGH-1", 500);        // 优先级5（最高），耗时500ms
  pool.enqueue(TaskPriority::NORMAL, test_task, "NORMAL-1", 800);    // 优先级3（默认），耗时800ms
  pool.enqueue(TaskPriority::HIGH, test_task, "HIGH-2", 600);        // 优先级5（最高），耗时600ms
  pool.enqueue(TaskPriority::BELOW_NORMAL, test_task, "BELOW-1", 700); // 优先级2，耗时700ms

  // 等待1.5秒，让上述任务部分执行，观察优先级效果
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  std::cout << "\n=== Step 2: Test tasks with return value (verify future) ===" << std::endl;
  // 提交带返回值的任务，验证future获取结果的功能
  futures.emplace_back(pool.enqueue(TaskPriority::ABOVE_NORMAL, add_task, 10, 20)); // 优先级4，10+20
  futures.emplace_back(pool.enqueue(add_task, 30, 40)); // 默认优先级3，30+40（兼容旧接口）

  std::cout << "\n=== Step 3: Test pool status monitoring (verify get_status) ===" << std::endl;
  // 实时监控线程池状态：每隔300ms输出一次，观察活跃线程数、等待任务数变化
  for (int i = 0; i < 5; ++i) {
    ThreadPool::PoolStatus status = pool.get_status();
    std::cout << "Monitor " << (i+1) << ": "
              << "ActiveThreads=" << status.active_threads << " | "
              << "PendingTasks=" << status.pending_tasks << " | "
              << "CompletedTasks=" << status.completed_tasks << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  std::cout << "\n=== Step 4: Get task return values (verify future.get()) ===" << std::endl;
  // 等待带返回值的任务完成，通过future.get()获取结果（get()会阻塞直到任务完成）
  for (size_t i = 0; i < futures.size(); ++i) {
    int result = futures[i].get();
    std::cout << "Future result " << (i+1) << ": " << result << std::endl;
  }

  std::cout << "\n=== Step 5: Wait for all tasks to complete ===" << std::endl;
  // 等待所有任务完成（可选，线程池析构时会自动等待，这里为了看最终状态）
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 6. 输出线程池最终状态
  ThreadPool::PoolStatus final_status = pool.get_status();
  std::cout << "\n=== Final Pool Status ===" << std::endl;
  std::cout << "ActiveThreads: " << final_status.active_threads << std::endl;
  std::cout << "PendingTasks: " << final_status.pending_tasks << std::endl;
  std::cout << "CompletedTasks: " << final_status.completed_tasks << std::endl;

  return 0;
}
