#include "thread_pool.hpp"  // 包含线程池类的声明
#include <cerrno>        // 错误码处理（异常提示用）
#include <iostream>       // 控制台输出（异常提示用）

/**
 * 线程池构造函数实现
 * 核心工作：1. 初始化原子变量；2. 创建指定数量的工作线程；3. 每个线程进入循环等待任务
 * @param thread_count 工作线程数量（传入0时默认创建1个线程，避免无工作线程）
 */
ThreadPool::ThreadPool(size_t thread_count) 
  : stop_(false),  // 初始化停止标志：线程池未停止
    active_threads_(0),  // 初始化活跃线程数：0个（线程未开始工作）
    completed_tasks_(0)  // 初始化已完成任务数：0个
{
  // 安全校验：如果传入的线程数为0，默认创建1个线程（避免无工作线程导致任务无法执行）
  if (thread_count == 0) {
    thread_count = 1;
  }

  // 循环创建thread_count个工作线程，并存入workers_容器
  for (size_t i = 0; i < thread_count; ++i) {
    workers_.emplace_back([this]() {  // 每个线程的执行逻辑（lambda捕获this指针，访问线程池成员）
      // 线程核心循环：持续等待并执行任务，直到线程池关闭且任务队列为空
      for (;;) {
        // 修复点：用自定义构造函数初始化空任务（避免默认构造报错）
        Task task(0, nullptr);  // 存储从队列取出的任务，初始化为优先级0+空函数

        // 1. 加锁访问任务队列（unique_lock支持条件变量等待，自动管理锁的释放/获取）
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);

          // 2. 条件变量等待：线程阻塞在这里，直到满足以下任一条件：
          // - 线程池停止（stop_为true）
          // - 任务队列非空（有任务可执行）
          // 等待期间会释放锁，避免阻塞其他线程访问队列
          condition_.wait(lock, [this]() {
            return stop_ || !tasks_.empty();
          });

          // 3. 退出条件：线程池已停止，且任务队列已空（所有任务都执行完毕）
          if (stop_ && tasks_.empty()) {
            return;  // 线程退出，结束循环
          }

          // 4. 取出任务：从优先级队列顶部取出最高优先级的任务（std::move转移所有权，避免拷贝）
          task = std::move(tasks_.top());
          tasks_.pop();  // 从队列中移除已取出的任务

          // 5. 活跃线程数+1：任务开始执行，统计活跃线程
          active_threads_++;
        } // 解锁：任务取出后立即释放锁，避免任务执行时间过长阻塞队列操作

        // 6. 执行任务：在无锁环境下执行，提高并发效率
        try {
          task.func();  // 调用任务的执行逻辑（包装的可调用对象）
        } catch (const std::exception& e) {
          // 异常捕获：单个任务执行失败不会导致整个线程崩溃，仅输出错误信息
          std::cerr << "Task execution failed: " << e.what() << std::endl;
        } catch (...) {
          // 捕获所有未预期的异常，保证线程池稳定性
          std::cerr << "Task execution failed: Unknown exception" << std::endl;
        }

        // 7. 任务完成后的统计更新：
        active_threads_--;  // 活跃线程数-1（任务执行完毕，线程变为空闲）
        completed_tasks_++; // 已完成任务数+1（统计总完成量）
      }
    });
  }
}

/**
 * 线程池析构函数实现
 * 作用：确保线程池关闭，回收所有资源（工作线程、队列等）
 * 逻辑：直接调用shutdown()，避免代码重复
 */
ThreadPool::~ThreadPool() {
  shutdown();
}

/**
 * 关闭线程池的实现
 * 核心逻辑：1. 设置stop_为true（禁止新任务提交）；2. 唤醒所有等待线程；3. 等待所有线程退出
 */
void ThreadPool::shutdown() {
  // 1. 加锁设置stop_标志（确保线程安全，避免多线程同时调用shutdown）
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_ = true;  // 设置为true后，enqueue会拒绝新任务，线程循环会检查退出条件
  } // 自动解锁

  // 2. 唤醒所有等待的工作线程：让线程检查退出条件（stop_为true且队列空则退出）
  condition_.notify_all();

  // 3. 等待所有工作线程退出（join()会阻塞直到线程执行完毕）
  for (std::thread& worker : workers_) {
    // 检查线程是否可join（避免重复join导致崩溃）
    if (worker.joinable()) {
      worker.join();
    }
  }
}

/**
 * 获取线程池状态的实现（监控接口）
 * 逻辑：加锁访问任务队列和原子变量，组装状态信息后返回
 * 注意：const成员函数，不修改线程池状态，仅读取
 */
ThreadPool::PoolStatus ThreadPool::get_status() const {
  // 加锁保护：tasks_.size()的访问需要线程安全（队列可能被其他线程修改）
  std::unique_lock<std::mutex> lock(queue_mutex_);
  return {
    active_threads_,    // 原子变量，直接读取（无锁安全）
    tasks_.size(),      // 等待任务数：队列当前大小
    completed_tasks_    // 原子变量，直接读取（无锁安全）
  };
}
