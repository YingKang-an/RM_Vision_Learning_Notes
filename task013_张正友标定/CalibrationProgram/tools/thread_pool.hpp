/**
 * @file thread_pool.hpp
 * @brief 线程池管理头文件
 */

#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>

class ThreadPool {
public:
  // 构造函数，指定线程数量
  ThreadPool(size_t numThreads);
  
  // 析构函数
  ~ThreadPool();
  
  // 添加任务到线程池
  template<class F, class... Args>
  auto enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>;
  
  // 获取线程池大小
  size_t size() const { return workers.size(); }
  
  // 获取任务队列大小
  size_t taskCount() const { return tasks.size(); }
  
  // 停止线程池
  void stop();

private:
  std::vector<std::thread> workers;           // 工作线程
  std::queue<std::function<void()>> tasks;    // 任务队列
  std::mutex queueMutex;                      // 队列互斥锁
  std::condition_variable condition;          // 条件变量
  std::atomic<bool> stopFlag;                 // 停止标志
};

// 模板函数实现
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
  -> std::future<typename std::result_of<F(Args...)>::type> {
  
  using return_type = typename std::result_of<F(Args...)>::type;
  
  auto task = std::make_shared<std::packaged_task<return_type()>>(
    std::bind(std::forward<F>(f), std::forward<Args>(args)...)
  );
  
  std::future<return_type> result = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queueMutex);
    if (stopFlag) {
      throw std::runtime_error("enqueue on stopped ThreadPool");
    }
    tasks.emplace([task](){ (*task)(); });
  }
  condition.notify_one();
  return result;
}

#endif // THREAD_POOL_HPP