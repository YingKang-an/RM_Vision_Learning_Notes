/**
 * @file thread_pool.cpp
 * @brief 线程池管理实现
 */

#include "thread_pool.hpp"
#include <iostream>

// 构造函数
ThreadPool::ThreadPool(size_t numThreads) : stopFlag(false) {
  for (size_t i = 0; i < numThreads; ++i) {
    workers.emplace_back([this] {
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lock(this->queueMutex);
          this->condition.wait(lock, [this] {
            return this->stopFlag || !this->tasks.empty();
          });
          
          if (this->stopFlag && this->tasks.empty()) {
            return;
          }
          
          task = std::move(this->tasks.front());
          this->tasks.pop();
        }
        task();
      }
    });
  }
}

// 析构函数
ThreadPool::~ThreadPool() {
  stop();
}

// 停止线程池
void ThreadPool::stop() {
  {
    std::unique_lock<std::mutex> lock(queueMutex);
    stopFlag = true;
  }
  condition.notify_all();
  for (std::thread &worker : workers) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}