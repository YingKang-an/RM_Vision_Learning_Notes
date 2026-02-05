/**
 * @file   \b     _生产者消费者.cpp
 * @author \b     YinKang'an
 * @date          2026-02-05
 * @version       V3-2-2
 * @brief         生产者消费者队列
 *
 * @details
 *  1.泛型编程:          基于模板设计,支持任意数据类型.
 *  2.左值右值与移动语义:  内部采用移动语义取出元素,最小化数据拷贝,提升吞吐性能.
 *  3.万能引用和完美转发:  push接口支持万能引用,通过std::forward保持参数值类别,实现零损耗传递.
 *  4.线程安全:          使用条件变量std::condition_variable,独占锁std::unique_lock保证多线程安全访问.
 *
 * @note
 *  编译需启用 C++20 及以上标准.
 */
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <condition_variable>

template <typename T>
class Queue {
private:
  const int max_size_;
  std::queue<T> queue_;
  mutable std::mutex mtx_;   /**< 使用mutable确保在const成员函数中也能锁定互斥锁 */
  std::condition_variable cv_;

public:
  explicit Queue(int max_size) : max_size_(max_size) { }  /**< 禁止隐式类型转换 */

  template <typename U>
  void push(U&& item) {  /**< 万能引用,一套接口接收所有参数,避免重载多版本 */
    std::unique_lock<std::mutex> lck(mtx_);
    cv_.wait(lck,[this]() {return this->queue_.size() < this->max_size_;});
    queue_.push(std::forward<U> (item)); /**< 完美转发,左值自动拷贝,右值自动移动,提高性能 */
    lck.unlock();
    std::cout << "The Producer " << std::this_thread::get_id() << " push: " << item << std::endl;
    cv_.notify_one();
  }

  T pop() {
    std::unique_lock<std::mutex> lck(mtx_);
    cv_.wait(lck,[this]() {return this->queue_.size() > 0;});
    T item = std::move(this->queue_.front()); /**< 移动语义取出元素,避免拷贝 */
    this->queue_.pop();
    lck.unlock();
    std::cout << "The Consumer " << std::this_thread::get_id() << " pop : " << item << std::endl;
    cv_.notify_one();
    return item;
  } 

  bool is_empty() const { /**< const承诺不修改队列状态 */
    std::unique_lock<std::mutex> lck(mtx_); /**< 加锁确保线程安全 */
    return this->queue_.empty();
  }

  bool get_size() const { /**< const承诺不修改队列状态 */
    std::unique_lock<std::mutex> lck(mtx_); /**< 加锁确保线程安全 */
    return this->queue_.size();
  }
};

int main() {
  Queue<float> q(3);  /**< 队列最大容量为3 */

  std::thread producer([&q]() {
    for (size_t i = 0; i < 15; ++i) {
      q.push(i * 0.3f);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  std::thread consumer([&q]() {
    for (size_t i = 0; i < 15; ++i) {
      float item = q.pop();
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  });

  producer.join();
  consumer.join();

  return 0;
}