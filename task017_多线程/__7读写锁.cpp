#include <shared_mutex>
# include <iostream>
# include <thread>
# include <mutex>
# include <vector>

// C++中的读写锁(也称为共享锁和独占锁)是一种同步机制,
// 用于控制对共享资源的访问,允许多个线程同时读取资源,但在写入资源时只允许一个线程独占访问.
// 读读之间不互斥,读写之间互斥,写写之间互斥

std::shared_mutex rw_mutex;  /**< 读写锁 */
int shared_data = 2;         /**< 共享数据 */

void reader() {
  std::shared_lock<std::shared_mutex> lock(rw_mutex); /**< 申请共享锁 */
  std::cout << "读取线程ID: " << std::this_thread::get_id() << ", 读取数据: " << shared_data << std::endl;
}

void writer(int value) {
  std::unique_lock<std::shared_mutex> lock(rw_mutex); /**< 申请独占锁 */
  shared_data = value;
  std::cout << "写入线程ID: " << std::this_thread::get_id() << ", 写入数据: " << shared_data << std::endl;
}

int main() {
  std::vector<std::thread> Threads; /**< 存储线程的向量 */
  //创建5个读取线程
  for (int i = 0; i < 5; i++) {
    Threads.emplace_back(reader);  /**< 启动读取线程并加入线程向量 */
  }
  //创建2个写入线程
  for (int i = 0; i < 2; i++) {
    Threads.emplace_back(writer, i); /**< 启动写入线程并加入线程向量 */
  }
  //等待所有线程执行完毕
  for (auto& t : Threads) {
    t.join();
  }
  return 0;
}



