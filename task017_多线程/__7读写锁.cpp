#include <shared_mutex>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>

// C++中的读写锁(也称为共享锁和独占锁)是一种同步机制,
// 用于控制对共享资源的访问,允许多个线程同时读取资源,但在写入资源时只允许一个线程独占访问.
// 读读之间不互斥,读写之间互斥,写写之间互斥

std::shared_mutex rw_mutex;  /**< 读写互斥锁 */
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
  for (int i = 0; i < 5; i++) {
    //创建5个读取线程
    Threads.emplace_back(reader);  /**< 启动读取线程并加入线程向量 */
    //创建5个写入线程
    Threads.emplace_back(writer, i); /**< 启动写入线程并加入线程向量 */
  }
  //等待所有线程执行完毕
  for (auto& t : Threads) {
    t.join();    /**< join()逻辑:如果线程已经执行完毕,join()会直接返回.如果线程还在运行,join()会阻塞主线程到线程结束. */
  }
  return 0;
}

// 读取线程ID: 140737345746624, 读取数据: 2
// 写入线程ID: 140737337353920, 写入数据: 0
// 写入线程ID: 140737320568512, 写入数据: 1
// 读取线程ID: 140737328961216, 读取数据: 1
// 读取线程ID: 140737312175808, 读取数据: 1
// 写入线程ID: 140737303783104, 写入数据: 2
// 读取线程ID: 140737219917504, 读取数据: 2
// 写入线程ID: 140737211524800, 写入数据: 3
// 读取线程ID: 140737203132096, 读取数据: 3
// 写入线程ID: 140737194739392, 写入数据: 4
// [1] + Done                       "/usr/bin/gdb" --interpreter=mi --tty=${DbgTerm} 0<"/tmp/Microsoft-MIEngine-In-oyh1na5o.v2t" 1>"/tmp/Microsoft-MIEngine-Out-irvhgcuq.hxy"