#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>  /**< 条件变量 */

// 条件变量是一种同步原语,用于在线程之间协调共享资源的访问.
// 它允许一个线程等待特定条件的满足(如某个值的变化),而另一个线程在条件满足时通知(或唤醒)等待的线程.
// 这种机制可以防止线程忙等待,从而提高系统效率. 

std::mutex mtx;               /**< 互斥锁 */
std::condition_variable cv;   /**< 条件变量 */

// 条件变量存在"虚假唤醒"的问题:即使没有其他线程调用notify,操作系统也可能随机唤醒等待的线程.
// 如果没有状态标记(flag)线程被唤醒后无法判断"条件是否真的满足",会导致逻辑错误.

bool flag = false;            /**< 共享变量 标志位,用于检测条件是否满足 */

//根据共享变量组建逻辑 检测条件
void myprint(int i) {
  //此函数要访问共享变量,要保证线程安全 必须要加锁!!
  std::unique_lock<std::mutex> lck(mtx);  /**< 线程中间要等待,就要放弃锁,这也意味着之前要锁上,中间要解锁(不用lock_guard的原因) */
  while (!flag) {
    // 条件不成立,进入阻塞状态,释放锁,等待别人的唤醒
    cv.wait(lck);                                           /**< 条件变量等待 */
    // 这行代码是"魔法核心",干了3件关键的事:
    // ① 立刻释放互斥锁lck(让其他线程能访问flag,不然主线程改不了flag,死锁)
    // ② 让当前线程“阻塞休眠”（不占CPU，解决忙等问题）
    // ③ 等被cv.notify_all()唤醒后，会**重新抢互斥锁lck**，抢到后再重新判断!flag
  
  }
  std::cout << std::this_thread::get_id() << " 线程被唤醒,flag = " << flag << ", id =  " << i << std::endl;
}

void update_flag() {
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  // 此函数也要修改共享变量,要保证线程安全 要加锁!!
  std::unique_lock<std::mutex> lck(mtx);
  flag = true;
  cv.notify_all();       /**< 通知所有等待的线程 */
}

int main() {
  std::vector<std::thread> mybox;
  for (size_t i = 0; i < 10; i++) {
    mybox.emplace_back(myprint, i);  /**< emplace_back 直接在容器末尾构造对象,避免了拷贝构造 */
  }
  //准备好唤醒的条件
  update_flag();

  for (std::thread& t : mybox) {
    t.join();                        /**< 阻塞 */
  }
}