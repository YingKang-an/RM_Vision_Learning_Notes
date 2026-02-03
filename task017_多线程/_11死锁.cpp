#include <iostream>
#include <mutex>
#include <thread>

// 死锁(Deadlock)是指两个或多个线程在执行过程中因争夺资源而造成的一种互相等待的现象.
// 如果没有外力干涉,它们将永远无法继续执行下去.
// 死锁会导致程序或系统的严重性能问题,甚至使整个系统陷入停滞.

// 死锁通常由以下四个必要条件引起,这四个条件同时满足时,系统就会产生死锁:
// 1. 互斥条件(Mutual Exclusion):
// 资源一次只能被一个线程占用.
// 2. 请求和保持条件(Hold and Wait):
// 线程已经持有至少一个资源,同时又申请新的资源,而新资源被其他线程占有.
// 3. 不剥夺条件(No Preemption):
// 已获得的资源在未使用完之前,不能被强行剥夺,只能在使用完毕后由线程自己释放.
// 4. 循环等待条件(Circular Wait):
// 存在一个线程循环等待的链,链中的每个线程都持有下一个线程需要的资源.

// 以下是一个简单的死锁示例:
std::mutex mutex1, mutex2;

void task1() {
  std::lock_guard<std::mutex> lock1(mutex1);  /**< 线程1获取mutex1 */
  std::this_thread::sleep_for(std::chrono::seconds(1));  // 模拟工作
  std::lock_guard<std::mutex> lock2(mutex2);  /**< 线程1试图获取mutex2,但线程2已经获取了mutex2,因此线程1等待线程2释放mutex2 */
  std::cout << "Task 1 completed." << std::endl;
}

void task2() {
  std::lock_guard<std::mutex> lock2(mutex2);  /**< 线程2获取mutex2 */
  std::this_thread::sleep_for(std::chrono::seconds(1));  // 模拟工作
  std::lock_guard<std::mutex> lock1(mutex1);  /**< 线程2试图获取mutex1,但线程1已经获取了mutex1,因此线程2等待线程1释放mutex1 */
  std::cout << "Task 2 completed." << std::endl;
}

int main() {
  std::thread t1(task1);
  std::thread t2(task2);

  t1.join();
  t2.join();

  return 0;
}

//如果想避免死锁,可以采取以下措施:
// 1. 避免嵌套锁:
// 尽量避免在一个线程中获取多个锁,而是在多个线程中分别获取锁.
// 2. 按固定顺序获取锁:
// 所有线程在获取锁时,都按照固定的顺序来获取,避免循环等待.
// 3. 使用超时机制:
// 当一个线程尝试获取锁时,如果不能立即获取,可以设置一个超时时间,超过这个时间后,线程可以放弃获取锁,继续执行其他任务.
// 4. 使用死锁检测和恢复机制:
// 一些操作系统或库提供了死锁检测和恢复机制,可以在死锁发生时自动检测并尝试恢复.
// 5. 使用高级工具