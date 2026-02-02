# include <iostream>
# include <thread>
# include <mutex>

// 因为mutex 在管理方面有瑕疵,因此出现了一个互斥量封装器lock_guard来智能地管理mutex.
// 而lock_guard只是简单的管理，功能比较弱，有瑕疵。因此需要搞出来一个功能更强大的东西出来。而这个东西就是unique_lock 。
// 本质上来说lock_guard 和unique_lock都是为了更好地使用各种锁而诞生的。但是unique_lock更为灵活，功能更强大，可做的操作比较多。

int count_1 = 0;
int count_2 = 0;

std::mutex mtx;

void sum() {
  {
    std::lock_guard<std::mutex> lock(mtx);
    for (size_t i = 0; i < 10000; i++) {
      count_1++;
    }
  } // lock_guard 锁的粒度过大,中途无法解锁,不支持条件变量,不支持递归锁  ->  unique_lock 锁的粒度可以小,可以中途解锁,支持条件变量,支持递归锁
  for (size_t i = 0; i < 10000; i++) {
    count_2++;
  }
}

int main() {
  std::thread t1(sum);
  t1.join();
  std::cout << count_1 << std::endl;
  std::cout << count_2 << std::endl;
  return 0;
}

std::mutex mtx_1;
std::unique_lock<std::mutex> lock_1(mtx_1);                   /**< 自动锁定 */
std::unique_lock<std::mutex> lock_2(mtx_1, std::defer_lock);  /**< 延迟锁定,不自动锁定,需要手动调用 lock() 方法锁定 */
std::unique_lock<std::mutex> lock_3(mtx_1, std::try_to_lock); /**< 尝试锁定,如果无法锁定,则返回失败,不阻塞线程 */
std::unique_lock<std::mutex> lock_4(mtx_1, std::adopt_lock);  /**<  adopt_lock 接管已经锁定的锁 */
 
void example_defer_lock() {
  std::unique_lock<std::mutex> lock(mtx_1, std::defer_lock);
  //...一些其他操作
  lock.lock(); // 手动锁定
  std::cout << "Locked with defer_lock: " << lock.owns_lock() << std::endl;
  //lock.unlock(); // 手动解锁,也可以在 unique_lock 析构时自动解锁
}

void example_try_to_lock() {
  std::unique_lock<std::mutex> lock(mtx_1, std::try_to_lock);
  //lock.unlock();  !!!如果没有锁上,则不能解锁,否则会导致程序崩溃 
  if (lock.owns_lock()) {  /**< 检查是否拥有mtx_1的所有权 */
    std::cout << "Locked with try_to_lock: " << lock.owns_lock() << std::endl; /**< 成功锁定 */
  } else {
    std::cout << "Failed to lock with try_to_lock: " << lock.owns_lock() << std::endl; /**< 失败锁定 */
  }
}

void example_adopt_lock() {
  mtx_1.lock(); /**< 手动锁定 mtx_1 */
  std::unique_lock<std::mutex> lock(mtx_1, std::adopt_lock); /**< 接管已经锁定的锁 */
  std::cout << "Locked with adopt_lock: " << lock.owns_lock() << std::endl;
  //lock.unlock(); /**< 手动解锁,也可以在 unique_lock 析构时自动解锁 */
}