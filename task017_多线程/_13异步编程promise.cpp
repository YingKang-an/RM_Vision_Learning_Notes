// std::promise是一个用于设置异步操作结果的机制.
// 它允许你在一个线程中设置值或异常,然后在另一个线程中通过std::future对象检索这些值或异常. 
// std::promise通常与std::async,std::packaged_task或std::thread结合使用,以在异步操作中传递结果.
// 一个Promise对象是为了保存某个特定的结果值分配的,这个结果值是在未来的某个时间点计算得到的.
// 结果值可以是任意类型的值(void类型除外);也可以是指向常量和非常量(对象指针、函数或者成员函数)的指针;甚至是引用类型的"实值".
// 一个Promise对象可以创建一个或多个std::future对象,用来承载该Promise对象所保存的结果值. 
// 只有当set_value()或者set_exception()被调用,这些结果值才能被执行. 
// 一旦被执行,这些结果值就可以通过std::future::get函数添加到std::future对象中.

#include <iostream>
#include <thread>
#include <future>
#include <chrono>

void async_task(std::promise<int> prom) {
// 模拟异步操作,将结果设置到promise中
  std::this_thread::sleep_for(std::chrono::seconds(2));
  int result = 42;
  prom.set_value(result);
}

int main() {
  std::promise<int> prom;
  std::future<int> fut = prom.get_future();    /**< 获取与prom关联的future对象 */
  // std::move 的作用是把一个左值(这里的prom)转换成右值,触发移动构造
  // 移动不是复制,它是把原对象的资源(内部指针,状态等)转移给新对象,原对象变成空壳.
  // 这样就实现了所有权转移:主线程把promise的控制权交给子线程,自己不再持有,符合独占设计.
  std::thread t(async_task, std::move(prom));  /**< 启动异步线程,将promise传递到线程中 */

  // 在主线程中等待异步线程完成
  std::cout << "the result is: " << fut.get() << std::endl;
  t.join();  /**< 等待异步线程完成后 !!释放线程资源!! */

  return 0;
}

// 成员函数
// get_future():返回一个std::future对象,该对象与 std::promise 对象共享状态.
// 可以通过这个std::future对象来检索异步操作的结果.​
// set_value(T value):设置异步操作的结果.
// 调用此方法后,与 std::promise 关联的std::future对象将变为fulfilled状态,
// 并且可以通过调用 std::future::get() 来检索结果.
// set_exception(std::exception_ptr p):设置异步操作中抛出的异常.
// 调用此方法后,与 std::promise 关联的 std::future 对象将变为 rejected 状态,
// 并且可以通过调用 std::future::get() 来重新抛出异常.


// 注意事项 
// 1. std::promise 的生命周期: 确保 std::promise 对象在 std::future 对象需要它之前保持有效. 
// 一旦 std::promise 对象被销毁,任何尝试通过 std::future 对象访问其结果的操作都将失败.
// 2. 线程安全: std::promise 的 set_value 和 set_exception 方法是线程安全的,
// 但你应该避免在多个线程中同时调用它们,因为这通常意味着你的设计存在问题.
// 3. 异常处理: 当使用 std::promise 时,要特别注意异常处理. 
// 如果 std::promise 的 set_exception 方法没有被调用,但异步操作中确实发生了异常,
// 那么这些异常将不会被捕获,并可能导致程序崩溃.
// 因此,在使用 std::promise 时,建议始终在异步操作中捕获异常,
// 并使用 std::promise 的 set_exception 方法将异常设置到 std::future 对象中.
// 4. 性能考虑: 虽然 std::promise 和 std::future 提供了强大的异步编程能力,但它们也引入了额外的开销.
// 在性能敏感的应用程序中,要仔细考虑是否真的需要它们
// 5. std::move 的使用: 在将 std::promise 对象传递给线程函数时,通常需要使用 std::move 来避免不必要的复制. 
// 这是因为 std::promise 对象通常包含非托管资源(如共享状态),复制它们可能是昂贵的或不必要的.