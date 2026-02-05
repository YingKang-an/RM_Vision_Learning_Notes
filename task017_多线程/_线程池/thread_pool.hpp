#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

// 线程池依赖的C++11标准库头文件
#include <vector>             // 存储工作线程的容器
#include <queue>              // 优先级队列的底层依赖
#include <thread>             // 线程相关操作（创建、join等）
#include <mutex>              // 互斥锁，保证线程安全
#include <condition_variable> // 条件变量，用于线程间通信
#include <future>             // future/packaged_task，用于获取任务返回值
#include <functional>         // function，用于存储任意可调用对象（任务）
#include <stdexcept>          // 异常类（如runtime_error）
#include <atomic>             // 原子变量，无锁线程安全
#include <utility>            // forward/move，用于完美转发

/**
 * 任务优先级枚举类
 * 优先级从低到高：LOW(1) < BELOW_NORMAL(2) < NORMAL(3) < ABOVE_NORMAL(4) < HIGH(5)
 * 数字越大，任务越先被执行
 */
enum class TaskPriority {
  LOW = 1,          // 低优先级：非紧急任务（如日志归档）
  BELOW_NORMAL = 2, // 次低优先级：普通非核心任务
  NORMAL = 3,       // 默认优先级：大多数常规任务
  ABOVE_NORMAL = 4, // 次高优先级：重要任务
  HIGH = 5          // 高优先级：紧急核心任务（如用户请求处理）
};

/**
 * 线程池核心类
 * 功能：管理一组预先创建的工作线程，接收任务并按优先级调度执行，提供状态监控接口
 * 设计原则：线程安全、接口简洁、支持任意可调用对象（函数、lambda、类成员函数等）
 */
class ThreadPool {
public:
  /**
   * 构造函数：创建线程池并初始化工作线程
   * @param thread_count 工作线程数量（默认值为CPU核心数，充分利用硬件资源）
   */
  explicit ThreadPool(size_t thread_count = std::thread::hardware_concurrency());

  /**
   * 析构函数：关闭线程池，等待所有任务完成后释放资源
   * 注意：析构时会自动调用shutdown()，确保资源不泄露
   */
  ~ThreadPool();

  /**
   * 提交带优先级的任务到线程池
   * @tparam F 任务函数类型（自动推导，无需手动指定）
   * @tparam Args 任务函数的参数类型（自动推导）
   * @param priority 任务优先级（TaskPriority枚举值）
   * @param f 任务函数（任意可调用对象）
   * @param args 任务函数的参数（可变参数）
   * @return std::future<typename std::result_of<F(Args...)>::type> 
   *         用于获取任务执行结果的future对象（任务异步执行，通过get()阻塞获取结果）
   */
  template<class F, class... Args>
  auto enqueue(TaskPriority priority, F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;

  /**
   * 提交默认优先级的任务到线程池（兼容无优先级需求的场景）
   * 本质：调用带优先级的enqueue，默认优先级为NORMAL(3)
   * @param f 任务函数
   * @param args 任务函数的参数
   * @return 用于获取结果的future对象
   */
  template<class F, class... Args>
  auto enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;

  /**
   * 线程池状态结构体（监控接口返回值）
   * 包含线程池当前的核心运行状态，用于调试和监控
   */
  struct PoolStatus {
    size_t active_threads;    // 正在执行任务的活跃线程数
    size_t pending_tasks;     // 已提交但未执行的等待任务数
    size_t completed_tasks;   // 自线程池创建以来已完成的总任务数
  };

  /**
   * 获取线程池当前状态
   * @return PoolStatus结构体，包含活跃线程数、等待任务数、已完成任务数
   * 线程安全：通过互斥锁保护队列访问，不会影响任务执行
   */
  PoolStatus get_status() const;

  /**
   * 关闭线程池
   * 行为：1. 不再接受新任务；2. 等待队列中已有的任务全部执行完成；3. 回收工作线程资源
   * 注意：调用后enqueue会抛出异常，析构时会自动调用，无需手动重复调用
   */
  void shutdown();

private:
  /**
   * 任务包装器结构体：封装任务的优先级和执行逻辑
   * 修复点：添加接收priority和func的构造函数，解决emplace构造失败的问题
   * 作用：为优先级队列提供排序依据，将"优先级+任务函数"绑定为一个整体
   */
  struct Task {
    int priority;                // 任务优先级（存储枚举的整型值，便于比较）
    std::function<void()> func;  // 任务执行逻辑（无参数无返回值，统一任务接口）

    /**
     * 自定义构造函数（核心修复！！！）
     * 解决：原代码中emplace(int, lambda)无匹配构造函数的编译错误
     * @param p 任务优先级（整型）
     * @param f 任务执行函数（std::function<void()>类型）
     */
    Task(int p, std::function<void()> f) 
      : priority(p), func(std::move(f)) // 用move减少拷贝，提高效率
    {}

    /**
     * 优先级队列排序规则：重载<运算符
     * std::priority_queue是大顶堆，返回true时当前任务优先级低于other，会被排到后面
     * 最终效果：优先级数字越大的任务，越先被取出执行
     */
    bool operator<(const Task& other) const {
      return priority < other.priority;
    }
  };

  std::vector<std::thread> workers_;       // 工作线程容器：存储所有预先创建的工作线程
  std::priority_queue<Task> tasks_;        // 优先级任务队列：按优先级存储待执行任务，队首是最高优先级任务
  mutable std::mutex queue_mutex_;         // 队列同步互斥锁：保护tasks_的读写操作，mutable允许const接口调用
  std::condition_variable condition_;       // 条件变量：用于线程间通知（有新任务/线程池关闭时唤醒等待线程）
  std::atomic<bool> stop_;                 // 线程池停止标志（原子变量，无锁线程安全，避免数据竞争）
  std::atomic<size_t> active_threads_;     // 活跃线程数（原子变量，无需加锁，实时统计正在工作的线程）
  std::atomic<size_t> completed_tasks_;    // 已完成任务数（原子变量，无需加锁，统计总完成任务量）
};

/**
 * 带优先级任务提交的模板实现（必须放在头文件，因为模板编译需要可见性）
 * 核心逻辑：1. 包装任务为packaged_task（用于获取返回值）；2. 封装为Task对象加入优先级队列；3. 唤醒空闲线程
 */
template<class F, class... Args>
auto ThreadPool::enqueue(TaskPriority priority, F&& f, Args&&... args)
  -> std::future<typename std::result_of<F(Args...)>::type> {

  // 推导任务函数的返回值类型（用于定义future和packaged_task）
  using return_type = typename std::result_of<F(Args...)>::type;

  // 1. 包装任务为shared_ptr<packaged_task>：
  // - packaged_task将可调用对象包装为"无参数无返回值"的函数，同时关联future
  // - 用shared_ptr是因为任务可能被多个线程间接引用（队列存储+future等待），延长生命周期避免悬空
  auto task = std::make_shared<std::packaged_task<return_type()>>(
    // std::bind绑定任务函数和参数，std::forward实现完美转发（保留参数的左值/右值属性）
    std::bind(std::forward<F>(f), std::forward<Args>(args)...)
  );

  // 2. 获取与packaged_task关联的future对象（用于后续获取任务结果）
  std::future<return_type> res = task->get_future();

  // 3. 加锁保护任务队列操作（避免多线程同时修改tasks_）
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    // 检查线程池是否已停止：如果已停止，禁止提交新任务，抛出异常
    if (stop_) {
      throw std::runtime_error("Cannot enqueue task: ThreadPool has been shut down");
    }

    // 4. 封装Task对象并加入优先级队列：
    // - static_cast<int>(priority)将枚举转换为整型，用于排序
    // - lambda表达式捕获task（shared_ptr），执行时调用(*task)()触发任务执行
    // - 调用修复后的Task构造函数：Task(priority, func)
    tasks_.emplace(
      static_cast<int>(priority),
      [task]() { (*task)(); }
    );
  } // 自动解锁：unique_lock出作用域时释放mutex

  // 5. 通知一个空闲线程：有新任务加入，可立即执行
  condition_.notify_one();

  // 6. 返回future对象，供调用者获取任务结果
  return res;
}

/**
 * 默认优先级任务提交的模板实现（兼容接口）
 * 直接调用带优先级的enqueue，指定优先级为NORMAL(3)，简化无优先级需求的调用
 */
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
  -> std::future<typename std::result_of<F(Args...)>::type> {

  return enqueue(TaskPriority::NORMAL, std::forward<F>(f), std::forward<Args>(args)...);
}

#endif // THREAD_POOL_HPP
