#include <iostream>
#include <cstdlib>
#include <vector>

class Demo {
public:
  Demo(int n) {
    m_size = n;
    p_arr = (int*)malloc(n * sizeof(int));
    if (nullptr == p_arr) {
      std::cerr << "内存分配失败" << std::endl;
      return;
    }
    for (int i = 0; i < n; i++) {
      p_arr[i] = i;
    }
    std::cout << "默认构造" << std::endl;
  }

  Demo(const Demo& other) {  
    m_size = other.m_size;
    p_arr = (int*)malloc(m_size * sizeof(int));  /**< 深拷贝 */
    if (nullptr == p_arr) {
      std::cerr << "内存分配失败" << std::endl;
      return;
    }
    for (int i = 0; i < m_size; i++) {
      p_arr[i] = other.p_arr[i];
    }
    std::cout << "拷贝构造" << std::endl;
  }

  Demo& operator= (const Demo& other) {
    if (this != &other) {  /**< 拷贝赋值,意味着在调用时,要拷贝赋值的对象已经存在 */
      // 1. 先释放当前对象的旧资源
      if (nullptr != p_arr) {
        free(p_arr);
        p_arr = nullptr;
      }
      // 2. 分配新内存并拷贝内容
      m_size = other.m_size;
      p_arr = (int*)malloc(m_size * sizeof(int));
      if (nullptr == p_arr) {
        std::cerr << "内存分配失败" << std::endl;
        m_size = 0; // 保证对象处于有效状态
        return *this;
      }
      for (int i = 0; i < m_size; ++i) {
        p_arr[i] = other.p_arr[i];
      }
      std::cout << "拷贝赋值" << std::endl;
    }
  return *this; /**< return一个对象,而函数声明返回的是引用时,编译器会自动把它当成"返回该对象的引用" */
  }

  //#######################################################
  //   拷贝构造 和 拷贝赋值 中 malloc 函数 是对系统的交互
  //   频繁的 拷贝构造 和 拷贝赋值 会导致性能问题.
  //#######################################################

  Demo(Demo&& other) {
    // 获得 other 资源所有权
    m_size = other.m_size;
    p_arr = other.p_arr;
    // 移除 other 资源所有权
    other.p_arr = nullptr;
    other.m_size = 0;

    std::cout << "移动构造" << std::endl;
  }
  
  Demo& operator= (Demo&& other) {
    if (this != &other) {
      if (nullptr != p_arr) {  /**< 先释放当前对象的旧资源 */
        // 如果不处理,指针会被覆盖,会"没人管",无法free,最终导致内存泄漏
        free(p_arr);           /**< 防止内存泄漏 */
        p_arr = nullptr;
      }
      // 获得 other 资源所有权
      m_size = other.m_size;
      p_arr = other.p_arr;
      // 移除 other 资源所有权
      other.m_size = 0;
      other.p_arr = nullptr;

      std::cout << "移动赋值" << std::endl;
    }
    return *this;
  }

  ~Demo() {
    if (nullptr != p_arr) {
      free(p_arr);
      p_arr = nullptr;
    }
    std::cout << "析构函数" << std::endl;
  }

public:
  int* p_arr;   /**< 指向动态分配数组的指针 */
  int m_size;   /**< 指向动态分配数组的大小 */
};

Demo createDemo() {
  Demo demo_1(10);
  Demo demo_2(15);
  if (demo_1.m_size > demo_2.m_size) {
    return demo_1;
  } else {
    return demo_2;
  }
}

// 任务1 返回局部对象
void task_01() {
  Demo demo = createDemo();
}

// 任务2 值方式传参
void do_logic(Demo demo) {   /**< 不可以写"引用"!createDemo() 返回的是临时对象,是右值,不能用引用接收 */
  // 模拟一些逻辑操作
}
void task_02() {
  do_logic(createDemo());
}

// 任务3 引用方式传参
void do_logic_(const Demo& demo) { }
void task_03() {
  do_logic_(createDemo());
}

// 任务4 容器操作
void task_04() {
  std::vector<Demo> vec;
  vec.push_back(Demo(10));
  vec.emplace_back(15);
}
 
// 任务5 交换对象
void task_05() {
  Demo demo1_(10);
  Demo demo2_(15);
  std::swap(demo1_, demo2_);
}

// 任务6 对象复值
void task_06() {
  Demo demo(10);
  demo = createDemo();
}

// 任务7 容器中对象赋值
void task_07() {
  std::vector<Demo> vec;
  vec.push_back(Demo(10));
  vec[0] = Demo(20);
}

// 任务8 调用拷贝构造函数
void task_08() {
  Demo demo(10);
  Demo demo_2 = demo;
}

// 任务9 调用拷贝赋值函数
void task_09() {
  Demo demo_1(10);
  Demo demo_2(20);
  demo_2 = demo_1;
}

int main() {

  std::cout << "-----task_01-----" << std::endl;
  task_01();
  std::cout << "-----task_02-----" << std::endl;
  task_02();
  std::cout << "-----task_03-----" << std::endl;
  task_03();
  std::cout << "-----task_04-----" << std::endl;
  task_04();
  std::cout << "-----task_05-----" << std::endl;
  task_05();
  std::cout << "-----task_06-----" << std::endl;
  task_06();
  std::cout << "-----task_07-----" << std::endl;
  task_07();
  std::cout << "-----task_08-----" << std::endl;
  task_08();
  std::cout << "-----task_09-----" << std::endl;
  task_09();

  return 0;
}


// -----task_01-----
// 默认构造
// 默认构造
// 移动构造
// 析构函数
// 析构函数
// 析构函数
// -----task_02-----
// 默认构造
// 默认构造
// 移动构造
// 析构函数
// 析构函数
// 析构函数
// -----task_03-----
// 默认构造
// 默认构造
// 移动构造
// 析构函数
// 析构函数
// 析构函数
// -----task_04-----
// 默认构造
// 移动构造
// 析构函数
// 默认构造
// 拷贝构造 < vector 扩容
// 析构函数
// 析构函数
// 析构函数
// -----task_05-----
// 默认构造
// 默认构造
// 移动构造
// 移动赋值
// 移动赋值
// 析构函数
// 析构函数
// 析构函数
// -----task_06-----
// 默认构造
// 默认构造
// 默认构造
// 移动构造
// 析构函数
// 析构函数
// 移动赋值
// 析构函数
// 析构函数
// -----task_07-----
// 默认构造
// 移动构造
// 析构函数
// 默认构造
// 移动赋值
// 析构函数
// 析构函数
// -----task_08-----
// 默认构造
// 拷贝构造
// 析构函数
// 析构函数
// -----task_09-----
// 默认构造
// 默认构造
// 拷贝赋值
// 析构函数
// 析构函数
// [1] + Done                       "/usr/bin/gdb" --interpreter=mi --tty=${DbgTerm} 0<"/tmp/Microsoft-MIEngine-In-b5t30yrj.h1y" 1>"/tmp/Microsoft-MIEngine-Out-t1tiuh05.fwm"