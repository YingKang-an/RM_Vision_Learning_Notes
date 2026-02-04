// 以下7个场景,会创建很多临时对象,大量持有动态资源,调用拷贝构造(没有移动构造),导致性能问题.

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

  Demo& operator=(const Demo& other) {
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

  // Demo(Demo&& other) {
  //   m_size = other.m_size;
  //   p_arr = other.p_arr;
  //   other.p_arr = nullptr;
  //   other.m_size = 0;
  //   std::cout << "移动构造函数" << std::endl;
  // }
  
  // Demo& operator=(Demo&& other) {
  //   if (this != &other) {
  //     // 先释放
  //     // 先释放自己的旧内存
  //     if (p_arr != nullptr) {
  //       free(p_arr);
  //       p_arr = nullptr;
  //     }
  //     // 把 other 的资源“偷”过来
  //     m_size = other.m_size;
  //     p_arr = other.p_arr;
  //     // 把 other 置空，防止它析构时把内存释放了
  //     other.m_size = 0;
  //     other.p_arr = nullptr;
  //     std::cout << "移动赋值函数" << std::endl;
  //   }
  //   return *this;
  // }

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
  // 执行 createDemo()：
  // 1. 构造 demo_1、demo_2
  // 2. return demo_2 → 拷贝构造【临时返回对象A】
  // 3. 析构 demo_2、demo_1
  // 4. createDemo() 执行完毕，返回【临时返回对象A】
  // 现在执行：Demo demo = 【临时返回对象A】
  // 5. 用【临时返回对象A】初始化 demo → 拷贝构造【临时对象B】（即 demo）
  // 6. 【临时返回对象A】使命完成 → 析构
  // 7. task_01 结束 → demo（临时对象B）析构
}

// 任务2 值方式传参
void do_logic(Demo demo) {   /**< 不可以写"引用"!createDemo() 返回的是临时对象,是右值,不能用引用接收 */
  // 模拟一些逻辑操作
}
void task_02() {
  do_logic(createDemo());
  // 1.createDemo() 执行完毕，返回【临时返回对象A】
  // 2.【临时返回对象A】使用拷贝构造函数到 do_logic()的参数 demo
}

// 任务3 引用方式传参
void do_logic_(const Demo& demo) {
//  C++ 专门给 const 引用开了个“绿灯”：
//  const 左值引用可以"绑定"临时对象,并延长临时对象的生命周期.
// 意思是：
// - 临时对象本来执行完这行就死
// - 但被 const 引用绑住后,它会活到这个引用失效为止
// - 而且不拷贝,直接用原对象
}
void task_03() {
  do_logic_(createDemo());
  // 1. createDemo() 内部：
  //    - 构造 demo_1、demo_2
  //    - return demo_2 → 拷贝构造【临时返回对象A】
  //    - 析构 demo_2、demo_1
  // 2. 执行 do_logic_(【临时返回对象A】)：
  //    - const Demo& demo 直接绑定【临时返回对象A】
  //    - **不拷贝，不调用任何构造函数！**
  // 3. do_logic_ 结束，demo 失效，【临时返回对象A】析构
}

// 任务4 容器操作
void task_04() {
  std::vector<Demo> vec;
  vec.push_back(Demo(10));
  // 默认构造函数
  // 拷贝构造函数
  // 析构函数
  // 析构函数
  vec.emplace_back(15);
  // 默认构造函数
  // 析构函数
  //------------------
  //  1. 如果所有元素的移动构造都加了  noexcept 
  //  → 全部用移动构造"扩容"(最快)
  //  2. 只要有一个元素的移动构造没加  noexcept 
  //  → 全部用拷贝构造"扩容"(最安全)
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

  return 0;
}

// -----task_01-----
// 默认构造
// 默认构造
// 拷贝构造
// 析构函数
// 析构函数
// 析构函数
// -----task_02-----
// 默认构造
// 默认构造
// 拷贝构造
// 析构函数
// 析构函数
// 析构函数
// -----task_03-----
// 默认构造
// 默认构造
// 拷贝构造
// 析构函数
// 析构函数
// 析构函数
// -----task_04-----
// 默认构造
// 拷贝构造
// 析构函数
// 默认构造
// 拷贝构造
// 析构函数
// 析构函数
// 析构函数
// -----task_05-----
// 默认构造
// 默认构造
// 拷贝构造
// 拷贝赋值
// 拷贝赋值
// 析构函数
// 析构函数
// 析构函数
// -----task_06-----
// 默认构造
// 默认构造
// 默认构造
// 拷贝构造
// 析构函数
// 析构函数
// 拷贝赋值
// 析构函数
// 析构函数
// -----task_07-----
// 默认构造
// 拷贝构造
// 析构函数
// 默认构造
// 拷贝赋值
// 析构函数
// 析构函数
//--------------------------------------------------------------------------------------------
// 在_1.cpp中,我们提到了7个场景,会创建很多临时对象,大量持有动态资源,调用拷贝构造(没有移动构造),导致性能问题.
// 代价非常昂贵!!!!

// 在这么多的临时对象中,可以试着"!资源转移!"
// 在类里面,有很多动态资源,但是临时对象的动态资源马上就要销毁了
// 不如直接把动态资源转移到别人那里,可以大大提高程序效率

// int* p_arr;   /**< 指向动态分配数组的指针 */
// int m_size;   /**< 指向动态分配数组的大小 */
// 直接把"旧对象"或"即将废弃的对象"的p_arr和m_size赋值给新对象,然后把旧对象的p_arr设置为nullptr
// 这样就避免了调用拷贝构造,提高了效率  
