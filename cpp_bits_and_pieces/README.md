# cpp_bits_and_pieces
C++ 进阶知识点与实战碎片 | Modern C++ Bits & Pieces

## 📋 仓库定位
作为计算机专业学生，同时也是机器人团队视觉组成员，我深刻意识到**扎实的 C++ 功底是视觉算法与工程开发的基石**。为了系统性地攻克现代 C++ 进阶特性、沉淀学习成果，我创建了这个文件夹。
这里将记录我从 C++ 基础到进阶、从语法原理到工程实践的全链路学习笔记，所有内容均以**可直接运行的代码示例 + 详细注释**呈现，既是个人知识库，也是视觉组同学共同学习、提升编程能力的技术阵地。

## 🎯 知识体系架构
本仓库围绕「**夯实基础 → 掌握核心 → 落地实战**」的学习路径，构建完整的现代 C++ 知识体系，精准匹配视觉开发与工程编程需求：
### 1. 基础进阶：筑牢编程根基
聚焦类与对象、模板编程两大核心，吃透面向对象底层原理与泛型编程思想，解决视觉开发中复杂数据结构、算法封装的基础问题；
### 2. 核心新特性：掌握现代 C++ 能力
重点攻克移动语义、智能指针、可调用对象、异步并发四大高频特性，提升代码性能、内存安全性与并发处理能力，适配视觉算法多线程、实时性需求；
### 3. 前沿特性：紧跟技术趋势
补充 C++20/23 实用新特性，提前掌握协程、模块、范围库等前沿能力，为后续视觉工程迭代、性能优化储备技术；
### 4. 工程规范：培养专业编码习惯
沉淀现代 C++ 编码最佳实践、避坑指南，从「能写代码」到「写好代码」，培养符合工业标准的工程化思维，适配团队协作与项目开发。

## 🚀 学习目标
1.  彻底掌握 C++11/14/17 核心特性，理解底层实现原理，告别「只会用不懂理」的碎片化学习；
2.  提升代码质量与性能，写出内存安全、高效简洁、可维护的 C++ 代码，支撑视觉算法开发与工程落地；
3.  构建个人专属 C++ 知识库，方便随时回顾、迭代优化，同时为团队同学提供学习参考；
4.  以练促学，通过大量实战代码，将理论知识转化为编程能力，适配机器人视觉组的开发需求。

## 📌 补充说明
- 内容持续更新：随着学习深入，会不断补充新知识点、实战案例（如视觉算法常用数据结构、多线程图像处理等）；
- 注重实战落地：所有示例均围绕「视觉开发、工程编程」场景设计，拒绝脱离实际的纯理论讲解；
- 团队共享开放：欢迎视觉组及其他感兴趣的同学一起完善内容、交流心得，共同提升 C++ 编程水平。

## 📂 仓库目录结构
```bash
cpp_bits_and_pieces/
├── 类与对象深度/                # 面向对象底层原理与进阶特性
│   ├── 特殊成员函数.cpp        # 六大函数：构造/析构/拷贝/移动/赋值/移动赋值（含右值引用版）
│   ├── 静态成员.cpp            # 静态变量/函数、类级共享、static 底层逻辑
│   ├── 多态与虚函数.cpp        # 虚函数/纯虚函数、虚表/虚指针原理、抽象类
│   ├── 继承与访问控制.cpp      # 访问权限、菱形继承、虚继承（解决数据冗余）
│   ├── 委托构造.cpp            # 委托构造函数、继承构造、构造函数复用
│   ├── explicit与类型转换.cpp  # 显式构造、类型转换运算符、explicit operator bool
│   ├── const成员函数.cpp       # const 正确性、mutable、this 指针常量性
│   └── noexcept与异常安全.cpp   # noexcept 修饰符/运算符、构造/析构异常安全、RAII 保证
├── 模板编程/                   # 泛型编程与模板元编程核心
│   ├── 函数模板.cpp            # 基础函数模板、模板参数推导（C++17 类模板推导）
│   ├── 类模板.cpp              # 类模板、成员模板、模板友元、模板特化前置
│   ├── 模板特化.cpp            # 全特化、偏特化、模板重载决议规则
│   ├── 可变参数模板.cpp        # 参数包展开、sizeof...、C++17 折叠表达式
│   ├── enable_if.cpp           # SFINAE 原理、条件启用模板、编译期分支
│   ├── type_traits基础.cpp     # 类型萃取：is_same/is_integral/remove_reference 等
│   ├── constexpr_if.cpp        # C++17 编译期分支、替代 SFINAE 简化代码
│   └── 概念Concepts.cpp        # C++20 Concepts、约束模板、替代 enable_if 更优雅
├── 移动语义与完美转发/         # 现代 C++ 内存优化核心
│   ├── 右值引用基础.cpp        # 左值/右值/纯右值/将亡值、std::move 本质
│   ├── 移动构造与移动赋值.cpp  # 移动语义、资源转移、noexcept 与移动语义
│   ├── 完美转发.cpp            # 万能引用、std::forward、转发引用规则
│   ├── move与forward本质.cpp   # move 是强制类型转换、forward 条件转发对比
│   ├── 拷贝消除RVO.cpp         # 返回值优化、NRVO、编译器自动优化规则
│   └── 值类别总结.cpp          # 左值/右值/将亡值/纯右值 完整体系+判断方法
├── 智能指针/                   # 自动内存管理与指针安全
│   ├── unique_ptr.cpp          # 独占指针、make_unique、移动语义、自定义删除器
│   ├── shared_ptr.cpp          # 共享指针、引用计数、make_shared、控制块原理
│   ├── weak_ptr.cpp            # 弱引用、解决循环引用、lock()/expired() 使用场景
│   ├── 循环引用问题.cpp        # shared_ptr 循环泄漏案例、weak_ptr 解决方案
│   ├── 自定义删除器.cpp        # unique_ptr/shared_ptr 删除器、lambda 实现删除器
│   └── enable_shared_from_this.cpp # this 转 shared_ptr、避免重复创建控制块
├── 可调用对象封装/             # 函数式编程与回调封装
│   ├── function.cpp            # std::function 包装函数/lambda/成员函数、空判断
│   ├── bind.cpp                # std::bind 绑定参数/成员函数、占位符使用、bind 陷阱
│   ├── ref_cref.cpp            # std::ref/std::cref、引用传递给 bind/function
│   ├── lambda表达式.cpp        # lambda 捕获规则、泛型 lambda、mutable/constexpr lambda
│   ├── 函数对象仿函数.cpp      # 重载 operator()、与 std::function 性能对比
│   └── invoke.cpp              # C++17 std::invoke、统一调用函数/成员函数/lambda
├── 异步并发/                   # 多线程与异步编程实战
│   ├── future.cpp              # std::future、get/wait/wait_for、获取异步结果
│   ├── packaged_task.cpp       # 包装任务、绑定 future、线程池任务封装核心
│   ├── promise.cpp             # 手动设置 future 值/异常、线程间传递结果
│   ├── async.cpp               # std::async、launch 策略、自动创建线程执行任务
│   ├── thread基础.cpp          # std::thread、join/detach、线程 ID、线程局部存储
│   ├── mutex.cpp               # std::mutex、lock_guard/unique_lock/scoped_lock 对比
│   ├── condition_variable.cpp  # 条件变量、生产者消费者模型、线程同步实战
│   ├── atomic.cpp              # 原子操作、无锁编程、memory_order 内存序
│   └── 线程池入门.cpp          # 简易线程池、任务队列、future 结果返回、线程复用
├── 异常处理/                   # 错误处理与异常安全
│   ├── 异常基础.cpp            # try/catch/throw、标准异常体系（std::exception 派生类）
│   ├── 自定义异常.cpp          # 继承 std::exception、重写 what()、异常携带上下文
│   ├── noexcept.cpp            # noexcept 修饰符/运算符、异常规格演进、noexcept 与移动
│   ├── 异常安全.cpp            # 强异常安全/基本异常安全/无异常安全、RAII 保证
│   └── 栈展开.cpp              # 栈展开过程、析构函数调用规则、异常传播
├── STL进阶/                    # 标准库底层与高效使用
│   ├── vector底层.cpp          # 扩容机制、迭代器失效规则、emplace_back 优于 push_back
│   ├── list与forward_list.cpp  # 双向/单向链表、哨兵节点、迭代器稳定性、性能对比
│   ├── map与set.cpp            # 红黑树底层、有序容器、lower_bound/upper_bound 高效查找
│   ├── unordered_map.cpp       # 哈希表、桶机制、哈希函数、碰撞处理、性能调优
│   ├── 迭代器基础.cpp          # 迭代器类型、iterator_traits、迭代器适配器
│   ├── 算法基础.cpp            # sort/find/for_each/transform、算法复杂度
│   ├── 算法谓词与lambda.cpp    # 算法配合谓词/lambda、自定义排序/过滤规则
│   └── emplace系列.cpp         # emplace/emplace_back 原地构造、减少拷贝/移动
├── 编译期编程/                 # 编译期计算与类型校验
│   ├── constexpr基础.cpp       # C++11 constexpr 函数/变量、编译期常量
│   ├── constexpr扩展.cpp       # C++14/17/20 constexpr 增强、编译期函数复杂度提升
│   ├── consteval.cpp           # C++20 立即函数、强制编译期执行、区别 constexpr
│   ├── 编译期字符串.cpp        # C++20 constexpr string、编译期字符串处理
│   ├── 编译期容器.cpp          # C++20 constexpr vector/array、编译期容器操作
│   └── 类型萃取.cpp            # 高级类型萃取：is_same/remove_const/decay 实战
├── C++20新特性/                # 前沿实用特性（工业界逐步落地）
│   ├── concepts.cpp            # 约束模板、标准概念（integral/range等）、自定义概念
│   ├── ranges.cpp              # 视图、过滤/转换/生成、延迟计算、ranges 算法
│   ├── coroutines.cpp          # 协程基础、co_await/co_yield/co_return、简单协程实现
│   ├── format.cpp              # std::format、std::print、类型安全格式化（替代 printf/cout）
│   ├── spaceship.cpp           # 三路比较运算符 operator<=>、默认比较规则
│   └── modules.cpp             # C++20 模块、import/export、替代头文件（编译器支持说明）
├── C++23新特性/                # 最新实用特性（前瞻学习）
│   ├── flat_map.cpp            # flat_map/flat_set、连续内存有序容器、性能优化
│   ├── mdspan.cpp              # 多维数组视图、数值计算/视觉领域必备
│   ├── expected.cpp            # 错误处理、替代异常/错误码、携带错误信息
│   ├── print.cpp               # std::print/std::println、轻量级格式化输出
│   ├── 显式对象参数.cpp        # explicit object parameter、简化 this 处理
│   └── 堆内存检查.cpp          # std::allocation_guard、堆内存泄漏检测
└── 现代C++编码规范/            # 工程化最佳实践与避坑指南
    ├── RAII原则.cpp            # 资源获取即初始化、智能指针/锁/文件/网络连接管理
    ├── const正确性.cpp         # const 变量/函数/参数、顶层/底层 const、const 最佳实践
    ├── 避免裸指针.cpp          # 用智能指针/span 替代裸指针、指针安全边界
    ├── 优先使用emplace.cpp     # emplace 系列 vs push 系列、减少拷贝/移动开销
    ├── 接口设计原则.cpp        # 最小依赖、高内聚低耦合、值语义优先、接口稳定性
    └── 现代C++避坑指南.cpp     # 常见陷阱（悬垂引用/迭代器失效/线程安全）、最佳实践
```

## 🚀 学习建议
1. **循序渐进**：按目录顺序从“类与对象深度”→“模板编程”→“核心新特性”→“前沿特性”学习，先夯实基础再攻坚难点；
2. **动手实践**：每个 `.cpp` 文件均可独立编译运行（需 C++17 及以上编译器），建议本地编译调试，结合注释理解底层原理；
3. **重点突破**：优先掌握“移动语义”“智能指针”“异步并发”三大模块（工业界高频使用），再补充模板与编译期编程；
4. **工程落地**：学习完基础特性后，结合“现代 C++ 编码规范”模块，重构自己的代码，落地最佳实践；
5. **版本适配**：优先掌握 C++11/14/17（编译器支持完善），C++20/23 作为前沿储备，按需学习。

## 🛠️ 开发与调试环境（Ubuntu + VS Code）
- 系统：Ubuntu 24.04 / 22.04
- 编译器：GCC 9+ / Clang 10+（支持 C++17/20/23）
- 编辑器：VS Code
- 调试方式：VS Code 按 **F5** 一键调试（依赖 `launch.json` + `tasks.json`）
- 编译命令示例（终端）：
  ```bash
  g++ -std=c++20 文件名.cpp -o 可执行文件 -pthread -g
  ```
  （并发模块必须加 `-pthread`，调试加 `-g`）
- 调试工具：GDB（VS Code 自动调用）

## 📌 补充说明
- 代码规范：所有示例代码遵循 Google C++ 风格指南，注释清晰，变量命名语义化，便于阅读与扩展；
- 避坑指南：关键知识点均标注“陷阱”“注意事项”，避免踩工业界常见坑；
- 持续更新：仓库会持续补充实战案例（如线程池优化、STL 性能调优），欢迎 Star 关注，一起完善现代 C++ 学习体系。

---
*点滴积累，终成体系 —— 从 C++ 进阶到精通，从可运行的代码碎片开始*