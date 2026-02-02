学习纯C++环境下如何用yaml-cpp库**读取和写入YAML文件**

### 前置准备
#### 1. 确认yaml-cpp已安装
先在终端执行以下命令，确认库已安装（对应你之前CMakeLists.txt里的配置）：
```bash
# 检查头文件是否存在
ls /usr/include/yaml-cpp/yaml.h
# 检查静态库是否存在
ls /usr/lib/x86_64-linux-gnu/libyaml-cpp.a
```
如果提示不存在，先执行安装：
```bash
sudo apt update && sudo apt install libyaml-cpp-dev
```

#### 2. 极简CMakeLists.txt（仅支持yaml-cpp）
创建`CMakeLists.txt`，确保编译能链接yaml-cpp：
```cmake
cmake_minimum_required(VERSION 3.18)
project(yaml_demo LANGUAGES CXX)

# C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找yaml-cpp或直接指定路径
set(yaml_cpp_INCLUDE_DIRS "/usr/include")
set(yaml_cpp_LIBRARIES "/usr/lib/x86_64-linux-gnu/libyaml-cpp.a")

# 校验yaml-cpp
if(NOT EXISTS ${yaml_cpp_INCLUDE_DIRS}/yaml-cpp/yaml.h)
    message(FATAL_ERROR "yaml-cpp头文件未找到！")
endif()
if(NOT EXISTS ${yaml_cpp_LIBRARIES})
    message(FATAL_ERROR "yaml-cpp静态库未找到！")
endif()

# 生成可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 配置头文件和链接库
target_include_directories(${PROJECT_NAME} PRIVATE ${yaml_cpp_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${yaml_cpp_LIBRARIES})

# 输出到build/bin（可选）
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

---

### 一、核心操作1：读取YAML文件
#### 步骤1：创建测试用的YAML文件
在项目根目录创建`test_config.yaml`，内容如下：
```yaml
# 基础键值对
name: "YinKang'an"
age: 18
score: 99
is_student: true

# 嵌套层级
address:
  province: "辽宁省"
  city: "大连市"
  detail: "甘井子区"

# 列表/数组
hobbies:
  - C
  - C++
  - OpenCV

# 复杂列表（每个元素是对象）
scores:
  - subject: "高等数学"
    score: 90
  - subject: "线性代数"
    score: 92
```

#### 步骤2：C++读取代码（main.cpp）
```cpp
#include <iostream>
#include <fstream>
// 必须包含yaml-cpp头文件
#include <yaml-cpp/yaml.h>

int main() {
    // ===================== 1. 加载YAML文件 =====================
    YAML::Node config;  // YAML节点，核心数据结构，类似JSON的Object
    try {
        // 加载文件（路径：项目根目录的test_config.yaml）
        // 如果编译后可执行文件在build/bin，需把test_config.yaml复制到build/bin目录
        config = YAML::LoadFile("test_config.yaml");
    } catch (const YAML::BadFile& e) {
        // 捕获文件不存在/读取失败的异常，避免程序崩溃
        std::cerr << "❌ 加载YAML文件失败：" << e.what() << std::endl;
        return -1;
    }

    // ===================== 2. 读取基础类型（键值对） =====================
    // as<T>()：将节点值转换为指定类型
    // as<T>(默认值)：如果字段不存在，使用默认值（推荐，避免崩溃）
    std::string name = config["name"].as<std::string>();
    int age = config["age"].as<int>(18);  // 字段不存在则返回18
    float score = config["score"].as<float>();
    bool is_student = config["is_student"].as<bool>();

    std::cout << "===== 基础数据 =====" << std::endl;
    std::cout << "姓名：" << name << std::endl;
    std::cout << "年龄：" << age << std::endl;
    std::cout << "分数：" << score << std::endl;
    std::cout << "是否学生：" << (is_student ? "是" : "否") << std::endl;

    // ===================== 3. 读取嵌套层级 =====================
    std::string province = config["address"]["province"].as<std::string>();
    std::string city = config["address"]["city"].as<std::string>();
    std::string detail = config["address"]["detail"].as<std::string>();

    std::cout << "\n===== 嵌套数据 =====" << std::endl;
    std::cout << "地址：" << province << "-" << city << "-" << detail << std::endl;

    // ===================== 4. 读取列表/数组 =====================
    std::cout << "\n===== 简单列表（爱好） =====" << std::endl;
    // 遍历列表节点（size()获取列表长度）
    for (int i = 0; i < config["hobbies"].size(); i++) {
        std::string hobby = config["hobbies"][i].as<std::string>();
        std::cout << "爱好" << i+1 << "：" << hobby << std::endl;
    }

    // ===================== 5. 读取复杂列表（对象列表） =====================
    std::cout << "\n===== 复杂列表（成绩） =====" << std::endl;
    // 迭代器遍历
    for (const auto& score_node : config["scores"]) {
        std::string subject = score_node["subject"].as<std::string>();
        int score_val = score_node["score"].as<int>();
        std::cout << subject << "：" << score_val << std::endl;
    }

    return 0;
}
```

#### 编译运行
```bash
# 创建build目录
mkdir build && cd build
# 生成编译文件
cmake ..
# 编译
make -j4
# 运行（可执行文件在bin目录）
./bin/yaml_demo
```

#### 预期输出
```
===== 基础数据 =====
姓名：张三
年龄：20
分数：95.5
是否学生：是

===== 嵌套数据 =====
地址：广东省-深圳市-南山区XX路

===== 简单列表（爱好） =====
爱好1：篮球
爱好2：编程
爱好3：阅读

===== 复杂列表（成绩） =====
数学：98
语文：92
```

---

### 二、核心操作2：写入YAML文件
#### C++写入代码（替换main.cpp）
```cpp
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main() {
    // ===================== 1. 创建YAML节点 =====================
    YAML::Node config;  // 根节点

    // ===================== 2. 写入基础类型 =====================
    config["name"] = "李四";
    config["age"] = 22;
    config["score"] = 88.8;
    config["is_student"] = false;  // 布尔值

    // ===================== 3. 写入嵌套层级 =====================
    config["address"]["province"] = "北京市";
    config["address"]["city"] = "北京市";
    config["address"]["detail"] = "海淀区XX街";

    // ===================== 4. 写入简单列表 =====================
    // push_back：向列表添加元素
    config["hobbies"].push_back("游泳");
    config["hobbies"].push_back("听歌");
    config["hobbies"].push_back("旅行");

    // ===================== 5. 写入复杂列表（对象列表） =====================
    // 创建第一个成绩节点
    YAML::Node score1;
    score1["subject"] = "英语";
    score1["score"] = 90;
    config["scores"].push_back(score1);

    // 创建第二个成绩节点
    YAML::Node score2;
    score2["subject"] = "物理";
    score2["score"] = 85;
    config["scores"].push_back(score2);

    // ===================== 6. 保存到文件 =====================
    try {
        // 打开文件（输出模式，不存在则创建，存在则覆盖）
        std::ofstream fout("output_config.yaml");
        if (!fout.is_open()) {
            std::cerr << "❌ 打开文件失败！" << std::endl;
            return -1;
        }
        // 将YAML节点写入文件
        fout << config;
        // 关闭文件
        fout.close();
        std::cout << "✅ YAML文件写入成功！路径：output_config.yaml" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "❌ 写入YAML文件失败：" << e.what() << std::endl;
        return -1;
    }

    return 0;
}
```

#### 编译运行后，生成的`output_config.yaml`内容
```yaml
address:
  city: 北京市
  detail: 海淀区XX街
  province: 北京市
age: 22
hobbies:
  - 游泳
  - 听歌
  - 旅行
is_student: false
name: 李四
score: 88.8
scores:
  - score: 90
    subject: 英语
  - score: 85
    subject: 物理
```

---

### 三、核心知识点总结
1. **核心数据结构**：`YAML::Node` 是所有操作的核心，可表示键值对、列表、嵌套对象；
2. **读取关键**：
   - `YAML::LoadFile("文件名")` 加载文件；
   - `node["键"].as<T>()` 转换类型，推荐加默认值 `as<T>(默认值)` 防崩溃；
   - 列表用 `size()` 取长度，迭代器/下标遍历；
3. **写入关键**：
   - 直接给 `node["键"]` 赋值写入基础类型；
   - 列表用 `push_back()` 添加元素；
   - 用 `ofstream` 配合 `fout << node` 保存到文件；
4. **异常处理**：读取/写入时加 `try-catch`，捕获文件不存在、格式错误等异常。

### 避坑指南
1. 读取文件时，确保YAML文件路径和可执行文件路径匹配（推荐把YAML文件放到可执行文件同级目录）；
2. 写入文件时，确保程序有文件写入权限（比如不要写到`/root`等需要管理员权限的目录）；
3. YAML语法严格：冒号后加空格、缩进用空格（不要用Tab），否则读取会失败。