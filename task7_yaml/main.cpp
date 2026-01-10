#include <iostream>
// 写入 YAML 时必须包含<fstream>，因为需要
// 用std::ofstream创建文件流、将 YAML 内容写入磁盘
#include <fstream>
// 必须包含yaml-cpp头文件
#include <yaml-cpp/yaml.h>

int main() {
    // ===================== 1. 加载YAML文件 =====================
    YAML::Node config;  // 创建YAML节点，核心数据结构
    try {
        // 加载文件（路径：项目根目录的../test_config.yaml）
        config = YAML::LoadFile("../config.yaml");
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

    // ===================== 4. 读取 列表/数组 =====================
    std::cout << "\n===== 简单列表（爱好） =====" << std::endl;
    // 遍历列表节点（size()获取列表长度）
    for (int i = 0; i < config["hobbies"].size(); i++) {
        std::string hobby = config["hobbies"][i].as<std::string>();
        std::cout << "爱好" << i+1 << "：" << hobby << std::endl;
    }

    // ===================== 5. 读取复杂列表(对象列表) =====================
    std::cout << "\n===== 复杂列表（成绩） =====" << std::endl;
    // 迭代器遍历
    for (const auto& score_node : config["scores"]) {
        std::string subject = score_node["subject"].as<std::string>();
        int score_val = score_node["score"].as<int>();
        std::cout << subject << "：" << score_val << std::endl;
    }

    return 0;
}