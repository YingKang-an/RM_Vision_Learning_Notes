/**
 * @brief 订阅发布方发布的消息，并在终端输出
 * @date  2026-02-11
 * step：
 *   1.包含头文件
 *   2.初始化ros2客户端
 *   3.定义节点类
 *     3.1.创建订阅方
 *     3.2.解析并输出数据
 *   4.调用spin函数，并传入节点对象指针
 *   5.释放资源
 */

// 1.包含头文件
#include <rclcpp/rclcpp.hpp>
#include "base_interfaces/msg/student.hpp"
#include <iostream>

// 3.定义节点类
class ListenerStu : public rclcpp::Node {
public:
  ListenerStu() : Node("ListenerStu_node_cpp") { 
  // 3.1.创建订阅方
  subscription_ = this -> create_subscription<base_interfaces::msg::Student>("chatter_stu", rclcpp::QoS(10), std::bind(&ListenerStu::do_cb, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<base_interfaces::msg::Student>::SharedPtr subscription_;

private:
  void do_cb(const base_interfaces::msg::Student::SharedPtr stu) {
  // 3.2.解析并输出数据
  RCLCPP_INFO(this -> get_logger(), "订阅的学生信息:name = %s,age = %d, height = %.2f", stu -> name.c_str(), stu -> age, stu -> height);
  } 
};
 
int main(int argc, char* argv[]) {
  // 2.初始化ros2客户端
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<ListenerStu>());
  // 5.释放资源
  rclcpp::shutdown();
  return 0;
}