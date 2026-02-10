/**
 * @brief 订阅发布方发布的消息，并在终端输出
 * @data  2026-02-10
 * step：
 *   1.包含头文件
 *   2.初始化ros2客户端
 *   3.定义节点类
 *     3.1.创建发布方
 *     3.2.解析并输出数据
 *   4.调用spin函数，并传入节点对象指针
 *   5.释放资源
 */

// 1.包含头文件
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

// 3.定义节点类
class Listener : public rclcpp::Node {
public:
  Listener() : Node("listener_node_cpp") {
    RCLCPP_INFO(this -> get_logger(), "订阅方创建！");
    // 3.1.创建发布方
    subscription_ = this -> create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&Listener::do_cb, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

private:
  void do_cb(const std_msgs::msg::String &msg) {
    // 3.2.解析并输出数据
    RCLCPP_INFO(this -> get_logger(), "订阅到的消息是：%s",msg.data.c_str());
  }
};
 
int main(int argc, char* argv[]) {
  // 2.初始化ros2客户端
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<Listener>());
  // 5.释放资源
  rclcpp::shutdown();
  return 0;
}