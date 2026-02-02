#include <iostream>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc,argv);    // 初始化ROS2,分配资源
  auto node = std::make_shared<rclcpp::Node>("cpp_node");  // 创建节点
  RCLCPP_INFO(node->get_logger(),"你好,C++节点!");  // 打印日志
  rclcpp::spin(node);  // 运行节点

  rclcpp::shutdown();  // 关闭节点
  return 0;
}