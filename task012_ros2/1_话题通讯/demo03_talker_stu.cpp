/**
 * @brief 以某个固定频率发送学生信息
 * @date  2026-02-11
 * step：
 *   1.包含头文件
 *   2.初始化ros2发布端
 *   3.定义节点类
 *     3.1.创建发布方
 *     3.2.创建定时器
 *     3.3.组织消息并发布
 *   4.调用spin函数，并传入节点对象指针
 *   5.释放资源
 */

// 1.包含头文件
#include <rclcpp/rclcpp.hpp>
#include "base_interfaces/msg/student.hpp"
#include <chrono>

// 3.定义节点类
class TalkerStu : public rclcpp::Node {
public:
  TalkerStu() : Node("TalkerStu_node_cpp"), count_(0) {
  // 3.1.创建发布方
  publisher_ = this -> create_publisher<base_interfaces::msg::Student>("chatter_stu", 10);
  // 3.2.创建定时器
  timer_ = this -> create_wall_timer(std::chrono::seconds(3),std::bind(&TalkerStu::on_timer, this));
  }

private:
  rclcpp::Publisher<base_interfaces::msg::Student>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

private:
  void on_timer() {
    // 3.3.组织消息并发布
    auto stu = base_interfaces::msg::Student();
    stu.name = "zhangsan";
    stu.age = 8;
    stu.height = 2.20;

    publisher_ -> publish(stu);
    RCLCPP_INFO(this -> get_logger(),"发布的消息(%s,%d,%.2f),%ld", stu.name.c_str(), stu.age, stu.height, count_++);
  }
};

int main(int argc, char* argv[]) {
  // 2.初始化ros2
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<TalkerStu>());
  // 5.释放资源
  rclcpp::shutdown();
  return 0;
}