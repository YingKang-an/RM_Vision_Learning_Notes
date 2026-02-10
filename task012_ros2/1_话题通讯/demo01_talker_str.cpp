/**
 * @brief 以某个固定频率发送字符串“hello world！”，后缀编号，每发布一条消息，编号+1
 * @date  2026-02-10
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
#include <std_msgs/msg/string.hpp>
#include <chrono>

// 3.定义节点类
class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker_node_cpp"), count_(0) {
    RCLCPP_INFO(this -> get_logger(),"发布节点创建！");
    // 3.1.创建发布方
    publisher_ = this -> create_publisher<std_msgs::msg::String>("chatter",10);
    // 3.2.创建定时器
    timer_ = this -> create_wall_timer(std::chrono::seconds(1), std::bind(&Talker::on_timer, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

private:
  void on_timer() {
    // 3.3.组织消息并发布
    auto message = std_msgs::msg::String();
    message.data = "hello world! " + std::to_string(count_++);
    RCLCPP_INFO(this -> get_logger(),"发布方发布的消息：%s",message.data.c_str());
    publisher_ -> publish(message);
  }
};

int main(int argc, char* argv[]) {
  // 2.初始化ros2
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针
  rclcpp::spin(std::make_shared<Talker>());
  // 5.释放资源
  rclcpp::shutdown();
  return 0;
}