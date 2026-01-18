#include "rclcpp/rclcpp.hpp"

class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("demo_ros2_node")
    {
        // 创建定时器，每秒执行一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DemoNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Demo ROS 2节点已启动！");
    }

private:
    void timer_callback()
    {
        static int count = 0;
        RCLCPP_INFO(this->get_logger(), "Hello ROS 2! 计数: %d", count++);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点并运行
    auto node = std::make_shared<DemoNode>();
    rclcpp::spin(node);
    
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}