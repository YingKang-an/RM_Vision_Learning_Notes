#include <rclcpp/rclcpp.hpp>

class CppNode : public rclcpp::Node {
public:
    CppNode() : Node("cpp_ros_node") {
        // ⏰ 创建定时器，每秒执行一次
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CppNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "🎉 C++ ROS2节点已启动！");
    }

private:
    void timer_callback() {
        static int count = 0;
        RCLCPP_INFO(this->get_logger(), "⚡ C++节点运行中，计数: %d", count++);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);    // 🔧 初始化ROS2
    auto node = std::make_shared<CppNode>();
    RCLCPP_INFO(node->get_logger(), "👋 你好,C++节点!");
    rclcpp::spin(node);          // 🏃‍♂️ 运行节点
    rclcpp::shutdown();          // 🛑 关闭节点
    return 0;
}
