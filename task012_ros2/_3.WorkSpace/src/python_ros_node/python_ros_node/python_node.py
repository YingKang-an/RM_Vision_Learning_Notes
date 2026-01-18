import rclpy
from rclpy.node import Node

class PythonNode(Node):
    def __init__(self):
        super().__init__("python_ros_node")
        self.counter = 0
        # ⏰ 创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("🎉 Python ROS2节点已启动！")

    def timer_callback(self):
        self.get_logger().info(f"🐍 Python节点运行中，计数: {self.counter}")
        self.counter += 1

def main():
    rclpy.init()  # 🔧 初始化工作
    node = PythonNode()
    node.get_logger().info("👋 你好,Python节点!")
    node.get_logger().warn("⚠️ 这是警告信息!")
    node.get_logger().error("❌ 这是错误信息!")
    
    try:
        rclpy.spin(node) # 🏃‍♂️ 运行节点
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown() # 🛑 关闭节点

if __name__ == "__main__":
    main()
