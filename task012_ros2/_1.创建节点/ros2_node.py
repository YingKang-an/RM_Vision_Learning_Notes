import rclpy
from rclpy.node import Node

def main():
    rclpy.init()  # 初始化公作,分配资源
    node = Node('python_node')  # 创建节点
    node.get_logger().info('你好,python 节点!') # 打印info日志
    node.get_logger().warn('你好,python 节点!') # 打印warn日志
    node.get_logger().error('你好,python 节点!') # 打印error日志
    rclpy.spin(node) # 运行节点
    rclpy.shutdown() # 关闭节点

if __name__ == '__main__':
    main()