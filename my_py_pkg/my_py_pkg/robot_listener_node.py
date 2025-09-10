#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotListenerNode(Node):
    def __init__(self):
        super().__init__("robot_listener")
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.listener_callback, 10)
        self.get_logger().info("Robot Listener Node has been started.")
    def listener_callback(self, msg: String):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()