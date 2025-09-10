#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
    
class RobotNewsStationNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("hybrid_robot") # MODIFY NAME
        self.robot_name_ = "Astro"
        self.publisher_ = self.create_publisher(String, "hybrid_topic", 10)
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.listener_callback, 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot Hybrid News Station Node has been started.")

    def publish_news(self):
        msg = String()
        msg.data = f"Hi, this is {self.robot_name_}. All systems are operational."
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

    def listener_callback(self, msg: String):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()