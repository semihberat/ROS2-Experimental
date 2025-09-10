#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
    
class RobotNewsStationNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_news_station") # MODIFY NAME
        self.robot_name_ = "R2-D2"
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station Node has been started.")

    def publish_news(self):
        msg = String()
        msg.data = f"Hi, this is {self.robot_name_}. All systems are operational."
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()