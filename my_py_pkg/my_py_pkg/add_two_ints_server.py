#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial
class AddTwoIntsServer(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("add_two_ints_srv") # MODIFY NAME
        self.server = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)
        self.get_logger().info("Add Two Ints server has been started.")

    def add_two_ints_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} and {request.b} have been received.")
        self.get_logger().info(f"This is the response from the server side: {response.sum}")
        return response

        
        
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()