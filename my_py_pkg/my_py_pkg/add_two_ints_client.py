import rclpy 
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

#Callback fonksiyonuna ekstra argümanlar geçirebilmek için partial fonksiyonunu kullanıyoruz.
# future objesi, servis çağrısının sonucunu temsil eder ve callback fonksiyonunda bu objeye erişim sağlar.
from functools import partial

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        self.get_logger().info("Add Two Ints server is ready.")

    def call_add_two_ints(self, a: int, b: int):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("service not available, waiting again...")
        request = AddTwoInts.Request()  
        request.a = a
        request.b = b 
        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.add_two_ints_callback, request=request))

    def add_two_ints_callback(self, future, request):
        response = future.result()
        self.get_logger().info(f"This is the client side: {request.a} + {request.b} = {response.sum}")
        self.get_logger().info(f"{response.sum} has been received from the server.")


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    node.call_add_two_ints(5,3)
    node.call_add_two_ints(10,20)
    node.call_add_two_ints(1,7)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  