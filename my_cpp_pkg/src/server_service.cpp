#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("add_two_ints_server") // MODIFY NAME
    {
        service_ = this -> create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",std::bind(&MyCustomNode::callback_add_two_ints, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this -> get_logger(), "Service server ready to add two ints.");
    }
 
private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
    void callback_add_two_ints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                               example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response -> sum = request -> a + request -> b;
        RCLCPP_INFO(this -> get_logger(), "Incoming request\na: %ld b: %ld", request -> a, request -> b);
        RCLCPP_INFO(this -> get_logger(), "Sending back response: %ld", response -> sum);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}