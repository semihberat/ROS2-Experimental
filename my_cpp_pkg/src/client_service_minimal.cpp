#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client");
    auto client = node -> create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(1s)){
        RCLCPP_WARN(node -> get_logger(), "Service not available waiting again...");
    }
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request -> a = 6;
    request -> b = 2;

    auto future = client-> async_send_request(request);
    rclcpp::spin_until_future_complete(node, future);

    auto response = future.get();
    RCLCPP_INFO(node -> get_logger(), "Result of %ld + %ld = %ld", request -> a, request -> b, response -> sum);
    
}