#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("client_side") // MODIFY NAME
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }
    void addTwoInts(int a, int b){
        while(!client_->wait_for_service(1s)){
            RCLCPP_WARN(this->get_logger(),"Waiting for the server...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        client_-> async_send_request(request,std::bind(&AddTwoIntsClient::addTwoIntsCallback, this, _1));
    }


private:
    void addTwoIntsCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
    };
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>(); // MODIFY NAME
    node->addTwoInts(3,5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}