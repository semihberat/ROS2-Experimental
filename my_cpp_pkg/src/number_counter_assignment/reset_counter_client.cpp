#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class ResetCounterClient : public rclcpp::Node
{
public:
    ResetCounterClient() : Node("client_side") // MODIFY NAME
    {
        client_ = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
    }
    void resetCounter(bool reset){
        while(!client_->wait_for_service(1s)){
            RCLCPP_WARN(this->get_logger(),"Waiting for the server...");
        }
        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = reset;

        client_-> async_send_request(request,std::bind(&ResetCounterClient::resetCounterCallback, this, _1));
    }


private:
    void resetCounterCallback(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future){
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Success: %d", (int)response->success);
    };
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetCounterClient>(); // MODIFY NAME
    node->resetCounter(true);
    node->resetCounter(false);
    node->resetCounter(true);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}