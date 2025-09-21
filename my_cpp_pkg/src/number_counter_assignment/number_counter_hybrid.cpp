#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class NumberCounterHybrid : public rclcpp::Node
{
public:
    NumberCounterHybrid()
        : Node("number_counter"), count_(0)
    {
        service_ = this -> create_service<example_interfaces::srv::SetBool>(
            "reset_counter",std::bind(&NumberCounterHybrid::callback_reset_numbers, this, std::placeholders::_1, std::placeholders::_2));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscription_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterHybrid::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
            500ms, std::bind(&NumberCounterHybrid::timer_callback, this));
    }

private:
    // private definitions
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    int64_t count_;

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
    // publisher callback function
    void timer_callback()
    {
        auto message = example_interfaces::msg::Int64();
        message.data = count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
        publisher_->publish(message);
    }

    // subscriber callback function
    void topic_callback(const example_interfaces::msg::Int64 &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg.data);
    }

    
    // service callback function
    void callback_reset_numbers(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                       example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request -> data){
            count_ = 0;
            response -> success = true;
            response -> message = "Counter reset to zero.";
            RCLCPP_INFO(this -> get_logger(), "Counter reset to zero as requested.");
        }else{
            response -> success = false;
            response -> message = "Counter not reset.";
            RCLCPP_INFO(this -> get_logger(), "Counter not reset as requested.");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberCounterHybrid>());
    rclcpp::shutdown();
    return 0;
}