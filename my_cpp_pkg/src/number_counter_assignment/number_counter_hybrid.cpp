#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class NumberCounterHybrid : public rclcpp::Node
{
public:
    NumberCounterHybrid()
        : Node("number_counter"), count_(0)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscription_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterHybrid::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
            500ms, std::bind(&NumberCounterHybrid::timer_callback, this));
    }

private:

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

    // private definitions
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    int64_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberCounterHybrid>());
    rclcpp::shutdown();
    return 0;
}