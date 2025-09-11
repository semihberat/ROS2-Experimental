#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class NumberPublisher : public rclcpp::Node
{
  public:
    NumberPublisher()
    : Node("number_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&NumberPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = example_interfaces::msg::Int64();
      message.data = count_++;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int64_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberPublisher>());
  rclcpp::shutdown();
  return 0;
}