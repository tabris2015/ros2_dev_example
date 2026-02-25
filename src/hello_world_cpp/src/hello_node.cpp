#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloWorldCppNode : public rclcpp::Node
{
public:
  HelloWorldCppNode()
  : Node("hello_world_cpp_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&HelloWorldCppNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Hello World Cpp Node has started!");
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World: " + std::to_string(this->get_clock()->now().nanoseconds());
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: \"%s\"", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorldCppNode>());
  rclcpp::shutdown();
  return 0;
}
