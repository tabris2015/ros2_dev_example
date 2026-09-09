// Copyright YEAR Jose Laruta

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// Starter node for lesson LESSON_NN. Replace with the lesson's real node.
class TopicNode : public rclcpp::Node
{
public:
  TopicNode()
  : Node("TOPIC")
  {
    timer_ = create_wall_timer(1s, [this]() {on_timer();});
    RCLCPP_INFO(get_logger(), "TOPIC started");
  }

private:
  /// Log and count one heartbeat.
  void on_timer()
  {
    ++count_;
    RCLCPP_INFO(get_logger(), "heartbeat #%d", count_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int count_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicNode>());
  rclcpp::shutdown();
  return 0;
}
