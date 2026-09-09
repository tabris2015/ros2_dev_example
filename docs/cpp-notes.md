# C++ notes

Language explanations for the C++ side of each lesson, kept out of the lesson
READMEs so those stay about ROS 2. One section per lesson. Read it after the
lesson, once the constructs have context.

## Lesson 01: nodes

Written together with lesson 1. Planned topics: `std::shared_ptr` and
`std::make_shared`, the rclcpp `::SharedPtr` aliases, deriving from
`rclcpp::Node`, lambdas capturing `[this]` and why that is safe when the node
owns the timer, `std::bind` once, `std::chrono_literals`, `const T &` versus
`T::SharedPtr` callback parameters, `RCLCPP_INFO` versus `RCLCPP_INFO_STREAM`.
