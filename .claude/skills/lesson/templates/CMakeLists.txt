cmake_minimum_required(VERSION 3.8)
project(PACKAGE_NAME)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Exercise solutions live in solution/ and are off by default:
#   colcon build --cmake-args -DTUT_BUILD_SOLUTIONS=ON
option(TUT_BUILD_SOLUTIONS "Build exercise solutions" OFF)

# ---------- Dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---------- C++ nodes ----------
add_executable(TOPIC_cpp src/TOPIC.cpp)
target_link_libraries(TOPIC_cpp rclcpp::rclcpp ${std_msgs_TARGETS})

install(TARGETS
  TOPIC_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Python nodes ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/TOPIC_py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Launch and config ----------
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---------- Exercise solutions ----------
if(TUT_BUILD_SOLUTIONS)
  ament_python_install_package(${PROJECT_NAME}_solution
    PACKAGE_DIR solution/${PROJECT_NAME}_solution
  )
  # add_executable(relay_cpp solution/src/relay.cpp)
  # target_link_libraries(relay_cpp rclcpp::rclcpp ${std_msgs_TARGETS})
  # install(TARGETS relay_cpp DESTINATION lib/${PROJECT_NAME})
  # install(PROGRAMS solution/scripts/relay_py DESTINATION lib/${PROJECT_NAME})
endif()

# ---------- Testing ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Source files carry a one-line copyright, not the full license block
  # ament_copyright wants. cpplint still checks that the line exists.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests for non-ROS logic go here:
  # find_package(ament_cmake_pytest REQUIRED)
  # ament_add_pytest_test(test_something test/test_something.py)
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_something test/test_something.cpp)
endif()

ament_package()
