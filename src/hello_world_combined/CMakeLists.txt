cmake_minimum_required(VERSION 3.8)
project(hello_world_combined)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- Dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---------- C++ executable ----------
add_executable(hello_node_cpp src/hello_node.cpp)
ament_target_dependencies(hello_node_cpp rclcpp std_msgs)

install(TARGETS hello_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Python module ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/hello_node_py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Testing ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pytest_copyright test/test_copyright.py)
  ament_add_pytest_test(pytest_flake8 test/test_flake8.py)
  ament_add_pytest_test(pytest_pep257 test/test_pep257.py)
endif()

ament_package()
