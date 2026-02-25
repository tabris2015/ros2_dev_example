---
name: new-node
description: Scaffold a new ROS 2 node in an existing package. Creates the node source file and wires up entry points.
argument-hint: "<node-name> <package-name> [python|cpp]"
disable-model-invocation: true
---

# Scaffold a New ROS 2 Node

Add a new node to an existing package with proper boilerplate and entry point registration.

## Arguments

- `$ARGUMENTS[0]` — **required** node name (snake_case, e.g. `lidar_processor`)
- `$ARGUMENTS[1]` — **required** package name (must already exist under `src/`)
- `$ARGUMENTS[2]` — optional language: `python` or `cpp` (inferred from package build type if omitted)

## Steps

1. Read the target package's `package.xml` to determine build type
2. If language is not specified, infer it:
   - `ament_python` → Python
   - `ament_cmake` without `ament_cmake_python` → C++
   - `ament_cmake` with `ament_cmake_python` → ask or default to Python
3. Create the node file and wire up the entry point

## Python node template

Follow the pattern from `src/hello_world/hello_world/hello_node.py`:

**Create** `src/<package>/<package>/<node_name>.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class <NodeClassName>(Node):
    def __init__(self):
        super().__init__('<node_name>_node')
        self.get_logger().info('<NodeClassName> has started!')


def main(args=None):
    rclpy.init(args=args)
    node = <NodeClassName>()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Update** `setup.py` entry_points to add:
```python
'<node_name> = <package>.<node_name>:main'
```

## C++ node template

Follow the pattern from `src/hello_world_cpp/src/hello_node.cpp`:

**Create** `src/<package>/src/<node_name>.cpp`:
```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"

class <NodeClassName> : public rclcpp::Node
{
public:
  <NodeClassName>()
  : Node("<node_name>_node")
  {
    RCLCPP_INFO(this->get_logger(), "<NodeClassName> has started!");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<<NodeClassName>>());
  rclcpp::shutdown();
  return 0;
}
```

**Update** `CMakeLists.txt` to add:
```cmake
add_executable(<node_name> src/<node_name>.cpp)
ament_target_dependencies(<node_name> rclcpp std_msgs)

install(TARGETS <node_name>
  DESTINATION lib/${PROJECT_NAME}
)
```

## Naming convention

- Node name: `snake_case` (e.g. `lidar_processor`)
- Class name: `PascalCase` + `Node` suffix (e.g. `LidarProcessorNode`)
- ROS node name: `snake_case_node` (e.g. `lidar_processor_node`)

## After creation

- Report the created file and updated build file
- Suggest running `/build <package-name>` to verify
- Remind user to run the node with: `ros2 run <package> <node_name>`
