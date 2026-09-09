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

Match the style of the existing Python nodes under `src/` (typed, docstrings,
the Jazzy init/shutdown idiom):

**Create** `src/<package>/<package>/<node_name>.py`:
```python
#!/usr/bin/env python3
"""<One line: what this node does.>"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class <NodeClassName>(Node):
    """<What the node owns and publishes/subscribes.>"""

    def __init__(self) -> None:
        super().__init__('<node_name>')
        self.get_logger().info('<node_name> started')


def main(args: list[str] | None = None) -> None:
    """Entry point: init, spin, and shut down cleanly."""
    rclpy.init(args=args)
    node = <NodeClassName>()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

**Update** `setup.py` entry_points to add:
```python
'<node_name> = <package>.<node_name>:main'
```

## C++ node template

Match the style of the existing C++ nodes under `src/` (Doxygen comments,
lambdas for callbacks, modern CMake targets):

**Create** `src/<package>/src/<node_name>.cpp`:
```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"

/// <One line: what this node does.>
class <NodeClassName> : public rclcpp::Node
{
public:
  <NodeClassName>()
  : Node("<node_name>")
  {
    RCLCPP_INFO(get_logger(), "<node_name> started");
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

**Update** `CMakeLists.txt` to add (modern targets, not `ament_target_dependencies`):
```cmake
add_executable(<node_name> src/<node_name>.cpp)
target_link_libraries(<node_name> rclcpp::rclcpp ${std_msgs_TARGETS})

install(TARGETS <node_name>
  DESTINATION lib/${PROJECT_NAME}
)
```

## Naming convention

- Node name and ROS node name: `snake_case` (e.g. `lidar_processor`); no `_node` suffix
- Class name: `PascalCase` + `Node` suffix (e.g. `LidarProcessorNode`)
- In combined packages, executables carry a language suffix: `lidar_processor_py`, `lidar_processor_cpp`

## After creation

- Report the created file and updated build file
- Suggest running `/build <package-name>` to verify
- Remind user to run the node with: `ros2 run <package> <node_name>`
