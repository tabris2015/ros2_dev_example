---
name: launch
description: Generate or update a ROS 2 Python launch file. Use when the user needs launch files for nodes.
argument-hint: "<launch-file-name> [package-name]"
disable-model-invocation: true
---

# Generate a ROS 2 Launch File

Create a Python launch file that follows ROS 2 conventions.

## Arguments

- `$ARGUMENTS[0]` — **required** launch file name (e.g. `my_robot_launch` or `my_robot_launch.py`)
- `$ARGUMENTS[1]` — optional package name (if omitted, ask or infer from context)

## Steps

1. Determine the target package and its build type from `package.xml`
2. Create the launch file in the correct location
3. Update the build system to install the launch file

## Launch file location

- **ament_python packages**: `src/<pkg>/launch/<name>.py`
- **ament_cmake packages**: `src/<pkg>/launch/<name>.py`

Ensure the filename ends with `_launch.py` or `launch.py` by convention.

## Launch file template

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='<node_executable>',
            name='<node_name>',
            output='screen',
            parameters=[
                # {'param_name': 'param_value'},
            ],
            remappings=[
                # ('/original_topic', '/remapped_topic'),
            ],
        ),
    ])
```

## Wiring up installation

### For ament_python (`setup.py`)

Add to `data_files` in `setup.py`:
```python
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
```

And add imports at the top of `setup.py`:
```python
import os
from glob import glob
```

### For ament_cmake (`CMakeLists.txt`)

Add before `ament_package()`:
```cmake
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

## After creation

- Report the created launch file path
- Show how to run it: `ros2 launch <package> <launch_file.py>`
- Suggest running `/build <package>` first
