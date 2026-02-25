---
name: msg
description: Create a new ROS 2 message, service, or action interface definition. Scaffolds the .msg/.srv/.action file and wires up rosidl generation.
argument-hint: "<InterfaceName> [msg|srv|action] <package-name>"
disable-model-invocation: true
---

# Create a ROS 2 Interface Definition

Scaffold a `.msg`, `.srv`, or `.action` file and wire up the build system for code generation.

## Arguments

- `$ARGUMENTS[0]` — **required** interface name in PascalCase (e.g. `RobotStatus`)
- `$ARGUMENTS[1]` — optional type: `msg` (default), `srv`, or `action`
- `$ARGUMENTS[2]` — optional package name (if omitted, ask or infer)

## Pre-checks

1. The target package **must** use `ament_cmake` build type. If it's `ament_python`, warn the user that message generation requires CMake and suggest creating a separate `_msgs`/`_interfaces` package.
2. Check if `rosidl_generate_interfaces` is already in `CMakeLists.txt`.

## File locations

- Messages: `src/<pkg>/msg/<InterfaceName>.msg`
- Services: `src/<pkg>/srv/<InterfaceName>.srv`
- Actions: `src/<pkg>/action/<InterfaceName>.action`

## Template contents

### Message (.msg)

```
# <InterfaceName> message definition
# Field definitions below (type name):

string data
float64 value
```

### Service (.srv)

```
# Request
string input
---
# Response
bool success
string message
```

### Action (.action)

```
# Goal
int32 target
---
# Result
bool success
string message
---
# Feedback
float32 progress
```

## Build system changes

### package.xml — add these dependencies if not already present:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>rosidl_default_runtime</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

For action interfaces, also add:
```xml
<depend>action_msgs</depend>
```

### CMakeLists.txt — add interface generation:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/<InterfaceName>.msg"
  # Add more interfaces here as needed
)
```

If `rosidl_generate_interfaces` already exists, append the new interface file to the existing list instead of creating a new block.

For action interfaces:
```cmake
find_package(action_msgs REQUIRED)
```

## After creation

- Report the created interface file and build system changes
- Remind user to rebuild: `/build <package>`
- Show how to verify: `ros2 interface show <package>/msg/<InterfaceName>`
- Note: other packages that use this interface must add `<depend><package></depend>` to their `package.xml`
