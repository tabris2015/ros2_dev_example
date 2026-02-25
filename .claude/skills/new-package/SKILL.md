---
name: new-package
description: Scaffold a new ROS 2 package with proper structure. Creates package.xml, build files, and directory layout.
argument-hint: "<package-name> [python|cpp|combined]"
disable-model-invocation: true
allowed-tools: Bash(mkdir *), Bash(touch *), Bash(chmod *)
---

# Scaffold a New ROS 2 Package

Create a complete ROS 2 package under `src/` with the correct structure, build files, and test scaffolding.

## Arguments

- `$ARGUMENTS[0]` — **required** package name (snake_case, e.g. `my_robot_driver`)
- `$ARGUMENTS[1]` — optional package type: `python` (default), `cpp`, or `combined`

## Before creating

1. Verify the package name uses snake_case
2. Check that `src/$ARGUMENTS[0]` does not already exist

## Reference examples

Read these existing packages to match this workspace's conventions exactly:

- **Python**: `src/hello_world/` — `setup.py`, `setup.cfg`, `package.xml`, `hello_world/hello_node.py`
- **C++**: `src/hello_world_cpp/` — `CMakeLists.txt`, `package.xml`, `src/hello_node.cpp`
- **Combined**: `src/hello_world_combined/` — `CMakeLists.txt`, `package.xml`, Python module + C++ source + scripts

## Package structures

### Python package (`ament_python`)

```
src/<pkg>/
├── package.xml              # build_type: ament_python, depend: rclpy std_msgs
├── setup.py                 # entry_points with console_scripts
├── setup.cfg                # [develop] and [install] script dirs
├── resource/<pkg>           # empty marker file for ament index
├── <pkg>/
│   └── __init__.py          # empty
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### C++ package (`ament_cmake`)

```
src/<pkg>/
├── package.xml              # buildtool_depend: ament_cmake, depend: rclcpp std_msgs
├── CMakeLists.txt           # C++17, -Wall -Wextra -Wpedantic, ament_lint_auto
├── src/                     # source files go here
└── include/<pkg>/           # public headers
```

### Combined package (`ament_cmake` + `ament_cmake_python`)

```
src/<pkg>/
├── package.xml              # buildtool: ament_cmake + ament_cmake_python
├── CMakeLists.txt           # C++ exe + ament_python_install_package + pytest
├── src/                     # C++ sources
├── <pkg>/
│   └── __init__.py          # Python module
├── scripts/                 # Python entry point scripts
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## Template files

Refer to the templates in this skill's [templates/](templates/) directory for exact file contents. Read them and adapt the package name.

## After creation

- Report what was created
- Suggest the user run `/build <package-name>` to verify it compiles
