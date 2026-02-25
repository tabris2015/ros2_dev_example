# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 Humble workspace with three packages implementing the same publisher node pattern. Both nodes publish `std_msgs/String` messages on the `chatter` topic at 1 Hz.

## Build Commands

```bash
# Source ROS 2 (already done in devcontainer .bashrc)
source /opt/ros/humble/setup.bash

# Build entire workspace (run from workspace root)
colcon build

# Build a single package
colcon build --packages-select hello_world
colcon build --packages-select hello_world_cpp
colcon build --packages-select hello_world_combined

# Source the workspace overlay after building
source install/setup.bash

# Run tests
colcon test
colcon test --packages-select hello_world
colcon test --packages-select hello_world_cpp
colcon test --packages-select hello_world_combined

# View test results
colcon test-result --verbose
```

## Running Nodes

```bash
# After sourcing install/setup.bash
ros2 run hello_world hello_node             # Python node (standalone package)
ros2 run hello_world_cpp hello_node         # C++ node (standalone package)
ros2 run hello_world_combined hello_node_py   # Python node (combined package)
ros2 run hello_world_combined hello_node_cpp  # C++ node (combined package)
```

## Architecture

- **`src/hello_world/`** — Python-only package (`ament_python` build type). Node class in `hello_world/hello_node.py`, entry point defined in `setup.py`. Tests use `pytest` with ament linters (flake8, pep257, copyright).
- **`src/hello_world_cpp/`** — C++-only package (`ament_cmake` build type). Node in `src/hello_node.cpp`, C++17 with `-Wall -Wextra -Wpedantic`. Tests use `ament_lint_auto`.
- **`src/hello_world_combined/`** — Mixed C++/Python package (`ament_cmake` + `ament_cmake_python`). Contains both nodes in a single package. C++ node built via CMake, Python module installed via `ament_python_install_package()`, Python entry point in `scripts/hello_node_py`. Tests use both `ament_lint_auto` and `ament_cmake_pytest`.

All packages depend on `rclcpp`/`rclpy` and `std_msgs`.

## Development Environment

Devcontainer based on `osrf/ros:humble-desktop-full` with host networking and privileged mode. VS Code debug configurations exist in `.vscode/launch.json` for all packages — Python nodes use debugpy, C++ nodes use GDB (cppdbg). C++ IntelliSense uses compile commands from `build/hello_world_combined/compile_commands.json`.

## Claude Code Skills

Custom skills are defined in `.claude/skills/`. Use these slash commands for common ROS 2 workflows:

| Skill | Usage | Description |
|---|---|---|
| `/build` | `/build hello_world` or `/build --all` | Build packages with colcon, source overlay |
| `/test` | `/test hello_world_cpp` | Build + test + report results |
| `/new-package` | `/new-package my_driver cpp` | Scaffold a new ROS 2 package (python/cpp/combined) |
| `/new-node` | `/new-node lidar_proc my_driver cpp` | Add a node to an existing package |
| `/launch` | `/launch bringup_launch my_robot` | Generate a Python launch file |
| `/msg` | `/msg RobotStatus msg my_driver` | Create .msg/.srv/.action with rosidl wiring |

## Hooks

Configured in `.claude/settings.json`:

- **PreToolUse (Bash)**: Blocks accidental `rm -rf build/ install/ log/` — use `/build` instead
- **PostToolUse (Edit/Write)**: Auto-formats C++ files with `ament_uncrustify` after edits

## MCP Servers

### ROS 2 MCP Server (optional)

A project-scoped MCP config is in `.mcp.json` for the [LCAS ros2_mcp](https://github.com/LCAS/ros2_mcp) server. To use it:

1. Install in a colcon workspace: `cd src && git clone https://github.com/LCAS/ros2_mcp.git && cd .. && rosdep install -i --from-paths src/ && colcon build`
2. Launch the MCP server: `ros2 launch ros2_mcp mcp_server.launch.py` (serves on `http://localhost:8000/sse`)
3. Claude Code will auto-detect it via `.mcp.json`

Override the URL with: `export ROS2_MCP_URL=http://your-host:port/sse`

### GitHub MCP Server (recommended)

Add per-user (not committed): `claude mcp add --transport http github https://api.githubcopilot.com/mcp/`

Then authenticate: `/mcp` in Claude Code and follow the browser flow.
