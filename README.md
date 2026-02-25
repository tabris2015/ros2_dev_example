# Hello ROS 2

A ROS 2 Humble template workspace with three example packages, a full devcontainer setup, and Claude Code AI-assisted development tooling (skills, hooks, and MCP integration).

Use this as a starting point for new ROS 2 projects.

## Table of Contents

- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Packages](#packages)
- [Building and Testing](#building-and-testing)
- [Running Nodes](#running-nodes)
- [Debugging](#debugging)
- [Claude Code Integration](#claude-code-integration)
  - [Skills (Slash Commands)](#skills-slash-commands)
  - [Hooks](#hooks)
  - [MCP Servers](#mcp-servers)
- [Development Workflow](#development-workflow)
- [Using as a Template](#using-as-a-template)

---

## Quick Start

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
- (Optional) [Claude Code](https://claude.ai/code) CLI or VS Code extension

### Open in Devcontainer

1. Clone the repository:
   ```bash
   git clone <repo-url> hello_ros2
   cd hello_ros2
   ```
2. Open in VS Code:
   ```bash
   code .
   ```
3. When prompted, click **Reopen in Container** (or run `Dev Containers: Reopen in Container` from the command palette).

The devcontainer is based on `osrf/ros:humble-desktop-full` and includes:
- ROS 2 Humble (auto-sourced in every terminal)
- Python 3, pip, GDB
- VS Code extensions: RDE Pack, Python, C++ tools, GitLens, Claude Code

### First Build

Once inside the container:

```bash
colcon build
source install/setup.bash
```

### Run a Node

```bash
ros2 run hello_world hello_node
```

You should see `Publishing: "Hello World: <timestamp>"` at 1 Hz.

---

## Project Structure

```
hello_ros2/
├── src/                              # ROS 2 packages
│   ├── hello_world/                  # Python-only package
│   ├── hello_world_cpp/              # C++-only package
│   └── hello_world_combined/         # Mixed C++/Python package
├── .claude/                          # Claude Code configuration
│   ├── skills/                       # Custom slash commands
│   │   ├── build/SKILL.md
│   │   ├── test/SKILL.md
│   │   ├── new-package/SKILL.md      # + templates/
│   │   ├── new-node/SKILL.md
│   │   ├── launch/SKILL.md
│   │   └── msg/SKILL.md
│   ├── hooks/                        # Automation hooks
│   │   ├── guard-workspace-clean.sh
│   │   └── format-cpp.sh
│   ├── settings.json                 # Project hooks config
│   └── settings.local.json           # Local permission overrides
├── .devcontainer/                    # Docker devcontainer
│   ├── devcontainer.json
│   └── Dockerfile
├── .vscode/                          # VS Code config
│   ├── launch.json                   # Debug configurations
│   └── settings.json                 # Editor settings
├── .mcp.json                         # MCP server config (ROS 2)
├── CLAUDE.md                         # AI assistant instructions
└── README.md                         # This file
```

---

## Packages

All three packages implement the same node pattern — a publisher that sends `std_msgs/String` messages on the `chatter` topic at 1 Hz — demonstrating the three ROS 2 package types.

### hello_world (Python)

| Item | Value |
|---|---|
| Build type | `ament_python` |
| Node class | `hello_world/hello_node.py` → `HelloWorldNode` |
| Entry point | `setup.py` → `hello_node = hello_world.hello_node:main` |
| Dependencies | `rclpy`, `std_msgs` |
| Tests | pytest (flake8, pep257, copyright) |

### hello_world_cpp (C++)

| Item | Value |
|---|---|
| Build type | `ament_cmake` |
| Node class | `src/hello_node.cpp` → `HelloWorldCppNode` |
| Standard | C++17 with `-Wall -Wextra -Wpedantic` |
| Dependencies | `rclcpp`, `std_msgs` |
| Tests | ament_lint_auto (cpplint, cppcheck, uncrustify) |

### hello_world_combined (Mixed C++/Python)

| Item | Value |
|---|---|
| Build type | `ament_cmake` + `ament_cmake_python` |
| C++ node | `src/hello_node.cpp` → executable `hello_node_cpp` |
| Python node | `hello_world_combined/hello_node.py` → script `hello_node_py` |
| Dependencies | `rclcpp`, `rclpy`, `std_msgs` |
| Tests | ament_lint_auto + pytest (flake8, pep257, copyright) |

---

## Building and Testing

### Build

```bash
# Build everything
colcon build

# Build a single package
colcon build --packages-select hello_world

# Build with symlinks (faster iteration for Python)
colcon build --symlink-install

# Always source the overlay after building
source install/setup.bash
```

### Test

```bash
# Test everything
colcon test

# Test a single package
colcon test --packages-select hello_world_cpp

# View detailed results
colcon test-result --verbose
```

---

## Running Nodes

After building and sourcing the overlay:

```bash
# Python node (standalone)
ros2 run hello_world hello_node

# C++ node (standalone)
ros2 run hello_world_cpp hello_node

# Python node (combined package)
ros2 run hello_world_combined hello_node_py

# C++ node (combined package)
ros2 run hello_world_combined hello_node_cpp
```

In a separate terminal, verify messages are flowing:

```bash
ros2 topic echo /chatter
```

---

## Debugging

VS Code launch configurations are provided in `.vscode/launch.json` for all four node variants:

| Configuration | Type | Debugger |
|---|---|---|
| Debug: hello_node (Python) | Python | debugpy |
| Debug: hello_node (C++) | C++ | GDB |
| Debug: hello_node_py (Combined) | Python | debugpy |
| Debug: hello_node_cpp (Combined) | C++ | GDB |

**To debug:**
1. Build the workspace first (`colcon build`)
2. Open the Run and Debug panel (Ctrl+Shift+D)
3. Select the configuration from the dropdown
4. Press F5

For C++ debugging, the executable must exist in `install/` — rebuild if it's missing.

---

## Claude Code Integration

This project includes a full Claude Code setup for AI-assisted ROS 2 development. Claude Code can build, test, scaffold packages, generate launch files, and more — all through natural language or slash commands.

### Skills (Slash Commands)

Skills are defined in `.claude/skills/` and invoked with `/` in Claude Code:

#### `/build` — Build packages

```
/build hello_world          # Build one package
/build --all                # Build entire workspace
```

Automatically sources ROS 2, runs `colcon build --symlink-install`, and sources the overlay. Reports errors with file paths and line numbers.

#### `/test` — Run tests

```
/test hello_world_cpp       # Test one package
/test                       # Test everything
```

Builds first, then runs `colcon test` and `colcon test-result --verbose`. Parses and summarizes failures.

#### `/new-package` — Scaffold a package

```
/new-package my_robot_driver python      # Python package
/new-package my_robot_driver cpp         # C++ package
/new-package my_robot_driver combined    # Mixed C++/Python package
```

Creates the full directory structure under `src/` with `package.xml`, build files (`setup.py` or `CMakeLists.txt`), test scaffolding, and resource markers. Templates matching the existing packages are in `.claude/skills/new-package/templates/`.

#### `/new-node` — Add a node to a package

```
/new-node lidar_processor my_robot_driver cpp
/new-node path_planner my_robot_driver python
```

Creates the node source file with standard boilerplate (init, spin, shutdown, logger) and wires up the entry point in `setup.py` or `CMakeLists.txt`.

#### `/launch` — Generate a launch file

```
/launch bringup_launch my_robot
```

Creates a Python launch file in `src/<pkg>/launch/` with `LaunchDescription` and `Node` actions, and updates the build system to install it.

#### `/msg` — Create an interface definition

```
/msg RobotStatus msg my_robot_driver       # .msg file
/msg GetPose srv my_robot_driver            # .srv file
/msg FollowPath action my_robot_driver      # .action file
```

Creates the `.msg`/`.srv`/`.action` file, adds `rosidl_generate_interfaces()` to `CMakeLists.txt`, and adds the required `rosidl_default_generators` dependencies to `package.xml`.

### Hooks

Hooks are automatic actions configured in `.claude/settings.json`:

| Hook | Trigger | What it does |
|---|---|---|
| Workspace clean guard | Before any Bash command | Blocks `rm -rf build/ install/ log/` to prevent accidental deletion. Use `/build` to rebuild instead. |
| C++ auto-format | After editing `.cpp`/`.hpp`/`.h` files | Runs `ament_uncrustify --reformat` to maintain consistent C++ style. |

### MCP Servers

MCP (Model Context Protocol) servers give Claude Code live access to external tools.

#### ROS 2 MCP Server (optional)

The `.mcp.json` file configures the [LCAS ros2_mcp](https://github.com/LCAS/ros2_mcp) server, which lets Claude introspect a running ROS 2 system — list topics, echo messages, inspect interfaces, and monitor system health.

**Setup:**

1. Clone into the workspace:
   ```bash
   cd src
   git clone https://github.com/LCAS/ros2_mcp.git
   cd ..
   ```

2. Install dependencies:
   ```bash
   rosdep install -i --from-paths src/
   ```

3. Build:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Start the MCP server:
   ```bash
   ros2 launch ros2_mcp mcp_server.launch.py
   ```

5. Claude Code will auto-detect it via `.mcp.json` (serves on `http://localhost:8000/sse`).

To use a custom URL, set the environment variable before launching Claude Code:
```bash
export ROS2_MCP_URL=http://your-host:port/sse
```

#### GitHub MCP Server (recommended)

For PR reviews, issue tracking, and CI integration:

```bash
claude mcp add --transport http github https://api.githubcopilot.com/mcp/
```

Then authenticate inside Claude Code:
```
/mcp
```

This is user-scoped (not committed to the repo) since it requires individual authentication.

---

## Development Workflow

Here's the recommended workflow for developing with this template, both with and without Claude Code.

### Without Claude Code

1. **Create a package:**
   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp std_msgs
   cd ..
   ```

2. **Write your node** in `src/my_package/src/my_node.cpp`

3. **Build:**
   ```bash
   colcon build --packages-select my_package
   source install/setup.bash
   ```

4. **Test:**
   ```bash
   colcon test --packages-select my_package
   colcon test-result --verbose
   ```

5. **Run:**
   ```bash
   ros2 run my_package my_node
   ```

### With Claude Code

1. **Scaffold a package:**
   ```
   /new-package my_package cpp
   ```

2. **Add a node:**
   ```
   /new-node my_node my_package cpp
   ```

3. **Describe what the node should do** in natural language:
   ```
   Make my_node subscribe to /scan (sensor_msgs/LaserScan) and publish
   the closest obstacle distance on /closest_obstacle (std_msgs/Float64)
   ```

4. **Build and test:**
   ```
   /build my_package
   /test my_package
   ```

5. **Add a launch file:**
   ```
   /launch my_package_launch my_package
   ```

6. **Create custom messages if needed:**
   ```
   /msg ObstacleInfo msg my_package
   ```

7. **Debug issues** by asking Claude Code naturally:
   ```
   The node crashes when /scan has no ranges. Can you add a guard?
   ```

### Starting a New Project from This Template

1. Fork or clone this repository
2. Remove the example packages you don't need:
   ```bash
   rm -rf src/hello_world src/hello_world_cpp src/hello_world_combined
   ```
3. Clean build artifacts:
   ```bash
   rm -rf build/ install/ log/
   ```
4. Create your first package:
   ```
   /new-package my_robot_driver cpp
   ```
5. Update `CLAUDE.md` with your project-specific architecture notes
6. The skills, hooks, and MCP config carry over — no reconfiguration needed

---

## License

TODO: Add license.
