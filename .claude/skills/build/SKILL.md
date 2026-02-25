---
name: build
description: Build ROS 2 packages using colcon. Use when the user asks to build, compile, or rebuild packages.
argument-hint: "[package-name or --all] [--release]"
allowed-tools: Bash(colcon *), Bash(source *)
---

# Build ROS 2 Packages

Build one or more ROS 2 packages in this workspace using colcon.

## Arguments

- `$ARGUMENTS` — optional package name(s), `--all`, and/or `--release`
- If a specific package name is given, use `--packages-select <name>`
- If `--all` or no argument is given, build the entire workspace
- If `--release` is given, override the default Debug build type with `Release`

## Steps

1. Ensure ROS 2 is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Run the build from the workspace root (`/workspaces/hello_ros2`):
   ```bash
   # For a specific package (Debug, the default):
   colcon build --symlink-install --packages-select <package-name>

   # For entire workspace (Debug, the default):
   colcon build --symlink-install

   # Release build — append this cmake arg to override the default:
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. Source the workspace overlay after a successful build:
   ```bash
   source install/setup.bash
   ```

4. If the build fails:
   - Read the error output carefully
   - Report the failing file path and line number
   - Suggest a fix based on the error message

## Notes

- Always run from the workspace root directory
- Use `--symlink-install` for faster Python iteration (symlinks instead of copies)
- The default build type is `Debug` (set in `colcon_defaults.yaml`). Use `--release` to build with optimizations
- The workspace has three packages: `hello_world` (Python), `hello_world_cpp` (C++), `hello_world_combined` (mixed)
