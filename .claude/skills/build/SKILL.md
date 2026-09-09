---
name: build
description: Build ROS 2 packages using colcon. Use when the user asks to build, compile, or rebuild packages.
argument-hint: "[package-name or --all] [--release] [--solutions]"
allowed-tools: Bash(colcon *), Bash(source *), Bash(git rev-parse *)
---

# Build ROS 2 packages

Build one or more packages in this workspace with colcon.

## Arguments

- `$ARGUMENTS`: optional package name(s), `--all`, `--release`, `--solutions`
- A package name becomes `--packages-select <name>`
- `--all` or no argument builds the whole workspace
- `--release` overrides the default Debug build type
- `--solutions` also builds the exercise solutions (`-DTUT_BUILD_SOLUTIONS=ON`)

## Steps

1. Source ROS 2 (the devcontainer shell already does this):
   ```bash
   source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
   ```

2. Run from the workspace root, which is `git rev-parse --show-toplevel`:
   ```bash
   # One package (Debug, the default)
   colcon build --symlink-install --packages-select <package-name>

   # Whole workspace
   colcon build --symlink-install

   # Release
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

   # With exercise solutions
   colcon build --symlink-install --cmake-args -DTUT_BUILD_SOLUTIONS=ON
   ```

3. Source the overlay after a successful build:
   ```bash
   source install/setup.bash
   ```

4. If the build fails, read the error, report the failing file path and line
   number, and suggest a fix.

## Notes

- Always pass `--symlink-install`. Switching a package between symlink and plain
  builds breaks its `install/<pkg>`; the fix is deleting `build/<pkg>` and
  `install/<pkg>`, which the guard hook blocks, so ask the user first.
- The default build type is Debug (from `colcon_defaults.yaml`).
- `ament_python` packages print a setuptools deprecation warning on stderr on
  Ubuntu 24.04. colcon reports "--- stderr" for them. That is not a failure.
- List packages with `colcon list --names-only`.
