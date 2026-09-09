---
name: test
description: Run ROS 2 package tests and report results. Use when the user asks to test, validate, or check packages.
argument-hint: "[package-name or --all]"
allowed-tools: Bash(colcon *), Bash(source *), Bash(cat *), Bash(git rev-parse *)
---

# Test ROS 2 packages

Build, test, and report results for one or more packages.

## Arguments

- `$ARGUMENTS`: optional package name(s) or `--all`
- A package name becomes `--packages-select <name>`
- `--all` or no argument tests the whole workspace

## Steps

1. Source ROS 2:
   ```bash
   source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
   ```

2. Build first so tests see the latest code (from `git rev-parse --show-toplevel`):
   ```bash
   colcon build --symlink-install [--packages-select <package-name>]
   ```

3. Source the overlay:
   ```bash
   source install/setup.bash
   ```

4. Run the tests:
   ```bash
   colcon test --return-code-on-test-failure [--packages-select <package-name>]
   ```

5. Show details:
   ```bash
   colcon test-result --verbose
   ```

6. On failure, read `log/latest_test/<package>/stdout_stderr.log`, report which
   tests failed with file paths and messages, and suggest fixes.

## Test types in this workspace

- Python: pytest with ament linters (flake8, pep257; copyright is skipped by design)
- C++: `ament_lint_auto` (cpplint, cppcheck, uncrustify, xmllint, lint_cmake)
- Combined packages run both
- Lesson packages may add unit tests (pytest, gtest) for non-ROS logic
