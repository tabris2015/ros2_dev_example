---
name: test
description: Run ROS 2 package tests and report results. Use when the user asks to test, validate, or check packages.
argument-hint: "[package-name or --all]"
allowed-tools: Bash(colcon *), Bash(source *), Bash(cat *)
---

# Test ROS 2 Packages

Run tests for one or more ROS 2 packages and report results.

## Arguments

- `$ARGUMENTS` — optional package name(s) or `--all`
- If a specific package name is given, use `--packages-select <name>`
- If `--all` or no argument is given, test the entire workspace

## Steps

1. Source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Build first to ensure latest code is compiled:
   ```bash
   colcon build --symlink-install [--packages-select <package-name>]
   ```

3. Source the overlay:
   ```bash
   source install/setup.bash
   ```

4. Run the tests:
   ```bash
   colcon test [--packages-select <package-name>]
   ```

5. Get detailed results:
   ```bash
   colcon test-result --verbose
   ```

6. If any tests fail:
   - Read the test log files under `log/latest_test/`
   - For pytest failures: check `log/latest_test/<package>/stdout_stderr.log`
   - Report which specific tests failed with file paths and error messages
   - Suggest fixes

## Test types in this workspace

- **Python packages**: pytest with ament linters (flake8, pep257, copyright)
- **C++ packages**: ament_lint_auto (cpplint, cppcheck, uncrustify, xmllint)
- **Combined packages**: both Python pytest and C++ lint tests
