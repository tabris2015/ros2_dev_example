# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

`modern-ros2`: a ROS 2 Jazzy tutorial series. One package per lesson
(`src/tutNN_topic`), Python and C++ side by side, building toward a fake
differential-drive robot with a synthetic camera. The curriculum, the decisions
behind it, and the phases are in `docs/plan.md`. The package README is the
lesson. The primary reader is the author, learning; the repo is public.

## Environment

Devcontainer with two targets from one Dockerfile (`.devcontainer/Dockerfile`):
`base` (headless, `ros:jazzy-ros-base` plus tooling) and `full` (adds
`ros-jazzy-desktop`, rqt, PlotJuggler, xacro). Both build on amd64 and arm64.
The workspace is mounted at `/workspaces/modern-ros2`. Every shell sources
`/opt/ros/jazzy/setup.bash` and, when it exists, `install/setup.bash`
(`$WS` points at the workspace). `RCUTILS_CONSOLE_OUTPUT_FORMAT` is
`[{severity}] [{name}]: {message}` so README output blocks are stable.

Ubuntu 24.04, Python 3.12 (`/opt/ros/jazzy/lib/python3.12/site-packages`).
No `pip install` for ROS dependencies; declare them in `package.xml` and let
`rosdep` handle them (`.devcontainer/post-create.sh` runs it).

## Build, test, run

```bash
colcon build --symlink-install                                 # Debug + compile_commands.json (colcon_defaults.yaml)
colcon build --symlink-install --packages-select tut01_nodes
colcon build --symlink-install --cmake-args -DTUT_BUILD_SOLUTIONS=ON   # exercise solutions too
source install/setup.bash
colcon test --return-code-on-test-failure && colcon test-result --verbose
ros2 run <pkg> <node>_py                                       # or <node>_cpp
ros2 launch <pkg> <name>.launch.py lang:=cpp
```

Always pass `--symlink-install`. Switching a package between symlink and plain
builds breaks its `install/<pkg>`; the fix is deleting `build/<pkg>` and
`install/<pkg>`, which the guard hook blocks, so ask the user to do it.
`ament_python` packages print a setuptools deprecation warning on stderr on
24.04; colcon reports "--- stderr" but the build succeeded.

## Lesson package contract

Scaffold with `/lesson <NN> <topic>`. The result:

```
src/tutNN_topic/
  CMakeLists.txt  package.xml  README.md
  src/<node>.cpp                 -> executable <node>_cpp
  tutNN_topic/<node>.py          -> module, run via scripts/<node>_py
  launch/<node>.launch.py        lang:=py|cpp
  config/params.yaml
  solution/                      built only with -DTUT_BUILD_SOLUTIONS=ON
    src/*.cpp  tutNN_topic_solution/*.py  scripts/*_py  README.md
```

README sections, always in this order: Concept, Expected output (verbatim
terminal blocks), Break it on purpose, Node graph (mermaid), CLI cheat sheet,
In the wild, Exercises. A lesson commit also touches `docs/journal.md`,
`docs/cpp-notes.md` (one section per lesson), the curriculum table in the root
`README.md`, and this file if commands changed. CI builds with
`-DTUT_BUILD_SOLUTIONS=ON`, so solutions must compile and lint.

Shared packages outside the numbering, once they exist: `tut_interfaces`
(lesson 2) and `tut_robot_sim` (lesson 4). Later lessons depend on them.

## Conventions

- Python: full type hints, docstrings, f-strings. Jazzy idiom: `rclpy.init`;
  `try: rclpy.spin(node)`; `except (KeyboardInterrupt, ExternalShutdownException)`;
  `finally: node.destroy_node(); rclpy.try_shutdown()`. Never `with rclpy.init()`
  (that is Kilted+), never `rclpy.shutdown()` after Ctrl-C.
- C++17. Lambdas capturing `[this]` for callbacks; `std::bind` appears once, in
  lesson 1, explained. Doxygen comments on classes and methods. Modern CMake:
  `target_link_libraries(x rclcpp::rclcpp ${std_msgs_TARGETS})`, not
  `ament_target_dependencies`.
- Every source file starts with `// Copyright 2026 Jose Laruta` (or `#`). C++
  packages set `ament_cmake_copyright_FOUND TRUE` before
  `ament_lint_auto_find_test_dependencies()`; cpplint still requires the line.
  Python-only packages keep `test_copyright.py` skipped. No full license blocks.
- Names in code are relative (`chatter`, not `/chatter`). The Python and C++
  node share a name; executables are `<node>_py` and `<node>_cpp`.
- Lint is `ament_lint_auto` with `ament_lint_common` over the whole package,
  `solution/` included, so launch files and solutions need docstrings too.
- C++ language explanations go in `docs/cpp-notes.md`, never inline in a lesson
  README.
- "In the wild" links: open and check each one before it lands in a README.
- Prose (READMEs, docs, commit messages) follows the `unslop` skill.

## Skills

| Skill | Usage |
|---|---|
| `/build` | `/build tut01_nodes`, `/build --release`, `/build --solutions` |
| `/test` | `/test tut01_nodes`, `/test` |
| `/lesson` | `/lesson 04 params` |
| `/new-package` | `/new-package my_pkg combined` |
| `/new-node` | `/new-node talker my_pkg cpp` |
| `/launch` | `/launch bringup my_pkg` |
| `/msg` | `/msg WheelSpeeds msg tut_interfaces` |

## Hooks

`.claude/settings.json`: PreToolUse on Bash blocks `rm -rf` of `build/`,
`install/`, `log/` (use `git clean` on untracked paths, or ask the user);
PostToolUse on Edit/Write runs `ament_uncrustify --reformat` on C++ files.

## CI

`.github/workflows/ci.yml` runs rosdep, `colcon build` (solutions on) and
`colcon test` inside `ros:jazzy-ros-base` on `ubuntu-24.04` and
`ubuntu-24.04-arm` for every push to `main` and every PR. `docker.yml` builds
both image targets natively on both architectures when `.devcontainer/`
changes, or on demand. `main` stays green; do not start the next lesson on a
red build.

## Journal

`docs/journal.md`, dated entries, newest first. The user writes what confused
them per lesson. Claude adds setup gotchas that cost real time.
