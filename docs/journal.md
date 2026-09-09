# Journal

Running notes, newest first. What confused me, what I got wrong, what finally
made it click. Setup gotchas that cost real time go here too.

## 2026-09-09: Phase 0, the foundation

- `osrf/ros:jazzy-desktop-full` is amd64-only. The `full` image is built up from
  `ros:jazzy-ros-base` plus `ros-jazzy-desktop` so the same Dockerfile runs on a
  Jetson. CI confirmed both targets build natively on arm64.
- On Jazzy, `ament_lint_auto` with `ament_lint_common` runs `ament_copyright` and
  cpplint on every C++ package. Both fail on a source file without a copyright
  line, and `ament_copyright` rejects SPDX-only headers. Settled on a one-line
  `// Copyright 2026 Jose Laruta` plus `set(ament_cmake_copyright_FOUND TRUE)`.
- Ubuntu 24.04 images ship an `ubuntu` user at uid 1000, so `useradd --uid 1000`
  fails until that user is removed.
- A bare `build/` in `.gitignore` also ignores `.claude/skills/build/`. Patterns
  are anchored now.
- PEP 668: `pip install` refuses to touch system Python on 24.04. ROS Python
  dependencies go through `rosdep` and apt, never pip.
