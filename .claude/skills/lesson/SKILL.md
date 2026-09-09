---
name: lesson
description: Scaffold a lesson package src/tutNN_topic (combined ament_cmake + ament_cmake_python) with the README skeleton, launch/, config/, solution/ and starter nodes in both languages.
argument-hint: "<NN> <topic>"
disable-model-invocation: true
allowed-tools: Bash(bash .claude/skills/lesson/scaffold.sh *), Bash(git rev-parse *), Read, Glob
---

# Scaffold a lesson package

Creates `src/tut<NN>_<topic>` in the combined layout every lesson uses, with a
starter node in Python and C++, a launch file, a parameter file, the README
skeleton with the six required sections, and the `solution/` layout.

## Arguments

- `$ARGUMENTS[0]`: lesson number, one or two digits (`4` and `04` both give `tut04_`)
- `$ARGUMENTS[1]`: topic in snake_case (`params`, `launch_basics`)

## Steps

1. Run the scaffolder from the repository root:
   ```bash
   bash .claude/skills/lesson/scaffold.sh <NN> <topic>
   ```
   It refuses to overwrite an existing package and prints the files it created.

2. Read the generated `README.md` and tell the user which sections to fill in.

3. Remind the user of the follow-ups the scaffolder does not do:
   - Replace the starter nodes with the lesson's real nodes (same name in both
     languages, executables suffixed `_py` and `_cpp`).
   - If the lesson has no launch content yet, delete `launch/` and `config/`
     and the matching `install(DIRECTORY ...)` line.
   - Add the lesson to the curriculum table in the root `README.md`.
   - Fill the `if(TUT_BUILD_SOLUTIONS)` block once solutions exist.

4. Suggest `/build tut<NN>_<topic>` and then `/test tut<NN>_<topic>`.

## Conventions baked into the templates

- One-line copyright header on every source file; `ament_copyright` is disabled
  in CMake, cpplint still checks the line exists.
- Modern CMake targets (`target_link_libraries` with `rclcpp::rclcpp`), not
  `ament_target_dependencies`.
- Typed Python with the Jazzy shutdown idiom (`try_shutdown`,
  `ExternalShutdownException`). Lambdas for C++ callbacks.
- Lint runs through `ament_lint_auto` over the whole package, including
  `solution/`. No duplicate pytest lint trio.
- Python solutions live in `solution/<pkg>_solution/` and install as that
  module when `TUT_BUILD_SOLUTIONS=ON`, so they run with `ros2 run` too.
