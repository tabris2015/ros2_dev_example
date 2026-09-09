# modern-ros2

A series of ROS 2 Jazzy lessons, in Python and C++ side by side, that build one
system: a fake differential-drive robot with a synthetic camera. Each lesson is
a package. Later lessons depend on earlier ones. Written for the author's own
learning and published for anyone walking the same road.

## Quick start

You need Docker and VS Code with the Dev Containers extension.

1. Clone and open:
   ```bash
   git clone git@github.com:tabris2015/modern-ros2.git
   code modern-ros2
   ```
2. Choose "Reopen in Container" and pick one of the two configurations:
   - `base`: headless. ROS 2 core, build and lint tooling, keyboard teleop. For a
     Jetson, SSH sessions, or CI. About 2.5 GB.
   - `full`: `base` plus rviz2, rqt, PlotJuggler, xacro. Needs an X server; run
     `xhost +local:` on the host once before opening. About 6 GB.
3. Build and test everything:
   ```bash
   colcon build --symlink-install
   colcon test --return-code-on-test-failure && colcon test-result --verbose
   ```
4. Reload the VS Code window once after the first build so the debugger sees the
   workspace overlay. Then start with lesson 1.

Both images build for amd64 and arm64. CI proves it on every change under
`.devcontainer/`.

## Curriculum

Status: planned, in progress, done. Lessons 3 to 9 accumulate into one working
robot; the plant is an integrator node, no simulator required.

| # | Package | Covers | Status |
|---|---|---|---|
| 01 | `tut01_nodes` | Nodes, topics, timers, the three package layouts | in progress |
| 02 | `tut02_interfaces` | msg/srv/action, rosidl, service client and server | planned |
| 03 | `tut03_launch_basics` | LaunchDescription, Node, param files | planned |
| 04 | `tut04_params` | Descriptors, ranges, set-callbacks, YAML. Plant node, teleop, PlotJuggler | planned |
| 05 | `tut05_qos` | Compatibility matrix, durability, sensor profiles. Camera node | planned |
| 06 | `tut06_executors` | Callback groups, MultiThreadedExecutor, the deliberate deadlock | planned |
| 07 | `tut07_sync` | message_filters, ApproximateTime and ExactTime | planned |
| 08 | `tut08_tf` | TF2, URDF, xacro, robot_state_publisher, rviz | planned |
| 09 | `tut09_time` | ROS time vs steady time, use_sim_time, `/clock` | planned |
| 10 | `tut10_actions` | Goal, feedback, cancel. `NavigateToWaypoint` | planned |
| 11 | `tut11_launch` | Arguments, namespaces, remapping, event handlers, Python/XML/YAML | planned |
| 12 | `tut12_lifecycle` | Managed nodes, ordered bringup, diagnostics | planned |
| 13 | `tut13_composition` | Components, containers, intra-process, type adaptation, loaned messages | planned |
| 14 | `tut14_capstone` | The whole robot as lifecycle components in one container | planned |
| 15 | `tut15_testing` | pytest, gtest, launch_testing, the CI explained | planned |
| 16 | `tut16_bags` | rosbag2, MCAP vs sqlite3, reading bags from code, regression tests | planned |
| 17 | `tut17_tools` | rqt, ros2 doctor, tf2_tools, introspection | planned |
| 18 | `tut18_multimachine` | Domain IDs, discovery, rmw selection, laptop plus Jetson | planned |
| 19 | `tut19_driver` | A real serial or USB device node, reconnection, error handling | planned |
| 20-22 | `tut20_control_*` | ros2_control: hardware interface, controller manager, diff_drive_controller | planned |
| A | appendix | Gazebo swapped in for the plant node | planned |
| B | appendix | ros2_mcp, agent introspection of a live graph | planned |
| C | appendix | [C++ notes](docs/cpp-notes.md) | in progress |
| D | appendix | Where next: Nav2, MoveIt, real-time, SROS2 | planned |

The full plan, with the decisions behind it and the phases, is in
[docs/plan.md](docs/plan.md). The author's running notes are in
[docs/journal.md](docs/journal.md).

## How a lesson is laid out

One package per lesson, `tutNN_topic`, in the combined `ament_cmake` plus
`ament_cmake_python` layout. The Python and C++ node share a node name and
behave the same; executables are `<node>_py` and `<node>_cpp`, and launch files
take `lang:=py|cpp`. Lesson 1 is the exception with three packages, because the
three layouts are what it teaches.

The package README is the lesson, and every one has the same sections:

- Concept
- Expected output, verbatim
- Break it on purpose
- Node graph
- CLI cheat sheet
- In the wild: real repositories using the pattern, each link checked
- Exercises, with solutions in `solution/` that build only with
  `--cmake-args -DTUT_BUILD_SOLUTIONS=ON`

Explanations of the C++ language itself stay out of the lesson READMEs and live
in [docs/cpp-notes.md](docs/cpp-notes.md), one section per lesson.

## Repository layout

```
src/                  lesson packages (tutNN_topic), later also tut_interfaces and tut_robot_sim
docs/                 plan.md, journal.md, cpp-notes.md
.devcontainer/        one Dockerfile with base and full targets, one devcontainer.json per target
.github/workflows/    ci.yml (colcon build and test on amd64 and arm64), docker.yml (image builds)
.claude/              Claude Code skills and hooks, see below
colcon_defaults.yaml  Debug builds with compile_commands.json by default
```

## Claude Code

The repository ships tooling for [Claude Code](https://claude.ai/code). It is
optional; nothing in the lessons depends on it.

| Skill | What it does |
|---|---|
| `/build [pkg] [--release] [--solutions]` | `colcon build --symlink-install`, reports errors with file and line |
| `/test [pkg]` | build, `colcon test`, `colcon test-result --verbose`, summarizes failures |
| `/lesson <NN> <topic>` | scaffolds a lesson package with both nodes, launch, config, README skeleton, solutions |
| `/new-package <name> [python|cpp|combined]` | scaffolds a plain package |
| `/new-node <node> <pkg> [python|cpp]` | adds a node and wires the entry point |
| `/launch <name> <pkg>` | adds a launch file and installs it |
| `/msg <Name> [msg|srv|action] <pkg>` | adds an interface and the rosidl wiring |

Two hooks: one blocks `rm -rf` on `build/`, `install/`, `log/`; one runs
`ament_uncrustify` on edited C++ files.

## Notes

- Docker Desktop on macOS has no real host networking, so DDS discovery between
  host and container does not work there. Everything inside one container still
  does.
- The `hello_world*` packages are the Humble-era template this repository grew
  from. Lesson 1 replaces them.

## License

Apache-2.0. See [LICENSE](LICENSE).
