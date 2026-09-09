# modern-ros2: plan

A series of ROS 2 Jazzy lessons, in Python and C++, that build one system: a
fake differential-drive robot with a synthetic camera. Each lesson is a package.
Later lessons depend on earlier ones. Primary audience is the author; the repo
is public.

## Decisions

- Distro: Jazzy. Base image split into `base` (headless, Jetson) and `full` (rviz), both multi-arch.
- One package per lesson, named `tutNN_topic`, Python and C++ side by side.
- Two shared unnumbered packages: `tut_interfaces` (lesson 2) and `tut_robot_sim` (lessons 4 and 5).
- The package README is the lesson. Every README has: concept, expected output,
  break it on purpose, mermaid node graph, CLI cheat sheet, in the wild.
- Exercises with solutions in `solution/`. One running `docs/journal.md`.
- Python typed. C++17, lambdas, doc comments. C++ language notes in an appendix, not inline.
- CI builds and tests every lesson from day one. One commit per lesson.
- Lessons take about an hour.

## Phases

0. Foundation. Archive the ONNX work, delete `ml_inference`, rename to `modern-ros2`,
   Jazzy bump, two Dockerfiles, CI, docs scaffolding, `/lesson` skill.
1. Foundations. Lessons 1 to 4: nodes, interfaces, launch basics, parameters.
   Ends with a robot you drive from the keyboard.
2. What people get wrong. Lessons 5 to 7: QoS, executors, time sync. Camera node arrives.
3. The robot takes shape. Lessons 8 to 10: TF and URDF in rviz, sim time, waypoint action.
4. Structure. Lessons 11 to 14: launch in depth, lifecycle, composition, capstone.
5. Engineering. Lessons 15 to 17: testing, bags, tools.
6. Real systems. Lessons 18 to 22: multi-machine with a Jetson, hardware driver, ros2_control.
7. Appendices. Gazebo, ros2_mcp, C++ notes, where next.

## Lessons

| # | Package | Covers |
|---|---|---|
| 01 | `tut01_nodes` | Nodes, topics, timers, the three package layouts |
| 02 | `tut02_interfaces` | msg/srv/action, rosidl, service client and server |
| 03 | `tut03_launch_basics` | LaunchDescription, Node, param files |
| 04 | `tut04_params` | Descriptors, ranges, set-callbacks, YAML. Plant node, teleop, PlotJuggler |
| 05 | `tut05_qos` | Compatibility matrix, durability, sensor profiles. Camera node |
| 06 | `tut06_executors` | Callback groups, MultiThreadedExecutor, the deliberate deadlock |
| 07 | `tut07_sync` | message_filters, ApproximateTime and ExactTime |
| 08 | `tut08_tf` | TF2, URDF, xacro, robot_state_publisher, rviz |
| 09 | `tut09_time` | ROS time vs steady time, use_sim_time, `/clock` |
| 10 | `tut10_actions` | Goal, feedback, cancel. `NavigateToWaypoint` |
| 11 | `tut11_launch` | Arguments, namespaces, remapping, event handlers, Python/XML/YAML |
| 12 | `tut12_lifecycle` | Managed nodes, ordered bringup, diagnostics |
| 13 | `tut13_composition` | Components, containers, intra-process, type adaptation, loaned messages |
| 14 | `tut14_capstone` | The whole robot as lifecycle components in one container |
| 15 | `tut15_testing` | pytest, gtest, launch_testing, the existing CI explained |
| 16 | `tut16_bags` | rosbag2, MCAP vs sqlite3, reading bags from code, regression tests |
| 17 | `tut17_tools` | rqt, ros2 doctor, tf2_tools, introspection |
| 18 | `tut18_multimachine` | Domain IDs, discovery, rmw selection, laptop plus Jetson |
| 19 | `tut19_driver` | A real serial or USB device node, reconnection, error handling |
| 20-22 | `tut20_control_*` | ros2_control: hardware interface, controller manager, diff_drive_controller |
| A | appendix | Gazebo swapped in for the plant node |
| B | appendix | ros2_mcp, agent introspection of a live graph |
| C | appendix | C++ notes |
| D | appendix | Where next: Nav2, MoveIt, real-time, SROS2 |

## Out of scope

Nav2, MoveIt, SROS2, real-time. Pointers only, in appendix D.
