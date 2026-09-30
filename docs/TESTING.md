# Testing and validation

## Verified live run

[ROS 2 Gazebo integration run #9](https://github.com/Jillian06/ros2-autonomous-navigation/actions/runs/36685203210) completed successfully on 2026-09-30 using Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic. The robot physically moved around the world walls and stopped 0.140 m from the requested goal.

The [raw recording](../media/demo_results.json) contains 92 path poses, 1,760 world-pose messages, 353 LiDAR scans with 46,701 finite samples, and 900 nonzero velocity commands. The final command is zero. Observer wall time includes ROS/Gazebo startup and is not a benchmark.

The [trajectory plot](../media/trajectory.png) shows the received A* path, Gazebo world pose, and wheel odometry. The [16.6-second GIF](../media/gazebo_demo.gif) is a Gazebo GUI capture at 3× speed. The [screenshot](../media/gazebo_demo.png) comes from the same capture. Run/commit/artifact provenance is preserved in [run_provenance.json](../media/run_provenance.json).

## What success checks

- A nontrivial A* path is published.
- Live LiDAR scans contain finite obstacle ranges.
- Nonzero velocity commands are issued.
- The controller stops near the requested goal.
- ROS /odom is within 0.15 m of the goal.
- Gazebo /ground_truth exists and is within 0.20 m of the goal.

The controller uses ideal simulation localization: ROS /odom is bridged from Gazebo /ground_truth. The separate /wheel_odom stream is recorded for comparison and does not determine success. This explicitly verifies physical movement rather than assuming spinning wheels mean the chassis moved.

The known map inflates static obstacles to provide clearance. This validation is one deterministic simulator scenario, not a collision-safety proof, robustness benchmark, SLAM/localization evaluation, or physical-robot test.

## Reproduce the run

On Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-rviz2 python3-colcon-common-extensions python3-matplotlib xvfb ffmpeg
bash scripts/ci_demo.sh
```

The script builds the package, subscribes before launch, captures the GUI, writes JSON/PNG/GIF evidence, and asserts the success conditions. The Actions workflow runs the same script.

For an interactive desktop:

```bash
ros2 launch autonomous_navigation simulation.launch.py
ros2 topic hz /odom
ros2 topic hz /ground_truth
ros2 topic hz /wheel_odom
ros2 topic hz /scan
ros2 topic echo /planned_path --once
ros2 topic echo /cmd_vel --once
```

## Unit-tested core

Seven simulator-independent tests cover path endpoints, obstacle detours, blocked goals, diagonal corner-cut rejection, wrapped heading error, and bounded controller behavior.

```bash
./scripts/run_unit_tests.sh
```
