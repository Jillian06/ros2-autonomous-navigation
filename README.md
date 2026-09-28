# ROS 2 Autonomous Navigation

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Core Tests](https://github.com/Jillian06/ros2-autonomous-navigation/actions/workflows/test.yml/badge.svg)](https://github.com/Jillian06/ros2-autonomous-navigation/actions/workflows/test.yml)

A compact robotics software project connecting a **from-scratch A\*** global planner and **feedback path follower** to a differential-drive robot in **ROS 2 Jazzy + Gazebo Harmonic**.

The design keeps planning and control simulator-independent, then integrates them through ROS 2 messages and Gazebo interfaces.

## Highlights

- Custom **8-connected A\*** planner on nav_msgs/OccupancyGrid
- Euclidean heuristic, obstacle rejection, and diagonal corner-cut prevention
- ROS 2 pipeline: /map + /odom + /goal_pose -> /planned_path
- Feedback waypoint tracking: /planned_path + /odom -> /cmd_vel
- Differential-drive Gazebo model with **360° 2-D LiDAR**
- ROS <-> Gazebo bridge for velocity, odometry, and laser scans
- Deterministic demo map and goal
- Simulator-independent unit tests + GitHub Actions CI
- Architecture, algorithms, validation, and technical walkthrough docs

## Architecture

```text
/map -------------------------+
                              |
/odom --------------------+   v
                          |  +------------------+
/goal_pose -------------->+->| Custom A* Planner|----> /planned_path
                             +------------------+              |
                                                               v
/odom ------------------------------------------------> +-------------+
                                                       | Path Follower|
                                                       +------+------+
                                                              |
                                                           /cmd_vel
                                                              |
                                                              v
                                                +-------------------------+
                                                | Gazebo diff-drive robot |
                                                | + 360° LiDAR            |
                                                +-----------+-------------+
                                                            |
                                                      /odom, /scan
```

See [Architecture](docs/ARCHITECTURE.md) and [Algorithms](docs/ALGORITHMS.md).

## Repository layout

```text
.
├── autonomous_navigation/
│   ├── autonomous_navigation/   # planner, controller, ROS 2 nodes
│   ├── config/                  # ROS-Gazebo bridge
│   ├── launch/                  # navigation + simulation launch
│   ├── models/                  # differential-drive robot + LiDAR
│   ├── worlds/                  # deterministic Gazebo world
│   └── test/                    # planning/control unit tests
├── docs/
├── media/
├── scripts/
└── .github/workflows/
```

## Quick start

### Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 python3-colcon-common-extensions python3-rosdep
```

### Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Jillian06/ros2-autonomous-navigation.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
ros2 launch autonomous_navigation simulation.launch.py
```

Inspect interfaces:

```bash
ros2 topic echo /planned_path --once
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel
```

## Algorithms

### A* planning

Priority: **f(n) = g(n) + h(n)**, with Euclidean h(n), cost 1 for cardinal moves, sqrt(2) for diagonal moves, and diagonal corner cutting rejected.

### Feedback path following

The controller selects a look-ahead waypoint, computes distance and wrapped heading error, and outputs bounded linear/angular velocity. Forward speed is attenuated when heading error is large.

## Testing

```bash
./scripts/run_unit_tests.sh
```

The suite covers open-grid planning, obstacle detours, blocked goals, diagonal corner cutting, angle wrapping, and controller command behavior. See [Testing & validation](docs/TESTING.md).

## Validation status

The planning/control core is covered by automated unit tests. The repository also includes Gazebo model/world/bridge/launch integration.

**Full graphical Gazebo execution is not claimed as verified yet.** Screenshots, GIFs, and performance numbers should only be added after running the complete stack on a ROS 2 Jazzy + Gazebo Harmonic machine and completing the validation checklist.

## Scope

**Implemented:** known occupancy map, custom global planning, feedback path following, differential-drive simulation model, LiDAR interface, ROS/Gazebo bridge, automated tests.

**Not claimed:** SLAM, AMCL/EKF state estimation, dynamic obstacle avoidance, learned navigation, or physical-robot deployment.

## Next steps

- Add verified Gazebo/RViz demo media
- Benchmark A* against Dijkstra/Nav2 baselines
- Add LiDAR-derived local obstacle updates and replanning
- Integrate SLAM Toolbox + localization
- Compare baseline tracking against pure pursuit / MPC

## License

MIT — see [LICENSE](LICENSE).
