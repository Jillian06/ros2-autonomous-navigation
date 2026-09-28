# ROS 2 Autonomous Navigation — Custom A* + Feedback Control

A portfolio-scale autonomous mobile-robot stack connecting **ROS 2**, **Gazebo Harmonic**, a custom **A\*** global planner, LiDAR sensing, and feedback path following.

> Target platform: Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

## Demo pipeline

```text
Gazebo differential-drive robot
      │
      ├── odometry ───────────────┐
      ├── 360° LiDAR              │
      │                           ▼
known OccupancyGrid + goal → custom A* → nav_msgs/Path
                                      │
                                      ▼
                               path follower
                                      │
                                      ▼
                                  /cmd_vel
                                      │
                                      └────→ Gazebo robot
```

## Highlights
- Custom, unit-tested 8-connected A* implementation
- Collision checking with diagonal corner-cut prevention
- ROS 2 planner node: `/map` + `/odom` + `/goal_pose` → `/planned_path`
- Feedback path follower: `/planned_path` + `/odom` → `/cmd_vel`
- Gazebo Harmonic differential-drive robot with 360° LiDAR
- `ros_gz_bridge` configuration for velocity, odometry and laser scan
- Reproducible demo world and deterministic occupancy grid
- GitHub Actions tests for the simulator-independent planning/control core

## Repository
```text
lsy_autonomous_navigation/
├── lsy_autonomous_navigation/
│   ├── planning.py              # A* core
│   ├── control.py               # feedback controller core
│   ├── astar_planner_node.py    # ROS 2 planner wrapper
│   ├── path_follower_node.py    # ROS 2 controller wrapper
│   ├── demo_map_node.py         # known demo occupancy grid
│   └── demo_goal_node.py        # reproducible demo goal
├── models/robot.sdf             # differential-drive + LiDAR
├── worlds/demo_world.sdf
├── config/bridge.yaml
├── launch/
└── test/
```

## Install
Install ROS 2 Jazzy and Gazebo Harmonic, then:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <YOUR-GITHUB-REPOSITORY-URL>
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the full simulation
```bash
ros2 launch lsy_autonomous_navigation simulation.launch.py
```
The demo publishes a known map and goal `(4.5, 4.0)`. The planner computes a collision-free path and the controller drives the simulated robot toward it.

Useful inspection commands:
```bash
ros2 topic echo /planned_path
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /cmd_vel
```

## Run only the custom navigation stack
```bash
ros2 launch lsy_autonomous_navigation autonomous_navigation.launch.py
```
Supply `/map`, `/odom`, and `/goal_pose` from any compatible platform.

## Test
Pure planning/control tests do not require ROS:
```bash
PYTHONPATH=lsy_autonomous_navigation pytest -q \
  lsy_autonomous_navigation/test/test_planning.py \
  lsy_autonomous_navigation/test/test_control.py
```

ROS workspace tests:
```bash
colcon test --packages-select lsy_autonomous_navigation
colcon test-result --verbose
```

## Algorithms
A* prioritizes each grid cell by

`f(n) = g(n) + h(n)`

with Euclidean `h(n)`. Cardinal moves cost `1`; diagonal moves cost `sqrt(2)`. Diagonal transitions that cut across occupied corners are rejected.

The baseline controller tracks a look-ahead waypoint using distance and wrapped heading error. Linear and angular velocity commands are bounded, and forward speed is attenuated under large heading error.

## Scope and limitations
This project deliberately uses a **known occupancy map** aligned with the demo world; it does **not** claim SLAM. LiDAR is simulated and bridged to ROS 2 for visualization and future mapping/perception extensions. Dynamic obstacle avoidance, EKF sensor fusion, Nav2 benchmarking, and learned local navigation are natural extensions.

## Why this project
The goal is to expose the full path from an algorithmic planning idea to a modular robot-software implementation: map representation → planning → path publication → feedback control → simulated robot actuation.

## License
MIT
