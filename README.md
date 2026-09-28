# ROS 2 Autonomous Navigation — Custom A* Planning & Feedback Control

A modular autonomous mobile-robot project built around **ROS 2 Jazzy**, **Gazebo Harmonic**, a from-scratch **A\*** global planner, 2-D LiDAR interfaces, and feedback path following.

> **Platform:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · Python

## What is implemented

- **Custom 8-connected A\*** occupancy-grid planner with unit tests
- Diagonal corner-cut prevention and path reconstruction
- ROS 2 planner interface: `/map` + `/odom` + `/goal_pose` → `/planned_path`
- Feedback path follower: `/planned_path` + `/odom` → `/cmd_vel`
- Differential-drive Gazebo robot with simulated **360° LiDAR**
- ROS ↔ Gazebo bridge configuration for velocity, odometry, scan, and clock
- Deterministic demo world, known occupancy grid, and reproducible goal
- Simulator-independent tests and GitHub Actions CI
- Architecture, algorithm, and validation documentation

## System architecture

```text
                                  /goal_pose
                                      |
                                      v
/map ----------------------> +-------------------+
                             |  Custom A* Planner | -----> /planned_path
/odom ---------------------> +-------------------+               |
       |                                                         v
       |                                                +----------------+
       +----------------------------------------------> | Path Follower  |
                                                        +-------+--------+
                                                                |
                                                             /cmd_vel
                                                                |
                                                                v
                                                    +-----------------------+
                                                    | Gazebo diff-drive bot |
                                                    | + 360° LiDAR          |
                                                    +-----------+-----------+
                                                                |
                                                          /odom, /scan
```

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for design details.

## Repository structure

```text
.
├── lsy_autonomous_navigation/
│   ├── lsy_autonomous_navigation/
│   │   ├── planning.py              # simulator-independent A* core
│   │   ├── control.py               # feedback-control core
│   │   ├── astar_planner_node.py    # ROS 2 planner node
│   │   ├── path_follower_node.py    # ROS 2 controller node
│   │   ├── demo_map_node.py         # deterministic occupancy grid
│   │   └── demo_goal_node.py        # reproducible navigation goal
│   ├── models/robot.sdf             # differential-drive robot + LiDAR
│   ├── worlds/demo_world.sdf        # static test environment
│   ├── config/bridge.yaml           # ros_gz_bridge topics
│   ├── launch/                      # simulation/navigation launch files
│   └── test/                        # planning/control unit tests
├── docs/                             # architecture, algorithms, validation
├── media/                            # verified demo artifacts go here
├── scripts/run_unit_tests.sh
└── .github/workflows/test.yml
```

## Quick start

### 1. Dependencies

Install ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04, then install the ROS/Gazebo bridge and build tools:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 python3-colcon-common-extensions python3-rosdep
```

### 2. Clone and build

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

### 3. Run the demo stack

```bash
ros2 launch lsy_autonomous_navigation simulation.launch.py
```

The deterministic demo publishes a known occupancy grid and a goal. The custom planner computes a collision-free `nav_msgs/Path`; the controller converts tracking error into bounded velocity commands for the simulated differential-drive robot.

Inspect the interfaces:

```bash
ros2 topic echo /planned_path --once
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel
```

### 4. Run only the navigation nodes

```bash
ros2 launch lsy_autonomous_navigation autonomous_navigation.launch.py
```

This allows the planner/controller to be connected to another platform that supplies compatible `/map`, `/odom`, and `/goal_pose` topics.

## Tests

Run the simulator-independent test suite:

```bash
./scripts/run_unit_tests.sh
```

or:

```bash
PYTHONPATH=lsy_autonomous_navigation pytest -q \
  lsy_autonomous_navigation/test/test_planning.py \
  lsy_autonomous_navigation/test/test_control.py
```

The CI workflow runs these tests on every push/pull request.

## Algorithms

### A* global planner

The planner prioritizes cells by

```text
f(n) = g(n) + h(n)
```

using Euclidean `h(n)`, unit cardinal cost, and `sqrt(2)` diagonal cost. A diagonal move is rejected if it would cut through occupied adjacent cells.

### Feedback path follower

The controller selects a look-ahead waypoint, computes distance and wrapped heading error, then generates bounded linear/angular velocity commands. Forward speed is reduced for large heading error.

See [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md).

## Validation status

The planning/control core is unit tested independently of ROS/Gazebo. The repository includes the simulation model, bridge, world, and launch integration; **full graphical Gazebo integration should be validated on a ROS 2 Jazzy + Gazebo Harmonic machine before publishing demo screenshots or performance numbers**. The validation checklist is in [`docs/TESTING.md`](docs/TESTING.md).

## Scope

This project intentionally distinguishes implemented functionality from future extensions:

**Implemented:** known occupancy map, custom global planning, path following, differential-drive simulation model, LiDAR interface, ROS/Gazebo bridge, tests.

**Not claimed:** SLAM, AMCL/EKF sensor fusion, Nav2 local costmaps, dynamic obstacle avoidance, learned navigation, or physical-robot deployment.

## Natural extensions

- SLAM Toolbox + saved-map workflow
- AMCL or EKF localization
- LiDAR-derived local obstacle layer and dynamic replanning
- Nav2 planner/controller benchmark against the custom baseline
- camera perception with OpenCV/PyTorch
- PID, pure-pursuit, MPC, or learned local control
- physical differential-drive deployment

## Why build the planner instead of only launching Nav2?

The project is designed to expose the algorithm-to-system boundary. Implementing the global planner and baseline controller directly makes planning assumptions, collision checks, ROS message flow, and control behavior inspectable and testable; mature stacks such as Nav2 can then be used as meaningful comparison baselines rather than black boxes.

## License

MIT. See [`LICENSE`](LICENSE).
