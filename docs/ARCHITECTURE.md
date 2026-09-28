# Architecture

The project separates simulator-independent algorithms from ROS 2 integration.

```text
                           +-------------------+
                           |   Goal Pose       |
                           +---------+---------+
                                     |
                                     v
+------------------+       +---------+---------+       +------------------+
| Occupancy Grid   | ----> | Custom A* Planner| ----> | nav_msgs/Path    |
+------------------+       +---------+---------+       +---------+--------+
                                     ^                           |
                                     |                           v
+------------------+                 |                 +---------+--------+
| Gazebo Robot     | -- odometry ----+---------------->| Path Follower    |
| Diff drive/LiDAR |                                   +---------+--------+
+--------+---------+                                             |
         ^                                                       v
         +------------------------ /cmd_vel <---------------------+
         |
         +---- /scan (sensor interface / extension hook)
```

## Design choices

### Custom global planner
The planner is implemented independently of Nav2 to make the algorithmic behavior inspectable and testable. It operates on an occupancy grid and supports 8-connected motion.

### Corner-cut prevention
A diagonal transition is rejected when either adjacent cardinal cell is occupied. This prevents a point-grid planner from producing paths that pass through obstacle corners.

### Controller
The baseline path follower uses a look-ahead waypoint and proportional distance/heading feedback. It is intentionally small enough to inspect and replace with PID, pure pursuit, MPC, or a learned policy.

### Simulation boundary
Gazebo provides differential-drive dynamics, odometry, and a 2-D LiDAR. `ros_gz_bridge` connects simulator topics to ROS 2.

## Current scope
The included demo uses a known occupancy grid. SLAM, AMCL/EKF localization, local costmaps, and dynamic obstacle avoidance are not claimed as implemented features.
