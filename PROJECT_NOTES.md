# Technical notes

## Design choices
- **Custom A\*** instead of a Nav2 planner plugin to expose the planning algorithm and make it independently testable.
- **8-connected grid** with Euclidean heuristic and no diagonal corner cutting.
- **Bounded proportional path follower** as a transparent baseline controller.
- **ROS 2 nodes are thin wrappers** around pure-Python planning/control functions.
- **Gazebo Harmonic bridge** separates simulator transport from ROS 2 interfaces.

## Limitations
The demo map is known a priori and aligned with the Gazebo world; this repository does not claim SLAM. The LiDAR stream is exposed for visualization and future perception/mapping extensions. Dynamic obstacle avoidance and state-estimation fusion are future work.
