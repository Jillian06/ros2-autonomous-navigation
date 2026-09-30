# Verified demo evidence

Recorded from a live ROS 2 Jazzy + Gazebo Harmonic run on 2026-09-30.

- [gazebo_demo.gif](gazebo_demo.gif): 16.6-second Gazebo GUI capture, 3× playback.
- [gazebo_demo.png](gazebo_demo.png): a frame from that capture.
- [trajectory.png](trajectory.png): received A* path, physical world pose, and wheel odometry.
- [demo_results.json](demo_results.json): actual ROS message counts and sampled trajectories.
- [run_provenance.json](run_provenance.json): source commit, Actions run, artifact digest, and capture details.

The robot physically reached the goal with 0.140 m error. ROS /odom uses ideal Gazebo world-pose localization; /wheel_odom is comparison only. LiDAR is live but does not update the known map in this demo.
