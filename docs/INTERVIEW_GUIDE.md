# Technical walkthrough

This note is a compact explanation of the project for code review or interview discussion.

## Why custom A*?

The project intentionally implements the global planner instead of hiding planning inside a larger navigation stack. This makes state expansion, collision checks, heuristic choice, and path reconstruction inspectable and testable.

## Why separate pure Python from ROS 2?

planning.py and control.py do not depend on ROS. Algorithm tests remain fast while the ROS nodes stay thin adapters around messages, frames, publishers, and subscribers.

## Planning assumptions

- 2-D occupancy grid
- 8-connected motion
- cardinal cost = 1
- diagonal cost = sqrt(2)
- Euclidean heuristic
- cells >= 50 are occupied
- unknown cells are rejected by default
- diagonal moves cannot pass between blocked cardinal neighbors

## Controller

The baseline controller tracks a look-ahead waypoint using distance and heading error. Angular velocity is proportional to heading error; forward velocity is reduced when the robot is poorly aligned.

## Main limitations

The map is known in advance. LiDAR is available as an interface but is not yet used for SLAM or a local dynamic costmap. The controller is a transparent baseline rather than an optimal tracking controller.

## Useful extensions

A strong next comparison is custom A* + baseline control versus Nav2 planning/control, followed by localization, replanning, and a verified Gazebo experiment.
