# Testing and validation

## Unit-tested core

The simulator-independent planning and control modules are tested for:
- A* path endpoints on an open grid
- obstacle detours
- blocked-goal handling
- diagonal corner-cut rejection
- wrapped heading error
- bounded controller behavior

Run:

```bash
PYTHONPATH=autonomous_navigation pytest -q autonomous_navigation/test/test_planning.py autonomous_navigation/test/test_control.py
```

## Integration validation checklist

Before publishing simulator results, validate on Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic:

```bash
ros2 launch autonomous_navigation simulation.launch.py
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /planned_path --once
ros2 topic hz /cmd_vel
```

Confirm:

1. Gazebo robot spawns at the expected pose.
2. /odom updates while the robot moves.
3. /scan reports finite obstacle ranges.
4. /planned_path avoids occupied grid cells.
5. /cmd_vel is published while tracking and goes to zero at the goal.
6. The robot reaches the goal without intersecting static obstacles.

Only add screenshots, GIFs, or quantitative performance claims after this integration run is actually completed.
