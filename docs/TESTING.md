# Testing and validation

## Unit-tested core
The simulator-independent planning and control modules have unit tests for:

- A* path existence and endpoints
- obstacle avoidance
- no-path cases
- diagonal corner-cut rejection
- heading-error wrapping
- controller command bounds / basic behavior

Run:

```bash
PYTHONPATH=lsy_autonomous_navigation pytest -q \
  lsy_autonomous_navigation/test/test_planning.py \
  lsy_autonomous_navigation/test/test_control.py
```

## Integration validation checklist
Before presenting simulator results, validate on Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic:

```bash
ros2 launch lsy_autonomous_navigation simulation.launch.py
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /planned_path --once
ros2 topic hz /cmd_vel
```

Then confirm:

1. Gazebo robot spawns at the expected pose.
2. `/odom` updates as the robot moves.
3. `/scan` contains finite ranges around obstacles.
4. `/planned_path` avoids occupied cells.
5. `/cmd_vel` is published while tracking and approaches zero at the goal.
6. Robot reaches the goal without intersecting static obstacles.

Do not add screenshots or numerical performance claims until this integration run has actually been completed.
