#!/usr/bin/env bash
# Run only on a full Ubuntu 24.04 runner with ROS 2 Jazzy and Gazebo Harmonic.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths autonomous_navigation
source install/setup.bash
set -u
mkdir -p media
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
Xvfb :99 -screen 0 1280x900x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
x_pid=$!
sleep 3
ros2 launch autonomous_navigation simulation.launch.py >media/launch.log 2>&1 &
launch_pid=$!
cleanup() { kill "$launch_pid" "$x_pid" 2>/dev/null || true; }
trap cleanup EXIT
sleep 10
{ ros2 topic list; timeout 8 gz topic -l; } >media/topics.log 2>&1 || true
{
  ros2 topic info -v /odom
  timeout 5 gz topic -i -t /model/autonomy_bot/odometry || true
  timeout 5 gz topic -e -t /model/autonomy_bot/odometry || true
  timeout 5 ros2 topic echo /odom --once || true
} >media/odom_diagnostics.log 2>&1
/usr/bin/python3 scripts/record_demo.py --duration 120 --output media/demo_results.json &
observer_pid=$!
# A real capture of the virtual display. A video frame becomes the still image.
ffmpeg -hide_banner -loglevel error -y -f x11grab -framerate 5 -video_size 1280x900 -i :99 -t 16 \
  -vf 'scale=900:-2:flags=lanczos' -c:v libx264 -preset ultrafast -pix_fmt yuv420p /tmp/gazebo.mp4
ffmpeg -hide_banner -loglevel error -y -ss 7 -i /tmp/gazebo.mp4 -frames:v 1 media/gazebo_demo.png
ffmpeg -hide_banner -loglevel error -y -i /tmp/gazebo.mp4 -vf 'fps=5,scale=900:-2:flags=lanczos,palettegen' /tmp/palette.png
ffmpeg -hide_banner -loglevel error -y -i /tmp/gazebo.mp4 -i /tmp/palette.png \
  -lavfi 'fps=5,scale=900:-2:flags=lanczos[x];[x][1:v]paletteuse' media/gazebo_demo.gif
wait "$observer_pid"
/usr/bin/python3 scripts/render_trajectory.py media/demo_results.json --output media/trajectory.png
/usr/bin/python3 - <<'PY'
import json
with open('media/demo_results.json', encoding='utf-8') as f:
    result = json.load(f)
for key in ('path_poses', 'scan_messages', 'nonzero_cmd_messages', 'final_goal_error_m'):
    print(f'{key}: {result[key]}')
assert result['path_poses'] > 2, 'No A* path was published'
assert result['scan_messages'] > 0 and result['finite_scan_samples'] > 0, 'No LiDAR scan'
assert result['nonzero_cmd_messages'] > 10, 'Robot never received movement commands'
assert result['goal_reached_within_0_15_m'], 'Robot did not reach the goal'
PY
