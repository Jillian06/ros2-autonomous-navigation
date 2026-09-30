#!/usr/bin/env python3
"""Record one live ROS 2 navigation run; no synthetic poses or scan data."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class DemoObserver(Node):
    def __init__(self):
        super().__init__('demo_observer')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Path, '/planned_path', self.on_path, latched)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, latched)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.poses = []
        self.truth_poses = []
        self.goal = None
        self.path_poses = 0
        self.path_xy = []
        self.scans = 0
        self.finite_scan_samples = 0
        self.minimum_scan_m = math.inf
        self.nonzero_cmds = 0
        self.last_cmd = None

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.poses.append((time.monotonic(), p.x, p.y))

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth_poses.append((time.monotonic(), p.x, p.y))

    def on_scan(self, msg):
        self.scans += 1
        finite = [v for v in msg.ranges if math.isfinite(v)]
        self.finite_scan_samples += len(finite)
        if finite:
            self.minimum_scan_m = min(self.minimum_scan_m, min(finite))

    def on_path(self, msg):
        self.path_poses = len(msg.poses)
        self.path_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def on_cmd(self, msg):
        self.last_cmd = (msg.linear.x, msg.angular.z)
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.nonzero_cmds += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--output', default='media/demo_results.json')
    args = parser.parse_args()
    rclpy.init()
    observer = DemoObserver()
    start = time.monotonic()
    reached = False
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(observer, timeout_sec=0.1)
            if observer.poses and observer.goal:
                _, x, y = observer.poses[-1]
                stopped = observer.last_cmd is not None and max(map(abs, observer.last_cmd)) < 1e-4
                truth_ok = observer.truth_poses and math.hypot(observer.truth_poses[-1][1]-observer.goal[0], observer.truth_poses[-1][2]-observer.goal[1]) < 0.20
                if observer.nonzero_cmds and stopped and truth_ok and math.hypot(x-observer.goal[0], y-observer.goal[1]) < 0.15:
                    reached = True
                    break
    finally:
        elapsed = time.monotonic() - start
        poses = observer.poses
        goal = observer.goal
        result = {
            'elapsed_wall_s': round(elapsed, 2),
            'goal_reached_within_0_15_m': reached,
            'goal_xy_m': goal,
            'start_xy_m': list(poses[0][1:]) if poses else None,
            'end_xy_m': list(poses[-1][1:]) if poses else None,
            'final_goal_error_m': round(math.hypot(poses[-1][1]-goal[0], poses[-1][2]-goal[1]), 3) if poses and goal else None,
            'ground_truth_goal_error_m': round(math.hypot(observer.truth_poses[-1][1]-goal[0], observer.truth_poses[-1][2]-goal[1]), 3) if observer.truth_poses and goal else None,
            'ground_truth_messages': len(observer.truth_poses),
            'ground_truth_end_xy_m': list(observer.truth_poses[-1][1:]) if observer.truth_poses else None,
            'ground_truth_trajectory_xy_m': [[round(x, 3), round(y, 3)] for _, x, y in observer.truth_poses[::max(1, len(observer.truth_poses)//300)]],
            'odom_messages': len(poses),
            'path_poses': observer.path_poses,
            'planned_path_xy_m': [[round(x, 3), round(y, 3)] for x, y in observer.path_xy],
            'scan_messages': observer.scans,
            'finite_scan_samples': observer.finite_scan_samples,
            'minimum_scan_m': round(observer.minimum_scan_m, 3) if math.isfinite(observer.minimum_scan_m) else None,
            'nonzero_cmd_messages': observer.nonzero_cmds,
            'last_command_linear_angular': observer.last_cmd,
            'trajectory_xy_m': [[round(x, 3), round(y, 3)] for _, x, y in poses[::max(1, len(poses)//300)]],
        }
        with open(args.output, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2)
            f.write('\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ('trajectory_xy_m', 'planned_path_xy_m')}, indent=2))
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
