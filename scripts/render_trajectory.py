#!/usr/bin/env python3
"""Plot recorded ROS 2 path/odometry; source data: record_demo.py output."""
import argparse
import json

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording', help='JSON emitted by record_demo.py')
    parser.add_argument('--output', default='media/trajectory.png')
    args = parser.parse_args()
    with open(args.recording, encoding='utf-8') as f:
        run = json.load(f)
    planned = run['planned_path_xy_m']
    actual = run['ground_truth_trajectory_xy_m']
    wheel = run['wheel_trajectory_xy_m']
    if not planned or len(actual) < 2:
        raise ValueError('Recording has no planned path or moving odometry')

    fig, ax = plt.subplots(figsize=(8, 7))
    for cx, cy in [(-1.25, -1.1), (1.45, 1.8)]:
        ax.add_patch(Rectangle((cx - .25, cy - 2.75), .5, 5.5,
                               color='#394b6b', label='Gazebo wall' if cx < 0 else None))
    ax.plot(*zip(*planned), '--', color='#e69f00', linewidth=2, label='A* planned path')
    ax.plot(*zip(*actual), color='#0072b2', linewidth=2.5, label='Gazebo world pose')
    ax.plot(*zip(*wheel), ':', color='#cc79a7', linewidth=1.5, label='Wheel odometry')
    ax.scatter(*actual[0], c='#009e73', s=90, label='Start', zorder=5)
    ax.scatter(*run['goal_xy_m'], c='#d55e00', marker='*', s=180, label='Goal', zorder=5)
    ax.set(xlabel='x (m)', ylabel='y (m)', xlim=(-3, 6), ylim=(-3, 6),
           title='ROS 2 + Gazebo navigation: planned and measured trajectory')
    ax.set_aspect('equal')
    ax.grid(alpha=.25)
    ax.legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)
    print(args.output)


if __name__ == '__main__':
    main()
