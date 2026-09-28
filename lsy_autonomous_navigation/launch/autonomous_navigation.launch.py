from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='lsy_autonomous_navigation', executable='astar_planner', output='screen'),
        Node(package='lsy_autonomous_navigation', executable='path_follower', output='screen'),
    ])
