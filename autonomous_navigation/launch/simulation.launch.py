import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=get_package_share_directory('autonomous_navigation')
    world=os.path.join(share,'worlds','demo_world.sdf'); model=os.path.join(share,'models','robot.sdf'); bridge=os.path.join(share,'config','bridge.yaml')
    return LaunchDescription([
      ExecuteProcess(cmd=['gz','sim','-v','4','-r',world],output='screen'),
      TimerAction(period=2.0,actions=[ExecuteProcess(cmd=['ros2','run','ros_gz_sim','create','-world','demo','-file',model,'-name','autonomy_bot'],output='screen')]),
      Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':bridge}],output='screen'),
      Node(package='autonomous_navigation',executable='demo_map',output='screen'),
      Node(package='autonomous_navigation',executable='astar_planner',output='screen'),
      Node(package='autonomous_navigation',executable='path_follower',output='screen'),
      Node(package='autonomous_navigation',executable='demo_goal',output='screen'),
    ])
