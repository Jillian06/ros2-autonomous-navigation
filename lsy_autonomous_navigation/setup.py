from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lsy_autonomous_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jillian Yang',
    maintainer_email='maintainer@example.com',
    description='Custom A* planner and path follower for ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'astar_planner = lsy_autonomous_navigation.astar_planner_node:main',
            'path_follower = lsy_autonomous_navigation.path_follower_node:main',
            'demo_map = lsy_autonomous_navigation.demo_map_node:main',
            'demo_goal = lsy_autonomous_navigation.demo_goal_node:main',
        ],
    },
)
