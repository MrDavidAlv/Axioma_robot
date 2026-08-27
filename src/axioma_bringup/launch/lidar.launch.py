#!/usr/bin/env python3
"""Start the physical YDLIDAR X3 PRO with the parameters this robot needs.

The stock ydlidar_ros2_driver launch defaults to params/ydlidar.yaml, which is
an X4 profile. Pointing it at the wrong profile is what produces an endless
stream of "Failed to get scan"; see config/ydlidar_x3_pro.yaml for the details.

    ros2 launch axioma_bringup lidar.launch.py
    ros2 launch axioma_bringup lidar.launch.py port:=/dev/ttyUSB1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('axioma_bringup'),
        'config', 'ydlidar_x3_pro.yaml')

    port = LaunchConfiguration('port')
    params = LaunchConfiguration('params_file')

    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[params, {'port': port}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial port the LiDAR is on'),
        DeclareLaunchArgument('params_file', default_value=params_file,
                              description='Driver parameters'),
        lidar,
    ])
