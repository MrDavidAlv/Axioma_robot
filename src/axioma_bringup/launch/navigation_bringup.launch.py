#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_axioma_gazebo = get_package_share_directory('axioma_gazebo')
    pkg_axioma_navigation = get_package_share_directory('axioma_navigation')

    # Map path as string (not LaunchConfiguration)
    map_file = os.path.join(pkg_axioma_navigation, 'maps', 'SlamToolboxSimulation.yaml')

    # 1. Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_gazebo, 'launch', 'complete_simulation.launch.py')
        )
    )

    # 2. Launch Navigation2 with explicit map argument
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    # 3. Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch,
        rviz_node
    ])
