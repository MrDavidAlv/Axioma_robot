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

    # 1. Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_gazebo, 'launch', 'complete_simulation.launch.py')
        )
    )

    # 2. Launch SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_navigation, 'launch', 'slam.launch.py')
        )
    )

    # 3. Launch RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Launch teleop for manual control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e'
    )

    # Create launch description
    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        rviz_node,
        teleop_node
    ])
