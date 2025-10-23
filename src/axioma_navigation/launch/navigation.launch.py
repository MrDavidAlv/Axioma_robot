import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_axioma_navigation = get_package_share_directory('axioma_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths
    nav2_params_file = os.path.join(pkg_axioma_navigation, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_axioma_navigation, 'maps', 'SlamToolboxSimulation.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=nav2_params_file)
    map_yaml = LaunchConfiguration('map', default=map_file)

    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_yaml
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('autostart', default_value='true',
                            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file,
                            description='Full path to the ROS2 parameters file'),
        DeclareLaunchArgument('map', default_value=map_file,
                            description='Full path to map yaml file'),
        nav2_bringup_launch
    ])
