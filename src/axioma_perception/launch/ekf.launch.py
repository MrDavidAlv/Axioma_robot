import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Attitude-aware localization: robot_localization's EKF.

    Publishes odom -> base_footprint from a fused estimate instead of from
    dead-reckoned wheel counts, so the transform stays right when the robot is
    on a slope. Which sensors get fused is decided entirely by the config file,
    not by this launch file:

        ekf_sim.yaml   simulation   IMU + wheel odometry      (default)
        ekf_zed.yaml   real robot   IMU + ZED 2i tracking

    Both configs and the reasoning behind the split are in config/.

    This node claims odom -> base_footprint, and a frame gets exactly one
    parent. Whatever was publishing that transform before has to be switched
    off when this runs: in simulation that is axioma_gazebo's odom_to_tf node,
    which simulation.launch.py disables whenever use_ekf is true; on the real
    robot it is the ZED wrapper's own publish_tf, which zed2i.launch.py turns
    off. Two publishers of one transform is not a merged estimate, it is a TF
    tree that flickers between two answers.

    The fused output stays on /odometry/filtered and is deliberately not
    remapped to /odom: in simulation /odom is the raw wheel odometry this
    filter *subscribes* to, and remapping the output onto it would feed the
    filter its own estimate. Consumers that want the fused pose should read
    TF, which is what SLAM Toolbox and Nav2 already do.
    """

    pkg_axioma_perception = get_package_share_directory('axioma_perception')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [pkg_axioma_perception, 'config', 'ekf_sim.yaml']),
            description='EKF config: ekf_sim.yaml (IMU + wheels) or '
                        'ekf_zed.yaml (IMU + ZED 2i tracking)'
        ),
        ekf_node,
    ])
