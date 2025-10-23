from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch rqt_robot_steering for teleoperation."""

    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        parameters=[{
            'default_topic': '/cmd_vel'
        }]
    )

    return LaunchDescription([
        rqt_robot_steering
    ])
