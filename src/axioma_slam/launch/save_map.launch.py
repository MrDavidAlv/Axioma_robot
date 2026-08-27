import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _default_maps_dir():
    """Directory the map should be written to.

    get_package_share_directory points into install/, so by default the map
    landed exactly where the next colcon build would overwrite it. When the
    workspace is built with --symlink-install the installed files are symlinks
    back into the source tree, so resolve one of them and write straight into
    src/, where the map stays under version control.
    """
    share_maps = os.path.join(
        get_package_share_directory('axioma_navigation'), 'maps')
    for probe in ('ground_truth.yaml', 'mapa.yaml'):
        path = os.path.join(share_maps, probe)
        if os.path.islink(path):
            return os.path.dirname(os.path.realpath(path))
    return share_maps


def generate_launch_description():
    """Save the map SLAM Toolbox currently holds to disk."""

    maps_dir = _default_maps_dir()
    default_map_path = os.path.join(maps_dir, 'mapa')

    map_path = LaunchConfiguration('map_path', default=default_map_path)

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-f', map_path,
                   '--ros-args', '-p', 'save_map_timeout:=20.0']
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value=default_map_path,
                              description='Map path, without extension'),
        LogInfo(msg=['Saving map to ', map_path, '.{pgm,yaml}']),
        map_saver
    ])
