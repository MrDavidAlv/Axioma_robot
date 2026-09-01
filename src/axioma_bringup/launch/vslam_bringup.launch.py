#!/usr/bin/env python3
"""Orchestrator: simulation + RGB-D visual SLAM + RViz + teleop.

One command for the whole VSLAM demo, the way slam_bringup.launch.py is for
SLAM Toolbox:

    ros2 launch axioma_bringup vslam_bringup.launch.py
    ros2 launch axioma_bringup vslam_bringup.launch.py world:=terrain
    ros2 launch axioma_bringup vslam_bringup.launch.py odom_source:=ekf

The teleop GUI comes up with it because visual SLAM has nothing to do until
the robot moves: a stationary camera adds no nodes to the pose graph and
closes no loops.

Which world matters more here than it does for the LiDAR demos. The office
world is a single floor, so the map RTAB-Map builds could have been built in
2D; the terrain world is two levels joined by ramps, which is the case a 2D
occupancy grid cannot express and the reason the pose graph is left free in
six degrees of freedom.

odom_source:=visual, the default, hands odom -> base_footprint to the camera,
so the simulation is started with publish_odom_tf:=false. The EKF still runs
and still publishes /odometry/filtered, which makes the two estimates
comparable in a single run - drive a lap and compare against
`ign model -m axioma -p`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Where each world wants the robot to start. The office world is happy at its
# origin; the terrain world's origin is the strip of floor between the two
# ramps, so the robot starts at the foot of the north one instead, the same
# place terrain_demo.launch.py uses.
SPAWN = {
    'office': {'x': '0.0', 'y': '0.0', 'z': '0.1', 'yaw': '0.0'},
    'terrain': {'x': '-5.0', 'y': '2.5', 'z': '0.15', 'yaw': '0.0'},
}


def launch_setup(context, *args, **kwargs):
    """Resolve the arguments before building the include.

    This is an OpaqueFunction and not the usual pile of substitutions for one
    specific reason. IncludeLaunchDescription performs its launch_arguments
    inside the *included* scope, one after another in dictionary order, so an
    argument whose value reads LaunchConfiguration('world') sees whatever the
    earlier 'world' entry just set - the full path to the .world file, not the
    short name this file takes. Spawn coordinates derived that way silently
    fall through to their else branch and the robot appears at the origin.
    Resolving here, in the parent context, removes the ordering trap entirely.
    """
    pkg_axioma_gazebo = get_package_share_directory('axioma_gazebo')
    pkg_axioma_slam = get_package_share_directory('axioma_slam')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    world = LaunchConfiguration('world').perform(context)
    odom_source = LaunchConfiguration('odom_source').perform(context)
    rtabmap_viz = LaunchConfiguration('rtabmap_viz').perform(context)

    world_file = os.path.join(pkg_axioma_gazebo, 'worlds', world + '.world')
    spawn = SPAWN[world]

    # Visual odometry owns odom -> base_footprint when it is the source; a
    # frame gets exactly one parent. The EKF keeps running either way, so
    # /odometry/filtered stays available to compare against.
    publish_odom_tf = 'false' if odom_source == 'visual' else 'true'

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_file,
            'spawn_x': spawn['x'],
            'spawn_y': spawn['y'],
            'spawn_z': spawn['z'],
            'spawn_yaw': spawn['yaw'],
            'publish_odom_tf': publish_odom_tf,
            # RViz draws RTAB-Map's assembled map here, so the simulation does
            # not also need to reproject every depth frame into /zed2i/points.
            'publish_points': 'false',
        }.items()
    )

    vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_slam, 'launch', 'vslam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_source': odom_source,
            'rtabmap_viz': rtabmap_viz,
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_axioma_slam, 'rviz', 'vslam.rviz')],
        parameters=[{'use_sim_time': use_sim_time == 'true'}]
    )

    teleop_gui = Node(
        package='axioma_teleop_gui',
        executable='teleop_gui',
        name='axioma_teleop_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time == 'true'}]
    )

    return [simulation, vslam, rviz, teleop_gui]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('world', default_value='terrain',
                              choices=sorted(SPAWN),
                              description='Which world to map'),
        DeclareLaunchArgument('odom_source', default_value='visual',
                              choices=['visual', 'ekf'],
                              description='Camera-only odometry, or map on '
                                          'top of the fused EKF pose'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              choices=['true', 'false'],
                              description="RTAB-Map's own diagnostic window"),
        OpaqueFunction(function=launch_setup),
    ])
