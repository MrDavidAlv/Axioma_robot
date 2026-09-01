#!/usr/bin/env python3
"""RGB-D visual SLAM with RTAB-Map, driven by the ZED 2i.

Agnostic of simulation or real robot, the way slam.launch.py is: it consumes
an RGB image, a registered depth image and a camera_info, and it produces a
map. axioma_bringup/vslam_bringup.launch.py is the orchestrator that pairs it
with the simulation and RViz.

    ros2 launch axioma_slam vslam.launch.py
    ros2 launch axioma_slam vslam.launch.py odom_source:=ekf
    ros2 launch axioma_slam vslam.launch.py localization:=true

Two odometry sources, and the choice decides who owns odom -> base_footprint,
because a frame gets exactly one parent:

    visual (default)  rgbd_odometry tracks features between frames and owns
                      the transform. This is visual odometry in the strict
                      sense - nothing but the camera - so the caller must stop
                      whatever else was publishing that transform. In
                      simulation that is simulation.launch.py's
                      publish_odom_tf:=false, which vslam_bringup passes for
                      you and which leaves the EKF running as a topic so the
                      two estimates can be compared in one run.

    ekf               RTAB-Map maps on top of axioma_perception's fused pose
                      and rgbd_odometry never starts. Slower to drift and
                      immune to a featureless wall, but the camera is then
                      only building the map, not localising the robot.

The camera topics default to the simulated ZED. On the real robot the ZED
wrapper publishes the same three under /zed2i/zed_node/, so override
rgb_topic, depth_topic and camera_info_topic rather than editing this file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_axioma_slam = get_package_share_directory('axioma_slam')
    params_file = os.path.join(pkg_axioma_slam, 'config', 'vslam_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_source = LaunchConfiguration('odom_source')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # 'visual' and 'ekf' as a condition, once, instead of at three call sites.
    use_visual_odom = PythonExpression(["'", odom_source, "' == 'visual'"])

    camera_remaps = [
        ('rgb/image', rgb_topic),
        ('depth/image', depth_topic),
        ('rgb/camera_info', camera_info_topic),
    ]

    visual_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=camera_remaps + [('odom', '/vslam/odom')],
        condition=IfCondition(use_visual_odom),
    )

    # RTAB-Map reads its odometry from a topic, and that topic has to be the
    # same estimate TF carries, or map -> odom corrects a pose nobody uses.
    rtabmap_common = [params_file, {
        'use_sim_time': use_sim_time,
        'database_path': database_path,
        # Mapping appends to the database; localization opens it read-only and
        # only ever publishes map -> odom.
        #
        # ParameterValue(value_type=str) is not decoration: launch_ros infers a
        # parameter's type from the string, so a bare 'false' arrives as a bool
        # and RTAB-Map, whose parameters are all strings, aborts on the type
        # mismatch before it publishes anything.
        'Mem/IncrementalMemory': ParameterValue(
            PythonExpression(["'false' if '", localization,
                              "' == 'true' else 'true'"]), value_type=str),
        'Mem/InitWMWithAllNodes': ParameterValue(
            PythonExpression(["'true' if '", localization,
                              "' == 'true' else 'false'"]), value_type=str),
    }]

    # The odometry topic follows the source, so one node definition covers
    # both choices. Mapping and localization stay separate only because
    # --delete_db_on_start is an argument, not a parameter, and localization
    # must not delete the database it was asked to reuse.
    odom_topic = PythonExpression(
        ["'/vslam/odom' if '", odom_source, "' == 'visual' "
         "else '/odometry/filtered'"])

    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_common,
        remappings=camera_remaps + [('odom', odom_topic)],
        arguments=['--delete_db_on_start'],
        condition=UnlessCondition(localization),
    )

    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_common,
        remappings=camera_remaps + [('odom', odom_topic)],
        condition=IfCondition(localization),
    )

    # RTAB-Map's own window: the feature matches and the loop-closure candidates
    # frame by frame. Off by default because it is a diagnostic, and because it
    # costs as much real time factor as the simulation it is diagnosing.
    viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=camera_remaps + [('odom', odom_topic)],
        condition=IfCondition(rtabmap_viz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument(
            'odom_source', default_value='visual',
            choices=['visual', 'ekf'],
            description='visual: rgbd_odometry owns odom -> base_footprint. '
                        'ekf: map on top of axioma_perception, camera does '
                        'not localise'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            choices=['true', 'false'],
            description='Reuse database_path read-only instead of mapping'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(os.path.expanduser('~'), '.ros',
                                       'axioma_vslam.db'),
            description='Where the RTAB-Map database lives'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              choices=['true', 'false'],
                              description="RTAB-Map's own diagnostic window"),
        DeclareLaunchArgument('rgb_topic', default_value='/zed2i/image',
                              description='RGB image'),
        DeclareLaunchArgument('depth_topic', default_value='/zed2i/depth_image',
                              description='Depth registered to the RGB image'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/zed2i/camera_info',
                              description='Intrinsics for both images'),
        visual_odometry,
        rtabmap_mapping,
        rtabmap_localization,
        viz,
    ])
