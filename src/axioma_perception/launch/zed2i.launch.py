import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError


def generate_launch_description():
    """The real ZED 2i, on the real robot. Not used by the simulation.

    Stereolabs' wrapper is not in rosdistro and is not a declared dependency of
    this package: it is built from source against a locally installed ZED SDK,
    so declaring it would break `rosdep install` for anyone who only ever runs
    the simulation. That is why this file resolves it at launch time and says
    so plainly when it is absent, instead of the package failing to build.

    Two of the arguments passed below are not preferences, they are what keeps
    the TF tree single-rooted:

      publish_urdf:=false   axioma_description already publishes every zed2i_*
                            frame, from the same macro Stereolabs ships. Left
                            true, the wrapper spawns a second
                            robot_state_publisher for a camera floating on its
                            own, unattached to the chassis.

      publish_tf:=false     the wrapper would otherwise publish
      publish_map_tf:=false odom -> camera and map -> odom itself. The EKF in
                            ekf.launch.py owns odom -> base_footprint and SLAM
                            Toolbox owns map -> odom. The camera's tracking
                            still arrives, as the /zed2i/zed_node/odom topic
                            ekf_zed.yaml subscribes to — it just does not get
                            to write to TF.

    Typical use on the robot, alongside the description and the EKF:

        ros2 launch axioma_perception zed2i.launch.py
        ros2 launch axioma_perception ekf.launch.py \
            use_sim_time:=false \
            config_file:=<share>/axioma_perception/config/ekf_zed.yaml
    """

    camera_name = LaunchConfiguration('camera_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    try:
        pkg_zed_wrapper = get_package_share_directory('zed_wrapper')
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg=(
                '\n'
                'axioma_perception: zed_wrapper was not found.\n'
                '\n'
                'This launch file drives the physical ZED 2i and needs '
                'Stereolabs\' ROS 2 wrapper, which is built from source '
                'against the ZED SDK rather than installed from apt:\n'
                '\n'
                '    https://github.com/stereolabs/zed-ros2-wrapper\n'
                '\n'
                'Nothing in the simulation needs it. To run the same EKF '
                'against Gazebo instead:\n'
                '\n'
                '    ros2 launch axioma_gazebo simulation.launch.py\n'
            )),
        ])

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': camera_name,
            'publish_urdf': 'false',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed2i',
            description='Camera namespace. Must match the frame prefix in '
                        'axioma.urdf.xacro and the topic in ekf_zed.yaml'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time. False: this is the real camera'
        ),
        zed,
    ])
