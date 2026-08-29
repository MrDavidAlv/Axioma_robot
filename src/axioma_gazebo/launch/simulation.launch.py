import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Simulation launch file.
    Launches Gz Sim + ros_gz_bridge + Robot State Publisher, and one of
    odom_to_tf or the EKF depending on use_ekf.
    Single source of truth for all simulation setup.
    """

    # Package directories
    pkg_axioma_gazebo = get_package_share_directory('axioma_gazebo')
    pkg_axioma_description = get_package_share_directory('axioma_description')
    pkg_axioma_perception = get_package_share_directory('axioma_perception')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_axioma_gazebo, 'worlds', 'office.world')
    xacro_file = os.path.join(pkg_axioma_description, 'urdf', 'axioma.urdf.xacro')
    sdf_file = os.path.join(pkg_axioma_gazebo, 'models', 'axioma_v2', 'model.sdf')

    # Set Gz Sim resource path for models and worlds
    gz_models_path = os.path.join(pkg_axioma_gazebo, 'models')
    gz_worlds_path = os.path.join(pkg_axioma_gazebo, 'worlds')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gz_models_path + ':' + gz_worlds_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_models_path + ':' + gz_worlds_path

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)
    use_ekf = LaunchConfiguration('use_ekf', default='true')
    # Where the robot lands. The office world is happy with the origin; the
    # terrain world's origin sits between the two ramps, on the strip of floor
    # separating the warehouse from the yard platform, which is a silly place
    # to start a lap of a loop circuit.
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.1')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')

    # Gz Sim (server + GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn robot in Gz Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'axioma',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen'
    )

    # ROS <-> Gz Bridge
    # /tf bridge (Pose_V) removed — doesn't provide odom->base_footprint.
    # Whichever of odom_to_tf or the EKF is active publishes that transform.
    # joint_states uses direct topic (SDF has <topic>joint_states</topic>).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Chassis IMU. This is what makes the robot's roll and pitch
            # observable rather than assumed — see axioma_perception.
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # ZED 2i. Bridged for inspection in RViz, not because anything
            # subscribes: the EKF's position source in simulation is the
            # wheels, since the ZED SDK's tracking has no Gazebo equivalent.
            '/zed2i/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed2i/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed2i/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/zed2i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # Robot state publisher (xacro from axioma_description).
    # The description is xacro since the ZED 2i was mounted; reading it raw
    # hands robot_state_publisher a document full of ${...} that KDL rejects.
    robot_desc = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # odom -> base_footprint, one publisher or the other, never both.
    #
    # odom_to_tf republishes the DiffDrive plugin's odometry as a transform
    # verbatim, which assumes the world is a plane: on a ramp it reports the
    # distance driven as horizontal distance and the robot's z never changes.
    # The EKF replaces it by integrating the same wheel speeds through the
    # attitude the IMU measures, so the pose follows the slope. Since both
    # write the same transform and a frame gets exactly one parent, use_ekf
    # picks between them.
    odom_to_tf = Node(
        package='axioma_gazebo',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_ekf)
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_perception, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': PathJoinSubstitution(
                [pkg_axioma_perception, 'config', 'ekf_sim.yaml']),
        }.items(),
        condition=IfCondition(use_ekf)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='World file'
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Spawn X'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Spawn Y'),
        DeclareLaunchArgument('spawn_z', default_value='0.1',
                              description='Spawn Z'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Spawn yaw, radians'),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Fuse the IMU with wheel odometry through '
                        'axioma_perception. False falls back to odom_to_tf, '
                        'which republishes the raw wheel odometry and assumes '
                        'the ground is flat'
        ),
        gz_sim,
        spawn_entity,
        bridge,
        robot_state_publisher,
        odom_to_tf,
        ekf,
    ])
