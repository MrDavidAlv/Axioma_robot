import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Simulation launch file.
    Launches Gz Sim + ros_gz_bridge + Robot State Publisher + odom_to_tf.
    Single source of truth for all simulation setup.
    """

    # Package directories
    pkg_axioma_gazebo = get_package_share_directory('axioma_gazebo')
    pkg_axioma_description = get_package_share_directory('axioma_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_axioma_gazebo, 'worlds', 'office.world')
    urdf_file = os.path.join(pkg_axioma_description, 'urdf', 'axioma.urdf')
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
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # ROS <-> Gz Bridge
    # /tf bridge (Pose_V) removed — doesn't provide odom->base_link.
    # odom_to_tf node handles that transform instead.
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
        ],
        output='screen'
    )

    # Robot state publisher (URDF from axioma_description)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

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

    # odom -> base_link TF from /odom topic
    odom_to_tf = Node(
        package='axioma_gazebo',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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
        gz_sim,
        spawn_entity,
        bridge,
        robot_state_publisher,
        odom_to_tf,
    ])
