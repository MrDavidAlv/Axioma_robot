import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """The whole rough-terrain stack, in one window each: drive it and watch it.

    Brings up the terrain world, the robot, the EKF, RViz and the teleop GUI
    together, so the thing the perception work is *for* is visible rather than
    inferred from a log:

      * Gazebo      the warehouse, the two ramps and the yard's three surfaces
      * RViz        the fused pose, the ZED 2i point cloud, the LiDAR, the TF
      * teleop GUI  something to drive it with

    The robot starts lined up with the north ramp: hold forward and it climbs.
    The lap worth driving from there: cross the yard east over asphalt, dirt
    and sand, and come back down the south ramp at y = -2.5. On the ramps, watch the fused pose in RViz climb
    while the robot's own wheel odometry insists it is still on the floor. On
    the sand, watch a commanded straight line stop being one.

    Set use_ekf:=false to see the same lap without the filter — the robot's
    idea of itself stays flat on the ground the entire way up.
    """

    pkg_axioma_gazebo = get_package_share_directory('axioma_gazebo')

    world_file = os.path.join(pkg_axioma_gazebo, 'worlds', 'terrain.world')
    rviz_config = os.path.join(pkg_axioma_gazebo, 'rviz', 'terrain_demo.rviz')

    use_ekf = LaunchConfiguration('use_ekf')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_axioma_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'use_ekf': use_ekf,
            # In the warehouse, lined up with the north ramp and facing it, so
            # the first thing anyone does with this demo — hold forward — is
            # the thing it exists to show. The world's origin is the strip
            # between the warehouse and the platform, which is no place to
            # start a lap.
            'spawn_x': '-5.0',
            'spawn_y': '2.5',
            'spawn_z': '0.15',
            'spawn_yaw': '0.0',
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )

    teleop_gui = Node(
        package='axioma_teleop_gui',
        executable='teleop_gui',
        name='axioma_teleop_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_teleop)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='Fuse the IMU. False shows the same lap with the raw '
                        'flat-world wheel odometry instead'
        ),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Open RViz'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Open the teleop GUI'),
        LogInfo(msg='Terrain demo: north ramp up at y=+2.5, south ramp down at '
                    'y=-2.5. Yard is asphalt (mu 0.9) / dirt (0.55) / sand (0.30) '
                    'west to east.'),
        simulation,
        rviz,
        teleop_gui,
    ])
