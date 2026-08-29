#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_axioma_description = get_package_share_directory('axioma_description')

    # Ruta al archivo xacro
    xacro_file = os.path.join(pkg_axioma_description, 'urdf', 'axioma.urdf.xacro')

    # Ruta al archivo de configuración de RViz2
    rviz_config_file = os.path.join(pkg_axioma_description, 'rviz', 'display.rviz')

    # Procesar el xacro. La descripcion dejo de ser URDF plano cuando se monto
    # la ZED 2i: leer el archivo tal cual entrega un documento lleno de ${...}
    # que KDL rechaza, y el RobotModel de RViz aparece vacio sin decir por que.
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Argumentos del launch
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui if true'
    )

    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc
        }]
    )

    # Nodo joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Nodo RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
