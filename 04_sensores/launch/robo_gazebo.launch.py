import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _criar_simulacao(context):
    pkg_share = get_package_share_directory('sensores')
    model_xacro = os.path.join(pkg_share, 'models', 'my_robot', 'model.urdf.xacro')
    escala = LaunchConfiguration('escala_ruido').perform(context)
    robot_description = xacro.process_file(
        model_xacro, mappings={'escala_ruido': escala}).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true',
                          'world': LaunchConfiguration('world').perform(context)}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    cmd_vel_relay = Node(
        package='sensores',
        executable='cmd_vel_relay',
        output='screen',
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, diff_drive_spawner],
        )
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        start_controllers,
        cmd_vel_relay,
    ]


def generate_launch_description():
    mundo_padrao = os.path.join(
        get_package_share_directory('sensores'), 'worlds', 'sensores.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=mundo_padrao,
            description='Mundo do Gazebo (precisa ter <spherical_coordinates> para o GPS)'),
        DeclareLaunchArgument(
            'escala_ruido', default_value='1.0',
            description='Multiplica o desvio-padrão do ruído dos sensores (0 = sem ruído)'),
        OpaqueFunction(function=_criar_simulacao),
    ])
