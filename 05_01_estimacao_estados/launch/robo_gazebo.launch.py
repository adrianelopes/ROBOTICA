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
    pkg_share = get_package_share_directory('estimacao_estados')
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

    # Raiz da árvore de transformações (REP-105: world -> odom -> chassis -> sensores).
    # O mundo do Gazebo é a única frame que ninguém publica: o `odom` vem do
    # diff_drive_controller e os links do robô vêm do robot_state_publisher.
    # O robô nasce na origem do mundo (spawn_entity com x=y=0), então o `odom` começa
    # coincidindo com o `world`. Não use use_sim_time aqui: transformada estática não
    # depende do relógio e o nó pode subir antes do /clock do Gazebo.
    tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_para_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'odom',
        ],
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
        package='estimacao_estados',
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
        tf_world_odom,
        spawn_entity,
        start_controllers,
        cmd_vel_relay,
    ]


def generate_launch_description():
    mundo_padrao = os.path.join(
        get_package_share_directory('estimacao_estados'), 'worlds', 'sensores.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=mundo_padrao,
            description='Mundo do Gazebo (precisa ter <spherical_coordinates> para o GPS)'),
        DeclareLaunchArgument(
            'escala_ruido', default_value='1.0',
            description='Multiplica o desvio-padrão do ruído dos sensores (0 = sem ruído)'),
        OpaqueFunction(function=_criar_simulacao),
    ])
