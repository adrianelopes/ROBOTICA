"""
SLAM 2D com o slam_toolbox (modo online assíncrono) no robô my_robot.

Sobe o Gazebo com a sala fechada (worlds/slam_simples.world), o slam_toolbox e o RViz.
Dirija o robô em outro terminal com o teleop_twist_keyboard (publica em /cmd_vel, e o
cmd_vel_relay repassa para o diff_drive_controller).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('estimacao_estados')
    mundo_padrao = os.path.join(pkg_share, 'worlds', 'slam_simples.world')
    params_padrao = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # Robô no Gazebo, SEM a TF estática world -> odom: no SLAM o pai do odom é o map.
    simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robo_gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'escala_ruido': LaunchConfiguration('escala_ruido'),
            'publicar_world_odom': 'false',
        }.items(),
    )

    # Nó do repositório SteveMacenski/slam_toolbox (compilado em ~/ros2_ws/src/slam_toolbox).
    # Assina /scan e a TF odom -> chassis; publica /map e a TF map -> odom.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=mundo_padrao,
            description='Mundo do Gazebo usado no mapeamento'),
        DeclareLaunchArgument(
            'escala_ruido', default_value='1.0',
            description='Multiplica o ruído dos sensores (0 = sem ruído)'),
        DeclareLaunchArgument(
            'slam_params', default_value=params_padrao,
            description='Arquivo YAML de parâmetros do slam_toolbox'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abre o RViz com o mapa, o lidar e a árvore TF'),
        simulacao,
        # Espera o Gazebo publicar /clock e os controladores subirem (odom -> chassis);
        # antes disso o slam_toolbox só reclamaria de TF ausente.
        TimerAction(period=5.0, actions=[slam_toolbox, rviz]),
    ])
