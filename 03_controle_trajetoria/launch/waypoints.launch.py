import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _criar_seguidor(context):
    parametros = [LaunchConfiguration('params_file').perform(context)]
    controlador = LaunchConfiguration('controller').perform(context)
    if controlador:
        parametros.append({'controlador': controlador})

    return [Node(
        package='controle_trajetoria',
        executable='seguidor_waypoints',
        name='seguidor_waypoints',
        output='screen',
        parameters=parametros,
    )]


def generate_launch_description():
    params_padrao = os.path.join(
        get_package_share_directory('controle_trajetoria'),
        'config', 'seguidor_waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=params_padrao,
            description='YAML com waypoints e ganhos dos controladores'),
        DeclareLaunchArgument(
            'controller', default_value='',
            description="'continuo' ou 'manobras' (vazio: usa o do YAML)"),
        OpaqueFunction(function=_criar_seguidor),
    ])
