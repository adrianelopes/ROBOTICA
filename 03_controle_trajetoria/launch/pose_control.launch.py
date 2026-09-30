import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('controle_trajetoria'), 'config')

    controller = LaunchConfiguration('controller')

    continuo = Node(
        package='controle_trajetoria',
        executable='pose_controller',
        name='pose_controller',  
        output='screen',
        parameters=[os.path.join(config_dir, 'pose_controller.yaml')],
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'continuo'"])),
    )

    manobras = Node(
        package='controle_trajetoria',
        executable='controle_pose_manobras',
        name='controle_pose',  
        output='screen',
        parameters=[os.path.join(config_dir, 'controle_pose_manobras.yaml')],
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'manobras'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='continuo',
            description="Controlador de pose: 'continuo' ou 'manobras'"),
        continuo,
        manobras,
    ])
