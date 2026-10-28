import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('sensores')
    params = os.path.join(pkg_share, 'config', 'sensores.yaml')
    escala = ParameterValue(LaunchConfiguration('escala_ruido'), value_type=float)

    imu_orientacao = Node(
        package='sensores',
        executable='imu_orientacao',
        name='imu_orientacao',
        output='screen',
        parameters=[params, {'escala_ruido': escala}],
    )

    gps_xyz = Node(
        package='sensores',
        executable='gps_xyz',
        name='gps_xyz',
        output='screen',
        parameters=[params],
    )

    scan_para_nuvem = Node(
        package='sensores',
        executable='scan_para_nuvem',
        name='scan_para_nuvem',
        output='screen',
        parameters=[params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'sensores.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'escala_ruido', default_value='1.0',
            description='Use o mesmo valor do robo_gazebo.launch.py'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abre o RViz com câmera, lidar e GPS'),
        imu_orientacao,
        gps_xyz,
        scan_para_nuvem,
        rviz,
    ])
