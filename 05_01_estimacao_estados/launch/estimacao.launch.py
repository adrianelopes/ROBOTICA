import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('sensores')
    escala = LaunchConfiguration('escala_ruido')

    # Gazebo + robô + controladores (o params_file sobe o /clock de 10 para 100 Hz)
    robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robo_gazebo.launch.py')),
        launch_arguments={
            'params_file': os.path.join(pkg_share, 'config', 'gazebo_params.yaml'),
            'escala_ruido': escala,
        }.items(),
    )

    # /imu/data (com ruído e covariância), /gps/xyz, /scan/points
    sensores = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensores.launch.py')),
        launch_arguments={
            'escala_ruido': escala,
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    # GPS + IMU -> /pose_estimada (medida bruta, com a covariância R do GPS)
    estimador = Node(
        package='sensores', executable='estimador_pose', name='estimador_pose',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'trajetoria_8.yaml'),
                    {'escala_ruido': ParameterValue(escala, value_type=float)}])

    # Filtro de Kalman estendido: /odom + /imu/data + /pose_estimada -> /odometry/filtered
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')])

    return LaunchDescription([
        DeclareLaunchArgument('escala_ruido', default_value='1.0',
                              description='Multiplica o desvio-padrão do ruído dos sensores'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abre o RViz da parte 1'),
        robo,
        sensores,
        estimador,
        ekf,
    ])