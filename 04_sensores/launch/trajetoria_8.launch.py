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

    # nós da parte 1: /imu/data (com ruído na orientação), /gps/xyz, /scan/points
    sensores = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensores.launch.py')),
        launch_arguments={
            'escala_ruido': escala,
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    params = os.path.join(pkg_share, 'config', 'trajetoria_8.yaml')

    estimador = Node(package='sensores', executable='estimador_pose',
                     name='estimador_pose', output='screen', parameters=[params])
    # os argumentos do launch vencem o YAML (a ordem da lista importa)
    trajetoria = Node(
        package='sensores', executable='trajetoria_8', name='trajetoria_8', output='screen',
        parameters=[params, {
            'topico_pose': LaunchConfiguration('topico_pose'),
            'omega': ParameterValue(LaunchConfiguration('omega'), value_type=float),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('escala_ruido', default_value='1.0',
                              description='Multiplica o desvio-padrão do ruído dos sensores'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abre o RViz da parte 1'),
        DeclareLaunchArgument('topico_pose', default_value='/pose_estimada',
                              description='Pose usada no controle (/odom = sem sensores)'),
        DeclareLaunchArgument('omega', default_value='0.5',
                              description='Frequência do 8 (rad/s); 0.2 = bem mais lento'),
        robo,
        sensores,
        estimador,
        trajetoria,
    ])