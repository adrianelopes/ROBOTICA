import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('sensores')
    escala = LaunchConfiguration('escala_ruido')

    # Gazebo + robô + sensores + estimador (/pose_estimada) + EKF (/odometry/filtered)
    estimacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'estimacao.launch.py')),
        launch_arguments={
            'escala_ruido': escala,
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    params = os.path.join(pkg_share, 'config', 'trajetoria_8.yaml')

    # os argumentos do launch vencem o YAML (a ordem da lista importa)
    trajetoria = Node(
        package='sensores', executable='trajetoria_8', name='trajetoria_8', output='screen',
        parameters=[params, {
            'topico_pose': LaunchConfiguration('topico_pose'),
            'omega': ParameterValue(LaunchConfiguration('omega'), value_type=float),
        }])

    # Árvore de transformações no terminal (world -> odom -> chassis -> sensores).
    # Só aparece quando todos os frames da simulação existem, então não polui a saída
    # com a árvore pela metade enquanto o Gazebo sobe.
    arvore = Node(
        package='sensores', executable='arvore_tf', name='arvore_tf', output='screen',
        parameters=[{
            'periodo': ParameterValue(LaunchConfiguration('tf_periodo'), value_type=float),
            'pdf': LaunchConfiguration('tf_pdf'),
        }],
        condition=IfCondition(LaunchConfiguration('arvore_tf')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('escala_ruido', default_value='1.0',
                              description='Multiplica o desvio-padrão do ruído dos sensores'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abre o RViz da parte 1'),
        DeclareLaunchArgument('topico_pose', default_value='/odometry/filtered',
                              description='Pose usada no controle: /odometry/filtered (EKF), '
                                          '/pose_estimada (GPS + IMU bruto) ou /odom (rodas)'),
        DeclareLaunchArgument('omega', default_value='0.2',
                              description='Frequência do 8 (rad/s); 0.2 = bem mais lento'),
        DeclareLaunchArgument('arvore_tf', default_value='true',
                              description='Imprime a árvore de transformações no terminal'),
        DeclareLaunchArgument('tf_periodo', default_value='2.0',
                              description='Intervalo (s) da checagem de frames faltando'),
        DeclareLaunchArgument('tf_pdf', default_value='',
                              description='Se preenchido, grava a árvore também em PDF '
                                          '(ex.: ~/arvore.pdf; precisa de graphviz)'),
        estimacao,
        trajetoria,
        arvore,
    ])
