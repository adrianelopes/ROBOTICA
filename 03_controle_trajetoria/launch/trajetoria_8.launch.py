import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # params_file chega ao gzserver e sobe o /clock de 10 para 100 Hz
    robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('controle_trajetoria'), 'launch', 'robo_gazebo.launch.py')),
        launch_arguments={'params_file': os.path.join(
            get_package_share_directory('controle_trajetoria'), 'config', 'gazebo_params.yaml')}.items(),
    )

    params = os.path.join(
        get_package_share_directory('controle_trajetoria'), 'config', 'trajetoria_8.yaml')

    no = Node(package='controle_trajetoria', executable='trajetoria_8', output='screen',
              parameters=[params, {'use_sim_time': True}])

    return LaunchDescription([robo, no])
