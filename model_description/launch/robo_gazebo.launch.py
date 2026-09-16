import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('model_description')
    model_sdf = os.path.join(pkg_share, 'models', 'my_robot', 'model.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-file', model_sdf,
            '-entity', 'my_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

    return LaunchDescription([gazebo, spawn_entity])
