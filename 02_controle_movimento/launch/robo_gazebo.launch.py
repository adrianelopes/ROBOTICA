import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('controle_movimento')
    model_xacro = os.path.join(pkg_share, 'models', 'my_robot', 'model.urdf.xacro')
    robot_description = xacro.process_file(model_xacro).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Publica robot_description, lido pelo spawn_entity e pelo gazebo_ros2_control
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
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
        package='controle_movimento',
        executable='cmd_vel_relay',
        output='screen',
    )

    teleop_keyboard = ExecuteProcess(
        cmd=['gnome-terminal', '--wait', '--',
             'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen',
    )

    # Os controladores só são ativados depois que o robô existe no Gazebo
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, diff_drive_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        start_controllers,
        cmd_vel_relay,
        teleop_keyboard,
    ])
