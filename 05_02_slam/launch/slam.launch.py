"""
SLAM 2D com o slam_toolbox (modo online assíncrono) no robô my_robot.

Sobe o Gazebo com a sala fechada (worlds/slam_simples.world), o slam_toolbox e o RViz.
Três cenários de ruído para comparar (argumento ``cenario``):

* ``sem_ruido``: lidar sem ruído e odometria groundtruth (tudo groundtruth);
* ``lidar``: ruído no lidar 2D, odometria groundtruth;
* ``lidar_odom``: ruído no lidar 2D e erro na odometria.

A TF odom -> chassis que o SLAM usa vem do nó ``odometria_slam`` (pose exata do Gazebo +
erro controlado por ``ruido_odom``). Com ``fonte_odom:=rodas`` ela volta a ser a do
diff_drive_controller (odometria real das rodas, sem controle sobre o erro).

Por padrão o robô faz sozinho o mesmo percurso pela sala (``percurso:=true``) e o nó
``avaliacao_slam`` grava mapa, trajetória e métricas em ``saida/<cenario>``. Com
``percurso:=false`` dirija pelo teleop_twist_keyboard (/cmd_vel) e salve com Ctrl+C.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# cenario -> (escala_ruido, ruido_lidar [m], ruido_odom [escala de slam_cenarios.yaml])
CENARIOS = {
    'sem_ruido': ('0.0', '0.0', '0.0'),
    'lidar': ('1.0', '0.03', '0.0'),
    'lidar_odom': ('1.0', '0.03', '1.0'),
}
DESCRICOES = {
    'sem_ruido': 'Sem ruído (lidar e odometria groundtruth)',
    'lidar': 'Ruído no lidar 2D, odometria groundtruth',
    'lidar_odom': 'Ruído no lidar 2D e na odometria',
}


def _verdadeiro(texto):
    return texto.strip().lower() in ('true', '1', 'yes', 'sim')


def _criar(context):
    pkg_share = get_package_share_directory('estimacao_estados')
    arg = {nome: LaunchConfiguration(nome).perform(context) for nome in (
        'cenario', 'world', 'escala_ruido', 'ruido_lidar', 'ruido_odom', 'fonte_odom',
        'slam_params', 'rviz', 'gui', 'percurso', 'avaliar', 'saida', 'encerrar')}

    cenario = arg['cenario']
    if cenario not in CENARIOS:
        raise RuntimeError(f'cenario:={cenario!r} inválido; use um de {sorted(CENARIOS)}')
    # Argumentos vazios herdam o valor do cenário; preenchidos sobrescrevem.
    escala, ruido_lidar, ruido_odom = (
        arg[nome] or padrao
        for nome, padrao in zip(('escala_ruido', 'ruido_lidar', 'ruido_odom'),
                                CENARIOS[cenario]))
    rodas = arg['fonte_odom'] == 'rodas'
    if arg['fonte_odom'] not in ('groundtruth', 'rodas'):
        raise RuntimeError("fonte_odom deve ser 'groundtruth' ou 'rodas'")
    parametros_cenario = os.path.join(pkg_share, 'config', 'slam_cenarios.yaml')
    saida = os.path.join(os.path.expanduser(arg['saida']), cenario)

    # Robô no Gazebo, SEM a TF estática world -> odom: no SLAM o pai do odom é o map.
    simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robo_gazebo.launch.py')),
        launch_arguments={
            # /clock a 1 kHz: carimbos de tempo da odometria sincronizados com o /scan
            'params_file': os.path.join(pkg_share, 'config', 'gazebo_params_slam.yaml'),
            'world': arg['world'],
            'escala_ruido': escala,
            'ruido_lidar': ruido_lidar,
            'publicar_world_odom': 'false',
            'odom_tf_rodas': 'true' if rodas else 'false',
            'gui': arg['gui'],
        }.items(),
    )

    acoes = [
        LogInfo(msg=f'[slam] cenario={cenario}: {DESCRICOES[cenario]} | escala_ruido={escala}'
                    f' ruido_lidar={ruido_lidar} m (x escala) | odometria='
                    + ('rodas (diff_drive_controller)' if rodas
                       else f'groundtruth com ruido_odom={ruido_odom}')),
        simulacao,
    ]

    if not rodas:
        acoes.append(Node(
            package='estimacao_estados',
            executable='odometria_slam',
            output='screen',
            parameters=[parametros_cenario, {'escala': float(ruido_odom)}],
        ))

    # Nó do repositório SteveMacenski/slam_toolbox (compilado em ~/ros2_ws/src/slam_toolbox).
    # Assina /scan e a TF odom -> chassis; publica /map e a TF map -> odom.
    atrasados = [Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[arg['slam_params']],
    )]
    if _verdadeiro(arg['rviz']):
        atrasados.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'slam.rviz')],
            parameters=[{'use_sim_time': True}],
        ))
    if _verdadeiro(arg['percurso']):
        atrasados.append(Node(
            package='estimacao_estados',
            executable='percurso_sala',
            output='screen',
            parameters=[parametros_cenario],
        ))
    if _verdadeiro(arg['avaliar']):
        avaliacao = Node(
            package='estimacao_estados',
            executable='avaliacao_slam',
            output='screen',
            parameters=[parametros_cenario, {
                'saida': saida,
                'mundo': arg['world'],
                'cenario': cenario,
                'descricao': DESCRICOES[cenario],
                'encerrar_ao_concluir': _verdadeiro(arg['encerrar']),
            }],
        )
        atrasados.append(avaliacao)
        if _verdadeiro(arg['encerrar']):
            acoes.append(RegisterEventHandler(OnProcessExit(
                target_action=avaliacao,
                on_exit=[EmitEvent(event=Shutdown(reason='avaliação do SLAM concluída'))])))

    # Espera o Gazebo publicar /clock e os controladores subirem (odom -> chassis);
    # antes disso o slam_toolbox só reclamaria de TF ausente.
    acoes.append(TimerAction(period=5.0, actions=atrasados))
    return acoes


def generate_launch_description():
    pkg_share = get_package_share_directory('estimacao_estados')
    mundo_padrao = os.path.join(pkg_share, 'worlds', 'slam_simples.world')
    params_padrao = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    saida_padrao = os.path.join('~', 'ros2_ws', 'src', '05_02_slam', 'resultados')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cenario', default_value='sem_ruido',
            description='sem_ruido | lidar | lidar_odom'),
        DeclareLaunchArgument(
            'world', default_value=mundo_padrao,
            description='Mundo do Gazebo usado no mapeamento'),
        DeclareLaunchArgument(
            'escala_ruido', default_value='',
            description='Multiplica o ruído de todos os sensores (vazio = do cenário)'),
        DeclareLaunchArgument(
            'ruido_lidar', default_value='',
            description='Desvio-padrão do lidar em m, antes da escala (vazio = do cenário)'),
        DeclareLaunchArgument(
            'ruido_odom', default_value='',
            description='Multiplica o erro de odometria de slam_cenarios.yaml '
                        '(0 = groundtruth; vazio = do cenário)'),
        DeclareLaunchArgument(
            'fonte_odom', default_value='groundtruth',
            description='groundtruth (odometria_slam, com ruido_odom) | rodas '
                        '(diff_drive_controller)'),
        DeclareLaunchArgument(
            'slam_params', default_value=params_padrao,
            description='Arquivo YAML de parâmetros do slam_toolbox'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abre o RViz com o mapa, o lidar e a árvore TF'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Abre a janela do Gazebo'),
        DeclareLaunchArgument(
            'percurso', default_value='true',
            description='O robô faz sozinho o percurso de slam_cenarios.yaml '
                        '(false = teleop)'),
        DeclareLaunchArgument(
            'avaliar', default_value='true',
            description='Grava mapa, trajetória e métricas (nó avaliacao_slam)'),
        DeclareLaunchArgument(
            'saida', default_value=saida_padrao,
            description='Pasta dos resultados (cada cenário numa subpasta)'),
        DeclareLaunchArgument(
            'encerrar', default_value='false',
            description='Fecha tudo depois de salvar os resultados do percurso'),
        OpaqueFunction(function=_criar),
    ])
